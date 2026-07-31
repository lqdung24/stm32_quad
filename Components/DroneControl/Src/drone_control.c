#include "drone_control.h"

#include "drone_cobs.h"
#include "drone_protocol.h"
#include "motor_pwm.h"

#include <stdbool.h>
#include <string.h>

#define CONTROL_UART_RX_CHUNK_SIZE          64U
#define CONTROL_UART_RING_SIZE              256U
#define CONTROL_UART_LOG_RING_SIZE          512U
#define CONTROL_UART_ENCODED_FRAME_SIZE     64U
#define CONTROL_ESC_MIN_PULSE_US            1000U
#define CONTROL_ESC_MAX_PULSE_US            2000U
#define CONTROL_DEG_TO_RAD                   0.01745329252f
#define CONTROL_ROLL_PITCH_MAX_RATE_RAD_S   (200.0f * CONTROL_DEG_TO_RAD)
#define CONTROL_YAW_MAX_RATE_RAD_S           (150.0f * CONTROL_DEG_TO_RAD)

typedef struct
{
  UART_HandleTypeDef *uart;
  MotorPwm_Handle_t motors;
  RateControl rate_control;
  volatile uint16_t rx_head;
  volatile uint16_t rx_tail;
  volatile uint16_t log_head;
  volatile uint16_t log_tail;
  uint8_t rx_chunk[CONTROL_UART_RX_CHUNK_SIZE];
  uint8_t rx_ring[CONTROL_UART_RING_SIZE];
  uint8_t log_ring[CONTROL_UART_LOG_RING_SIZE];
  uint8_t encoded_rx[CONTROL_UART_ENCODED_FRAME_SIZE];
  size_t encoded_rx_length;
  uint8_t tx_frame[DRONE_STATUS_PACKET_SIZE + 3U];
  DroneSystemState state;
  uint16_t error_flags;
  uint16_t active_session;
  uint16_t last_sequence;
  uint16_t requested_throttle;
  uint16_t applied_throttle;
  uint16_t status_sequence;
  uint16_t packets_this_second;
  uint16_t uart_rx_rate;
  uint32_t rate_window_ms;
  uint32_t last_valid_control_ms;
  uint32_t last_status_ms;
  bool initialized;
  bool has_session;
  bool has_sequence;
  bool has_valid_control;
  bool require_disarm_cycle;
} DroneControlContext;

static DroneControlContext control;

static void start_uart_receive(void);
static void process_uart_bytes(uint32_t now_ms);
static void process_raw_packet(const uint8_t *packet, size_t length,
                               uint32_t now_ms);
static void process_control_command(const DroneControlCommand *command,
                                    uint32_t now_ms);
static void enter_failsafe(uint16_t reason);
static void disarm_output(void);
static bool apply_throttle(uint16_t requested);
static void send_status(uint32_t now_ms);

/*
 * Initial bench gains. Output is a normalized mixer correction, not us.
 * These values must be tuned for the actual airframe before flight.
 */
static const RateControlConfig rate_control_config = {
    .pid = {
        [RATE_CONTROL_ROLL] = {
            .kp = 45.0f,
            .ki = 20.0f,
            .kd = 0.8f,
            .integral_limit = 60.0f,
            .output_limit = 200.0f,
            .derivative_cutoff_hz = 20.0f,
        },
        [RATE_CONTROL_PITCH] = {
            .kp = 45.0f,
            .ki = 20.0f,
            .kd = 0.8f,
            .integral_limit = 60.0f,
            .output_limit = 200.0f,
            .derivative_cutoff_hz = 20.0f,
        },
        [RATE_CONTROL_YAW] = {
            .kp = 35.0f,
            .ki = 10.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .output_limit = 150.0f,
            .derivative_cutoff_hz = 0.0f,
        },
    },
    .maximum_rate_rad_s = {
        [RATE_CONTROL_ROLL] = CONTROL_ROLL_PITCH_MAX_RATE_RAD_S,
        [RATE_CONTROL_PITCH] = CONTROL_ROLL_PITCH_MAX_RATE_RAD_S,
        [RATE_CONTROL_YAW] = CONTROL_YAW_MAX_RATE_RAD_S,
    },
};

void DroneControl_Init(UART_HandleTypeDef *uart, TIM_HandleTypeDef *motor_timer)
{
  const MotorPwm_Config_t motor_config = {
      .timer = motor_timer,
      .channel = {
          TIM_CHANNEL_1,
          TIM_CHANNEL_2,
          TIM_CHANNEL_3,
          TIM_CHANNEL_4,
      },
      .disarmed_pulse_us = CONTROL_ESC_MIN_PULSE_US,
      .minimum_pulse_us = CONTROL_ESC_MIN_PULSE_US,
      .maximum_pulse_us = CONTROL_ESC_MAX_PULSE_US,
  };

  memset(&control, 0, sizeof(control));
  control.uart = uart;
  control.state = DRONE_STATE_BOOT;
  control.rate_window_ms = HAL_GetTick();
  control.last_status_ms = control.rate_window_ms;

  if ((uart == NULL) || (motor_timer == NULL) ||
      !RateControl_Init(&control.rate_control, &rate_control_config) ||
      !MotorPwm_Init(&control.motors, &motor_config))
  {
    control.state = DRONE_STATE_ERROR;
    control.error_flags |= DRONE_ERROR_PWM_INIT;
  }
  else
  {
    control.state = DRONE_STATE_DISARMED;
    control.initialized = true;
  }
  start_uart_receive();
}

void DroneControl_UpdateBodyRates(float roll_rad_s,
                                  float pitch_rad_s,
                                  float yaw_rad_s,
                                  float dt_s)
{
  const float measured_rad_s[RATE_CONTROL_AXIS_COUNT] = {
      roll_rad_s,
      pitch_rad_s,
      yaw_rad_s,
  };

  if (!control.initialized ||
      (control.state != DRONE_STATE_ARMED) ||
      (control.applied_throttle == 0U))
  {
    RateControl_Reset(&control.rate_control);
    return;
  }

  if (!RateControl_Update(&control.rate_control, measured_rad_s, dt_s))
  {
    RateControl_Reset(&control.rate_control);
  }
}

bool DroneControl_GetRateControlDebug(RateControlDebug *debug)
{
  return RateControl_GetDebug(&control.rate_control, debug);
}

void DroneControl_Process(uint32_t now_ms)
{
  process_uart_bytes(now_ms);

  if (control.has_valid_control &&
      ((uint32_t)(now_ms - control.last_valid_control_ms) >=
       DRONE_CONTROL_LINK_TIMEOUT_MS))
  {
    control.has_valid_control = false;
    enter_failsafe(DRONE_ERROR_UART_LINK_LOST);
  }

  if ((uint32_t)(now_ms - control.rate_window_ms) >= 1000U)
  {
    control.uart_rx_rate = control.packets_this_second;
    control.packets_this_second = 0U;
    control.rate_window_ms = now_ms;
  }

  if ((uint32_t)(now_ms - control.last_status_ms) >=
      DRONE_CONTROL_STATUS_PERIOD_MS)
  {
    control.last_status_ms = now_ms;
    send_status(now_ms);
  }
}

void DroneControl_OnUartRxEvent(UART_HandleTypeDef *uart, uint16_t size)
{
  uint16_t i;

  if ((uart != control.uart) || (size > CONTROL_UART_RX_CHUNK_SIZE))
  {
    return;
  }
  for (i = 0U; i < size; ++i)
  {
    const uint16_t next =
        (uint16_t)((control.rx_head + 1U) % CONTROL_UART_RING_SIZE);
    const uint16_t log_next =
        (uint16_t)((control.log_head + 1U) % CONTROL_UART_LOG_RING_SIZE);

    if (log_next != control.log_tail)
    {
      control.log_ring[control.log_head] = control.rx_chunk[i];
      control.log_head = log_next;
    }

    if (next == control.rx_tail)
    {
      control.error_flags |= DRONE_ERROR_INVALID_PACKET;
      break;
    }
    control.rx_ring[control.rx_head] = control.rx_chunk[i];
    control.rx_head = next;
  }
  start_uart_receive();
}

void DroneControl_OnUartError(UART_HandleTypeDef *uart)
{
  if (uart == control.uart)
  {
    control.error_flags |= DRONE_ERROR_UART_LINK_LOST;
    start_uart_receive();
  }
}

size_t DroneControl_ReadUartRxLog(uint8_t *output, size_t capacity)
{
  size_t count = 0U;

  if (output == NULL)
  {
    return 0U;
  }

  while ((count < capacity) && (control.log_tail != control.log_head))
  {
    output[count++] = control.log_ring[control.log_tail];
    control.log_tail =
        (uint16_t)((control.log_tail + 1U) % CONTROL_UART_LOG_RING_SIZE);
  }
  return count;
}

static void start_uart_receive(void)
{
  if (control.uart != NULL)
  {
    (void)HAL_UARTEx_ReceiveToIdle_IT(control.uart,
                                     control.rx_chunk,
                                     sizeof(control.rx_chunk));
  }
}

static void process_uart_bytes(uint32_t now_ms)
{
  while (control.rx_tail != control.rx_head)
  {
    const uint8_t byte = control.rx_ring[control.rx_tail];
    control.rx_tail =
        (uint16_t)((control.rx_tail + 1U) % CONTROL_UART_RING_SIZE);

    if (byte == 0U)
    {
      uint8_t raw[DRONE_PROTOCOL_MAX_PACKET_SIZE];
      const size_t raw_length =
          DroneCobs_Decode(control.encoded_rx, control.encoded_rx_length,
                          raw, sizeof(raw));
      if (raw_length != 0U)
      {
        process_raw_packet(raw, raw_length, now_ms);
      }
      else if (control.encoded_rx_length != 0U)
      {
        control.error_flags |= DRONE_ERROR_INVALID_PACKET;
      }
      control.encoded_rx_length = 0U;
    }
    else if (control.encoded_rx_length < sizeof(control.encoded_rx))
    {
      control.encoded_rx[control.encoded_rx_length++] = byte;
    }
    else
    {
      control.encoded_rx_length = 0U;
      control.error_flags |= DRONE_ERROR_INVALID_PACKET;
    }
  }
}

static void process_raw_packet(const uint8_t *packet, size_t length,
                               uint32_t now_ms)
{
  DroneControlCommand command;
  const DroneProtocolResult result =
      DroneProtocol_DecodeControl(packet, length, &command);

  if (result != DRONE_PROTOCOL_OK)
  {
    control.error_flags |=
        (result == DRONE_PROTOCOL_CRC_ERROR)
            ? DRONE_ERROR_CRC
            : DRONE_ERROR_INVALID_PACKET;
    return;
  }
  process_control_command(&command, now_ms);
}

static void process_control_command(const DroneControlCommand *command,
                                    uint32_t now_ms)
{
  const bool arm_requested =
      (command->header.flags & DRONE_CONTROL_FLAG_ARM_REQUEST) != 0U;
  const bool emergency_stop =
      (command->header.flags & DRONE_CONTROL_FLAG_EMERGENCY_STOP) != 0U;
  const bool new_session =
      !control.has_session ||
      (command->header.session_id != control.active_session);

  if (new_session)
  {
    disarm_output();
    control.active_session = command->header.session_id;
    control.has_session = true;
    control.has_sequence = false;
    control.require_disarm_cycle = true;
    control.error_flags |= DRONE_ERROR_SESSION_CHANGED;
    if (control.state != DRONE_STATE_ERROR)
    {
      control.state = DRONE_STATE_DISARMED;
    }
  }

  if (control.has_sequence &&
      !DroneProtocol_IsSequenceNewer(command->header.sequence,
                                     control.last_sequence))
  {
    control.error_flags |= DRONE_ERROR_SEQUENCE;
    return;
  }

  control.has_sequence = true;
  control.last_sequence = command->header.sequence;
  control.last_valid_control_ms = now_ms;
  control.has_valid_control = true;
  ++control.packets_this_second;
  control.error_flags &= (uint16_t)~DRONE_ERROR_UART_LINK_LOST;
  control.requested_throttle = command->throttle;
  RateControl_SetCommand(&control.rate_control,
                         command->roll,
                         command->pitch,
                         command->yaw);

  if (control.state == DRONE_STATE_ERROR)
  {
    disarm_output();
    return;
  }

  if (emergency_stop)
  {
    enter_failsafe(DRONE_ERROR_FAILSAFE_ACTIVE);
    return;
  }

  /*
   * A zero-throttle command with ARM_REQUEST clear is the explicit disarm
   * cycle required after startup, session changes, e-stop, or failsafe.
   */
  if (!arm_requested)
  {
    disarm_output();
    control.state = DRONE_STATE_DISARMED;
    if (command->throttle == 0U)
    {
      control.require_disarm_cycle = false;
      control.error_flags &=
          (uint16_t)~(DRONE_ERROR_FAILSAFE_ACTIVE |
                      DRONE_ERROR_ARM_REJECTED);
    }
    return;
  }

  if (control.state == DRONE_STATE_FAILSAFE ||
      control.require_disarm_cycle)
  {
    control.error_flags |= DRONE_ERROR_ARM_REJECTED;
    disarm_output();
    return;
  }

  if (control.state == DRONE_STATE_DISARMED)
  {
    if (command->throttle != 0U)
    {
      control.error_flags |= DRONE_ERROR_ARM_REJECTED;
      return;
    }
    if (!MotorPwm_Arm(&control.motors))
    {
      control.state = DRONE_STATE_ERROR;
      control.error_flags |= DRONE_ERROR_PWM_INIT;
      return;
    }
    control.state = DRONE_STATE_ARMED;
  }

  if (control.state == DRONE_STATE_ARMED)
  {
    if (!apply_throttle(command->throttle))
    {
      control.state = DRONE_STATE_ERROR;
      control.error_flags |= DRONE_ERROR_PWM_INIT;
      disarm_output();
    }
  }
}

static void enter_failsafe(uint16_t reason)
{
  disarm_output();
  control.require_disarm_cycle = true;
  if (control.state != DRONE_STATE_ERROR)
  {
    control.state = DRONE_STATE_FAILSAFE;
  }
  control.error_flags |= (uint16_t)(reason | DRONE_ERROR_FAILSAFE_ACTIVE);
}

static void disarm_output(void)
{
  MotorPwm_Disarm(&control.motors);
  RateControl_Reset(&control.rate_control);
  control.applied_throttle = 0U;
}

static bool apply_throttle(uint16_t requested)
{
  uint8_t i;
  uint16_t pulses[MOTOR_PWM_MOTOR_COUNT] = {
      CONTROL_ESC_MIN_PULSE_US,
      CONTROL_ESC_MIN_PULSE_US,
      CONTROL_ESC_MIN_PULSE_US,
      CONTROL_ESC_MIN_PULSE_US,
  };
  uint16_t applied = requested;

  if (applied > DRONE_CONTROL_MAX_TEST_THROTTLE)
  {
    applied = DRONE_CONTROL_MAX_TEST_THROTTLE;
    control.error_flags |= DRONE_ERROR_THROTTLE_CLAMPED;
  }
  else
  {
    control.error_flags &= (uint16_t)~DRONE_ERROR_THROTTLE_CLAMPED;
  }

  const uint16_t pulse_us = (uint16_t)(
      CONTROL_ESC_MIN_PULSE_US +
      (((uint32_t)applied *
        (CONTROL_ESC_MAX_PULSE_US - CONTROL_ESC_MIN_PULSE_US)) /
       1000U));
  for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
  {
    pulses[i] = pulse_us;
  }
  if (!MotorPwm_SetAllPulseUs(&control.motors, pulses))
  {
    return false;
  }
  control.applied_throttle = applied;
  return true;
}

static void send_status(uint32_t now_ms)
{
  DroneSystemStatus status = {
      .header = {
          .sequence = control.status_sequence++,
          .session_id = control.active_session,
          .sender_time_ms = now_ms,
      },
      .last_control_sequence = control.last_sequence,
      .requested_throttle = control.requested_throttle,
      .applied_throttle = control.applied_throttle,
      .pwm_pulse_us = MotorPwm_GetPulseUs(&control.motors, 0U),
      .state = (uint8_t)control.state,
      .error_flags = control.error_flags,
      .uart_rx_rate = control.uart_rx_rate,
  };
  uint8_t raw[DRONE_STATUS_PACKET_SIZE];
  size_t encoded_length;

  if ((control.uart == NULL) ||
      (DroneProtocol_EncodeStatus(&status, raw) != DRONE_PROTOCOL_OK))
  {
    return;
  }
  encoded_length = DroneCobs_Encode(raw, sizeof(raw),
                                   control.tx_frame,
                                   sizeof(control.tx_frame) - 1U);
  if (encoded_length == 0U)
  {
    return;
  }
  control.tx_frame[encoded_length++] = 0U;
  (void)HAL_UART_Transmit_IT(control.uart, control.tx_frame,
                            (uint16_t)encoded_length);
}
