#include "drone_control.h"

#include "dp_cobs.h"
#include "dp_protocol.h"
#include "../../MotorMixer/Inc/motor_mixer.h"
#include "motor_pwm.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>

#define CONTROL_UART_RX_CHUNK_SIZE 64U
#define CONTROL_UART_RING_SIZE 256U
#define CONTROL_UART_LOG_RING_SIZE 512U
#define CONTROL_UART_ENCODED_FRAME_SIZE 64U
#define CONTROL_ESC_MIN_PULSE_US 1000U
#define CONTROL_ESC_MAX_PULSE_US 2000U
#define CONTROL_PILOT_IDLE_PULSE_US 1225U
#define CONTROL_PILOT_MAX_PULSE_US 1800U
/*
 * No-prop first-rotation measurements, expressed above the 1000 us disarmed
 * pulse. Rotation is viewed from above: M1/M4 CW and M2/M3 CCW. The active
 * floor adds 20 us so each motor stays reliably above its measured threshold.
 */
#define CONTROL_M1_FIRST_ROTATION_OFFSET_US 200U
#define CONTROL_M2_FIRST_ROTATION_OFFSET_US 205U
#define CONTROL_M3_FIRST_ROTATION_OFFSET_US 190U
#define CONTROL_M4_FIRST_ROTATION_OFFSET_US 205U
#define CONTROL_MOTOR_IDLE_MARGIN_US 20U
#define CONTROL_M1_IDLE_PULSE_US                                      \
    (CONTROL_ESC_MIN_PULSE_US + CONTROL_M1_FIRST_ROTATION_OFFSET_US + \
     CONTROL_MOTOR_IDLE_MARGIN_US)
#define CONTROL_M2_IDLE_PULSE_US                                      \
    (CONTROL_ESC_MIN_PULSE_US + CONTROL_M2_FIRST_ROTATION_OFFSET_US + \
     CONTROL_MOTOR_IDLE_MARGIN_US)
#define CONTROL_M3_IDLE_PULSE_US                                      \
    (CONTROL_ESC_MIN_PULSE_US + CONTROL_M3_FIRST_ROTATION_OFFSET_US + \
     CONTROL_MOTOR_IDLE_MARGIN_US)
#define CONTROL_M4_IDLE_PULSE_US                                      \
    (CONTROL_ESC_MIN_PULSE_US + CONTROL_M4_FIRST_ROTATION_OFFSET_US + \
     CONTROL_MOTOR_IDLE_MARGIN_US)
#define CONTROL_MOTOR_THRESHOLD_TEST_MODE 0U
#define CONTROL_DEG_TO_RAD 0.01745329252f
#define CONTROL_ROLL_PITCH_MAX_RATE_RAD_S (200.0f * CONTROL_DEG_TO_RAD)
#define CONTROL_YAW_MAX_RATE_RAD_S (150.0f * CONTROL_DEG_TO_RAD)

typedef struct
{
    UART_HandleTypeDef *uart;
    MotorPwm_Handle_t *motors;
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
    uint8_t tx_frame[CONTROL_UART_ENCODED_FRAME_SIZE];
    DroneSystemState state;
    uint16_t error_flags;
    uint16_t active_session;
    uint16_t last_sequence;
    uint16_t requested_throttle;
    uint16_t applied_throttle;
    uint8_t motor_test_selection;
    uint16_t status_sequence;
    uint16_t flight_telemetry_sequence;
    uint16_t packets_this_second;
    uint16_t uart_rx_rate;
    uint32_t rate_window_ms;
    uint32_t last_valid_control_ms;
    uint32_t last_status_ms;
    uint32_t last_flight_telemetry_ms;
    DroneFlightTelemetry flight_telemetry;
    volatile uint32_t mixer_telemetry_sequence;
    DroneMixerTelemetry mixer_telemetry;
    bool initialized;
    bool has_session;
    bool has_sequence;
    bool has_valid_control;
    bool flight_telemetry_available;
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
static float throttle_to_pwm(uint16_t throttle);
static bool apply_mixed_output(float roll_correction,
                               float pitch_correction,
                               float yaw_correction);
static void publish_mixer_telemetry(const MotorMixerResult *mix,
                                    const uint16_t pulses[MOTOR_PWM_MOTOR_COUNT],
                                    float roll_correction,
                                    float pitch_correction,
                                    float yaw_correction,
                                    bool active);
static void publish_disarmed_telemetry(void);
static bool send_status(uint32_t now_ms);
static bool send_flight_telemetry(void);
static bool try_send_packet(const uint8_t *raw, size_t raw_length);
static bool scale_to_i16(float value, float scale, int16_t *output);

static const MotorMixerOutputConfig mixer_output_config = {
    .disarmed_pulse_us = CONTROL_ESC_MIN_PULSE_US,
    .idle_pulse_us = {
        [MOTOR_MIXER_M1_FRONT_LEFT] = CONTROL_M1_IDLE_PULSE_US,
        [MOTOR_MIXER_M2_REAR_LEFT] = CONTROL_M2_IDLE_PULSE_US,
        [MOTOR_MIXER_M3_FRONT_RIGHT] = CONTROL_M3_IDLE_PULSE_US,
        [MOTOR_MIXER_M4_REAR_RIGHT] = CONTROL_M4_IDLE_PULSE_US,
    },
    .maximum_pulse_us = CONTROL_ESC_MAX_PULSE_US,
};

_Static_assert(CONTROL_PILOT_IDLE_PULSE_US >= CONTROL_M1_IDLE_PULSE_US,
               "pilot idle must cover M1 idle floor");
_Static_assert(CONTROL_PILOT_IDLE_PULSE_US >= CONTROL_M2_IDLE_PULSE_US,
               "pilot idle must cover M2 idle floor");
_Static_assert(CONTROL_PILOT_IDLE_PULSE_US >= CONTROL_M3_IDLE_PULSE_US,
               "pilot idle must cover M3 idle floor");
_Static_assert(CONTROL_PILOT_IDLE_PULSE_US >= CONTROL_M4_IDLE_PULSE_US,
               "pilot idle must cover M4 idle floor");
_Static_assert(CONTROL_PILOT_MAX_PULSE_US <= CONTROL_ESC_MAX_PULSE_US,
               "pilot maximum must leave valid actuator headroom");

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

void DroneControl_Init(UART_HandleTypeDef *uart, MotorPwm_Handle_t *motors)
{
    memset(&control, 0, sizeof(control));
    control.uart = uart;
    control.motors = motors;
    control.state = DRONE_STATE_BOOT;
    control.rate_window_ms = HAL_GetTick();
    control.last_status_ms = control.rate_window_ms;

    if ((uart == NULL) || (motors == NULL) ||
        !RateControl_Init(&control.rate_control, &rate_control_config) ||
        !MotorPwm_IsAttached(motors))
    {
        control.state = DRONE_STATE_ERROR;
        control.error_flags |= DRONE_ERROR_PWM_INIT;
    }
    else
    {
        control.state = DRONE_STATE_DISARMED;
        control.initialized = true;
        publish_disarmed_telemetry();
    }
    start_uart_receive();
}

bool DroneControl_UpdateBodyRates(float roll_rad_s,
                                  float pitch_rad_s,
                                  float yaw_rad_s,
                                  float dt_s)
{
#if CONTROL_MOTOR_THRESHOLD_TEST_MODE
    (void)roll_rad_s;
    (void)pitch_rad_s;
    (void)yaw_rad_s;
    (void)dt_s;
    RateControl_Reset(&control.rate_control);
    return false;
#else
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
        return false;
    }

    if (!RateControl_Update(&control.rate_control, measured_rad_s, dt_s))
    {
        RateControl_Reset(&control.rate_control);
        (void)apply_mixed_output(0.0f, 0.0f, 0.0f);
        return false;
    }

    if (!apply_mixed_output(
            control.rate_control.debug.output[RATE_CONTROL_ROLL],
            control.rate_control.debug.output[RATE_CONTROL_PITCH],
            control.rate_control.debug.output[RATE_CONTROL_YAW]))
    {
        control.state = DRONE_STATE_ERROR;
        control.error_flags |= DRONE_ERROR_PWM_INIT;
        disarm_output();
        return false;
    }
    return true;
#endif
}

bool DroneControl_GetRateControlDebug(RateControlDebug *debug)
{
    return RateControl_GetDebug(&control.rate_control, debug);
}

bool DroneControl_GetMixerTelemetry(DroneMixerTelemetry *telemetry)
{
    uint32_t sequence_before;
    uint32_t sequence_after;
    uint8_t axis;
    uint8_t motor;

    if ((telemetry == NULL) || !control.initialized)
    {
        return false;
    }

    do
    {
        sequence_before = control.mixer_telemetry_sequence;
        if ((sequence_before & 1U) != 0U)
        {
            continue;
        }
        __DMB();
        for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
        {
            telemetry->pid_output[axis] =
                control.mixer_telemetry.pid_output[axis];
        }
        for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
        {
            telemetry->motor_command[motor] =
                control.mixer_telemetry.motor_command[motor];
            telemetry->pulse_us[motor] =
                control.mixer_telemetry.pulse_us[motor];
        }
        telemetry->applied_throttle =
            control.mixer_telemetry.applied_throttle;
        telemetry->applied_collective =
            control.mixer_telemetry.applied_collective;
        telemetry->correction_scale =
            control.mixer_telemetry.correction_scale;
        telemetry->active = control.mixer_telemetry.active;
        telemetry->collective_shifted =
            control.mixer_telemetry.collective_shifted;
        telemetry->correction_scaled =
            control.mixer_telemetry.correction_scaled;
        __DMB();
        sequence_after = control.mixer_telemetry_sequence;
    } while ((sequence_before != sequence_after) ||
             ((sequence_after & 1U) != 0U));
    return true;
}

void DroneControl_PublishFlightTelemetrySample(uint32_t timestamp_ms,
                                               bool attitude_valid,
                                               float roll_deg,
                                               float pitch_deg,
                                               float yaw_deg,
                                               float gyro_roll_rad_s,
                                               float gyro_pitch_rad_s,
                                               float gyro_yaw_rad_s)
{
    const float attitude_deg[3] = {roll_deg, pitch_deg, yaw_deg};
    const float gyro_rad_s[3] = {
        gyro_roll_rad_s,
        gyro_pitch_rad_s,
        gyro_yaw_rad_s,
    };
    DroneFlightTelemetry telemetry = {
        .header = {
            .sequence = 0U,
            .session_id = control.active_session,
            .sender_time_ms = timestamp_ms,
        },
        .state = (uint8_t)control.state,
        .actuators_active = control.mixer_telemetry.active,
        .attitude_valid = attitude_valid,
    };
    uint8_t axis;
    uint8_t motor;

    if (!control.initialized || (telemetry.state > DRONE_STATE_ERROR))
    {
        return;
    }
    for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
    {
        if (!scale_to_i16(attitude_deg[axis], 100.0f,
                          &telemetry.attitude_cdeg[axis]) ||
            !scale_to_i16(gyro_rad_s[axis], 1000.0f,
                          &telemetry.gyro_mrad_s[axis]) ||
            !scale_to_i16(control.rate_control.debug.target_rad_s[axis],
                          1000.0f,
                          &telemetry.rate_setpoint_mrad_s[axis]) ||
            !scale_to_i16(control.mixer_telemetry.pid_output[axis],
                          100.0f,
                          &telemetry.pid_command_centi[axis]))
        {
            return;
        }
    }
    for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
    {
        telemetry.motor_pwm_us[motor] = control.mixer_telemetry.pulse_us[motor];
        if ((telemetry.motor_pwm_us[motor] < CONTROL_ESC_MIN_PULSE_US) ||
            (telemetry.motor_pwm_us[motor] > CONTROL_ESC_MAX_PULSE_US))
        {
            return;
        }
    }

    control.flight_telemetry = telemetry;
    control.flight_telemetry_available = true;
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
        if (send_status(now_ms))
        {
            control.last_status_ms = now_ms;
        }
    }

    if (control.flight_telemetry_available &&
        ((uint32_t)(now_ms - control.last_flight_telemetry_ms) >=
         DRONE_CONTROL_FLIGHT_TELEMETRY_PERIOD_MS) &&
        send_flight_telemetry())
    {
        control.last_flight_telemetry_ms = now_ms;
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

    if ((command->aux1 > DRONE_CONTROL_MOTOR_SELECT_MAX) ||
        (command->aux2 != 0U))
    {
        control.error_flags |= DRONE_ERROR_INVALID_PACKET;
        disarm_output();
        return;
    }

    if ((control.state == DRONE_STATE_ARMED) &&
        (command->aux1 != control.motor_test_selection) &&
        (command->throttle != 0U))
    {
        enter_failsafe(DRONE_ERROR_INVALID_PACKET);
        return;
    }

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
    control.motor_test_selection = (uint8_t)command->aux1;
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
        if (!MotorPwm_Arm(control.motors))
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
    MotorPwm_Disarm(control.motors);
    RateControl_Reset(&control.rate_control);
    control.applied_throttle = 0U;
    publish_disarmed_telemetry();
}

static bool apply_throttle(uint16_t requested)
{
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

    control.applied_throttle = applied;
    /*
     * A control packet may arrive between gyro samples. Retain the most recent
     * PID correction when applying the new collective so packet handling does
     * not briefly overwrite the stabilized output with equal motor commands.
     * RateControl_Reset() clears these values on disarm/failsafe/invalid input.
     */
    return apply_mixed_output(
        control.rate_control.debug.output[RATE_CONTROL_ROLL],
        control.rate_control.debug.output[RATE_CONTROL_PITCH],
        control.rate_control.debug.output[RATE_CONTROL_YAW]);
}

static float throttle_to_pwm(uint16_t throttle)
{
    float normalized;

    if (throttle == 0U)
    {
        return (float)CONTROL_ESC_MIN_PULSE_US;
    }

    normalized = (float)(throttle - 1U) /
                 (float)(DRONE_CONTROL_MAX_TEST_THROTTLE - 1U);
    return (float)CONTROL_PILOT_IDLE_PULSE_US +
           (normalized * (float)(CONTROL_PILOT_MAX_PULSE_US -
                                 CONTROL_PILOT_IDLE_PULSE_US));
}

static bool apply_mixed_output(float roll_correction,
                               float pitch_correction,
                               float yaw_correction)
{
    MotorMixerResult mix;
    uint16_t pulses[MOTOR_PWM_MOTOR_COUNT];
    const float collective_pwm =
        throttle_to_pwm(control.applied_throttle);
    const float collective_command =
        collective_pwm - (float)CONTROL_ESC_MIN_PULSE_US;
    const bool active =
        (control.state == DRONE_STATE_ARMED) &&
        (control.applied_throttle > 0U);

    if (!MotorMixer_MixQuadX(collective_command,
                             roll_correction,
                             pitch_correction,
                             yaw_correction,
                             &mix) ||
        !MotorMixer_MapToPulseUs(&mixer_output_config,
                                 mix.command,
                                 active,
                                 pulses))
    {
        return false;
    }
#if CONTROL_MOTOR_THRESHOLD_TEST_MODE
    if (active &&
        (control.motor_test_selection != DRONE_CONTROL_MOTOR_SELECT_ALL))
    {
        uint8_t motor;
        for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
        {
            if (control.motor_test_selection != (uint8_t)(motor + 1U))
            {
                mix.command[motor] = 0.0f;
                pulses[motor] = mixer_output_config.disarmed_pulse_us;
            }
        }
    }
#endif
    if (!MotorPwm_SetAllPulseUs(control.motors, pulses))
    {
        return false;
    }
    publish_mixer_telemetry(&mix,
                            pulses,
                            roll_correction,
                            pitch_correction,
                            yaw_correction,
                            active);
    return true;
}

static void publish_mixer_telemetry(
    const MotorMixerResult *mix,
    const uint16_t pulses[MOTOR_PWM_MOTOR_COUNT],
    float roll_correction,
    float pitch_correction,
    float yaw_correction,
    bool active)
{
    const float pid_output[RATE_CONTROL_AXIS_COUNT] = {
        roll_correction,
        pitch_correction,
        yaw_correction,
    };
    uint8_t axis;
    uint8_t motor;

    ++control.mixer_telemetry_sequence;
    __DMB();
    for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
    {
        control.mixer_telemetry.pid_output[axis] = pid_output[axis];
    }
    for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
    {
        control.mixer_telemetry.motor_command[motor] = mix->command[motor];
        control.mixer_telemetry.pulse_us[motor] = pulses[motor];
    }
    control.mixer_telemetry.applied_throttle = control.applied_throttle;
    control.mixer_telemetry.applied_collective = mix->collective_command;
    control.mixer_telemetry.correction_scale = mix->correction_scale;
    control.mixer_telemetry.active = active;
    control.mixer_telemetry.collective_shifted =
        mix->collective_shifted;
    control.mixer_telemetry.correction_scaled =
        mix->correction_scaled;
    __DMB();
    ++control.mixer_telemetry_sequence;
}

static void publish_disarmed_telemetry(void)
{
    MotorMixerResult mix = {
        .command = {0.0f, 0.0f, 0.0f, 0.0f},
        .collective_command = 0.0f,
        .correction_scale = 1.0f,
        .collective_shifted = false,
        .correction_scaled = false,
    };
    uint16_t pulses[MOTOR_PWM_MOTOR_COUNT];
    uint8_t motor;

    for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
    {
        pulses[motor] = MotorPwm_GetPulseUs(control.motors, motor);
    }
    publish_mixer_telemetry(&mix, pulses, 0.0f, 0.0f, 0.0f, false);
}

static bool send_status(uint32_t now_ms)
{
    DroneSystemStatus status = {
        .header = {
            .sequence = control.status_sequence,
            .session_id = control.active_session,
            .sender_time_ms = now_ms,
        },
        .last_control_sequence = control.last_sequence,
        .requested_throttle = control.requested_throttle,
        .applied_throttle = control.applied_throttle,
        .state = (uint8_t)control.state,
        .error_flags = control.error_flags,
        .uart_rx_rate = control.uart_rx_rate,
    };
    uint8_t motor;
    uint8_t raw[DRONE_STATUS_PACKET_SIZE];

    for (motor = 0U; motor < MOTOR_PWM_MOTOR_COUNT; ++motor)
    {
        status.pwm_pulse_us[motor] =
            MotorPwm_GetPulseUs(control.motors, motor);
    }

    if ((control.uart == NULL) ||
        (DroneProtocol_EncodeStatus(&status, raw) != DRONE_PROTOCOL_OK))
    {
        return false;
    }
    if (!try_send_packet(raw, sizeof(raw)))
    {
        return false;
    }
    ++control.status_sequence;
    return true;
}

static bool send_flight_telemetry(void)
{
    uint8_t raw[DRONE_FLIGHT_TELEMETRY_PACKET_SIZE];
    DroneFlightTelemetry telemetry = control.flight_telemetry;

    telemetry.header.sequence = control.flight_telemetry_sequence;
    if (DroneProtocol_EncodeFlightTelemetry(&telemetry, raw) !=
        DRONE_PROTOCOL_OK)
    {
        return false;
    }
    if (!try_send_packet(raw, sizeof(raw)))
    {
        return false;
    }
    ++control.flight_telemetry_sequence;
    return true;
}

static bool try_send_packet(const uint8_t *raw, size_t raw_length)
{
    size_t encoded_length;

    if ((raw == NULL) || (raw_length == 0U) ||
        (raw_length > DRONE_PROTOCOL_MAX_PACKET_SIZE) ||
        (control.uart == NULL) ||
        (control.uart->gState != HAL_UART_STATE_READY))
    {
        return false;
    }
    encoded_length = DroneCobs_Encode(raw, raw_length,
                                      control.tx_frame,
                                      sizeof(control.tx_frame) - 1U);
    if (encoded_length == 0U)
    {
        return false;
    }
    control.tx_frame[encoded_length++] = 0U;
    return HAL_UART_Transmit_IT(control.uart, control.tx_frame,
                                (uint16_t)encoded_length) == HAL_OK;
}

static bool scale_to_i16(float value, float scale, int16_t *output)
{
    float scaled;

    if ((output == NULL) || !isfinite(value) || !isfinite(scale))
    {
        return false;
    }
    scaled = value * scale;
    if (!isfinite(scaled) || (scaled < -32768.0f) || (scaled > 32767.0f))
    {
        return false;
    }
    *output = (int16_t)((scaled >= 0.0f) ? (scaled + 0.5f) :
                                               (scaled - 0.5f));
    return true;
}
