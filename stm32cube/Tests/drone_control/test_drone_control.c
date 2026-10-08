#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mahony.h"

/* Inspect internal state while exercising the public UART/sample entry points. */
#include "../../Components/DroneControl/Src/drone_control.c"

typedef struct
{
  float pending[4];
  float applied[4];
  bool started;
  bool fail_update;
  bool fail_start;
  bool busy_update;
  unsigned int positive_commits;
  unsigned int zero_commits;
  unsigned int starts;
} MockMotor;

static uint32_t mock_tick;
static uint8_t *mock_rx_buffer;
static uint16_t mock_rx_capacity;
static uint16_t next_sequence;
static MockMotor mock_motor;
static MotorOutput motors;
static UART_HandleTypeDef uart;
static uint8_t last_tx_packet[DRONE_PROTOCOL_MAX_PACKET_SIZE];
static size_t last_tx_length;

uint32_t HAL_GetTick(void)
{
  return mock_tick;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *handle,
                                              uint8_t *buffer,
                                              uint16_t length)
{
  assert(handle == &uart);
  mock_rx_buffer = buffer;
  mock_rx_capacity = length;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *handle,
                                      uint8_t *buffer,
                                      uint16_t length)
{
  assert(handle == &uart);
  assert(buffer != NULL && length > 0U);
  assert(buffer[length - 1U] == 0U);
  last_tx_length = DroneCobs_Decode(buffer, length - 1U,
                                    last_tx_packet, sizeof(last_tx_packet));
  assert(last_tx_length > 0U);
  return HAL_OK;
}

static MotorStatus mock_start(void *driver)
{
  MockMotor *motor = driver;
  motor->started = true;
  ++motor->starts;
  return motor->fail_start ? MOTOR_ERROR : MOTOR_OK;
}

static MotorStatus mock_stop(void *driver)
{
  MockMotor *motor = driver;
  motor->started = false;
  memset(motor->pending, 0, sizeof(motor->pending));
  memset(motor->applied, 0, sizeof(motor->applied));
  return MOTOR_OK;
}

static MotorStatus mock_set(void *driver, uint8_t index, float throttle)
{
  MockMotor *motor = driver;
  assert(index < 4U);
  motor->pending[index] = throttle;
  return MOTOR_OK;
}

static MotorStatus mock_set_all(void *driver, const float *throttle,
                                uint8_t count)
{
  MockMotor *motor = driver;
  assert(count == 4U);
  memcpy(motor->pending, throttle, sizeof(motor->pending));
  return MOTOR_OK;
}

static MotorStatus mock_update(void *driver)
{
  MockMotor *motor = driver;
  unsigned int index;
  bool positive = false;

  if (motor->fail_update)
  {
    return MOTOR_ERROR;
  }
  if (motor->busy_update)
  {
    return MOTOR_BUSY;
  }
  for (index = 0U; index < 4U; ++index)
  {
    assert(isfinite(motor->pending[index]));
    assert(motor->pending[index] >= 0.0f && motor->pending[index] <= 1.0f);
    motor->applied[index] = motor->started ? motor->pending[index] : 0.0f;
    positive |= motor->applied[index] > 0.0f;
  }
  motor->positive_commits += positive ? 1U : 0U;
  motor->zero_commits += positive ? 0U : 1U;
  return MOTOR_OK;
}

static bool mock_started(const void *driver)
{
  const MockMotor *motor = driver;
  return motor->started;
}

static float mock_throttle(const void *driver, uint8_t index)
{
  const MockMotor *motor = driver;
  return motor->applied[index];
}

static uint16_t mock_raw(const void *driver, uint8_t index)
{
  return (uint16_t)(1000.0f + 1000.0f * mock_throttle(driver, index));
}

static const MotorOps motor_ops = {
  .start = mock_start,
  .stop = mock_stop,
  .set_throttle = mock_set,
  .set_all_throttle = mock_set_all,
  .update = mock_update,
  .is_started = mock_started,
  .get_throttle = mock_throttle,
  .get_raw_output = mock_raw,
};

static void assert_near(float actual, float expected)
{
  assert(fabsf(actual - expected) < 0.0001f);
}

static void assert_stopped(void)
{
  unsigned int motor;
  for (motor = 0U; motor < 4U; ++motor)
  {
    assert_near(mock_motor.applied[motor], 0.0f);
  }
  assert(control.applied_throttle == 0U);
}

static void setup(void)
{
  mock_tick = 100U;
  next_sequence = 1U;
  memset(&mock_motor, 0, sizeof(mock_motor));
  memset(&uart, 0, sizeof(uart));
  last_tx_length = 0U;
  assert(MotorOutput_Init(&motors, &motor_ops, &mock_motor, 4U,
                          MOTOR_PROTOCOL_PWM) == MOTOR_OK);
  DroneControl_Init(&uart, &motors);
  assert(control.state == DRONE_STATE_DISARMED);
  assert_stopped();
}

static void setup_dshot(void)
{
  setup();
  motors.protocol = MOTOR_PROTOCOL_DSHOT;
}

static void queue_raw(const uint8_t *raw, size_t length)
{
  uint8_t encoded[64];
  size_t encoded_length = DroneCobs_Encode(raw, length, encoded,
                                          sizeof(encoded) - 1U);
  assert(encoded_length > 0U);
  encoded[encoded_length++] = 0U;
  assert(encoded_length <= mock_rx_capacity);
  memcpy(mock_rx_buffer, encoded, encoded_length);
  DroneControl_OnUartRxEvent(&uart, (uint16_t)encoded_length);
}

static void queue_command(uint16_t flags, uint16_t throttle,
                           int16_t roll, int16_t pitch, int16_t yaw)
{
  DroneControlCommand command = {
    .header = {
      .sequence = next_sequence++,
      .session_id = 42U,
      .flags = flags,
      .sender_time_ms = mock_tick,
    },
    .throttle = throttle,
    .roll = roll,
    .pitch = pitch,
    .yaw = yaw,
  };
  uint8_t raw[DRONE_CONTROL_PACKET_SIZE];
  assert(DroneProtocol_EncodeControl(&command, raw) == DRONE_PROTOCOL_OK);
  queue_raw(raw, sizeof(raw));
}

static void send_command(uint16_t flags, uint16_t throttle,
                          int16_t roll, int16_t pitch, int16_t yaw)
{
  queue_command(flags, throttle, roll, pitch, yaw);
  DroneControl_Process(mock_tick);
}

static void send_motor_test(uint16_t flags, uint16_t throttle,
                            uint16_t selection)
{
  DroneControlCommand command = {
    .header = {
      .sequence = next_sequence++,
      .session_id = 42U,
      .flags = flags,
      .sender_time_ms = mock_tick,
    },
    .throttle = throttle,
    .aux1 = selection,
  };
  uint8_t raw[DRONE_CONTROL_PACKET_SIZE];
  assert(DroneProtocol_EncodeControl(&command, raw) == DRONE_PROTOCOL_OK);
  queue_raw(raw, sizeof(raw));
  DroneControl_Process(mock_tick);
}

static bool sample(bool valid, float roll, float pitch)
{
  return DroneControl_UpdateFlightSample(mock_tick, valid, roll, pitch,
                                         0.0f, 0.0f, 0.0f, 1.0f / 1125.0f);
}

static void arm_angle(void)
{
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE, 0U, 0, 0, 0);
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_ARMED);
  assert_stopped();
}

static void run_angle(void)
{
  arm_angle();
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                200U, 0, 0, 0);
  assert(sample(true, 0.0f, 0.0f));
  assert(mock_motor.applied[0] > 0.0f);
}

static void test_startup_arm_requires_attitude_and_disarm_cycle(void)
{
  setup();
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_DISARMED);
  assert(control.require_disarm_cycle);
  assert_stopped();
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE, 0U, 0, 0, 0);
  (void)sample(false, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_DISARMED);
  assert((control.error_flags & DRONE_ERROR_ARM_REJECTED) != 0U);
  assert_stopped();
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                1U, 0, 0, 0);
  assert(control.state == DRONE_STATE_DISARMED);
  assert_stopped();
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_ARMED);
  assert_stopped();
}

static void test_angle_restoring_signs_through_mixer(void)
{
  setup();
  run_angle();
  assert(sample(true, 10.0f, 0.0f));
  assert(control.rate_control.debug.target_rad_s[RATE_CONTROL_ROLL] < 0.0f);
  assert(mock_motor.applied[0] < mock_motor.applied[2]);
  assert(mock_motor.applied[1] < mock_motor.applied[3]);
  assert_near(mock_motor.applied[0], mock_motor.applied[1]);
  RateControl_Reset(&control.rate_control);
  assert(sample(true, 0.0f, 10.0f));
  assert(control.rate_control.debug.target_rad_s[RATE_CONTROL_PITCH] < 0.0f);
  assert(mock_motor.applied[0] < mock_motor.applied[1]);
  assert(mock_motor.applied[2] < mock_motor.applied[3]);
}

static void test_mahony6_to_motor_restoring_response(void)
{
  Mahony_Handle_t filter;
  Mahony_Euler_t euler;
  const Mahony_Config_t config = {
    .kp = 2.0f, .ki = 0.05f, .integral_limit_rad_s = 0.1f,
    .accel_min_norm = 0.8f, .accel_max_norm = 1.2f,
  };
  setup();
  run_angle();
  Mahony_Init(&filter, &config);
  /* Gravity for +30 deg roll, no magnetometer: right side needs more lift. */
  assert(Mahony_InitFromAccel(&filter, 0.0f, 0.5f, 0.866025404f));
  assert(Mahony_GetEulerDegrees(&filter, &euler));
  assert(sample(true, euler.roll, euler.pitch));
  assert(mock_motor.applied[2] > mock_motor.applied[0]);
  assert(mock_motor.applied[3] > mock_motor.applied[1]);
  RateControl_Reset(&control.rate_control);
  /* Gravity for +30 deg pitch: rear motors must lower the raised nose. */
  assert(Mahony_InitFromAccel(&filter, -0.5f, 0.0f, 0.866025404f));
  assert(Mahony_GetEulerDegrees(&filter, &euler));
  assert(sample(true, euler.roll, euler.pitch));
  assert(mock_motor.applied[1] > mock_motor.applied[0]);
  assert(mock_motor.applied[3] > mock_motor.applied[2]);
}

static void test_zero_throttle_resets_then_recovers_pilot_targets(void)
{
  setup();
  run_angle();
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 500, -500, 500);
  assert_stopped();
  assert_near(control.rate_control.debug.output[0], 0.0f);
  assert(!sample(true, 0.0f, 0.0f));
  assert_near(control.rate_control.debug.target_rad_s[0], 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                200U, 500, -500, 500);
  assert(sample(true, 0.0f, 0.0f));
  assert(control.rate_control.debug.target_rad_s[0] > 0.0f);
  assert(control.rate_control.debug.target_rad_s[1] < 0.0f);
  assert_near(control.rate_control.debug.target_rad_s[2],
               75.0f * CONTROL_DEG_TO_RAD);
}

static void test_stale_sample_prevents_queued_positive_commit_and_rearm(void)
{
  unsigned int commits;
  setup();
  run_angle();
  commits = mock_motor.positive_commits;
  mock_tick += DRONE_CONTROL_ATTITUDE_TIMEOUT_MS;
  queue_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                 400U, 0, 0, 0);
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert(control.require_disarm_cycle);
  assert(mock_motor.positive_commits == commits);
  assert_stopped();
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
  arm_angle();
}

static void test_invalid_angle_samples_fail_safe(void)
{
  unsigned int test;
  for (test = 0U; test < 7U; ++test)
  {
    setup();
    run_angle();
    assert(!DroneControl_UpdateFlightSample(
        test == 6U ? mock_tick - 20U : mock_tick,
        test != 0U,
        test == 1U ? NAN : 0.0f,
        test == 2U ? 91.0f : 0.0f,
        test == 3U ? INFINITY : 0.0f, 0.0f, 0.0f,
        test == 4U ? 0.021f : (test == 5U ? 0.0001f : 0.001f)));
    assert(control.state == DRONE_STATE_FAILSAFE);
    assert_stopped();
  }
  setup();
  run_angle();
  assert(!DroneControl_UpdateBodyRates(0.0f, 0.0f, 0.0f, 0.001f));
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
}

static void test_mode_changes_disarm_and_estop_priority(void)
{
  setup();
  run_angle();
  send_command(DRONE_CONTROL_FLAG_ACRO_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
  setup();
  run_angle();
  send_command(DRONE_CONTROL_FLAG_ACRO_MODE, 0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_DISARMED);
  assert(!control.angle_mode && !control.require_disarm_cycle);
  assert_stopped();
  setup();
  run_angle();
  send_command(DRONE_CONTROL_FLAG_EMERGENCY_STOP, 200U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert((control.error_flags & DRONE_ERROR_INVALID_PACKET) == 0U);
  assert_stopped();
}

static void test_acro_compatibility_sequence_and_link_watchdog(void)
{
  float before[4];
  setup();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 500, -500, 500);
  assert(DroneControl_UpdateBodyRates(0.0f, 0.0f, 0.0f, 0.001f));
  assert(!control.angle_mode);
  assert_near(control.rate_control.debug.target_rad_s[0],
               100.0f * CONTROL_DEG_TO_RAD);
  assert_near(control.rate_control.debug.target_rad_s[1],
               -100.0f * CONTROL_DEG_TO_RAD);
  memcpy(before, mock_motor.applied, sizeof(before));
  --next_sequence;
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 400U, -1000, 1000, 0);
  assert(control.applied_throttle == 200U);
  assert(control.pilot_command[0] == 500);
  assert(memcmp(before, mock_motor.applied, sizeof(before)) == 0);
  mock_tick += DRONE_CONTROL_LINK_TIMEOUT_MS;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert((control.error_flags & DRONE_ERROR_UART_LINK_LOST) != 0U);
  assert_stopped();
}

static void test_zero_timestamp_and_tick_wrap(void)
{
  setup();
  mock_tick = 0U;
  arm_angle();
  mock_tick = 19U;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_ARMED);
  mock_tick = 20U;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
  setup();
  mock_tick = UINT32_MAX - 9U;
  arm_angle();
  mock_tick = 9U;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_ARMED);
  mock_tick = 10U;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
}

static void test_session_change_and_malformed_startup(void)
{
  DroneControlCommand command = {
    .header = {
      .sequence = 1U,
      .session_id = 99U,
      .flags = DRONE_CONTROL_FLAG_ARM_REQUEST | DRONE_CONTROL_FLAG_ANGLE_MODE,
    },
    .throttle = 300U,
  };
  uint8_t raw[DRONE_CONTROL_PACKET_SIZE];
  setup();
  assert(DroneProtocol_EncodeControl(&command, raw) == DRONE_PROTOCOL_OK);
  raw[20] ^= 1U;
  queue_raw(raw, sizeof(raw));
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_DISARMED);
  assert((control.error_flags & DRONE_ERROR_CRC) != 0U);
  assert_stopped();
  run_angle();
  assert(DroneProtocol_EncodeControl(&command, raw) == DRONE_PROTOCOL_OK);
  queue_raw(raw, sizeof(raw));
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_DISARMED);
  assert(control.require_disarm_cycle);
  assert_stopped();
}

static void test_imu_reinitialization_stops_before_blocking(void)
{
  setup();
  run_angle();
  DroneControl_InvalidateImuSample();
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert(!control.has_valid_attitude);
  assert(control.require_disarm_cycle);
  assert_stopped();
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  /* Reinitializing a gyro is unsafe in ACRO too. */
  setup();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
  DroneControl_InvalidateImuSample();
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  setup();
  (void)sample(true, 0.0f, 0.0f);
  DroneControl_InvalidateImuSample();
  assert(control.state == DRONE_STATE_DISARMED);
  assert(!control.has_valid_attitude);
  assert_stopped();
}

static void test_dshot_zero_prearm_then_positive_throttle(void)
{
  uint32_t elapsed;
  setup_dshot();
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE, 0U, 0, 0, 0);
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  assert(control.dshot_arming);
  assert(control.state == DRONE_STATE_DISARMED);
  assert(mock_motor.starts == 1U);
  assert_stopped();
  for (elapsed = 1U; elapsed < DRONE_CONTROL_DSHOT_ZERO_ARM_MS; ++elapsed)
  {
    ++mock_tick;
    (void)sample(true, 0.0f, 0.0f);
    if ((elapsed % 25U) == 0U)
    {
      send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                    0U, 0, 0, 0);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
    assert(control.dshot_arming);
    assert(control.state == DRONE_STATE_DISARMED);
    assert(mock_motor.positive_commits == 0U);
    assert(mock_motor.zero_commits == elapsed / MOTOR_OUTPUT_DSHOT_PERIOD_MS);
    assert_stopped();
  }
  assert(mock_motor.zero_commits >= DRONE_CONTROL_DSHOT_ZERO_MIN_FRAMES);
  ++mock_tick;
  (void)sample(true, 0.0f, 0.0f);
  DroneControl_Process(mock_tick);
  assert(!control.dshot_arming);
  assert(control.state == DRONE_STATE_ARMED);
  assert(mock_motor.positive_commits == 0U);
  assert(mock_motor.starts == 1U);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                200U, 0, 0, 0);
  assert(sample(true, 0.0f, 0.0f));
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == 0U);
  mock_tick += MOTOR_OUTPUT_DSHOT_PERIOD_MS;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits > 0U);
}

static void test_dshot_prearm_cancels_on_throttle_disarm_and_error(void)
{
  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  assert(control.dshot_arming);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 1U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert(control.require_disarm_cycle);
  assert_stopped();

  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  send_command(0U, 0U, 0, 0, 0);
  assert(!control.dshot_arming);
  assert(control.state == DRONE_STATE_DISARMED);
  assert_stopped();

  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  mock_motor.fail_update = true;
  mock_tick += MOTOR_OUTPUT_DSHOT_PERIOD_MS;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_ERROR);
  assert(!control.dshot_arming);
  assert_stopped();

  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  mock_motor.fail_start = true;
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_ERROR);
  assert(!mock_motor.started);
  assert_stopped();

  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST | DRONE_CONTROL_FLAG_EMERGENCY_STOP,
                0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
}

static void test_dshot_prearm_cancels_on_stale_imu_and_link(void)
{
  setup_dshot();
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE, 0U, 0, 0, 0);
  (void)sample(true, 0.0f, 0.0f);
  send_command(DRONE_CONTROL_FLAG_ANGLE_MODE | DRONE_CONTROL_FLAG_ARM_REQUEST,
                0U, 0, 0, 0);
  mock_tick += DRONE_CONTROL_ATTITUDE_TIMEOUT_MS;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  mock_tick += DRONE_CONTROL_LINK_TIMEOUT_MS;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  DroneControl_InvalidateImuSample();
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
}

static void test_dshot_prearm_requires_continuous_frames(void)
{
  uint32_t elapsed;
  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  for (elapsed = 1U; elapsed <= DRONE_CONTROL_DSHOT_ZERO_ARM_MS; ++elapsed)
  {
    ++mock_tick;
    if (elapsed >= 500U && elapsed < 530U)
    {
      mock_motor.busy_update = true;
    }
    else
    {
      mock_motor.busy_update = false;
    }
    if ((elapsed % 25U) == 0U)
    {
      send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
  }
  assert(control.dshot_arming);
  assert(control.state == DRONE_STATE_DISARMED);
  for (; elapsed < DRONE_CONTROL_DSHOT_ARM_TIMEOUT_MS; ++elapsed)
  {
    ++mock_tick;
    if ((elapsed % 25U) == 0U)
    {
      send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
    if (control.state == DRONE_STATE_ARMED)
    {
      break;
    }
  }
  assert(control.state == DRONE_STATE_ARMED);
  assert(mock_motor.positive_commits == 0U);
}

static void test_dshot_prearm_timeout_when_zero_dma_stalls(void)
{
  uint32_t elapsed;
  setup_dshot();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  mock_motor.busy_update = true;
  for (elapsed = 1U; elapsed <= DRONE_CONTROL_DSHOT_ARM_TIMEOUT_MS; ++elapsed)
  {
    ++mock_tick;
    if ((elapsed % 25U) == 0U)
    {
      send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
    assert(mock_motor.positive_commits == 0U);
  }
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert(control.require_disarm_cycle);
  assert_stopped();
}

static void test_motor_commit_error_stops_output(void)
{
  setup();
  run_angle();
  mock_motor.fail_update = true;
  assert(!sample(true, 10.0f, 0.0f));
  assert(control.state == DRONE_STATE_ERROR);
  assert_stopped();
}

static void test_motor_test_direct_output_and_safety(void)
{
  const uint16_t test = DRONE_CONTROL_FLAG_MOTOR_TEST;
  const uint16_t armed_test = test | DRONE_CONTROL_FLAG_ARM_REQUEST;
  float output;
  unsigned int motor;

  setup();
  send_motor_test(test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  assert(control.state == DRONE_STATE_ARMED); /* No IMU sample is required. */
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  output = mock_motor.applied[0];
  assert(output > 0.0f);
  for (motor = 1U; motor < 4U; ++motor)
  {
    assert_near(mock_motor.applied[motor], output);
  }
  assert(!DroneControl_UpdateBodyRates(3.0f, -3.0f, 2.0f, 0.001f));
  for (motor = 0U; motor < 4U; ++motor)
  {
    assert_near(mock_motor.applied[motor], output);
  }
  assert_near(control.rate_control.debug.output[RATE_CONTROL_ROLL], 0.0f);
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  assert_stopped();
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_M3);
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_M3);
  assert_near(mock_motor.applied[0], 0.0f);
  assert_near(mock_motor.applied[1], 0.0f);
  assert_near(mock_motor.applied[2], output);
  assert_near(mock_motor.applied[3], 0.0f);
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_M2);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  setup();
  send_motor_test(test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  setup();
  send_motor_test(test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_command(DRONE_CONTROL_FLAG_EMERGENCY_STOP, 0U, 0, 0, 0);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();

  setup();
  send_motor_test(test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_ALL);
  mock_tick += DRONE_CONTROL_LINK_TIMEOUT_MS;
  DroneControl_Process(mock_tick);
  assert(control.state == DRONE_STATE_FAILSAFE);
  assert_stopped();
}

static void test_motor_test_dshot_prearm_without_imu(void)
{
  const uint16_t test = DRONE_CONTROL_FLAG_MOTOR_TEST;
  const uint16_t armed_test = test | DRONE_CONTROL_FLAG_ARM_REQUEST;
  uint32_t elapsed;

  setup_dshot();
  send_motor_test(test, 0U, DRONE_CONTROL_MOTOR_SELECT_M2);
  send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_M2);
  assert(control.dshot_arming);
  for (elapsed = 1U; elapsed <= DRONE_CONTROL_DSHOT_ZERO_ARM_MS; ++elapsed)
  {
    ++mock_tick;
    if ((elapsed % 25U) == 0U)
    {
      send_motor_test(armed_test, 0U, DRONE_CONTROL_MOTOR_SELECT_M2);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
    assert(mock_motor.positive_commits == 0U);
  }
  assert(control.state == DRONE_STATE_ARMED);
  send_motor_test(armed_test, 200U, DRONE_CONTROL_MOTOR_SELECT_M2);
  mock_tick += MOTOR_OUTPUT_DSHOT_PERIOD_MS;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert_near(mock_motor.applied[0], 0.0f);
  assert(mock_motor.applied[1] > 0.0f);
  assert_near(mock_motor.applied[2], 0.0f);
  assert_near(mock_motor.applied[3], 0.0f);
}

/* Run the real pre-arm state machine; no direct state injection. */
static void arm_dshot_mode(uint32_t start_ms, uint16_t mode)
{
  uint32_t elapsed;
  setup_dshot();
  mock_tick = start_ms;
  send_command(mode, 0U, 0, 0, 0);
  if (mode == DRONE_CONTROL_FLAG_ANGLE_MODE)
  {
    (void)sample(true, 0.0f, 0.0f);
  }
  send_command(mode | DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  for (elapsed = 1U; elapsed <= DRONE_CONTROL_DSHOT_ZERO_ARM_MS; ++elapsed)
  {
    ++mock_tick;
    if (mode == DRONE_CONTROL_FLAG_ANGLE_MODE)
    {
      (void)sample(true, 0.0f, 0.0f);
    }
    if (elapsed % 25U == 0U)
    {
      send_command(mode | DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
    DroneControl_ServiceMotorOutput(mock_tick);
  }
  assert(control.state == DRONE_STATE_ARMED);
  assert(mock_motor.zero_commits == 500U);
  assert(mock_motor.positive_commits == 0U);
}

static void arm_dshot_acro(uint32_t start_ms)
{
  arm_dshot_mode(start_ms, 0U);
}

static void test_dshot_500hz_latest_bank_and_pid_rate(void)
{
  uint32_t elapsed;
  unsigned int motor;
  DroneMixerTelemetry telemetry;
  arm_dshot_acro(100U);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
  for (elapsed = 1U; elapsed <= 1000U; ++elapsed)
  {
    const float integral_before = control.rate_control.integral[RATE_CONTROL_ROLL];
    ++mock_tick;
    if (elapsed % 25U == 0U)
    {
      /* Multiple UART commands in one tick must not send extra frames. */
      send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 100U, 100, 0, 0);
      send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 300U, 300, -200, 100);
    }
    else
    {
      DroneControl_Process(mock_tick);
    }
    /* PID must still consume every 1 ms sample, independent of ESC cadence. */
    assert(DroneControl_UpdateBodyRates(0.1f, -0.1f, 0.0f, 0.001f));
    if (elapsed < 25U)
    {
      assert_near(control.rate_control.integral[RATE_CONTROL_ROLL],
                  integral_before - 0.002f);
    }
    DroneControl_ServiceMotorOutput(mock_tick);
    DroneControl_ServiceMotorOutput(mock_tick);
    assert(mock_motor.positive_commits == elapsed / 2U);
    if (elapsed % 2U == 0U)
    {
      assert(DroneControl_GetMixerTelemetry(&telemetry));
      for (motor = 0U; motor < 4U; ++motor)
      {
        assert_near(mock_motor.applied[motor], mock_motor.pending[motor]);
        assert_near(telemetry.motor_command[motor] / 1000.0f,
                    mock_motor.applied[motor]);
      }
    }
  }
  assert(mock_motor.positive_commits == 500U);
}

static void test_dshot_repeat_zero_and_motor_test_without_samples(void)
{
  unsigned int zero_before;
  unsigned int positive_before;
  unsigned int elapsed;
  arm_dshot_acro(100U);
  zero_before = mock_motor.zero_commits;
  for (elapsed = 1U; elapsed <= 10U; ++elapsed)
  {
    ++mock_tick;
    DroneControl_Process(mock_tick);
    DroneControl_ServiceMotorOutput(mock_tick);
  }
  assert(mock_motor.zero_commits == zero_before + 5U);
  assert_stopped();

  /* Enter MOTOR_TEST by an explicit DISARM, then complete another pre-arm. */
  send_motor_test(DRONE_CONTROL_FLAG_MOTOR_TEST, 0U,
                  DRONE_CONTROL_MOTOR_SELECT_M2);
  send_motor_test(DRONE_CONTROL_FLAG_MOTOR_TEST | DRONE_CONTROL_FLAG_ARM_REQUEST,
                  0U, DRONE_CONTROL_MOTOR_SELECT_M2);
  for (elapsed = 1U; elapsed <= DRONE_CONTROL_DSHOT_ZERO_ARM_MS; ++elapsed)
  {
    ++mock_tick;
    if (elapsed % 25U == 0U)
    {
      send_motor_test(DRONE_CONTROL_FLAG_MOTOR_TEST | DRONE_CONTROL_FLAG_ARM_REQUEST,
                      0U, DRONE_CONTROL_MOTOR_SELECT_M2);
    }
    DroneControl_Process(mock_tick);
    DroneControl_ServiceMotorOutput(mock_tick);
  }
  assert(control.state == DRONE_STATE_ARMED);
  send_motor_test(DRONE_CONTROL_FLAG_MOTOR_TEST | DRONE_CONTROL_FLAG_ARM_REQUEST,
                  200U, DRONE_CONTROL_MOTOR_SELECT_M2);
  positive_before = mock_motor.positive_commits;
  for (elapsed = 1U; elapsed <= 10U; ++elapsed)
  {
    ++mock_tick;
    DroneControl_Process(mock_tick);
    DroneControl_ServiceMotorOutput(mock_tick);
    assert(mock_motor.positive_commits == positive_before + elapsed / 2U);
  }
  assert(mock_motor.applied[1] > 0.0f);
  assert_near(mock_motor.applied[0], 0.0f);
  assert_near(mock_motor.applied[2], 0.0f);
  assert_near(mock_motor.applied[3], 0.0f);
}

static void test_dshot_busy_wrap_and_no_catchup_burst(void)
{
  unsigned int commits;
  arm_dshot_acro(UINT32_MAX - 1001U); /* Pre-arm ends at UINT32_MAX - 1. */
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 100U, 0, 0, 0);
  ++mock_tick;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == 0U);
  ++mock_tick;
  assert(mock_tick == 0U);
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == 1U);
  mock_motor.busy_update = true;
  mock_tick += 2U;
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == 1U);
  mock_motor.busy_update = false;
  ++mock_tick;
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 300U, 0, 0, 0);
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == 2U);
  assert_near(mock_motor.applied[0], throttle_to_mixer_command(300U) / 1000.0f);
  ++mock_tick;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == 2U);
  mock_tick += 9U;
  commits = mock_motor.positive_commits;
  DroneControl_ServiceMotorOutput(mock_tick);
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(mock_motor.positive_commits == commits + 1U);
}

static void test_dshot_periodic_output_stop_paths(void)
{
  unsigned int path;
  for (path = 0U; path < 6U; ++path)
  {
    unsigned int commits;
    const uint16_t mode = path == 5U ? DRONE_CONTROL_FLAG_ANGLE_MODE : 0U;
    arm_dshot_mode(100U, mode);
    send_command(mode | DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
    mock_tick += 2U;
    DroneControl_ServiceMotorOutput(mock_tick);
    assert(mock_motor.positive_commits == 1U);
    ++mock_tick; /* Stop paths must also work before the next ESC deadline. */
    send_command(mode | DRONE_CONTROL_FLAG_ARM_REQUEST, 300U, 0, 0, 0);
    switch (path)
    {
      case 0U: send_command(0U, 0U, 0, 0, 0); break;
      case 1U: send_command(DRONE_CONTROL_FLAG_EMERGENCY_STOP, 0U, 0, 0, 0); break;
      case 2U: DroneControl_InvalidateImuSample(); break;
      case 3U:
        mock_tick += DRONE_CONTROL_LINK_TIMEOUT_MS;
        DroneControl_ServiceMotorOutput(mock_tick);
        break;
      case 4U:
        mock_motor.fail_update = true;
        ++mock_tick;
        DroneControl_ServiceMotorOutput(mock_tick);
        assert(control.state == DRONE_STATE_ERROR);
        break;
      default:
        /* Simulate expiration between Process and the output step in ANGLE. */
        mock_tick += DRONE_CONTROL_ATTITUDE_TIMEOUT_MS;
        DroneControl_ServiceMotorOutput(mock_tick);
        break;
    }
    assert_stopped();
    assert(!mock_motor.started);
    commits = mock_motor.positive_commits;
    mock_tick += 2U;
    DroneControl_ServiceMotorOutput(mock_tick);
    assert(mock_motor.positive_commits == commits);
  }
}

static void telemetry_sample(float roll_rate)
{
  (void)DroneControl_UpdateFlightSample(mock_tick, false, 0.0f, 0.0f,
                                        roll_rate, 0.0f, 0.0f, 0.001f);
  DroneControl_PublishFlightTelemetrySample(mock_tick, false,
                                            0.0f, 0.0f, 0.0f,
                                            roll_rate, 0.0f, 0.0f);
}

static void assert_telemetry_matches_commit(float rate)
{
  int16_t expected_pid;
  unsigned int motor;
  DroneFlightTelemetry decoded;
  const uint16_t previous_sequence = control.flight_telemetry_sequence;
  assert(control.flight_telemetry_available);
  assert(control.flight_telemetry.output_sample_matched);
  assert(control.flight_telemetry.sample_id == control.flight_sample_id);
  assert(control.flight_telemetry.motor_commit_time_ms == mock_tick);
  assert(control.flight_telemetry.gyro_mrad_s[0] == (int16_t)(rate * 1000.0f));
  assert(scale_to_i16(control.rate_control.debug.output[0], 100.0f, &expected_pid));
  assert(control.flight_telemetry.pid_command_centi[0] == expected_pid);
  for (motor = 0U; motor < 4U; ++motor)
  {
    assert(control.flight_telemetry.motor_pwm_us[motor] == get_legacy_motor_output(motor));
  }
  assert(send_flight_telemetry());
  assert(last_tx_length == DRONE_FLIGHT_TELEMETRY_PACKET_SIZE);
  assert(DroneProtocol_DecodeFlightTelemetry(last_tx_packet, last_tx_length, &decoded) == DRONE_PROTOCOL_OK);
  assert(decoded.sample_id == control.flight_sample_id);
  assert(decoded.motor_commit_time_ms == mock_tick);
  assert(decoded.output_sample_matched);
  assert(control.flight_telemetry_sequence == (uint16_t)(previous_sequence + 1U));
  assert(!control.flight_telemetry_available);
}

static void test_telemetry_dshot_busy_latest_sample_and_stop(void)
{
  uint32_t first_id;
  arm_dshot_acro(100U);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
  ++mock_tick;
  telemetry_sample(0.1f);
  first_id = control.flight_sample_id;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(control.pending_flight_sample_available);
  assert(!control.flight_telemetry_available);

  ++mock_tick;
  telemetry_sample(0.9f);
  mock_motor.busy_update = true;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(!control.flight_telemetry_available);
  assert(control.motor_telemetry_commit_ms == mock_tick - 2U);

  ++mock_tick;
  telemetry_sample(0.3f);
  mock_motor.busy_update = false;
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(control.flight_telemetry.sample_id == first_id + 2U);
  assert(control.flight_telemetry.header.sender_time_ms == mock_tick);
  assert_telemetry_matches_commit(0.3f);
  assert(!control.pending_flight_sample_available);

  mock_tick += 2U;
  DroneControl_ServiceMotorOutput(mock_tick); /* Repeating a frame isn't a new sample. */
  assert(!control.flight_telemetry_available);
  ++mock_tick;
  telemetry_sample(0.4f);
  assert(control.pending_flight_sample_available);
  send_command(0U, 0U, 0, 0, 0);
  assert(!control.pending_flight_sample_available && !control.flight_telemetry_available);
  telemetry_sample(0.0f);
  assert(!control.flight_telemetry.output_sample_matched);
  assert(control.flight_telemetry.state == DRONE_STATE_DISARMED);
  assert(control.flight_telemetry.motor_commit_time_ms == mock_tick);
}

static void test_telemetry_pwm_busy_and_freshness(void)
{
  uint32_t committed_id;
  setup();
  send_command(0U, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 0U, 0, 0, 0);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
  telemetry_sample(0.2f);
  assert_telemetry_matches_commit(0.2f);
  committed_id = control.flight_sample_id;
  ++mock_tick;
  mock_motor.busy_update = true;
  telemetry_sample(0.9f);
  assert(!control.flight_telemetry_available);
  assert(control.pending_flight_sample_available);
  assert(control.flight_telemetry.sample_id == committed_id);
  ++mock_tick;
  mock_motor.busy_update = false;
  telemetry_sample(0.3f);
  assert_telemetry_matches_commit(0.3f);

  telemetry_sample(0.0f);
  mock_tick += CONTROL_TELEMETRY_MAX_SAMPLE_AGE_MS + 1U;
  assert(!send_flight_telemetry());
  assert(!control.flight_telemetry_available);
  telemetry_sample(0.0f);
  enter_failsafe(DRONE_ERROR_FAILSAFE_ACTIVE);
  assert(!control.flight_telemetry_available && !control.pending_flight_sample_available);
  telemetry_sample(0.0f);
  assert(control.flight_telemetry.state == DRONE_STATE_FAILSAFE);
  assert(!control.flight_telemetry.actuators_active);
}

static void test_telemetry_tick_and_sample_id_wrap(void)
{
  arm_dshot_acro(UINT32_MAX - 1001U);
  send_command(DRONE_CONTROL_FLAG_ARM_REQUEST, 200U, 0, 0, 0);
  control.flight_sample_id = UINT32_MAX - 1U;
  ++mock_tick;
  telemetry_sample(0.1f);
  assert(control.pending_flight_sample.sample_id == UINT32_MAX);
  ++mock_tick;
  assert(mock_tick == 0U);
  DroneControl_ServiceMotorOutput(mock_tick);
  assert(control.flight_telemetry.header.sender_time_ms == UINT32_MAX);
  assert_telemetry_matches_commit(0.1f);
  mock_tick += 2U;
  telemetry_sample(0.2f);
  assert(control.flight_sample_id == 0U);
  DroneControl_ServiceMotorOutput(mock_tick);
  assert_telemetry_matches_commit(0.2f);
  telemetry_sample(0.0f);
  DroneControl_InvalidateImuSample();
  assert(!control.flight_telemetry_available && !control.pending_flight_sample_available);
}

int main(void)
{
  test_startup_arm_requires_attitude_and_disarm_cycle();
  test_angle_restoring_signs_through_mixer();
  test_mahony6_to_motor_restoring_response();
  test_zero_throttle_resets_then_recovers_pilot_targets();
  test_stale_sample_prevents_queued_positive_commit_and_rearm();
  test_invalid_angle_samples_fail_safe();
  test_mode_changes_disarm_and_estop_priority();
  test_acro_compatibility_sequence_and_link_watchdog();
  test_zero_timestamp_and_tick_wrap();
  test_session_change_and_malformed_startup();
  test_imu_reinitialization_stops_before_blocking();
  test_dshot_zero_prearm_then_positive_throttle();
  test_dshot_prearm_cancels_on_throttle_disarm_and_error();
  test_dshot_prearm_cancels_on_stale_imu_and_link();
  test_dshot_prearm_requires_continuous_frames();
  test_dshot_prearm_timeout_when_zero_dma_stalls();
  test_motor_commit_error_stops_output();
  test_motor_test_direct_output_and_safety();
  test_motor_test_dshot_prearm_without_imu();
  test_dshot_500hz_latest_bank_and_pid_rate();
  test_dshot_repeat_zero_and_motor_test_without_samples();
  test_dshot_busy_wrap_and_no_catchup_burst();
  test_dshot_periodic_output_stop_paths();
  test_telemetry_dshot_busy_latest_sample_and_stop();
  test_telemetry_pwm_busy_and_freshness();
  test_telemetry_tick_and_sample_id_wrap();
  puts("drone control tests: PASS");
  return 0;
}
