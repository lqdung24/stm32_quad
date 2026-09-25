#include "motor.h"
#include "motor_dshot.h"
#include "motor_dshot300.h"
#include "motor_pwm.h"
#include "pwm_timer.h"
#include "../../Components/App/Inc/app_dshot_test.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t compare_value[MOTOR_OUTPUT_MAX_MOTORS];
static const uint32_t *last_dma_buffer;
static uint16_t last_slot_count;
static uint8_t last_channel_count;
static unsigned int dma_start_count;
static PwmTimer_Status service_status = PWM_TIMER_OK;

PwmTimer_Status PwmChannel_SetCompare(PwmChannel *channel,
                                      uint32_t compare_ticks)
{
  uint32_t index;

  assert(channel != NULL);
  index = channel->config.channel / 4U;
  assert(index < MOTOR_OUTPUT_MAX_MOTORS);
  compare_value[index] = compare_ticks;
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmTimer_StartCompareBurstDma(
    PwmTimer *timer,
    const uint32_t *interleaved_buffer,
    uint16_t slot_count,
    uint8_t channel_count)
{
  assert(timer != NULL);
  assert(!timer->dma_busy);
  timer->dma_busy = true;
  last_dma_buffer = interleaved_buffer;
  last_slot_count = slot_count;
  last_channel_count = channel_count;
  ++dma_start_count;
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmTimer_StopDma(PwmTimer *timer)
{
  assert(timer != NULL);
  timer->dma_busy = false;
  return PWM_TIMER_OK;
}

PwmTimer_Status PwmTimer_ServiceDma(PwmTimer *timer)
{
  assert(timer != NULL);
  if (service_status == PWM_TIMER_OK)
  {
    timer->dma_busy = false;
  }
  return service_status;
}

static void init_channels(PwmTimer *timer,
                          PwmChannel channel[MOTOR_OUTPUT_MAX_MOTORS])
{
  static const uint32_t channel_id[MOTOR_OUTPUT_MAX_MOTORS] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
  };
  unsigned int motor;

  memset(channel, 0, sizeof(PwmChannel) * MOTOR_OUTPUT_MAX_MOTORS);
  for (motor = 0U; motor < MOTOR_OUTPUT_MAX_MOTORS; ++motor)
  {
    channel[motor].config.timer = timer;
    channel[motor].config.channel = channel_id[motor];
    channel[motor].initialized = true;
    channel[motor].started = true;
  }
}

static uint16_t expected_packet(uint16_t value, int telemetry)
{
  uint16_t payload = (uint16_t)((value << 1U) | (telemetry ? 1U : 0U));
  uint16_t checksum =
      (uint16_t)((payload ^ (payload >> 4U) ^ (payload >> 8U)) & 0x0FU);

  return (uint16_t)((payload << 4U) | checksum);
}

static void assert_encoded_packet(const uint32_t *buffer,
                                  unsigned int motor,
                                  uint16_t packet,
                                  uint32_t zero_ticks,
                                  uint32_t one_ticks)
{
  unsigned int bit;

  for (bit = 0U; bit < MOTOR_DSHOT_PACKET_BITS; ++bit)
  {
    const uint16_t mask = (uint16_t)(1U << (15U - bit));
    const uint32_t expected = ((packet & mask) != 0U) ?
        one_ticks : zero_ticks;
    assert(buffer[bit * MOTOR_DSHOT_MOTOR_COUNT + motor] == expected);
  }
}

static void test_pwm_driver(void)
{
  PwmTimer timer = {0};
  PwmChannel channel[MOTOR_OUTPUT_MAX_MOTORS];
  MotorPwm driver;
  MotorOutput output = {0};
  const float throttle[MOTOR_OUTPUT_MAX_MOTORS] = {
    0.0f, 0.2f, 0.5f, 1.0f,
  };
  const float invalid[MOTOR_OUTPUT_MAX_MOTORS] = {
    0.0f, NAN, 0.5f, 1.0f,
  };
  MotorPwm_Config config;

  memset(compare_value, 0, sizeof(compare_value));
  timer.initialized = true;
  timer.config.period_ticks = 20000U;
  init_channels(&timer, channel);
  memset(&config, 0, sizeof(config));
  config.channel[0] = &channel[0];
  config.channel[1] = &channel[1];
  config.channel[2] = &channel[2];
  config.channel[3] = &channel[3];
  config.stop_compare = 1000U;
  config.minimum_compare = 1000U;
  config.maximum_compare = 2000U;
  config.idle_compare[0] = 1220U;
  config.idle_compare[1] = 1225U;
  config.idle_compare[2] = 1210U;
  config.idle_compare[3] = 1225U;

  assert(MotorPwm_Init(&driver, &output, &config) == MOTOR_OK);
  assert(MotorOutput_IsReady(&output));
  assert(!MotorOutput_IsStarted(&output));
  assert(MotorOutput_Start(&output) == MOTOR_OK);
  assert(MotorOutput_SetAllThrottle(&output,
                                    throttle,
                                    MOTOR_OUTPUT_MAX_MOTORS) == MOTOR_OK);
  assert(MotorOutput_Update(&output) == MOTOR_OK);
  assert(compare_value[0] == 1000U);
  assert(compare_value[1] == 1225U);
  assert(compare_value[2] == 1500U);
  assert(compare_value[3] == 2000U);
  assert(MotorOutput_SetAllThrottle(&output,
                                    invalid,
                                    MOTOR_OUTPUT_MAX_MOTORS) ==
         MOTOR_INVALID_ARGUMENT);
  assert(MotorOutput_Stop(&output) == MOTOR_OK);
  assert(!MotorOutput_IsStarted(&output));
  assert(compare_value[0] == 1000U);
  assert(compare_value[1] == 1000U);
  assert(compare_value[2] == 1000U);
  assert(compare_value[3] == 1000U);
}

static void test_dshot_driver(uint32_t bitrate_hz,
                               uint32_t period_ticks,
                               uint32_t zero_ticks,
                               uint32_t one_ticks)
{
  TIM_TypeDef instance = {0};
  DMA_HandleTypeDef dma = {0};
  TIM_HandleTypeDef htim = {0};
  PwmTimer timer = {0};
  PwmChannel channel[MOTOR_OUTPUT_MAX_MOTORS];
  MotorDshot driver;
  MotorOutput output = {0};
  MotorDshot_Config config;
  const float throttle[MOTOR_OUTPUT_MAX_MOTORS] = {
    0.0f, 0.0001f, 0.5f, 1.0f,
  };
  unsigned int slot;
  unsigned int motor;
  unsigned int previous_dma_start_count;

  htim.Instance = &instance;
  htim.hdma[TIM_DMA_ID_UPDATE] = &dma;
  timer.config.htim = &htim;
  timer.config.counter_clock_hz = 60000000U;
  timer.config.period_ticks = period_ticks;
  timer.initialized = true;
  init_channels(&timer, channel);
  memset(&config, 0, sizeof(config));
  config.timer = &timer;
  config.channel[0] = &channel[0];
  config.channel[1] = &channel[1];
  config.channel[2] = &channel[2];
  config.channel[3] = &channel[3];
  config.bitrate_hz = bitrate_hz;
  config.minimum_throttle_value = MOTOR_DSHOT_MIN_THROTTLE_VALUE;
  config.maximum_throttle_value = MOTOR_DSHOT_MAX_THROTTLE_VALUE;
  config.telemetry = false;

  dma_start_count = 0U;
  service_status = PWM_TIMER_OK;
  assert(timer.config.counter_clock_hz / bitrate_hz == period_ticks);
  if (bitrate_hz == MOTOR_DSHOT300_BITRATE_HZ)
  {
    assert(MotorDshot300_Init(&driver, &output, NULL) ==
           MOTOR_INVALID_ARGUMENT);
    assert(MotorDshot300_Init(NULL, &output, &config) ==
           MOTOR_INVALID_ARGUMENT);
    assert(MotorDshot300_Init(&driver, NULL, &config) ==
           MOTOR_INVALID_ARGUMENT);
    /* A valid DShot600 configuration must still be rejected by this adapter. */
    config.bitrate_hz = MOTOR_DSHOT600_BITRATE_HZ;
    timer.config.period_ticks = 100U;
    assert(MotorDshot300_Init(&driver, &output, &config) ==
           MOTOR_INVALID_ARGUMENT);
    config.bitrate_hz = bitrate_hz;
    timer.config.period_ticks = period_ticks;
    assert(MotorDshot300_Init(&driver, &output, &config) == MOTOR_OK);
  }
  else
  {
    assert(MotorDshot_Init(&driver, &output, &config) == MOTOR_OK);
  }
  assert(driver.zero_compare_ticks == zero_ticks);
  assert(driver.one_compare_ticks == one_ticks);
  assert(MotorOutput_Start(&output) == MOTOR_OK);
  assert(dma_start_count == 1U);
  assert(last_slot_count == MOTOR_DSHOT_FRAME_SLOTS);
  assert(last_channel_count == MOTOR_DSHOT_MOTOR_COUNT);
  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    assert_encoded_packet(last_dma_buffer,
                          motor,
                          expected_packet(0U, 0),
                          zero_ticks,
                          one_ticks);
  }

  assert(MotorOutput_SetAllThrottle(&output,
                                    throttle,
                                    MOTOR_OUTPUT_MAX_MOTORS) == MOTOR_OK);
  assert(MotorOutput_Update(&output) == MOTOR_OK);
  assert(driver.throttle_value[0] == 0U);
  assert(driver.throttle_value[1] == 48U);
  assert(driver.throttle_value[2] == 1048U);
  assert(driver.throttle_value[3] == 2047U);
  assert_encoded_packet(last_dma_buffer,
                        3U,
                        expected_packet(2047U, 0),
                        zero_ticks,
                        one_ticks);
  for (slot = MOTOR_DSHOT_PACKET_BITS;
       slot < MOTOR_DSHOT_FRAME_SLOTS;
       ++slot)
  {
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      assert(last_dma_buffer[slot * MOTOR_DSHOT_MOTOR_COUNT + motor] == 0U);
    }
  }

  service_status = PWM_TIMER_BUSY;
  previous_dma_start_count = dma_start_count;
  assert(MotorOutput_Update(&output) == MOTOR_BUSY);
  assert(dma_start_count == previous_dma_start_count);
  service_status = PWM_TIMER_OK;
  assert(MotorOutput_Stop(&output) == MOTOR_OK);
  assert(!MotorOutput_IsStarted(&output));
  assert(MotorOutput_GetRawOutput(&output, 3U) == 0U);
  assert(compare_value[0] == 0U);
  assert(compare_value[1] == 0U);
  assert(compare_value[2] == 0U);
  assert(compare_value[3] == 0U);
  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    assert_encoded_packet(last_dma_buffer, motor,
                          expected_packet(0U, 0), zero_ticks, one_ticks);
  }

  /* Exercise the bench sequencer through the real DShot encoder, including
   * HAL tick rollover, DMA backpressure and a latched output failure. */
  {
    AppDshotTest test = {0};
    const uint32_t boot_ms = UINT32_MAX - 1000U;
    AppDshotTest_Step(&test, &output, boot_ms);
    assert(test.started && !test.failed);
    AppDshotTest_Step(&test, &output, boot_ms + 14999U);
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      assert(MotorOutput_GetRawOutput(&output, motor) == 0U);
    }
    service_status = PWM_TIMER_BUSY;
    previous_dma_start_count = dma_start_count;
    AppDshotTest_Step(&test, &output, boot_ms + 15000U);
    assert(!test.failed);
    assert(dma_start_count == previous_dma_start_count);
    service_status = PWM_TIMER_OK;
    AppDshotTest_Step(&test, &output, boot_ms + 15001U);
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      assert(MotorOutput_GetRawOutput(&output, motor) == 648U);
      assert(MotorOutput_GetThrottle(&output, motor) == 0.30f);
      assert_encoded_packet(last_dma_buffer, motor,
                            expected_packet(648U, 0), zero_ticks, one_ticks);
    }
    service_status = PWM_TIMER_HAL_ERROR;
    AppDshotTest_Step(&test, &output, boot_ms + 15002U);
    assert(test.failed);
    assert(!MotorOutput_IsStarted(&output));
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      assert(MotorOutput_GetRawOutput(&output, motor) == 0U);
    }
    service_status = PWM_TIMER_OK;
    previous_dma_start_count = dma_start_count;
    AppDshotTest_Step(&test, &output, boot_ms + 21000U);
    assert(dma_start_count == previous_dma_start_count);
    assert(!MotorOutput_IsStarted(&output));
  }
}

int main(void)
{
  test_pwm_driver();
  test_dshot_driver(MOTOR_DSHOT300_BITRATE_HZ, 200U, 75U, 150U);
  test_dshot_driver(MOTOR_DSHOT600_BITRATE_HZ, 100U, 38U, 75U);
  puts("motor output tests: PASS");
  return 0;
}
