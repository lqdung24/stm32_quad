#include "../Inc/motor_dshot.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static MotorStatus MotorDshot_Start(void *driver_context);
static MotorStatus MotorDshot_Stop(void *driver_context);
static MotorStatus MotorDshot_SetThrottle(void *driver_context,
                                          uint8_t motor_index,
                                          float throttle);
static MotorStatus MotorDshot_SetAllThrottle(void *driver_context,
                                             const float *throttle,
                                             uint8_t motor_count);
static MotorStatus MotorDshot_Update(void *driver_context);
static bool MotorDshot_IsStarted(const void *driver_context);
static float MotorDshot_GetThrottle(const void *driver_context,
                                    uint8_t motor_index);
static uint16_t MotorDshot_GetRawOutput(const void *driver_context,
                                        uint8_t motor_index);
static bool MotorDshot_ConfigValid(const MotorDshot_Config *config);
static uint16_t MotorDshot_ThrottleToValue(const MotorDshot *driver,
                                           float throttle);
static uint16_t MotorDshot_MakePacket(uint16_t value, bool telemetry);
static uint8_t MotorDshot_Checksum(uint16_t payload);
static void MotorDshot_EncodePackets(MotorDshot *driver,
                                     const uint16_t *value);
static MotorStatus MotorDshot_StartFrame(MotorDshot *driver,
                                         const uint16_t *value);
static MotorStatus MotorDshot_ForceLow(MotorDshot *driver);
static MotorStatus MotorDshot_FromPwmStatus(PwmTimer_Status status);
static void MotorDshot_CleanDmaBuffer(MotorDshot *driver);

static const MotorOps motor_dshot_ops = {
  .start = MotorDshot_Start,
  .stop = MotorDshot_Stop,
  .set_throttle = MotorDshot_SetThrottle,
  .set_all_throttle = MotorDshot_SetAllThrottle,
  .update = MotorDshot_Update,
  .is_started = MotorDshot_IsStarted,
  .get_throttle = MotorDshot_GetThrottle,
  .get_raw_output = MotorDshot_GetRawOutput,
};

MotorStatus MotorDshot_Init(MotorDshot *driver,
                            MotorOutput *output,
                            const MotorDshot_Config *config)
{
  uint32_t ticks_per_bit;
  MotorStatus status;

  if ((driver == NULL) || (output == NULL) ||
      !MotorDshot_ConfigValid(config))
  {
    return MOTOR_INVALID_ARGUMENT;
  }

  ticks_per_bit = config->timer->config.counter_clock_hz /
                  config->bitrate_hz;
  memset(driver, 0, sizeof(*driver));
  driver->config = *config;
  /* Rounded integer forms of 37.5% and 75% duty cycles. */
  driver->zero_compare_ticks = (ticks_per_bit * 3U + 4U) / 8U;
  driver->one_compare_ticks = (ticks_per_bit * 3U + 2U) / 4U;
  if ((driver->zero_compare_ticks == 0U) ||
      (driver->zero_compare_ticks >= driver->one_compare_ticks) ||
      (driver->one_compare_ticks >= ticks_per_bit))
  {
    return MOTOR_INVALID_ARGUMENT;
  }

  status = MotorDshot_ForceLow(driver);
  if (status != MOTOR_OK)
  {
    return status;
  }
  driver->initialized = true;

  status = MotorOutput_Init(output,
                            &motor_dshot_ops,
                            driver,
                            MOTOR_DSHOT_MOTOR_COUNT,
                            MOTOR_PROTOCOL_DSHOT);
  if (status != MOTOR_OK)
  {
    driver->initialized = false;
  }
  return status;
}

static MotorStatus MotorDshot_Start(void *driver_context)
{
  MotorDshot *driver = (MotorDshot *)driver_context;
  uint16_t stop_value[MOTOR_DSHOT_MOTOR_COUNT] = {0U, 0U, 0U, 0U};
  MotorStatus status;

  if ((driver == NULL) || !driver->initialized)
  {
    return MOTOR_INVALID_ARGUMENT;
  }

  status = MotorDshot_FromPwmStatus(PwmTimer_StopDma(driver->config.timer));
  if (status != MOTOR_OK)
  {
    return status;
  }
  status = MotorDshot_ForceLow(driver);
  if (status != MOTOR_OK)
  {
    return status;
  }
  memset(driver->requested_throttle, 0, sizeof(driver->requested_throttle));
  memset(driver->applied_throttle, 0, sizeof(driver->applied_throttle));
  memset(driver->throttle_value, 0, sizeof(driver->throttle_value));
  driver->started = true;
  status = MotorDshot_StartFrame(driver, stop_value);
  if ((status != MOTOR_OK) && (status != MOTOR_BUSY))
  {
    driver->started = false;
  }
  return status;
}

static MotorStatus MotorDshot_Stop(void *driver_context)
{
  MotorDshot *driver = (MotorDshot *)driver_context;
  uint16_t stop_value[MOTOR_DSHOT_MOTOR_COUNT] = {0U, 0U, 0U, 0U};
  MotorStatus stop_status;
  MotorStatus low_status;
  MotorStatus frame_status;

  if ((driver == NULL) || !driver->initialized)
  {
    return MOTOR_INVALID_ARGUMENT;
  }

  driver->started = false;
  memset(driver->requested_throttle, 0, sizeof(driver->requested_throttle));
  memset(driver->applied_throttle, 0, sizeof(driver->applied_throttle));
  memset(driver->throttle_value, 0, sizeof(driver->throttle_value));

  stop_status = MotorDshot_FromPwmStatus(
      PwmTimer_StopDma(driver->config.timer));
  low_status = MotorDshot_ForceLow(driver);
  if ((stop_status != MOTOR_OK) || (low_status != MOTOR_OK))
  {
    return MOTOR_ERROR;
  }

  /* Send an explicit DShot value 0 after forcing a safe-low transition. */
  frame_status = MotorDshot_StartFrame(driver, stop_value);
  return frame_status;
}

static MotorStatus MotorDshot_SetThrottle(void *driver_context,
                                          uint8_t motor_index,
                                          float throttle)
{
  MotorDshot *driver = (MotorDshot *)driver_context;

  if ((driver == NULL) || !driver->initialized || !driver->started ||
      (motor_index >= MOTOR_DSHOT_MOTOR_COUNT) || !isfinite(throttle) ||
      (throttle < 0.0f) || (throttle > 1.0f))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  driver->requested_throttle[motor_index] = throttle;
  return MOTOR_OK;
}

static MotorStatus MotorDshot_SetAllThrottle(void *driver_context,
                                             const float *throttle,
                                             uint8_t motor_count)
{
  MotorDshot *driver = (MotorDshot *)driver_context;
  uint8_t motor;

  if ((driver == NULL) || !driver->initialized || !driver->started ||
      (throttle == NULL) || (motor_count != MOTOR_DSHOT_MOTOR_COUNT))
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    if (!isfinite(throttle[motor]) || (throttle[motor] < 0.0f) ||
        (throttle[motor] > 1.0f))
    {
      return MOTOR_INVALID_ARGUMENT;
    }
  }
  memcpy(driver->requested_throttle,
         throttle,
         sizeof(driver->requested_throttle));
  return MOTOR_OK;
}

static MotorStatus MotorDshot_Update(void *driver_context)
{
  MotorDshot *driver = (MotorDshot *)driver_context;
  uint16_t value[MOTOR_DSHOT_MOTOR_COUNT];
  PwmTimer_Status service_status;
  MotorStatus frame_status;
  uint8_t motor;

  if ((driver == NULL) || !driver->initialized || !driver->started)
  {
    return MOTOR_ERROR;
  }

  service_status = PwmTimer_ServiceDma(driver->config.timer);
  if (service_status == PWM_TIMER_BUSY)
  {
    return MOTOR_BUSY;
  }
  if (service_status != PWM_TIMER_OK)
  {
    return MOTOR_ERROR;
  }

  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    value[motor] = MotorDshot_ThrottleToValue(
        driver, driver->requested_throttle[motor]);
  }
  frame_status = MotorDshot_StartFrame(driver, value);
  if (frame_status == MOTOR_OK)
  {
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      driver->applied_throttle[motor] =
          driver->requested_throttle[motor];
      driver->throttle_value[motor] = value[motor];
    }
  }
  return frame_status;
}

static bool MotorDshot_IsStarted(const void *driver_context)
{
  const MotorDshot *driver = (const MotorDshot *)driver_context;

  return (driver != NULL) && driver->initialized && driver->started;
}

static float MotorDshot_GetThrottle(const void *driver_context,
                                    uint8_t motor_index)
{
  const MotorDshot *driver = (const MotorDshot *)driver_context;

  if ((driver == NULL) || !driver->initialized ||
      (motor_index >= MOTOR_DSHOT_MOTOR_COUNT))
  {
    return 0.0f;
  }
  return driver->applied_throttle[motor_index];
}

static uint16_t MotorDshot_GetRawOutput(const void *driver_context,
                                        uint8_t motor_index)
{
  const MotorDshot *driver = (const MotorDshot *)driver_context;

  if ((driver == NULL) || !driver->initialized ||
      (motor_index >= MOTOR_DSHOT_MOTOR_COUNT))
  {
    return 0U;
  }
  return driver->throttle_value[motor_index];
}

static bool MotorDshot_ConfigValid(const MotorDshot_Config *config)
{
  uint32_t ticks_per_bit;
  uint8_t motor;

  if ((config == NULL) || (config->timer == NULL) ||
      !config->timer->initialized || (config->bitrate_hz == 0U) ||
      (config->timer->config.htim == NULL) ||
      (config->minimum_throttle_value < MOTOR_DSHOT_MIN_THROTTLE_VALUE) ||
      (config->maximum_throttle_value > MOTOR_DSHOT_MAX_THROTTLE_VALUE) ||
      (config->minimum_throttle_value >= config->maximum_throttle_value) ||
      ((config->timer->config.counter_clock_hz % config->bitrate_hz) != 0U) ||
      (config->timer->config.htim->hdma[TIM_DMA_ID_UPDATE] == NULL))
  {
    return false;
  }

  ticks_per_bit = config->timer->config.counter_clock_hz /
                  config->bitrate_hz;
  if ((ticks_per_bit < 4U) ||
      (config->timer->config.period_ticks != ticks_per_bit))
  {
    return false;
  }

  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    if ((config->channel[motor] == NULL) ||
        !config->channel[motor]->initialized ||
        (config->channel[motor]->config.timer != config->timer) ||
        (config->channel[motor]->config.channel !=
         (TIM_CHANNEL_1 + ((uint32_t)motor * 4U))))
    {
      return false;
    }
  }
  return true;
}

static uint16_t MotorDshot_ThrottleToValue(const MotorDshot *driver,
                                           float throttle)
{
  float mapped;

  if (throttle <= 0.0f)
  {
    return 0U;
  }
  mapped = (float)driver->config.minimum_throttle_value +
      throttle * (float)(driver->config.maximum_throttle_value -
                         driver->config.minimum_throttle_value);
  return (uint16_t)(mapped + 0.5f);
}

static uint16_t MotorDshot_MakePacket(uint16_t value, bool telemetry)
{
  uint16_t payload = (uint16_t)((value << 1U) | (telemetry ? 1U : 0U));

  return (uint16_t)((payload << 4U) | MotorDshot_Checksum(payload));
}

static uint8_t MotorDshot_Checksum(uint16_t payload)
{
  return (uint8_t)((payload ^ (payload >> 4U) ^ (payload >> 8U)) & 0x0FU);
}

static void MotorDshot_EncodePackets(MotorDshot *driver,
                                     const uint16_t *value)
{
  uint16_t packet[MOTOR_DSHOT_MOTOR_COUNT];
  uint8_t motor;
  uint8_t slot;

  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    packet[motor] = MotorDshot_MakePacket(value[motor],
                                          driver->config.telemetry);
  }

  for (slot = 0U; slot < MOTOR_DSHOT_PACKET_BITS; ++slot)
  {
    uint16_t mask = (uint16_t)(1U << (15U - slot));
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      driver->dma_buffer[(uint32_t)slot * MOTOR_DSHOT_MOTOR_COUNT + motor] =
          ((packet[motor] & mask) != 0U) ?
              driver->one_compare_ticks : driver->zero_compare_ticks;
    }
  }
  for (; slot < MOTOR_DSHOT_FRAME_SLOTS; ++slot)
  {
    for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
    {
      driver->dma_buffer[(uint32_t)slot * MOTOR_DSHOT_MOTOR_COUNT + motor] =
          0U;
    }
  }
}

static MotorStatus MotorDshot_StartFrame(MotorDshot *driver,
                                         const uint16_t *value)
{
  MotorDshot_EncodePackets(driver, value);
  MotorDshot_CleanDmaBuffer(driver);
  return MotorDshot_FromPwmStatus(PwmTimer_StartCompareBurstDma(
      driver->config.timer,
      driver->dma_buffer,
      MOTOR_DSHOT_FRAME_SLOTS,
      MOTOR_DSHOT_MOTOR_COUNT));
}

static MotorStatus MotorDshot_ForceLow(MotorDshot *driver)
{
  uint8_t motor;
  bool ok = true;

  for (motor = 0U; motor < MOTOR_DSHOT_MOTOR_COUNT; ++motor)
  {
    if (PwmChannel_SetCompare(driver->config.channel[motor], 0U) !=
        PWM_TIMER_OK)
    {
      ok = false;
    }
  }
  return ok ? MOTOR_OK : MOTOR_ERROR;
}

static MotorStatus MotorDshot_FromPwmStatus(PwmTimer_Status status)
{
  if (status == PWM_TIMER_OK)
  {
    return MOTOR_OK;
  }
  if (status == PWM_TIMER_BUSY)
  {
    return MOTOR_BUSY;
  }
  if (status == PWM_TIMER_INVALID_ARGUMENT)
  {
    return MOTOR_INVALID_ARGUMENT;
  }
  return MOTOR_ERROR;
}

static void MotorDshot_CleanDmaBuffer(MotorDshot *driver)
{
#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  if ((SCB->CCR & SCB_CCR_DC_Msk) != 0U)
  {
    SCB_CleanDCache_by_Addr(driver->dma_buffer,
                           (int32_t)sizeof(driver->dma_buffer));
  }
#else
  (void)driver;
#endif
}
