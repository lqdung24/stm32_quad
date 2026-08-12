#include "app_rtos.h"

#include "app.h"
#include "cmsis_os2.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define APP_RTOS_FLIGHT_PERIOD_MS           1U
#define APP_RTOS_TELEMETRY_PERIOD_MS        5U
#define APP_RTOS_HOUSEKEEPING_PERIOD_MS   100U
#define APP_RTOS_RETRY_PERIOD_MS          1000U

#define APP_RTOS_FLIGHT_STACK_BYTES       4096U
#define APP_RTOS_TELEMETRY_STACK_BYTES    2048U
#define APP_RTOS_HOUSEKEEPING_STACK_BYTES 1024U

#if APP_RTOS_FLIGHT_ENABLE
static osThreadId_t flight_thread;
#endif
static osThreadId_t telemetry_thread;
static bool application_tasks_started;

#if APP_RTOS_FLIGHT_ENABLE
static void flight_task(void *argument);
#endif
static void telemetry_task(void *argument);
static uint32_t milliseconds_to_ticks(uint32_t period_ms);
static void delay_until_next_period(uint32_t *next_tick,
                                    uint32_t period_ticks);
static void terminate_thread(osThreadId_t *thread);

#if APP_RTOS_FLIGHT_ENABLE
static const osThreadAttr_t flight_attributes = {
    .name = "flightControl",
    .stack_size = APP_RTOS_FLIGHT_STACK_BYTES,
    .priority = osPriorityHigh,
};
#endif

static const osThreadAttr_t telemetry_attributes = {
    .name = "telemetry",
    .stack_size = APP_RTOS_TELEMETRY_STACK_BYTES,
    .priority = osPriorityBelowNormal,
};

void AppRtos_Bootstrap(void)
{
  uint32_t housekeeping_period_ticks;
  uint32_t next_housekeeping_tick;

  if (osKernelGetState() != osKernelRunning)
  {
    return;
  }

  if (application_tasks_started)
  {
    osThreadExit();
    return;
  }

  /*
   * Keep the CubeMX default task alive as the housekeeping/supervisor task.
   * This leaves a visible LED heartbeat even if a dynamically-created task
   * cannot be allocated or later fails.
   */
  telemetry_thread =
      osThreadNew(telemetry_task, NULL, &telemetry_attributes);
#if APP_RTOS_FLIGHT_ENABLE
  if (telemetry_thread != NULL)
  {
    flight_thread =
        osThreadNew(flight_task, NULL, &flight_attributes);
  }
#endif

  if ((telemetry_thread != NULL)
#if APP_RTOS_FLIGHT_ENABLE
      && (flight_thread != NULL)
#endif
     )
  {
    application_tasks_started = true;
  }
  else
  {
    /*
     * Retry from a clean state. Motor output remains at the disarmed value
     * set during App_Init(), while the default task continues blinking the
     * activity LED to prove that the scheduler is alive.
     */
    terminate_thread(&telemetry_thread);
#if APP_RTOS_FLIGHT_ENABLE
    terminate_thread(&flight_thread);
#endif
    App_HousekeepingStep(HAL_GetTick());
    (void)osDelay(milliseconds_to_ticks(APP_RTOS_RETRY_PERIOD_MS));
    return;
  }

  housekeeping_period_ticks =
      milliseconds_to_ticks(APP_RTOS_HOUSEKEEPING_PERIOD_MS);
  next_housekeeping_tick = osKernelGetTickCount();
  for (;;)
  {
    App_HousekeepingStep(HAL_GetTick());
    delay_until_next_period(&next_housekeeping_tick,
                            housekeeping_period_ticks);
  }
}

#if APP_RTOS_FLIGHT_ENABLE
static void flight_task(void *argument)
{
  const uint32_t period_ticks =
      milliseconds_to_ticks(APP_RTOS_FLIGHT_PERIOD_MS);
  uint32_t next_tick = osKernelGetTickCount();

  (void)argument;
  for (;;)
  {
    App_FlightControlStep(HAL_GetTick());
    delay_until_next_period(&next_tick, period_ticks);
  }
}
#endif

static void telemetry_task(void *argument)
{
  const uint32_t period_ticks =
      milliseconds_to_ticks(APP_RTOS_TELEMETRY_PERIOD_MS);
  uint32_t next_tick = osKernelGetTickCount();

  (void)argument;
  for (;;)
  {
    App_TelemetryStep(HAL_GetTick());
    delay_until_next_period(&next_tick, period_ticks);
  }
}

static uint32_t milliseconds_to_ticks(uint32_t period_ms)
{
  const uint32_t tick_frequency_hz = osKernelGetTickFreq();
  uint32_t ticks =
      (uint32_t)((((uint64_t)period_ms * tick_frequency_hz) + 999U) /
                 1000U);

  if (ticks == 0U)
  {
    ticks = 1U;
  }
  return ticks;
}

static void delay_until_next_period(uint32_t *next_tick,
                                    uint32_t period_ticks)
{
  *next_tick += period_ticks;
  if (osDelayUntil(*next_tick) != osOK)
  {
    /*
     * The task missed its absolute deadline. Resynchronize instead of
     * spinning at high priority and starving lower-priority tasks. Yielding
     * is insufficient because FreeRTOS only yields to ready tasks at the same
     * priority; a one-tick delay guarantees telemetry and housekeeping can
     * run after an overrun.
     */
    *next_tick = osKernelGetTickCount();
    (void)osDelay(1U);
  }
}

static void terminate_thread(osThreadId_t *thread)
{
  if ((thread != NULL) && (*thread != NULL))
  {
    (void)osThreadTerminate(*thread);
    *thread = NULL;
  }
}
