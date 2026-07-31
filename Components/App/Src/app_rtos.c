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

#define APP_RTOS_FLIGHT_STACK_BYTES       2048U
#define APP_RTOS_TELEMETRY_STACK_BYTES    1024U
#define APP_RTOS_HOUSEKEEPING_STACK_BYTES  512U

static osThreadId_t flight_thread;
static osThreadId_t telemetry_thread;
static osThreadId_t housekeeping_thread;
static bool application_tasks_started;

static void flight_task(void *argument);
static void telemetry_task(void *argument);
static void housekeeping_task(void *argument);
static uint32_t milliseconds_to_ticks(uint32_t period_ms);
static void delay_until_next_period(uint32_t *next_tick,
                                    uint32_t period_ticks);
static void terminate_thread(osThreadId_t *thread);

static const osThreadAttr_t flight_attributes = {
    .name = "flightControl",
    .stack_size = APP_RTOS_FLIGHT_STACK_BYTES,
    .priority = osPriorityHigh,
};

static const osThreadAttr_t telemetry_attributes = {
    .name = "telemetry",
    .stack_size = APP_RTOS_TELEMETRY_STACK_BYTES,
    .priority = osPriorityBelowNormal,
};

static const osThreadAttr_t housekeeping_attributes = {
    .name = "housekeeping",
    .stack_size = APP_RTOS_HOUSEKEEPING_STACK_BYTES,
    .priority = osPriorityLow,
};

void AppRtos_Bootstrap(void)
{
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
   * Create lower-priority support threads first. The high-priority flight
   * thread is created last so it cannot run and arm outputs before bootstrap
   * knows that every required thread exists.
   */
  housekeeping_thread =
      osThreadNew(housekeeping_task, NULL, &housekeeping_attributes);
  if (housekeeping_thread != NULL)
  {
    telemetry_thread =
        osThreadNew(telemetry_task, NULL, &telemetry_attributes);
  }
  if (telemetry_thread != NULL)
  {
    flight_thread =
        osThreadNew(flight_task, NULL, &flight_attributes);
  }

  if ((flight_thread != NULL) &&
      (telemetry_thread != NULL) &&
      (housekeeping_thread != NULL))
  {
    application_tasks_started = true;
    osThreadExit();
    return;
  }

  /*
   * Keep the CubeMX bootstrap task alive and retry from a clean state.
   * Motor output remains at the disarmed value set during App_Init().
   */
  terminate_thread(&housekeeping_thread);
  terminate_thread(&telemetry_thread);
  terminate_thread(&flight_thread);
  osDelay(milliseconds_to_ticks(APP_RTOS_RETRY_PERIOD_MS));
}

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

static void housekeeping_task(void *argument)
{
  const uint32_t period_ticks =
      milliseconds_to_ticks(APP_RTOS_HOUSEKEEPING_PERIOD_MS);
  uint32_t next_tick = osKernelGetTickCount();

  (void)argument;
  for (;;)
  {
    App_HousekeepingStep(HAL_GetTick());
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
     * spinning at high priority and starving lower-priority tasks.
     */
    *next_tick = osKernelGetTickCount();
    osThreadYield();
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
