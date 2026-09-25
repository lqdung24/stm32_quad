#include "usb_log.h"
#include "dshot_bench.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

extern USBD_HandleTypeDef hUsbDeviceFS;
/* Persistent storage: USB retains this pointer until transfer completion. */
static uint8_t tx[448];
static uint32_t last_ms, last_frames;
static int ready;
volatile uint32_t usb_log_init_error;

void UsbLog_Init(void)
{
  RCC_OscInitTypeDef osc = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  osc.HSI48State = RCC_HSI48_ON;
  osc.PLL.PLLState = RCC_PLL_NONE; /* Preserve the 225 MHz motor clock. */
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    usb_log_init_error = 1; return;
  }
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs = {0};
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB2;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
  crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&crs);
  MX_USB_DEVICE_Init();
  ready = 1;
}

void UsbLog_Step(void)
{
  uint32_t now = HAL_GetTick();
  if (!ready || (uint32_t)(now - last_ms) < 500U) return;
  uint32_t elapsed = now - last_ms;
  last_ms = now;
  uint32_t frames = dshot_bench.frames_completed;
  uint32_t rate = (uint32_t)(((uint64_t)(frames - last_frames) * 1000U) / elapsed);
  last_frames = frames;
  /* Formatting outside interrupt masking; direct-register frame has finished. */
  char line[sizeof(tx)];
  int n = snprintf(line, sizeof(line),
    "DSHOT_REG ms=%lu fault=%lu value=%lu frames=%lu fps=%lu "
    "tim_hz=%lu ticks=%lu hi=%lu/%lu frame_cycles=%lu max_cycles=%lu "
    "beacon=%lu sent=%lu/10 packet=0x%04lx stop=%lu psc=%lu arr=%lu cr1=0x%lx dier=0x%lx\r\n",
    (unsigned long)now, (unsigned long)dshot_bench.fault,
    (unsigned long)dshot_bench.last_value, (unsigned long)frames,
    (unsigned long)rate, (unsigned long)dshot_bench.timer_hz,
    (unsigned long)dshot_bench.bit_ticks, (unsigned long)dshot_bench.zero_ticks,
    (unsigned long)dshot_bench.one_ticks, (unsigned long)dshot_bench.frame_cycles,
    (unsigned long)dshot_bench.max_frame_cycles,
    (unsigned long)dshot_bench.beacon_command, (unsigned long)dshot_bench.beacon_sent,
    (unsigned long)dshot_bench.beacon_packet, (unsigned long)dshot_bench.stop_requested,
    (unsigned long)TIM3->PSC, (unsigned long)TIM3->ARR,
    (unsigned long)TIM3->CR1, (unsigned long)TIM3->DIER);
  if (n <= 0) return;
  if (n >= (int)sizeof(line)) n = (int)sizeof(line) - 1;
  /* USB reset/disconnect may free class data in IRQ; protect check + submit.
   * BUSY/disconnected drops this log, never waits or reuses an active buffer. */
  uint32_t mask = __get_PRIMASK();
  __disable_irq();
  USBD_CDC_HandleTypeDef *cdc = hUsbDeviceFS.pClassData;
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && cdc != NULL && cdc->TxState == 0U) {
    memcpy(tx, line, (size_t)n);
    (void)CDC_Transmit_FS(tx, (uint16_t)n);
  }
  __set_PRIMASK(mask);
}
