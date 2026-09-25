#include "dshot_bench.h"
#include "usb_log.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
volatile DshotBenchStatus dshot_bench;
TIM_TypeDef fake_tim;
USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef cdc;
static uint32_t tick,mask,count;
static uint8_t *held;
static uint16_t held_len;
int HAL_RCC_OscConfig(RCC_OscInitTypeDef *p){assert(p->PLL.PLLState==RCC_PLL_NONE);return HAL_OK;}
void HAL_RCCEx_CRSConfig(RCC_CRSInitTypeDef *p){assert(p->ReloadValue==47999);}
void MX_USB_DEVICE_Init(void){}
uint32_t HAL_GetTick(void){return tick;}
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t v){mask=v;}
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n){assert(mask==1);assert(!cdc.TxState);held=p;held_len=n;cdc.TxState=1;++count;return 0;}
int main(void){
 UsbLog_Init();tick=500;UsbLog_Step();assert(count==0 && mask==0);
 hUsbDeviceFS.dev_state=USBD_STATE_CONFIGURED;tick=1000;UsbLog_Step();assert(count==0);
 hUsbDeviceFS.pClassData=&cdc;dshot_bench.frames_completed=500;
 dshot_bench.last_value=1048;dshot_bench.timer_hz=225000000;
 tick=1500;UsbLog_Step();assert(count==1 && held_len<448);
 char saved[448]={0};memcpy(saved,held,held_len);
 assert(strstr(saved,"value=1048") && strstr(saved,"fps=1000"));
 tick=2000;dshot_bench.last_value=0;UsbLog_Step();assert(count==1);
 assert(memcmp(saved,held,held_len)==0); /* in-flight buffer cannot be overwritten */
 hUsbDeviceFS.pClassData=NULL;hUsbDeviceFS.dev_state=0;
 tick=2500;UsbLog_Step();assert(count==1);
 cdc.TxState=0;hUsbDeviceFS.pClassData=&cdc;hUsbDeviceFS.dev_state=3;
 dshot_bench.fault=11;tick=3000;UsbLog_Step();assert(count==2 && mask==0);
 memset(saved,0,sizeof(saved));memcpy(saved,held,held_len);assert(strstr(saved,"fault=11"));
 cdc.TxState=0;mask=1;tick=3500;UsbLog_Step();assert(count==3 && mask==1);
 puts("USB log: PASS (disconnected/null/busy/reconnect/fault/buffer ownership/IRQ restore)");
}
