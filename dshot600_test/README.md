# DShot600 bench — direct TIM3 registers

Tháo cánh trước khi cấp nguồn ESC. Nối sẵn SIG/GND, cấp nguồn ESC rồi reset
STM32: firmware gửi zero 3 giây, sau đó command beacon 1 (telemetry=1)
10 lần cách nhau 300 ms trên cả bốn chân. Giữa các command và sau lần thứ 10
vẫn gửi zero mỗi 1 ms; không tự lên ga.
Nếu cắm SIG muộn, reset STM32 để chạy lại phép thử. Không điều khiển qua web.

## Clock và chân

Clock đã generate dùng HSI 64 MHz, PLLM=4, PLLN=28, PLLFRACN=1024/8192,
PLLP=2: SYSCLK/HCLK=225 MHz, APB1=112.5 MHz, TIM3=225 MHz.
Driver đọc RCC kể cả TIMPRE, đặt PSC=0, ARR=374: 375 tick/bit, 600 kbit/s.
HIGH bit 0=141 tick (~0.627 us), bit 1=281 tick (~1.249 us).
Sai số clock vật lý vẫn phụ thuộc HSI; cần đo waveform trên board.

| Motor | GPIO | Channel |
|---|---|---|
| M1 | PA6 | TIM3_CH1 |
| M2 | PA7 | TIM3_CH2 |
| M3 | PB0 | TIM3_CH3 |
| M4 | PB1 | TIM3_CH4 |

## Đường phát không DMA

`Core/Src/dshot_bench.c` cấu hình trực tiếp GPIO MODER/AFR và TIM3
PSC/ARR/CCMR/CCER/CCR/EGR/CR1. Không gọi HAL PWM hay HAL DMA để phát.
TIM3 DIER=0, IRQ DMA1_Stream0 bị tắt trong Init. Các khai báo/khởi tạo DMA
CubeMX cũ còn trong mã sinh và .ioc nhưng không chạy truyền DMA; không cần
generate lại để dùng driver này. Bộ đệm CPU không cần clean D-cache cho DMA.

`DshotBench_Step()` gửi một frame mỗi 1 ms theo HAL TIM6 tick:

1. CPU mã hóa value thành 16 bit, checksum, telemetry=1 cho beacon và 0 cho stop, thêm 2 slot zero.
2. Khóa ngắt, nạp active CCR=0 qua UG; nạp bit đầu vào preload, bật counter.
3. Update đầu tiên kết thúc slot LOW dẫn và bắt đầu bit đầu bằng phần cứng.
4. Sau mỗi update, CPU ghi compare bit kế vào CCR1–CCR4 preload. Update
   tiếp theo nạp đồng thời cả bốn channel. Không dùng ngắt từng bit.
5. Khi đến slot zero cuối, dừng counter với active compare=0, khôi phục PRIMASK.

Hàm phát dùng GCC optimize O2 ngay cả trong Debug O0; ngắt bị khóa khoảng
30 us/frame, khoảng 3% thời gian ở 1 kHz. Timeout DWT 100 us nếu timer kẹt;
phát hiện tràn trong lúc nạp CCR thì chốt lỗi và ép bốn chân GPIO LOW.
Không tự khởi động lại sau lỗi. Không đảm bảo timing trước khi đo thực tế.

## Debug

Xem `dshot_bench`: `fault`, `last_value`, `frames_completed`, `timer_hz`,
`bit_ticks`, `zero_ticks`, `one_ticks`, `frame_cycles`, `max_frame_cycles`.
`dma_error` giữ lại để tương thích Expressions cũ, luôn 0 và không dùng nữa.
Ở 225 MHz, chia frame_cycles cho 225 để ra us.

- fault=0: không phát hiện lỗi; 1: timer không đúng TIM3; 4: clock không hợp lệ;
  6: DWT không chạy; 7: stop_requested; 9: timer/frame timeout;
  11: CPU không nạp xong CCR trước update kế tiếp.
- Đặt `dshot_bench.stop_requested=1` khi CPU đang chạy để dừng ở loop kế tiếp.
- Halt debugger không thực thi stop; ngắt nguồn ESC khi cần dừng độc lập MCU.
- LOW là mất tín hiệu, thời gian ESC ngừng quay phụ thuộc failsafe ESC.

## USB CDC log

Đã thêm USB_OTG_FS Device Only / CDC trên PA11 (D−), PA12 (D+).
USB dùng HSI48 48 MHz; `UsbLog_Init()` bật HSI48 và CRS đồng bộ USB2 SOF,
không thay đổi PLL1/TIM3 225 MHz. USB IRQ priority 6; USB không dùng DMA.
CubeMX 6.17 đã sinh stack USB trong thư mục tạm và stack đã được tích hợp.

Cắm cổng USB native nối PA11/PA12 của board (không phải cổng ST-LINK),
mở serial monitor ở 115200 8N1. CDC baud là thông tin line coding,
không thay đổi bitrate DShot. Firmware xuất một dòng mỗi 500 ms, kể cả khi fault.
Dòng ví dụ (không phải dữ liệu đo trên board):

```text
DSHOT_REG ms=5000 fault=0 value=0 frames=4900 fps=1000 tim_hz=225000000 ticks=375 hi=141/281 frame_cycles=6800 max_cycles=6850 beacon=1 sent=7/10 packet=0x0033 stop=0 psc=0 arr=374 cr1=0x0 dier=0x0
```

`cr1=0` bình thường giữa frame vì driver dừng counter ở LOW;
`dier=0` xác nhận không dùng request DMA/ngắt TIM3. `fps` là số frame phát
hoàn tất tính theo HAL tick trong khoảng log, không phải xác nhận ESC nhận.
Khi `fault!=0`, gửi lại nguyên dòng log để chẩn đoán. `value` là lệnh cuối
phát thành công, có thể vẫn giữ command trước fault trong khi chân đã bị ép LOW.

Logger dùng buffer TX static, không ghi đè khi TxState BUSY; rút cáp, chưa
configured hoặc host không đọc thì bỏ dòng, không chờ. Format chạy ngoài
vùng khóa ngắt, sau khi frame DShot đã hoàn tất. Logger không nhận lệnh ga.

Sau cập nhật project trong CubeIDE: Refresh (F5), Clean/Build để nhận thư mục
USB_DEVICE/Middlewares và include paths mới. Không cần generate lại để build.
Nếu regenerate sau này, giữ UsbLog_Init/Step trong USER CODE và bảo đảm
MX_USB_DEVICE_Init chỉ gọi một lần (UsbLog_Init đã gọi nó), mỗi IRQ chỉ forward
HAL một lần. Line coding CDC được lưu/đọc lại trong USER CODE của usbd_cdc_if.c.


## Build / test

```sh
cc -std=c11 -Wall -Wextra -Werror -ICore/Inc Tests/test_encode.c -o /tmp/dshot_encode
/tmp/dshot_encode
python3 Tests/build_firmware.py
```

Build script output `/tmp/dshot600_test_build/dshot600_test.elf`, dùng linker
FLASH hiện tại. CubeIDE tự build file viết tay trong Core/Src. Main đã có
Init và Step trong USER CODE; TIM6 IRQ phải gọi HAL_TIM_IRQHandler đúng một lần.

Host test kiểm packet của 2048 giá trị với cả hai telemetry bit, 4 channel và slot zero; không kiểm
bus latency/CCR preload thực tế. Bench tháo cánh: kiểm chu kỳ bit 1.667 us,
16 bit/packet, packet beacon 1=0x0033, frame khoảng 1 ms, stop và fault giữ LOW.
Chưa flash hoặc đo board.

Test logger host (mock USB, không xác nhận enumeration phần cứng):

```sh
cc -std=c11 -Wall -Wextra -Werror -ITests/usb_log -ICore/Inc \
  Tests/usb_log/test_usb_log.c Core/Src/usb_log.c -o /tmp/test_usb_log
/tmp/test_usb_log
```

Kiểm tra: disconnected, null class, BUSY, giữ nguyên buffer đang truyền,
rút/cắm lại, vẫn log fault và khôi phục PRIMASK. Chưa flash hay đo USB trên board.

## Thử ESC decode bằng beacon

API `DshotBench_RequestBeacon(command)` nhận command thô 1–5, trả 1 nếu nhận,
0 nếu không hợp lệ hoặc đang fault/stop/chưa init. Gọi từ main loop để thử lại;
mỗi yêu cầu bắt đầu lại 3 giây zero rồi gửi 10 beacon. Init tự yêu cầu command 1.
API này chỉ cho beacon, không phát các command thay đổi cấu hình ESC.
USB `sent` đếm frame beacon MCU đã phát, không phải ACK từ ESC;
`value` thường bằng 0 vì log lấy mẫu 500 ms. `packet` lưu packet beacon có telemetry.

Theo https://betaflight.com/docs/development/API/Dshot, beep cần chờ ít nhất
260 ms trước command tiếp theo; dùng 300 ms thay cho burst 10 frame ở 1 kHz.
Tiếng beep đáp đúng đợt thử là bằng chứng ESC hiểu command. Không beep chưa
chứng minh chắc chắn lỗi decode: cần motor nối ESC để phát âm, kiểm tra nguồn,
âm lượng beacon/config và đo SIG so với GND tại ESC nếu vẫn im lặng.
