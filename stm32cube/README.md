# Hướng dẫn đi dây drone STM32H743 + ESP32-S3

> **An toàn:** Tháo toàn bộ cánh quạt trước khi cấp nguồn hoặc thử motor.
> Không cấp nguồn motor/ESC từ STM32 hay ESP32.

## 1. Sơ đồ tổng quát

```text
Điện thoại
    │ Wi-Fi
    ▼
ESP32-S3 ── UART 460800 ── STM32H743 ── PWM 50 Hz ── ESC 1 ── Motor 1
    │                         │                       ESC 2 ── Motor 2
    │                         │                       ESC 3 ── Motor 3
    │                         │                       ESC 4 ── Motor 4
    │                         │
    GND───────────────────────GND──────────────────── GND các ESC

Pin/battery (+) ───────────────────────────────────── Nguồn động lực các ESC
Pin/battery (-) ───────────────────────────────────── GND nguồn động lực
```

Tất cả ESP32-S3, STM32 và ESC phải có **GND chung**.

## 2. ESP32-S3 nối STM32H743

| ESP32-S3 | STM32H743 | Chức năng |
|---|---|---|
| GPIO17, UART1 TX | PA10, USART1 RX | Lệnh điều khiển ESP32 → STM32 |
| GPIO18, UART1 RX | PA9, USART1 TX | Trạng thái STM32 → ESP32 |
| GND | GND | Mass tín hiệu chung |

Cấu hình UART hai bên:

- Baud rate: `460800`
- 8 data bits, no parity, 1 stop bit (`8N1`)
- Chế độ bất đồng bộ, TX/RX
- Mức logic `3.3 V`

Không nối UART vào điện áp `5 V`. Không dùng TX0/RX0 của ESP32-S3 nếu UART0
vẫn đang được dùng cho console/debug 115200 baud.

## 3. STM32 nối bốn ESC

Chiều quay được xác định khi nhìn từ trên xuống drone. Ngưỡng idle cấu hình
có thêm `20 µs` so với mức motor bắt đầu quay đo được khi tháo cánh.

| Motor | Vị trí | STM32H743 | Timer | Chiều quay | Bắt đầu quay | Idle cấu hình |
|---:|---|---|---|---|---:|---:|
| M1 | front-left | PA6 | TIM3_CH1 | Thuận (CW) | `1200 µs` | `1220 µs` |
| M2 | rear-left | PA7 | TIM3_CH2 | Ngược (CCW) | `1205 µs` | `1225 µs` |
| M3 | front-right | PB0 | TIM3_CH3 | Ngược (CCW) | `1190 µs` | `1210 µs` |
| M4 | rear-right | PB1 | TIM3_CH4 | Thuận (CW) | `1205 µs` | `1225 µs` |

Với mỗi ESC:

```text
STM32 PWM pin ───────── ESC Signal
STM32 GND ───────────── ESC Signal GND
Battery + ───────────── ESC Power +
Battery - ───────────── ESC Power -
Ba dây pha ESC ──────── Motor
```

PWM hiện được cấu hình:

- Tần số: `50 Hz`
- Disarm/throttle 0: `1000 µs`
- Command motor `N` tương ứng pulse danh nghĩa `1000 + N µs`; ví dụ command
  `200` là `1200 µs`, command `500` là `1500 µs`.
- Khi armed và collective dương, pulse thấp hơn idle riêng trong bảng sẽ được
  nâng lên idle đó (`1220/1225/1210/1225 µs`).
- PID rate và Quad-X mixer có thể tạo pulse khác nhau cho bốn motor, tối đa
  `2000 µs`; giới hạn collective `500` không phải giới hạn cứng sau mixer.

Nếu motor quay sai chiều, ngắt nguồn rồi đổi chéo **bất kỳ hai trong ba dây
pha** giữa ESC và motor.

### Dây đỏ 5 V/BEC của ESC

- Không nối dây 5 V/BEC vào chân `3V3`.
- Nếu STM32 và ESP32 đang được cấp nguồn bằng USB, để dây 5 V/BEC của ESC
  không kết nối.
- Nếu muốn dùng BEC để cấp nguồn board, chỉ dùng một nguồn 5 V đã kiểm tra và
  nối vào đúng chân `5V/VIN` theo sơ đồ của board. Không ghép song song nhiều
  đầu ra BEC nếu nhà sản xuất ESC không cho phép.

## 4. ICM20948 nối STM32 qua SPI1

| ICM20948 | STM32H743 | Chức năng |
|---|---|---|
| SCLK/SCL | PG11 | SPI1 SCK |
| SDO/AD0 | PG9 | SPI1 MISO |
| SDI/SDA | PB5 | SPI1 MOSI |
| CS/NCS | PA4 | Chip select |
| VCC | 3V3 | Nguồn logic |
| GND | GND | Mass chung |

Chỉ cấp `3.3 V` nếu module ICM20948 không có bộ ổn áp/chuyển mức riêng.

## 5. Checklist trước khi cấp điện

1. Tháo cánh quạt.
2. Dùng đồng hồ kiểm tra không chập `Battery +` với `GND`.
3. Xác nhận GPIO17 nối PA10 và GPIO18 nối PA9, không đấu TX với TX.
4. Nối GND ESP32, STM32 và cả bốn ESC với nhau.
5. Cấp nguồn logic cho ESP32/STM32 trước và kiểm tra web báo STM32 `ONLINE`.
6. Cấp nguồn động lực ESC.
7. Nhấn **DISARM / RESET**, sau đó giữ **ARM** một giây khi throttle bằng 0.
8. Tăng throttle chậm, kiểm tra từng motor và nút **DỪNG KHẨN CẤP**.

## 6. Lưu ý chân trên board

Các tên `PA6`, `PA7`, `PB0`, `PB1`, `PA9`, `PA10`, `PG9`, `PG11`, `PB5` và
`PA4` là tên chân của vi điều khiển STM32H743. Vị trí chân trên header phụ
thuộc board STM32 cụ thể; phải đối chiếu schematic/pinout của đúng board trước
khi cắm dây.
