# Air ESP32 - function flow

## Luồng tổng thể

`app_main` khởi tạo NVS, LED, UART nối STM32 và ESP-NOW nối Ground. Từ đó có hai chiều dữ liệu độc lập:

```text
Ground --ESP-NOW--> on_espnow_packet --UART/COBS--> STM32
STM32 --UART/COBS--> on_uart_packet --ESP-NOW--> Ground
                              |
                              +--> cập nhật LED theo trạng thái STM32
```

`air_link_monitor_task` chạy song song mỗi 50 ms. Nếu Ground vẫn gửi control nhưng STM32 không còn gửi status trong 300 ms, task phát status FAILSAFE giả về Ground và bật LED lỗi. Air chỉ làm bridge; quyết định motor/failsafe thật thuộc STM32.

## `main/main.c`

- `monotonic_ms()` lấy tick FreeRTOS và đổi sang mili-giây; dùng cho mọi timeout của link.
- `decode_control(packet, length, command)` gọi decoder DroneProtocol và chỉ trả `true` khi packet control hợp lệ hoàn toàn.
- `log_espnow_link_config()` đọc MAC local, MAC peer và channel từ transport rồi ghi log; lỗi đọc cấu hình làm dừng qua `ESP_ERROR_CHECK`.
- `update_air_led(status)` ánh xạ trạng thái STM32 sang LED: DISARMED xanh, ARMED đỏ, FAILSAFE/ERROR lỗi, BOOT/default chờ client.
- `remember_ground_control(command)` vào critical section, sao chép command cuối, timestamp và cờ đã nhận control để monitor task đọc nhất quán.
- `on_espnow_packet(packet, length)` xác thực packet là control; packet sai bị bỏ. Packet đúng được lưu làm heartbeat rồi chuyển nguyên byte qua `uart_transport_send_packet`.
- `on_uart_packet(packet, length)` thử decode status. Nếu đúng, cập nhật heartbeat STM32 và LED; dù packet là status hay telemetry, nó vẫn được chuyển nguyên vẹn qua ESP-NOW.
- `send_stm32_link_lost_status(last_control, sequence, now_ms)` dựng một `DroneSystemStatus` FAILSAFE với PWM 1000 us, giữ session/sequence control cuối, gắn cờ mất UART, encode rồi gửi Ground.
- `air_link_monitor_task(arg)` chụp trạng thái link dưới lock, tính freshness 300 ms; khi Ground online nhưng STM32 offline thì tăng sequence status giả, bật FAULT và gửi cảnh báo; ngủ 50 ms rồi lặp.
- `app_main()` phục hồi NVS nếu cần, khởi tạo LED, UART RX callback, ESP-NOW RX callback, tạo monitor task, đặt LED chờ và báo bridge sẵn sàng.

## `main/uart_transport.c`

- `uart_packet_received(packet, length, context)` adapter từ callback `packet_stream` sang callback hai tham số của Air.
- `uart_rx_task(arg)` đọc UART theo chunk 128 byte với timeout 20 ms và feed từng chunk vào bộ tách frame COBS.
- `uart_transport_start(callback)` kiểm tra callback, lấy port/pin/baud từ Kconfig, khởi tạo stream và mutex TX, cài driver UART 8-N-1, gán pin, rồi tạo RX task. Trả ngay lỗi của bước đầu tiên thất bại.
- `uart_transport_send_packet(packet, length)` kiểm tra kích thước, COBS-frame packet với delimiter `0x00`, lấy mutex tối đa 20 ms, ghi toàn frame rồi trả thành công chỉ khi số byte ghi đủ.

## Điểm an toàn cần giữ

- Không bỏ bước `DroneProtocol_DecodeControl` trước khi forward xuống STM32.
- `s_air_link` được dùng từ nhiều task/callback nên mọi đọc/ghi liên quan heartbeat phải nằm trong critical section.
- Status link-lost do Air sinh ra chỉ là thông báo cho Ground; đầu ra motor vẫn phải do watchdog STM32 đưa về an toàn.
