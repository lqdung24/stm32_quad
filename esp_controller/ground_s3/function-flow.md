# Ground ESP32-S3 - function flow

## Luồng tổng thể

```text
Web Serial --USB/COBS--> on_usb_packet --ESP-NOW--> Air
Air --ESP-NOW--> on_espnow_packet --USB/COBS--> browser
```

Hai task nền bổ sung: `ground_control_keepalive_task` phát lệnh DISARM mới mỗi 20 ms khi browser im quá 300 ms; `ground_link_log_task` tổng hợp tình trạng USB, ESP-NOW và STM32 mỗi giây.

## `main/main.c`

- `monotonic_ms()` đổi tick FreeRTOS thành mili-giây cho heartbeat/timeout.
- `decode_control(...)` xác thực packet control bằng DroneProtocol.
- `log_espnow_link_config()` đọc thông tin link, format một dòng MAC/channel rồi gửi cả ESP log lẫn USB debug.
- `on_usb_packet(packet, length)` bỏ packet không phải control hợp lệ; packet đúng cập nhật session, sequence, counters và timestamp dưới lock, sau đó forward nguyên packet qua ESP-NOW.
- `ground_control_keepalive_task(arg)` mỗi 20 ms kiểm tra control Web Serial còn mới không. Nếu quá 300 ms, nó dựng lệnh zero-throttle/disarm, giữ session hiện tại nhưng tăng sequence để STM32 không coi là duplicate, encode và gửi qua radio.
- `on_espnow_packet(packet, length)` ưu tiên decode status, nếu không thì telemetry. Status cập nhật heartbeat, ACK và `last_status`; telemetry tăng counter; packet khác tăng `invalid_packets`. Cuối cùng mọi packet vẫn được đưa lên USB cho browser.
- `link_text(online)` đổi boolean thành chuỗi `UP`/`DOWN` cho log.
- `ground_link_log_task(arg)` mỗi giây chụp state và transport stats; suy ra USB online từ control mới, ESP-NOW online từ ACK đúng session/độ trễ sequence, STM32 online từ status mới và không có cờ mất UART; tính tốc độ theo delta counter rồi log qua ESP và USB.
- `app_main()` khởi tạo/phục hồi NVS, LED, ESP-NOW và USB; tạo hai task keepalive/log, đặt LED chờ client và báo sẵn sàng.

## `main/usb_transport.c`

- `usb_packet_received(...)` adapter callback từ `packet_stream` sang callback của Ground.
- `usb_rx_task(arg)` đọc USB Serial/JTAG theo chunk 128 byte, feed vào COBS stream và lặp vô hạn.
- `usb_transport_start(callback)` cài USB driver, tạo mutex TX, khởi tạo stream và RX task; dừng ở lỗi đầu tiên.
- `usb_transport_send_packet(packet, length)` COBS-encode, khóa TX tối đa 20 ms, gửi frame trong tối đa 20 ms và chỉ trả `ESP_OK` nếu ghi đủ.
- `usb_transport_send_debug_line(line)` giới hạn dòng 255 ký tự, bọc text bằng delimiter `0x00` ở hai đầu cùng CRLF để protocol reader bỏ qua an toàn, rồi gửi dưới cùng mutex TX.

## Logic xác định link

- USB: browser có control hợp lệ trong 300 ms gần nhất.
- ESP-NOW: có status ACK cùng session, không chậm quá 16 packet và ACK mới dưới 300 ms.
- STM32: ESP-NOW đang online, status mới dưới 300 ms và không có `DRONE_ERROR_UART_LINK_LOST`.
