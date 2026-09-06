# ESP common component - function flow

## Luồng dữ liệu

```text
USB/UART bytes -> packet_stream_feed -> COBS decode -> app callback
app raw packet -> packet_stream_encode -> USB/UART bytes

Wi-Fi callback -> ESP-NOW queue -> espnow_rx_task -> app callback
app raw packet -> esp_now_send -> send callback -> statistics
```

## `src/packet_stream.c`

- `packet_stream_init(stream, callback, context)` xóa toàn bộ state, lưu callback/context; `stream == NULL` thì không làm gì.
- `packet_stream_feed(stream, bytes, length)` duyệt từng byte. Byte khác 0 được tích vào buffer; byte 0 kết thúc frame, gọi `DroneCobs_Decode`, rồi callback nếu decode ra packet khác rỗng. Frame quá dài bị reset và bỏ đến delimiter tiếp theo.
- `packet_stream_encode(packet, packet_length, frame, capacity)` kiểm tra con trỏ/kích thước, ghi delimiter đầu, COBS-encode payload, ghi delimiter cuối; trả tổng chiều dài hoặc 0 khi lỗi.

## `src/espnow_transport.c`

- `increment_stat(counter)` tăng counter trong critical section.
- `espnow_receive_callback(info, data, length)` chạy trong Wi-Fi callback: kiểm tra dữ liệu, kích thước, queue và đúng MAC peer; copy packet vào queue không-blocking; cập nhật `rx_enqueued`, `rx_rejected` hoặc `rx_queue_dropped`.
- `espnow_send_callback(info, status)` cập nhật số packet được radio xác nhận hoặc thất bại delivery.
- `espnow_rx_task(arg)` block chờ queue; mỗi packet được chuyển sang callback ứng dụng ngoài Wi-Fi callback context.
- `espnow_transport_start(callback, peer_mac, channel)` kiểm tra input; tạo event loop; bật Wi-Fi STA/RAM storage/channel; init ESP-NOW và peer không mã hóa; tạo queue, đăng ký callback, tạo RX task; đọc MAC local rồi đánh dấu started.
- `espnow_transport_send_packet(packet, length)` kiểm tra kích thước DroneProtocol, gọi `esp_now_send`, cập nhật submitted/submit-failed và trả nguyên mã lỗi.
- `espnow_transport_get_stats(stats)` sao chép snapshot counter dưới critical section; con trỏ null thì bỏ qua.
- `espnow_transport_get_link_info(info)` yêu cầu transport đã start, rồi trả MAC local/peer và channel.

## `src/status_led.c`

- `status_led_color_for(state, phase)` biến state và phase 100 ms thành RGB: boot nháy amber, waiting nháy blue, connected nháy cyan, disarmed xanh, armed đỏ liên tục, fault hai nhịp đỏ.
- `status_led_colors_equal(left, right)` tránh ghi phần cứng lại khi RGB không đổi.
- `status_led_is_on(color)` ở bản LED đơn sắc, gộp mọi màu khác đen thành mức GPIO ON.
- `status_led_task(arg)` chụp state dưới lock, reset phase khi state đổi, tính màu; chỉ cập nhật GPIO/WS2812 khi màu đổi; tăng phase và chờ notification hoặc timeout 100 ms.
- `status_led_init()` cấu hình GPIO đơn sắc hoặc tạo WS2812/RMT, đưa LED về tắt, tạo task; nếu tạo task lỗi thì rollback phần cứng; cuối cùng đánh dấu initialized.
- `status_led_set(state)` bỏ request trước init hoặc state ngoài enum; cập nhật state dưới lock và notify task chỉ khi có thay đổi.

## Cấu hình liên quan

`bridge_config.h` chứa MAC hai đầu và channel. `CMakeLists.txt` tái sử dụng trực tiếp DroneProtocol từ STM32 để hai firmware dùng cùng encoder/decoder, tránh hai bản protocol lệch nhau.
