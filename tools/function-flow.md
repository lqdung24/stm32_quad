# Telemetry tools - function flow

## `telemetry_plot.py`

- `crc16_ccitt_false(data)` tính cùng CRC-16/CCITT-FALSE với firmware.
- `Telemetry.armed` trả true khi state numeric là ARMED (`2`).
- `decode_packet(packet, host_time_s)` yêu cầu đúng 50 byte; kiểm tra CRC, magic/version/type/header/flags/state/PWM; unpack 12 int16 + 4 uint16, đổi fixed-point sang degree/rad/s/PID và trả immutable `Telemetry`.
- `Receiver.__init__(url, csv_path, history_seconds)` tạo deque khoảng 60 sample/s, lock/event/counters, mở CSV và ghi header.
- `Receiver.close()` signal stop rồi đóng CSV.
- `Receiver.add_packet(packet)` timestamp và decode; packet sai tăng invalid dưới lock. Packet đúng tính inter-arrival gap, phát hiện sequence drop có wrap-around, append sample, ghi/flush một CSV row.
- `Receiver.run()` lazy-import websocket; vòng ngoài tự reconnect mỗi giây; vòng trong nhận binary frame và gọi `add_packet`; luôn đóng socket ở `finally`.
- `plot(receiver, window_seconds)` lazy-import matplotlib, tạo bốn subplot. Callback `redraw` chụp samples/counters dưới lock, lọc cửa sổ thời gian, vẽ attitude, gyro+setpoint, PID và PWM, rồi cập nhật title trạng thái/link statistics. `FuncAnimation` gọi lại mỗi 50 ms.
- `self_test()` dựng packet telemetry hợp lệ, tính CRC, decode và assert các field/scale quan trọng.
- `main()` parse CLI; chạy self-test hoặc validate window; tạo Receiver và daemon receive thread; chạy plot hoặc loop record-only; Ctrl-C/finally luôn close receiver và join thread.

## `run_telemetry_plot.sh`

Script xác định thư mục chứa chính nó rồi `exec` Python với toàn bộ argument người dùng. Dùng `exec` để signal/exit code đi thẳng tới process Python.

## Context navigation tools

- `read_context(path)` được hiện thực bởi executable `tools/read_context`: chuẩn hóa path tương đối, từ chối absolute/path traversal và file không phải Markdown, rồi in đúng một tài liệu trong `.context`.
- `search_context(keyword)` được hiện thực bởi executable `tools/search_context`: tìm fixed-string, không phân biệt hoa thường trong các file Markdown của `.context`, trả kết quả dạng `path:line`; không có kết quả không được xem là lỗi.

```text
WebSocket binary frame -> decode/validate -> Receiver deque + CSV
                                      -> matplotlib redraw snapshot
```
