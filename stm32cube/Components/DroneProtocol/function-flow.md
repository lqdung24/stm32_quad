# DroneProtocol - function flow

## Pipeline packet

```text
struct -> kiểm tra range/flags -> encode header + payload -> CRC16 -> raw packet
raw packet -> kiểm tra size/magic/version/type/length/reserved/CRC -> decode payload -> struct
raw packet <-> COBS frame + delimiter 0x00 (ở tầng transport)
```

## `Src/dp_bytes.c`

- `DroneProtocol_ReadU16Le(data)` ghép hai byte little-endian thành `uint16_t`.
- `DroneProtocol_ReadI16Le(data)` gọi cách ghép 16-bit tương tự rồi giữ nguyên bit pattern dưới kiểu `int16_t`.
- `DroneProtocol_ReadU32Le(data)` ghép bốn byte little-endian thành `uint32_t`.
- `DroneProtocol_WriteU16Le(data, value)` tách `uint16_t` thành hai byte little-endian.
- `DroneProtocol_WriteI16Le(data, value)` giữ bit pattern signed qua cast rồi gọi writer 16-bit.
- `DroneProtocol_WriteU32Le(data, value)` tách `uint32_t` thành bốn byte little-endian.

## `Src/dp_cobs.c`

- `DroneCobs_Encode(input, input_length, output, capacity)` tạo các block COBS, thay byte 0 bằng code-length, tách block khi code đạt `0xFF`; kiểm tra capacity tại từng bước; trả số byte encode hoặc 0.
- `DroneCobs_Decode(input, input_length, output, capacity)` đọc code, copy `code-1` byte và chèn lại zero giữa block khi cần; từ chối code 0, block vượt input hoặc output thiếu chỗ.

## `Src/dp_protocol.c`

- `encode_header(header, output)` ghi magic, version, type, sequence, session, flags, payload length, reserved=0 và sender timestamp tại offset cố định.
- `decode_header(packet, length, required_type, required_payload_length, header)` kiểm tra null, exact size, magic, version, type, payload length, reserved và CRC trước khi populate header.
- `append_crc(packet, packet_length)` tính CRC trên mọi byte trừ hai byte cuối rồi ghi CRC little-endian.
- `DroneProtocol_Crc16CcittFalse(data, length)` chạy CRC-16/CCITT-FALSE với init `0xFFFF`, polynomial `0x1021`; null chỉ hợp lệ khi length=0.
- `DroneProtocol_EncodeControl(command, output)` validate flags, throttle/axes/aux; chuẩn hóa header type/length, ghi payload control và CRC.
- `DroneProtocol_DecodeControl(packet, length, command)` validate header/CRC, flags, đọc payload rồi kiểm tra range một lần nữa.
- `DroneProtocol_EncodeStatus(status, output)` validate throttle/state, ép status flags=0, ghi sequence ACK, PWM, state, error và UART rate, đặt reserved rồi CRC.
- `DroneProtocol_DecodeStatus(packet, length, status)` validate header/CRC/reserved, đọc payload và range-check throttle/state.
- `DroneProtocol_EncodeFlightTelemetry(telemetry, output)` validate state/flags/PWM; dựng flags từ state, actuator và attitude validity; ghi 3 trục attitude/gyro/setpoint/PID, 4 PWM và CRC.
- `DroneProtocol_DecodeFlightTelemetry(packet, length, telemetry)` validate header/flags/state; giải scale-field dạng integer và kiểm tra từng PWM 1000..2000 us.
- `DroneProtocol_IsSequenceNewer(candidate, reference)` so sánh sequence 16-bit có wrap-around: khác nhau và khoảng tiến nhỏ hơn nửa không gian `0x8000`.

Mọi decoder trả mã lỗi phân biệt null, size, magic, version, type, length, reserved, CRC, flags và range; caller không nên gộp bỏ thông tin này nếu cần telemetry lỗi chi tiết.
