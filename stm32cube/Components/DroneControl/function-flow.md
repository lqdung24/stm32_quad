# DroneControl - function flow

## Chuỗi điều khiển chính

```text
UART ISR -> RX ring -> COBS decode -> DroneProtocol decode
-> session/sequence/safety checks -> arm/disarm/failsafe state machine
-> throttle + latest PID correction -> Quad-X mixer -> MotorOutput

gyro sample -> RateControl_Update -> mixer/PWM
status + flight telemetry -> DroneProtocol -> COBS -> UART interrupt TX
```

## Public functions

- `DroneControl_Init(uart, motors)` xóa context, gắn `MotorOutput`, init PID và kiểm tra interface ready. Thành công vào DISARMED và publish telemetry an toàn; lỗi vào ERROR/PWM_INIT. Cuối cùng luôn arm UART receive-to-idle.
- `DroneControl_UpdateBodyRates(roll, pitch, yaw, dt)` chỉ chạy PID khi initialized, ARMED và throttle >0; ngược lại reset PID. PID input lỗi đưa correction về zero. Mixer/PWM lỗi chuyển ERROR, disarm. Threshold-test build cố ý tắt PID.
- `DroneControl_GetRateControlDebug(debug)` forward snapshot PID.
- `DroneControl_GetMixerTelemetry(telemetry)` chụp dữ liệu mixer bằng sequence counter chẵn/lẻ và memory barriers để tránh reader thấy bản ghi dang dở.
- `DroneControl_PublishFlightTelemetrySample(...)` scale attitude degree→centidegree, gyro/setpoint→mrad/s và PID→centi-unit; validate int16/PWM rồi cache sample để task gửi định kỳ.
- `DroneControl_Process(now_ms)` drain UART; nếu command hợp lệ cũ quá timeout thì failsafe; cập nhật packet rate mỗi giây; gửi status và flight telemetry theo chu kỳ khi UART sẵn sàng.
- `DroneControl_OnUartRxEvent(uart, size)` từ ISR callback: copy byte vào log ring nếu còn chỗ và RX ring; overflow đặt INVALID_PACKET; sau đó restart receive-to-idle.
- `DroneControl_OnUartError(uart)` đánh dấu UART_LINK_LOST và restart RX đúng UART.
- `DroneControl_ReadUartRxLog(output, capacity)` drain tối đa capacity byte từ log ring cho USB diagnostics.

## Decode và state machine

- `start_uart_receive()` gọi HAL receive-to-idle interrupt vào chunk buffer nếu có UART.
- `process_uart_bytes(now_ms)` drain RX ring; tích byte đến delimiter 0; COBS-decode và dispatch raw packet; frame lỗi/quá dài đặt INVALID_PACKET và reset accumulator.
- `process_raw_packet(packet, length, now_ms)` decode control; CRC lỗi đặt cờ CRC, lỗi khác đặt INVALID_PACKET; packet đúng vào state machine.
- `process_control_command(command, now_ms)` kiểm tra aux và không cho đổi motor-selection khi armed với throttle khác 0; session mới bắt buộc disarm cycle; từ chối sequence cũ/duplicate; cập nhật heartbeat/setpoint. E-stop vào failsafe. ARM clear luôn disarm, và zero-throttle mở khóa disarm cycle. ARM chỉ được nhận từ DISARMED với throttle=0; sau đó throttle được clamp/apply.
- `enter_failsafe(reason)` disarm, bắt buộc disarm cycle mới, chuyển FAILSAFE trừ khi đã ERROR và gắn cả reason lẫn FAILSAFE_ACTIVE.
- `disarm_output()` gọi `MotorOutput_Stop`, reset PID, zero applied throttle và publish telemetry inactive.

## Actuator và telemetry

- `apply_throttle(requested)` clamp theo test-throttle maximum, set/clear THROTTLE_CLAMPED, lưu applied value và remix với correction PID gần nhất để không tạo nhịp output đều giữa hai gyro sample.
- `throttle_to_mixer_command(throttle)` map 0 về 0; 1..max-test sang collective 225..800 trên thang mixer 0..1000, tương đương hành vi PWM 1225..1800 us cũ.
- `apply_mixed_output(roll, pitch, yaw)` mix Quad-X, chuẩn hóa bốn command sang 0..1, stage toàn bank qua `MotorOutput_SetAllThrottle`, commit bằng `MotorOutput_Update`; `MOTOR_BUSY` giữ bank mới cho lần kế tiếp mà không publish snapshot nửa cũ/nửa mới, còn commit thành công mới publish.
- `get_legacy_motor_output(index)` trả compare thật với PWM; với DShot nó map normalized output về 1000..2000 để giữ tương thích wire field `pwm_pulse_us` cho tới khi protocol có output-type riêng.
- `publish_mixer_telemetry(...)` ghi PID, motor command/PWM, collective và saturation flags trong sequence write transaction.
- `publish_disarmed_telemetry()` tạo mixer result zero, đọc pulse cache hiện tại và publish inactive.
- `send_status(now_ms)` dựng status từ state/error/control/PWM, encode và COBS-send; chỉ tăng sequence khi submit thành công.
- `send_flight_telemetry()` lấy sample cache, gán sequence, encode/send và chỉ tăng sequence khi thành công.
- `try_send_packet(raw, length)` kiểm tra bounds và UART READY, COBS-encode, thêm delimiter rồi dùng interrupt TX.
- `scale_to_i16(value, scale, output)` kiểm tra hữu hạn/range, scale và làm tròn đối xứng sang int16.

## Invariant an toàn

- Startup, session mới, e-stop và failsafe đều cần một lệnh DISARM rõ ràng với throttle 0 trước lần ARM kế tiếp.
- Command watchdog độc lập với estimator; mất gyro không được làm mất watchdog UART, và mất UART luôn disarm.
- Mọi lỗi actuator/mixer (trừ DMA busy hữu hạn) chuyển hệ thống sang ERROR và disarm ngay.
