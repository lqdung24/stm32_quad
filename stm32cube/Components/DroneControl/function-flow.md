# DroneControl - function flow

## Chuỗi điều khiển chính

```text
UART ISR -> RX ring -> COBS decode -> DroneProtocol decode
-> session/sequence/safety checks -> arm/disarm/failsafe state machine
-> pilot: throttle + PID correction -> Quad-X mixer -> MotorOutput
-> MOTOR_TEST: throttle + AUX1 selection -> MotorOutput

Mahony6 + gyro sample -> Angle outer P / Acro setpoint -> RateControl_Update -> mixer/MotorOutput
status + flight telemetry -> DroneProtocol -> COBS -> UART interrupt TX
```

## Public functions

- `DroneControl_Init(uart, motors)` xóa context, gắn `MotorOutput`, init PID và kiểm tra interface ready. Thành công vào DISARMED và publish telemetry an toàn; lỗi vào ERROR/PWM_INIT. Cuối cùng luôn arm UART receive-to-idle.
- `DroneControl_UpdateFlightSample(timestamp_ms, valid, roll_deg, pitch_deg, gyro_x, gyro_y, gyro_z, dt)` nhận attitude và gyro cùng mẫu. Cache validity/timestamp cả khi disarmed; ANGLE yêu cầu finite Euler/rates, `dt` 0.5..20 ms và tuổi mẫu <20 ms. Invalid/stale khi ARMED vào FAILSAFE ngay, kể cả throttle 0. Khi ARMED và throttle >0, tạo lại target từ pilot sticks đã lưu: ANGLE qua quaternion outer P, ACRO qua mapping rate; chạy rate PID và mixer. Không active thì reset PID. Mixer/output lỗi chuyển ERROR và disarm.
- `DroneControl_UpdateBodyRates(roll, pitch, yaw, dt)` wrapper tương thích ACRO với attitude invalid; không thể bỏ qua cổng attitude của ANGLE. ACRO giữ policy `dt` 0.5..50 ms; PID input lỗi reset correction về zero.
- `DroneControl_InvalidateImuSample()` xóa validity, sample telemetry và PID; nếu ARMED thì latch FAILSAFE/dừng output ở cả ANGLE và ACRO trước khi App chạy IMU setup/calibration có thể blocking.
- `DroneControl_GetRateControlDebug(debug)` forward snapshot PID.
- `DroneControl_GetMixerTelemetry(telemetry)` chụp dữ liệu mixer bằng sequence counter chẵn/lẻ và memory barriers để tránh reader thấy bản ghi dang dở.
- `DroneControl_PublishFlightTelemetrySample(...)` capture cùng mẫu IMU/setpoint/PID sau UpdateFlightSample, scale degree→centidegree, rates→mrad/s, PID→centi-unit và gắn sample_id. Active flight chờ commit thành công để ghép output/timestamp; PWM hoàn tất sau commit tức thời, DShot hoàn tất ở ServiceMotorOutput; DMA BUSY không ghép mẫu mới với output cũ. Test/zero/disarmed có output_sample_matched=false.
- `DroneControl_Process(now_ms)` kiểm tra ANGLE freshness/dừng output trước khi drain UART; nếu command hợp lệ cũ quá timeout thì failsafe; sau đó mỗi tick gọi `service_dshot_arming` để gửi DShot 0 mỗi 2 ms độc lập nhịp packet; cập nhật packet rate mỗi giây; gửi status và flight telemetry theo chu kỳ khi UART sẵn sàng.
- `DroneControl_ServiceMotorOutput(now_ms)` được flight task gọi sau sample IMU/PID. Với DShot ARMED, kiểm tra lại freshness/watchdog và gửi bank mới nhất mỗi 2 ms (500 Hz), kể cả throttle zero/MOTOR_TEST không có sample/packet mới. Lỡ deadline gửi một frame, không burst bù; DMA BUSY không cập nhật mốc thành công và được retry tick sau. PWM vẫn cập nhật ngay. Lỗi commit vào ERROR và stop. DISARM/e-stop/failsafe không phải chờ nhịp này.
- `DroneControl_OnUartRxEvent(uart, size)` từ ISR callback: copy byte vào log ring nếu còn chỗ và RX ring; overflow đặt INVALID_PACKET; sau đó restart receive-to-idle.
- `DroneControl_OnUartError(uart)` đánh dấu UART_LINK_LOST và restart RX đúng UART.
- `DroneControl_ReadUartRxLog(output, capacity)` drain tối đa capacity byte từ log ring cho USB diagnostics.

## Decode và state machine

- `start_uart_receive()` gọi HAL receive-to-idle interrupt vào chunk buffer nếu có UART.
- `process_uart_bytes(now_ms)` drain RX ring; tích byte đến delimiter 0; COBS-decode và dispatch raw packet; frame lỗi/quá dài đặt INVALID_PACKET và reset accumulator.
- `process_raw_packet(packet, length, now_ms)` decode control; CRC lỗi đặt cờ CRC, lỗi khác đặt INVALID_PACKET; packet đúng vào state machine.
- `process_control_command(command, now_ms)` kiểm tra aux và không cho đổi motor-selection khi armed với throttle khác 0; session mới bắt buộc disarm cycle; từ chối sequence cũ/duplicate; cập nhật heartbeat và lưu pilot sticks riêng khỏi PID reset. E-stop vào failsafe. ARM clear luôn disarm, và zero-throttle mở khóa disarm cycle. Đổi ANGLE↔ACRO khi vẫn ARM vào failsafe; đổi khi DISARM reset PID. ARM chỉ được nhận từ DISARMED với throttle=0 và attitude mới/hợp lệ nếu ANGLE. Với DShot, `MotorOutput_Start` gửi frame 0 đầu tiên, bật `dshot_arming` nhưng state vẫn DISARMED; packet ARM tiếp theo phải giữ throttle=0. PWM vào ARMED ngay. Kiểm tra lại freshness trước clamp/apply throttle.
- `service_dshot_arming(now_ms)` phát zero bank mỗi 2 ms qua `MotorOutput_SetAllThrottle/Update` từ flight task; `MOTOR_BUSY` chờ tick sau. Cần đủ 1000 ms và 100 frame thành công, mỗi khoảng cách frame ≤20 ms. Gap lớn bắt đầu lại cửa sổ liên tục; tổng 2000 ms chưa hoàn tất latch failsafe, lỗi output vào ERROR và disarm. Chỉ sau đủ điều kiện mới đổi state ARMED, lúc này packet kế tiếp mới được phép áp ga dương.
- `attitude_is_fresh(now_ms)` yêu cầu cờ valid và hiệu timestamp unsigned <20 ms (hỗ trợ tick wrap).
- `enter_failsafe(reason)` disarm, bắt buộc disarm cycle mới, chuyển FAILSAFE trừ khi đã ERROR và gắn cả reason lẫn FAILSAFE_ACTIVE.
- `disarm_output()` gọi `MotorOutput_Stop`, hủy `dshot_arming`, reset PID, zero applied throttle và publish telemetry inactive.

## Actuator và telemetry

- `apply_direct_motor_output(commit)` trong MOTOR_TEST stage cùng mức trực tiếp cho 4 motor hoặc chỉ AUX1 được chọn; chỉ commit/publish telemetry PID=0 khi `commit=true` và driver thành công.
- `apply_throttle(requested)` clamp theo test-throttle maximum, set/clear THROTTLE_CLAMPED, lưu applied value, reset PID ngay nếu throttle=0 và remix/stage với correction PID gần nhất; DShot không commit trong handler UART hay sample, mà dùng nhịp output 2 ms độc lập.
- `throttle_to_mixer_command(throttle)` map 0 về 0; 1..max-test sang collective 225..800 trên thang mixer 0..1000, tương đương hành vi PWM 1225..1800 us cũ.
- `apply_mixed_output(roll, pitch, yaw, commit)` mix Quad-X, chuẩn hóa bốn command sang 0..1, stage toàn bank qua `MotorOutput_SetAllThrottle`; PWM commit ngay, DShot chỉ commit bằng `MotorOutput_Update` khi `commit=true` tại bước output định kỳ; `MOTOR_BUSY` giữ bank mới cho lần kế tiếp mà không publish snapshot nửa cũ/nửa mới, còn commit thành công mới publish.
- `get_legacy_motor_output(index)` trả compare thật với PWM; với DShot nó map normalized output về 1000..2000 để giữ tương thích wire field `pwm_pulse_us` cho tới khi protocol có output-type riêng.
- `publish_mixer_telemetry(...)` ghi PID, motor command/PWM, collective và saturation flags trong sequence write transaction.
- `publish_disarmed_telemetry()` tạo mixer result zero, đọc pulse cache hiện tại và publish inactive.
- `send_status(now_ms)` dựng status từ state/error/control/PWM, encode và COBS-send; chỉ tăng sequence khi submit thành công.
- `send_flight_telemetry()` chọn snapshot hoàn chỉnh mới nhất, bỏ mẫu >50 ms hoặc khác state/session, gán sequence rồi encode/send type 8; chỉ tăng sequence khi gửi thành công và consume snapshot để không lặp mẫu. Stop/zero-throttle/IMU invalidation xóa cả pending và completed snapshot.
- `try_send_packet(raw, length)` kiểm tra bounds và UART READY, COBS-encode, thêm delimiter rồi dùng interrupt TX.
- `scale_to_i16(value, scale, output)` kiểm tra hữu hạn/range, scale và làm tròn đối xứng sang int16.

## Invariant an toàn

- Startup, session mới, e-stop và failsafe đều cần một lệnh DISARM rõ ràng với throttle 0 trước lần ARM kế tiếp.
- Command watchdog độc lập với estimator; mất gyro không được làm mất watchdog UART, và mất UART luôn disarm.
- Mọi lỗi actuator/mixer (trừ DMA busy hữu hạn) chuyển hệ thống sang ERROR và disarm ngay.

## Chế độ test motor

Cờ `MOTOR_TEST` loại trừ ANGLE/ACRO. Khi cờ này bật, `DroneControl_UpdateFlightSample` chỉ lưu trạng thái attitude rồi reset/bỏ qua PID; packet throttle stage lệnh output; DShot gửi lặp bank mới nhất ở 500 Hz qua `DroneControl_ServiceMotorOutput`, còn PWM cập nhật ngay. Có thể ARM dù chưa có mẫu IMU hợp lệ. Đổi cờ mode khi còn ARM gây FAILSAFE; đổi AUX1 với throttle dương cũng gây FAILSAFE. DISARM, E-STOP, DShot zero pre-arm và watchdog UART giữ nguyên.
