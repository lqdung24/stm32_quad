# RateControl - function flow

## Luồng PID mỗi sample

```text
stick [-1000..1000] -> target rad/s
target - gyro -> P
gyro derivative -> low-pass -> -Kd*d(measurement)
error integral -> clamp + conditional anti-windup
P + I + D -> output clamp -> MotorMixer
```

- `RateControl_Init(control, config)` validate gain/limit/rate cho cả ba trục; xóa state, copy config và đánh dấu initialized.
- `RateControl_Reset(control)` xóa debug rồi gọi `clear_dynamic_state`; config và cờ initialized được giữ nguyên.
- `RateControl_SetCommand(control, roll, pitch, yaw)` clamp từng command về ±1000, normalize và nhân maximum rate tương ứng để tạo target rad/s.
- `RateControl_Update(control, measured[3], dt)` yêu cầu initialized, input hữu hạn và `dt` trong 0.5..50 ms; input lỗi sẽ xóa dynamic state. Với dữ liệu đúng, lưu measurement và gọi `update_axis` cho ba trục.
- `RateControl_GetDebug(control, debug)` trả snapshot target, measurement và output khi handle hợp lệ.
- `config_valid(config)` yêu cầu mọi gain/limit/cutoff hữu hạn, gain và integral limit không âm, output limit/max rate dương.
- `clampf(value, min, max)` giới hạn scalar.
- `clear_dynamic_state(control)` xóa integral, previous measurement, derivative filter/init và output; target command được giữ.
- `update_axis(control, axis, measurement, dt)` tính error; derivative-on-measurement để tránh setpoint kick; lọc đạo hàm bậc một nếu cutoff >0; tạo integral candidate có clamp; ngừng tích phân nếu nó đẩy sâu hơn vào output saturation nhưng vẫn cho phép tích phân kéo ra; tổng P+I+D và clamp output.
