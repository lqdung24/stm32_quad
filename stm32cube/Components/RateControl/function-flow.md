# RateControl - function flow

## Luồng PID mỗi sample

```text
ACRO: stick [-1000..1000] -> target BODY rad/s
ANGLE: roll/pitch stick -> target ±30 degrees
       Mahony6 roll/pitch -> shortest BODY quaternion error
       2 * error.xy * 4/s -> joint limit 100 deg/s -> target BODY rad/s
       yaw stick -> configured BODY yaw rate (no heading hold)
target - gyro -> P
gyro derivative -> low-pass -> -Kd*d(measurement)
error integral -> clamp + conditional anti-windup
P + I + D -> output clamp -> MotorMixer
```

- `RateControl_Init(control, config)` validate gain/limit/rate cho cả ba trục; xóa state, copy config và đánh dấu initialized.
- `RateControl_Reset(control)` xóa debug rồi gọi `clear_dynamic_state`; config và cờ initialized được giữ nguyên.
- `RateControl_SetCommand(control, roll, pitch, yaw)` clamp từng command về ±1000, normalize và nhân maximum rate tương ứng để tạo target rad/s.
- `RateControl_SetTargetRates(control, targets)` kiểm tra cả ba trục hữu hạn trước khi commit, clamp theo maximum rate; lỗi reset toàn PID/setpoint.
- `RateControl_SetAngleCommand(control, roll, pitch, yaw, measured_roll_deg, measured_pitch_deg)` kiểm tra Euler hữu hạn trong roll ±180°, pitch ±90°; map sticks về target ±30°. Tạo quaternion current/target với cùng yaw, tính `conjugate(current) * target`, normalize và chọn dấu scalar không âm. Error x/y nhân `2 * 4/s` thành body roll/pitch rates, giới hạn độ lớn vector ở 100°/s. Yaw vẫn là lệnh tốc độ body của pilot. Tầng ngoài chỉ có P, không tích phân; tham số bench trong header cần tune trên khung thật.
- `RateControl_Update(control, measured[3], dt)` yêu cầu initialized, input hữu hạn và `dt` trong 0.5..50 ms; input lỗi sẽ xóa dynamic state. Với dữ liệu đúng, lưu measurement và gọi `update_axis` cho ba trục.
- `RateControl_GetDebug(control, debug)` trả snapshot target, measurement và output khi handle hợp lệ.
- `config_valid(config)` yêu cầu mọi gain/limit/cutoff hữu hạn, gain và integral limit không âm, output limit/max rate dương.
- `clampf(value, min, max)` giới hạn scalar.
- `clear_dynamic_state(control)` xóa integral, previous measurement, derivative filter/init và output; target command được giữ.
- `update_axis(control, axis, measurement, dt)` tính error; derivative-on-measurement để tránh setpoint kick; lọc đạo hàm bậc một nếu cutoff >0; tạo integral candidate có clamp; ngừng tích phân nếu nó đẩy sâu hơn vào output saturation nhưng vẫn cho phép tích phân kéo ra; tổng P+I+D và clamp output.
