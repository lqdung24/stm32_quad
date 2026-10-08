# Mahony (6-axis) - function flow

## Luồng filter

```text
init config -> init quaternion từ gravity BODY FRD trong norm gate
-> mỗi sample gyro rad/s + gravity g -> PI correction
-> tích phân quaternion -> normalize -> Euler độ cho Angle + telemetry
```

- `Mahony_Init(filter, config)` reset quaternion về identity (`q0=1`), integral và initialized. Gain/giới hạn phải hữu hạn, không âm; accel maximum phải lớn hơn minimum và epsilon. Config lỗi hoặc NULL để filter ở trạng thái không thể init.
- `Mahony_InitFromAccel(filter, ax, ay, az)` xóa initialized trước khi kiểm tra config, gravity hữu hạn và norm trong khoảng mở `(accel_min_norm, accel_max_norm)`. Normalize gravity, suy ra roll/pitch, tạo quaternion yaw=0, reset integral rồi đánh dấu initialized.
- `Mahony_Update(filter, gx, gy, gz, ax, ay, az, dt_s)` yêu cầu state/config hợp lệ, input hữu hạn và dt trong 0.5..20 ms. Lỗi sẽ xóa initialized và trả false; cần init lại trước khi sử dụng. Accel norm ngoài gate (kể cả vector zero) chỉ bỏ correction và vẫn tích phân gyro. Trong gate, normalize gravity, tính sai số cross-product với gravity dự đoán, cập nhật integral có clamp và cộng PI correction vào gyro. Tích phân quaternion rồi kiểm tra norm hữu hạn/không suy biến và normalize.
- `Mahony_GetEulerDegrees(filter, euler)` yêu cầu state hợp lệ, tính Euler vào biến tạm, clamp đối số `asin`, chỉ ghi output khi cả ba góc hữu hạn.
- `Mahony_ConfigValid(config)` kiểm tra gain/giới hạn/norm gate.
- `Mahony_StateValid(filter)` kiểm tra initialized, config, integral hữu hạn và quaternion unit với sai số norm bình phương tối đa 0.001.
- `Mahony_Clamp(value, min, max)` giới hạn integral và đối số lượng giác.

App dùng filter này trước cascade Angle/rate ở mỗi sample IMU mới. Gravity là accelerometer specific force đã hiệu chỉnh và đảo dấu; frame FRD, gyro rad/s và Euler độ. Mahony6 không có magnetometer nên yaw chỉ được tích phân từ gyro và sẽ drift; Angle giữ yaw-rate, không giữ heading.

Host regressions: `make -C stm32cube/Tests/mahony test` kiểm tra init/sign roll-pitch, gyro units và variable dt, gravity correction, norm gate, finite config/input/state/Euler, stall rejection và recovery qua init mới.
