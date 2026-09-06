# Mahony (6-axis) - function flow

## Luồng filter

```text
init config -> init quaternion từ accel -> mỗi sample gyro + accel correction
-> tích phân quaternion -> normalize -> Euler khi cần
```

- `Mahony_Init(filter, config)` validate con trỏ và gain/giới hạn; xóa state, copy config và đặt quaternion identity (`q0=1`).
- `Mahony_InitFromAccel(filter, ax, ay, az)` kiểm tra filter và norm accel; normalize gravity, suy ra roll/pitch ban đầu, tạo quaternion không yaw, xóa integral và đánh dấu initialized.
- `Mahony_Update(filter, gx, gy, gz, ax, ay, az, dt_s)` kiểm tra initialized, input hữu hạn và `dt>0`; nếu accel đủ norm thì normalize, tính sai số cross-product giữa gravity đo và gravity dự đoán, cập nhật integral có clamp và cộng PI correction vào gyro; tích phân quaternion theo gyro, normalize; norm quaternion hỏng thì bỏ initialized và trả false.
- `Mahony_GetEulerDegrees(filter, euler)` kiểm tra state/output; đổi quaternion thành roll/pitch/yaw, clamp đối số `asin` để tránh NaN, đổi rad sang degree.
- `Mahony_Clamp(value, min, max)` giới hạn scalar, dùng cho integral và đối số lượng giác.

Filter này không có magnetometer nên yaw chỉ được tích phân từ gyro và sẽ drift theo thời gian.
