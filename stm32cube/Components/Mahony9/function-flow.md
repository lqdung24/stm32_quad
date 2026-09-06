# Mahony9 (9-axis) - function flow

## Luồng filter

Mahony9 dùng gyro để propagate quaternion, accel để khóa roll/pitch và magnetometer để khóa heading. Khi mag không hợp lệ, caller có thể đi nhánh IMU-only mà vẫn giữ rate/attitude hoạt động.

- `Mahony9_Init(filter, config)` validate gain, integral limit và các ngưỡng norm; xóa state, copy config, đặt quaternion identity.
- `Mahony9_InitFromAccelMag(filter, accel, mag)` kiểm tra norm hai vector; normalize; suy ra roll/pitch từ gravity, tilt-compensate từ trường để tính yaw; tạo và normalize quaternion, reset integral/flags, đánh dấu initialized.
- `Mahony9_Update(filter, gyro, accel, mag, dt)` gọi core update với `allow_magnetometer=true`.
- `Mahony9_UpdateImu(filter, gyro, accel, dt)` gọi cùng core nhưng cấm dùng mag; hữu ích khi sample mag chưa sẵn sàng.
- `Mahony9_GetEulerDegrees(filter, euler)` chuyển quaternion sang roll/pitch/yaw độ, clamp giá trị cho `asin`, chỉ chạy khi initialized.
- `Mahony9_UpdateInternal(...)` validate input/dt; kiểm tra norm accel và mag theo config; cộng sai số gravity và magnetic reference vào vector correction; cập nhật/clamp integral PI; hiệu chỉnh gyro; tích phân quaternion và normalize. Nếu quaternion suy biến, xóa initialized. Cờ `magnetometer_used` phản ánh sample hiện tại.
- `Mahony9_NormAccepted(norm, minimum, maximum)` từ chối vector gần zero, dưới/equal min khi min bật, hoặc trên/equal max khi max hợp lệ.
- `Mahony9_Clamp(value, min, max)` giới hạn scalar cho integral và tính Euler.

```text
gyro -------------------------------> quaternion integration
accel -> norm gate -> gravity error -----^
mag   -> norm gate -> heading error ------^
                         PI correction
```
