# Attitude - function flow

Component này chuẩn hóa frame và đơn vị trước khi dữ liệu vào Mahony/PID. Quy ước body là FRD.

- `Attitude_MapSensorRawToBody(sensor_raw)` đổi frame accel/gyro sensor sang body bằng `x=-x`, `y=y`, `z=-z`.
- `Attitude_MapMagSensorRawToBody(sensor_raw)` đổi frame magnetometer riêng bằng `x=-x`, `y=-y`, `z=z`.
- `Attitude_CalibrateMagBodyCentiUt(mag_body_raw)` trừ hard-iron offset đã fit, nhân ma trận soft-iron 3x3, rồi trả vector integer centi-µT.
- `Attitude_CalibrateAccelBodyG(accel_body_g)` trừ bias sáu-mặt theo từng trục rồi nhân scale từng trục.
- `Attitude_AccelRawToBodyG(sensor_raw, lsb_per_g)` gọi mapping sensor→body, chia sensitivity nếu dương, sau đó gọi calibration accel. Sensitivity không hợp lệ tạo vector zero trước calibration.
- `Attitude_GyroRawToBodyRadS(sensor_raw, sensor_bias_raw, lsb_per_deg_s)` trừ gyro bias trong frame sensor, map sang body, chia LSB/(deg/s), rồi đổi độ/s sang rad/s; sensitivity không dương trả zero.
- `Attitude_SpecificForceToGravity(specific_force_body)` đảo dấu cả ba trục vì accelerometer đo specific force ngược hướng vector gravity mà attitude filter cần.

```text
raw accel -> map FRD -> scale g -> bias/scale calibration -> negate -> gravity -> Mahony
raw gyro  -> subtract bias -> map FRD -> deg/s -> rad/s -> PID + Mahony
raw mag   -> map FRD -> centi-uT -> hard/soft-iron calibration -> Mahony9
```
