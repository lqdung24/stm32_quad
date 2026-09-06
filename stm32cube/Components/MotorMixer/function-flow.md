# MotorMixer - function flow

- `MotorMixer_MixQuadX(collective, roll, pitch, yaw, result)` kiểm tra input hữu hạn và collective 0..1000; áp hệ số Quad-X để tạo correction cho M1..M4; nếu span correction lớn hơn 1000 thì scale đồng đều; dịch collective vào khoảng còn khả dụng để giữ correction; clamp từng motor 0..1000; trả cả command và cờ saturation (`collective_shifted`, `correction_scaled`).
- `MotorMixer_MapToPulseUs(config, command, active, pulse_us)` validate disarmed/idle/max và từng command; nếu inactive đưa cả bốn motor về disarmed pulse; nếu active map tuyến tính 0..1000 sang disarmed..maximum rồi áp idle floor riêng cho từng motor và làm tròn.

```text
collective + PID roll/pitch/yaw
        -> Quad-X signs
        -> scale correction nếu span quá lớn
        -> shift collective để giữ authority
        -> command[4] 0..1000
        -> inactive: 1000 us
           active: map PWM + per-motor idle floor
```

Thứ tự ưu tiên saturation là giữ tương quan correction trước, sau đó dịch collective; chỉ scale correction khi bản thân span không thể nằm trong toàn dải actuator.
