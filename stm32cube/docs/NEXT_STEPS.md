# Next Steps

> Ghi chú làm việc ngắn hạn: trạng thái hiện tại, việc cần làm tiếp và các lưu
> ý quan trọng. Chuyển quyết định lâu dài sang `DECISIONS.md`, lỗi đã xác nhận
> sang `KNOWN_ISSUES.md`, và lịch sử thực hiện sang `DEV_LOG.md`.

Cập nhật lần cuối: 2026-08-07

## Trạng thái hiện tại

| Hạng mục                | Trạng thái                | Ghi chú                                               |
| ------------------------- | --------------------------- | ------------------------------------------------------ |
| IMU ICM20948              | Đã tích hợp             | Đã có chuyển đổi trục BODY FRD và hiệu chuẩn |
| Mahony attitude           | Đã tích hợp             | Cần tiếp tục kiểm tra timing trên phần cứng     |
| ESP32–STM32 control link | Đã tích hợp             | Có CRC, sequence, session và timeout failsafe        |
| FreeRTOS                  | Đã tích hợp             | CMSIS-RTOS v2; flight, telemetry và housekeeping task |
| Rate PID roll/pitch/yaw   | Đã triển khai phần mềm | Chưa tune trên airframe                              |
| Quad-X mixer              | Đã triển khai phần mềm   | Có unit test dấu mixer và desaturation              |
| Motor output              | Bench-test only             | Có idle riêng và PID correction; chưa test rig       |
| Flight readiness          | Chưa sẵn sàng bay        | Chỉ thử khi tháo cánh hoặc dùng test rig         |

Chiều quay dưới đây được xác định khi nhìn từ trên xuống drone. Idle code cao
hơn ngưỡng bắt đầu quay đo khi tháo cánh `20 us`.

| Motor | Vị trí | Output | Chiều | Bắt đầu quay | Idle code | Dấu mixer |
|---|---|---|---|---:|---:|---|
| M1 | front-left | `TIM3_CH1/PA6` | CW | `1200 us` | `1220 us` | `+roll +pitch -yaw` |
| M2 | rear-left | `TIM3_CH2/PA7` | CCW | `1205 us` | `1225 us` | `+roll -pitch +yaw` |
| M3 | front-right | `TIM3_CH3/PB0` | CCW | `1190 us` | `1210 us` | `-roll +pitch +yaw` |
| M4 | rear-right | `TIM3_CH4/PB1` | CW | `1205 us` | `1225 us` | `-roll -pitch -yaw` |

## Việc cần làm tiếp

### P0 — Xác nhận phần cứng và mapping

- [X] Ghi rõ vị trí vật lý của Motor 1–4 theo bảng trên.
- [X] Xác nhận GPIO/timer channel tương ứng với từng motor thực tế.
- [X] Xác nhận chiều quay CW/CCW từng motor khi nhìn từ trên xuống.
- [X] Đo ngưỡng bắt đầu quay và đặt idle cao hơn `20 us` cho từng motor.
- [ ] Xác nhận đúng loại/chiều cánh quạt cho từng vị trí motor.
- [X] Kiểm tra dấu gyro: xoay dương roll, pitch, yaw và đối chiếu log BODY FRD.
- [ ] Đo chu kỳ IMU/rate PID thực tế và jitter trên phần cứng.
- [ ] Đo stack high-water mark của cả ba application task trên phần cứng.
- [ ] Thêm xử lý an toàn cho stack-overflow và malloc-failure hook trong CubeMX.
- [ ] Xác nhận ESC hỗ trợ tần số PWM nào trước khi thay đổi PWM 50 Hz.

### P1 — Giai đoạn 2: Quad-X mixer

- [X] Lập bảng mixer từ vị trí và chiều quay motor đã xác nhận.
- [X] Thêm unit test cho lệnh thuần roll, pitch, yaw và mapping PWM/idle.
- [ ] Kiểm tra dấu phản hồi: PID phải tạo mô-men chống lại chuyển động đo được.
- [X] Thiết kế saturation/desaturation và thứ tự ưu tiên
  roll/pitch/yaw/throttle.
- [X] Giữ toàn bộ mixer trong `Components/`; không sửa file CubeMX sinh tự động.
- [ ] Test không cánh: từng motor, từng trục, disarm và emergency stop.

### P2 — Sau khi bench-test mixer

- [ ] Chuyển rate loop sang trigger xác định bằng timer hoặc IMU data-ready.
- [ ] Xử lý IMU failure/reinitialization mà không block watchdog khi armed.
- [ ] Thêm telemetry cho target rate, measured rate và PID output.
- [ ] Tune rate PID trên test rig có dây giữ.
- [ ] Triển khai outer angle loop sau khi inner rate loop ổn định.

## Lưu ý bắt buộc

- Luôn tháo cánh khi kiểm tra motor order, mixer hoặc dấu PID.
- Các gain hiện tại chỉ là giá trị khởi đầu để bench test, không phải gain bay.
- Không cho phép integral tồn tại qua disarm, failsafe hoặc sensor invalid.
- Không sửa file do CubeMX sinh tự động, kể cả vùng `USER CODE`.
- Không kết luận “flight ready” chỉ vì build hoặc unit test thành công.
- Ghi rõ đơn vị tại mọi API: `rad/s`, `rad`, `s`, throttle chuẩn hóa hoặc `us`.

## Blocker / câu hỏi mở

- [X] Motor layout, timer channel và GPIO: đã ghi trong bảng trên.
- [X] Chiều quay motor nhìn từ trên: `CW, CCW, CCW, CW`.
- [X] Ngưỡng quay/idle: `1200/1220`, `1205/1225`, `1190/1210`,
  `1205/1225 us`.
- [ ] Loại và chiều cánh quạt tại từng motor:
- [ ] Model ESC và protocol/tần số hỗ trợ:
- [ ] Khối lượng, kích thước frame, motor, propeller và pin:
- [ ] Thiết bị/test rig dùng để tune:

## Ghi chú nhanh

Thêm ghi chú mới theo mẫu, rồi chuyển sang file chuyên biệt khi đã ổn định:

```text
[YYYY-MM-DD] [TODO|NOTE|BLOCKED|DONE]
- Nội dung:
- Lý do / bằng chứng:
- Bước tiếp theo:
```
