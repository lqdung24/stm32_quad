# MotorPWM - function flow

- `MotorPwm_Attach(motors, config)` validate timer, channel và giới hạn pulse; copy config, ghi compare disarmed cho cả bốn channel, cache pulse rồi đánh dấu attached.
- `MotorPwm_Arm(motors)` yêu cầu đã attach; trước khi set armed luôn ghi cả bốn output về disarmed để arm từ trạng thái xác định.
- `MotorPwm_Disarm(motors)` nếu attached thì ghi disarmed cho tất cả rồi xóa armed.
- `MotorPwm_SetPulseUs(motors, index, pulse)` chỉ cho phép khi attached+armed, index hợp lệ và pulse trong min..max; ghi CCR và cập nhật cache.
- `MotorPwm_SetAllPulseUs(motors, pulse[4])` validate toàn bộ mảng trước khi ghi bất kỳ channel nào, tránh trạng thái cập nhật nửa chừng; sau đó ghi/cập nhật cache cả bốn.
- `MotorPwm_GetPulseUs(motors, index)` trả pulse cache, hoặc 0 nếu handle/index không hợp lệ.
- `MotorPwm_IsAttached(motors)` kiểm tra handle và cờ attached.
- `MotorPwm_IsArmed(motors)` yêu cầu đồng thời attached và armed.
- `MotorPwm_ConfigValid(config)` kiểm tra timer, quan hệ `disarmed <= minimum < maximum`, maximum không vượt period và bốn channel thuộc TIM CH1..CH4.
- `MotorPwm_WriteCompare(motors, index, pulse)` helper ghi trực tiếp compare register channel đã map.
- `MotorPwm_WriteDisarmed(motors)` ghi disarmed pulse và cache cho mọi motor.

Invariant chính: public setter không thể phát PWM khi chưa arm; mọi disarm cập nhật phần cứng trước rồi mới cập nhật state.
