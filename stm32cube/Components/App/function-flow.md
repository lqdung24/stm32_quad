# STM32 App - function flow

## Luồng runtime

### Cấu hình mặc định: điều khiển bình thường bằng DShot300

- `APP_DSHOT_TEST_ENABLE=0` và `APP_MOTOR_OUTPUT_PROTOCOL=APP_MOTOR_OUTPUT_DSHOT300` trong `Inc/app.h`: `DroneControl_Init()` nhận motor output thật; flight task xử lý UART, ARM/DISARM, watchdog, IMU, PID và mixer. Không tự chạy ga sau 15 s.
- Đặt `APP_MOTOR_OUTPUT_PROTOCOL=APP_MOTOR_OUTPUT_PWM` hoặc `APP_MOTOR_OUTPUT_DSHOT600` để chọn giao thức khác mà vẫn giữ flow điều khiển bình thường.

### Chế độ bench tùy chọn (mặc định tắt)

- Chủ động đặt `APP_DSHOT_TEST_ENABLE=1` trong `Inc/app.h` để bật bench, mặc định chọn `APP_MOTOR_OUTPUT_DSHOT300`. Đặt `APP_MOTOR_OUTPUT_PROTOCOL=APP_MOTOR_OUTPUT_DSHOT600` để dùng cùng flow với DShot600. Tháo cánh trước khi cấp nguồn: cả bốn motor tự chạy, không cần ARM.
- Sau khi `App_Init()` hoàn tất, flight task gọi `AppDshotTest_Step()` mỗi 1 ms: bắt đầu bằng frame 0, tiếp tục gửi 0 trong 15000 ms rồi gửi `0.30f` (DShot value 648) cho M1..M4, gửi frame mỗi 2 ms (500 Hz). Đây là 30% lệnh ga, không phải 30% RPM hay duty tín hiệu DShot.
- `DroneControl_Init()` nhận motor NULL; flight step bỏ qua UART control/PID/IMU để tránh ghi đè và tránh sensor retry/calibration chặn luồng DShot. Telemetry điều khiển bay không phản ánh output test; xem `app_dshot_test`, `app_motor_driver.throttle_value` và `app_motor_driver.applied_throttle` bằng debugger.
- DMA BUSY được bỏ qua đến tick sau; lỗi output được chốt vào `app_dshot_test.failed`, gọi Stop và không tự khởi động lại. Ngắt nguồn ESC để dừng test; DISARM/e-stop/watchdog từ web không điều khiển motor trong chế độ này.
- Tắt test bằng `APP_DSHOT_TEST_ENABLE=0` rồi rebuild/flash; giao thức mặc định vẫn là DShot300. Cờ cũ `APP_DSHOT600_TEST_ENABLE=0` vẫn tắt bench; `=1` chọn bench DShot600 nếu chưa chọn protocol rõ ràng. Hai cờ bench không được mâu thuẫn.
- `Inc/app_dshot_test.h` chứa sequencer header-only để không sửa source list do CubeMX sinh. Host regression nằm trong `Tests/motor_output/test_motor_output.c`.

Luồng bên dưới áp dụng khi tắt bench test:

```text
CubeMX setup -> App_Set* -> App_Init -> App_Process/AppRtos_Bootstrap
  high-priority 1 ms: App_FlightControlStep
      UART commands -> DroneControl_Process
      gyro/accel -> body conversion -> Mahony6 -> Angle/Acro -> rate PID -> stage motor/telemetry
      DroneControl_ServiceMotorOutput -> DShot latest bank every 2 ms (500 Hz)
      mag 10 ms -> calibrated diagnostics (không tham gia Angle/Mahony6)
  telemetry 5 ms: logs/timing/mixer/UART diagnostics
  housekeeping 100 ms: heartbeat LED
```

## Setup và public entry points (`app.c`)

- `App_SetSpi`, `App_SetUsbTransmit`, `App_SetActivityLed`, `App_SetMotorTimer`, `App_SetControlUart` lưu các HAL handle/callback do CubeMX tạo để component dùng sau.
- `App_OnUsbReceive(data, length)` cố ý bỏ dữ liệu: USB CDC chỉ dành diagnostics, control an toàn chỉ nhận qua USART1 DroneProtocol.
- `App_OnUsbTransmitComplete()` xóa cờ USB busy để log kế tiếp được gửi.
- `App_Init()` bật cycle counter; tạo motor output đã chọn ở trạng thái stop; init DroneControl; init/calibrate ICM20948, mag diagnostics và Mahony6 attitude; đặt mọi timestamp/stat; toggle LED khởi động.
- `App_StartMotorOutput(motors)` lấy đúng timer input clock theo APB/TIMPRE, init một `PwmTimer`, config/start bốn `PwmChannel`, rồi bind `MotorPwm`, `MotorDshot300` hoặc `MotorDshot` theo `APP_MOTOR_OUTPUT_PROTOCOL`; lỗi giữa chừng stop mọi channel đã start.
- `App_Process()` chuyển quyền sang `AppRtos_Bootstrap`.
- `App_FlightControlStep(now_ms)` xử lý UART/failsafe trước; trước retry IMU mỗi giây khi lỗi, gọi `DroneControl_InvalidateImuSample()` để dừng motor đã ARM trước setup/calibration blocking; poll attitude/gyro mỗi 1 ms; gọi `DroneControl_ServiceMotorOutput(HAL_GetTick())` sau rate loop để gửi DShot mỗi 2 ms kể cả không có sample/packet mới; sau đó mới đọc mag mỗi 10 ms; tùy compile flag phát log IMU 20 ms.
- `App_TelemetryStep(now_ms)` theo compile flag phát timing/mixer log theo chu kỳ, log DShot mỗi 500 ms nếu USB rảnh (bitrate, fault, DMA error, raw value, frame count, timer clock), rồi drain UART RX debug log.
- `App_HousekeepingStep(now_ms)` toggle activity LED mỗi giây.
- `App_GetTimingStats(stats)` dùng sequence counter chẵn/lẻ và memory barrier để chụp snapshot không rách; đổi cycle sang µs và tính min/mean/max cùng rate milli-Hz.
- `HAL_UARTEx_RxEventCallback(...)` forward ISR callback vào `DroneControl_OnUartRxEvent`.
- `HAL_UART_ErrorCallback(...)` forward lỗi UART vào `DroneControl_OnUartError`.

## Sensor, estimator và diagnostics (`app.c`)

- `App_FlushControlUartLog()` lấy byte RX từ ring DroneControl, format hex và gửi USB nếu endpoint rảnh.
- `App_ReportTiming()` lấy timing snapshot, tính jitter peak-to-peak, format một dòng và đánh dấu USB busy khi submit thành công.
- `App_ReportMixer()` lấy snapshot mixer, scale số float để log integer, format throttle/PID/command/PWM/saturation rồi gửi USB.
- `App_FloatToTenths(value)` đổi float sang integer phần mười có làm tròn và saturate phù hợp cho log.
- `App_TryInitICM20948()` init SPI IMU, verify/read identity, cấu hình range/sample rate/DLPF, init Mahony6 và AK09916; reset mốc sample để lần đọc đầu dùng dt danh định; lưu status để retry thay vì chạy tiếp với state giả hợp lệ.
- `App_CalibrateGyro()` khi IMU OK, bỏ mẫu đầu rồi cộng nhiều mẫu gyro đứng yên để tạo bias; lỗi đọc làm calibration thất bại an toàn.
- `App_UpdateAttitude()` kiểm tra data-ready, đọc raw và đo thời gian; chuyển accel/gyro sang body/calibrated, đảo specific force thành gravity và tính `dt` từ DWT. Mahony6 init bằng gravity trong khoảng 0.8..1.2 g, không cần mag; mỗi sample sau update gyro + accel correction rồi lấy Euler độ. Sample có `dt` ngoài 0.5..20 ms hoặc filter lỗi được đánh dấu attitude invalid trước khi được phép init lại ở sample sau. Gọi `DroneControl_UpdateFlightSample()` đúng một lần cho mỗi raw sample mới: timestamp, validity, roll/pitch độ và gyro FRD rad/s. Angle nhận attitude cùng sample trước rate PID; Acro chỉ phụ thuộc gyro. Telemetry dùng cùng validity/Euler/gyro và PID, gắn ID/time mẫu; DShot chỉ hoàn tất snapshot sau commit thành công ở output step; lỗi raw read không tạo sample giả mà được watchdog/freshness xử lý. Cuối cùng ghi timing pipeline và PID.
- `App_UpdateMagnetometer()` yêu cầu IMU/mag init OK; đọc shadow AK09916, từ chối vector zero, map frame, đổi centi-µT, hiệu chỉnh hard/soft iron và đặt validity.
- `App_ReportICM20948()` format accel, gyro, nhiệt độ, mag và Euler; nếu shadow mag liên tục zero thì định kỳ gọi debug sâu.
- `App_ReportMagDebug()` đọc register master/slave/shadow và SLV4 probe rồi format kết quả chẩn đoán.
- `App_IcmLog(text, length)` compile-time gate cho log IMU.
- `App_UsbSend(text, length)` kiểm tra callback/input, clamp length theo buffer và submit USB.
- `App_GyroRawToMdps(raw)`, `App_TempRawToCentiC(raw)`, `App_MagRawToCentiUt(raw)` đổi raw sang đơn vị integer phục vụ log/calibration.
- `App_EnableCycleCounter()` mở DWT CYCCNT và lưu cờ hardware hỗ trợ.
- `App_CyclesToUs(cycles)` đổi cycle sang µs theo `SystemCoreClock`, trả 0 nếu clock chưa có.
- `App_ResetTimingStats()` dùng sequence+barrier, xóa counters/totals, đặt min về `UINT32_MAX` và reset mốc sample/PID.
- `App_RecordSampleTiming(...)` cập nhật period/read/pipeline min-max-total; chỉ thống kê PID khi update thành công và reset mốc PID qua khoảng disarmed/failsafe; bao toàn bộ write bằng sequence chẵn/lẻ.

## RTOS (`app_rtos.c`)

- `AppRtos_Bootstrap()` chỉ chạy khi kernel active; tạo telemetry task rồi flight task; nếu thiếu tài nguyên thì terminate sạch, giữ motor disarmed, blink và retry sau 1 s; khi thành công, default task trở thành housekeeping loop.
- `flight_task(arg)` gọi flight step theo absolute period 1 ms ở priority cao.
- `telemetry_task(arg)` gọi telemetry step theo absolute period 5 ms ở priority below-normal.
- `milliseconds_to_ticks(ms)` đổi period với phép làm tròn lên và tối thiểu một tick.
- `delay_until_next_period(next_tick, period)` dùng `osDelayUntil`; nếu miss deadline thì resync về tick hiện tại và delay một tick để không starvation task thấp hơn.
- `terminate_thread(thread)` terminate handle tồn tại và đưa handle về null.
