# STM32 App - function flow

## Luồng runtime

```text
CubeMX setup -> App_Set* -> App_Init -> App_Process/AppRtos_Bootstrap
  high-priority 1 ms: App_FlightControlStep
      UART commands -> DroneControl_Process
      gyro/accel -> attitude conversion -> rate PID + Mahony9 -> motor/telemetry
      mag 10 ms -> calibrated heading reference
  telemetry 5 ms: logs/timing/mixer/UART diagnostics
  housekeeping 100 ms: heartbeat LED
```

## Setup và public entry points (`app.c`)

- `App_SetSpi`, `App_SetUsbTransmit`, `App_SetActivityLed`, `App_SetMotorTimer`, `App_SetControlUart` lưu các HAL handle/callback do CubeMX tạo để component dùng sau.
- `App_OnUsbReceive(data, length)` cố ý bỏ dữ liệu: USB CDC chỉ dành diagnostics, control an toàn chỉ nhận qua USART1 DroneProtocol.
- `App_OnUsbTransmitComplete()` xóa cờ USB busy để log kế tiếp được gửi.
- `App_Init()` bật cycle counter; start PWM ở disarmed; init DroneControl; init/calibrate ICM20948, mag và attitude; đặt mọi timestamp/stat; toggle LED khởi động.
- `App_StartMotorPwm(motors)` init timer PWM, config/start bốn channel; nếu một start lỗi thì stop các channel đã start; attach MotorPWM với 1000..2000 us.
- `App_Process()` chuyển quyền sang `AppRtos_Bootstrap`.
- `App_FlightControlStep(now_ms)` xử lý UART/failsafe trước; retry IMU mỗi giây khi lỗi; poll attitude/gyro mỗi 1 ms; sau rate loop mới đọc mag mỗi 10 ms; tùy compile flag phát log IMU 20 ms.
- `App_TelemetryStep(now_ms)` theo compile flag phát timing/mixer log theo chu kỳ, rồi drain UART RX debug log.
- `App_HousekeepingStep(now_ms)` toggle activity LED mỗi giây.
- `App_GetTimingStats(stats)` dùng sequence counter chẵn/lẻ và memory barrier để chụp snapshot không rách; đổi cycle sang µs và tính min/mean/max cùng rate milli-Hz.
- `HAL_UARTEx_RxEventCallback(...)` forward ISR callback vào `DroneControl_OnUartRxEvent`.
- `HAL_UART_ErrorCallback(...)` forward lỗi UART vào `DroneControl_OnUartError`.

## Sensor, estimator và diagnostics (`app.c`)

- `App_FlushControlUartLog()` lấy byte RX từ ring DroneControl, format hex và gửi USB nếu endpoint rảnh.
- `App_ReportTiming()` lấy timing snapshot, tính jitter peak-to-peak, format một dòng và đánh dấu USB busy khi submit thành công.
- `App_ReportMixer()` lấy snapshot mixer, scale số float để log integer, format throttle/PID/command/PWM/saturation rồi gửi USB.
- `App_FloatToTenths(value)` đổi float sang integer phần mười có làm tròn và saturate phù hợp cho log.
- `App_TryInitICM20948()` init SPI IMU, verify/read identity, cấu hình range/sample rate/DLPF, init Mahony9 và AK09916; lưu status để retry thay vì chạy tiếp với state giả hợp lệ.
- `App_CalibrateGyro()` khi IMU OK, bỏ mẫu đầu rồi cộng nhiều mẫu gyro đứng yên để tạo bias; lỗi đọc làm calibration thất bại an toàn.
- `App_UpdateAttitude()` kiểm tra data-ready, đọc raw và đo thời gian; chuyển accel/gyro sang body/calibrated; tính `dt` từ DWT; luôn đưa gyro vào rate PID khi có sample. Nếu Mahony9 chưa init thì thử init bằng accel+mag; đã init thì update 9-axis hoặc IMU-only; publish telemetry và ghi timing ở mọi nhánh sample hợp lệ.
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
