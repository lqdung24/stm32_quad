# Web Controller - function flow

## Luồng điều khiển end-to-end

```text
pointer/keyboard -> command state -> createControl -> CRC -> COBS -> Web Serial
Web Serial -> COBS decode -> decodePacket
                         +-> decodeStatus -> ACK/link/state UI
                         +-> decodeFlightTelemetry -> plot/record/export
25 ms timer ----------------------------------------------------> send control
100 ms timer -> link watchdog -> local safe/e-stop khi mất link
```

## Protocol và connection (`app.js`)

- `$`, `fieldNodes` lấy DOM node theo id hoặc data attribute.
- `setStatus(name, text, tone)` cập nhật tất cả status field, thêm dot cho link và class màu.
- `setCommand(name, value)` cập nhật các field command đồng tên.
- `enterControlMode()` ẩn launch overlay rồi gọi connect.
- `enterFullscreen()` yêu cầu fullscreen nếu browser hỗ trợ; lỗi bị bỏ vì không ảnh hưởng safety.
- `crc16(data, length)` tính CRC-16/CCITT-FALSE giống firmware.
- `createControl()` dựng packet 30 byte little-endian; emergency ưu tiên E-STOP; chỉ phát throttle khi armed và deadman/pilot cho phép; chỉ phát axes ở pilot; tăng sequence, gắn session/timestamp và CRC.
- `sendNow()` nếu có writer thì COBS-encode control và write; lỗi write đi vào `serialDisconnected`.
- `cobsEncode(data)` tạo frame có delimiter đầu/cuối để đồng bộ cả protocol lẫn debug text.
- `cobsDecode(encoded)` phục hồi zero theo COBS; frame sai trả null.
- `newSession()` tạo session uint16 ngẫu nhiên khác 0, reset sequence/status/ACK và đưa local state về safe.
- `connect()` kiểm tra Web Serial, cho người dùng chọn/mở port, lấy writer, tạo session, start reader và gửi command đầu; lỗi hiển thị rồi cleanup.
- `readSerial()` đọc chunk liên tục, tách theo byte 0, giới hạn buffer, decode COBS rồi gọi `decodePacket`; finally luôn release reader và disconnect.
- `decodePacket(data)` kiểm tra magic/version rồi dispatch packet status hoặc flight telemetry theo type.
- `serialDisconnected()` dừng reader, release writer, xóa port/buffer, bật emergency local và báo USB disconnected.
- `decodeStatus(data)` kiểm tra exact length/header/CRC; đọc session, ACK, state/error/throttle/PWM/rate; cập nhật timestamp ACK nếu cùng session và lag <=16; render UI; nếu STM32 FAILSAFE/ERROR khi đang arm thì khóa local và gửi e-stop.
- `decodeFlightTelemetry(data)` kiểm tra size/header/flags/state/CRC/PWM; decode attitude, gyro, rate setpoint, PID tổng và bốn PWM; tính rate error; theo dõi sequence drop/rate; cập nhật history 60 giây và recorder có giới hạn 180.000 mẫu.

## Plot và lưu telemetry

- `updateTelemetryStats()` cập nhật packet/rate/drop/invalid/recorded và summary mới nhất.
- `telemetrySeries()` chọn series cho rate tracking + PID theo trục, attitude hoặc PWM.
- `renderTelemetryLegend(panels)` dựng legend DOM an toàn bằng `textContent`.
- `drawTelemetryPlot()` resize canvas theo device pixel ratio, lọc time window, autoscale từng panel và vẽ series; chỉ chạy khi panel đang mở.
- `telemetryRow(sample)` flatten sample và các rate error thành schema export ổn định.
- `csvValue(value)` quote/escape field CSV khi cần.
- `downloadTelemetry(format)` serialize record thành CSV hoặc TXT tab-separated, tạo Blob và kích hoạt browser download.
- `toggleTelemetryRecording()` bật/tắt ghi nhưng không ảnh hưởng luồng plot/control.
- `clearTelemetryData()` dừng ghi và reset history/counters/sequence tracking.

## UI state và input

- `updateCommandUi()` render throttle/axes hiện tại.
- `setKnob(knob, x, y)` đặt transform pixel cho joystick knob.
- `drawStickPositions()` map throttle/yaw và roll/pitch từ command domain sang bán kính hai joystick.
- `resetInputs()` bỏ deadman, zero throttle/axes/slider rồi redraw.
- `setMotorModeLocked(locked)` enable/disable chọn all/single motor.
- `updateControlAvailability()` chỉ bật slider ở test+armed; chỉ mở joystick ở pilot+armed.
- `updateMotorModeUi()` render radio mode, motor picker, label và card được chọn.
- `renderControlMode()` chuyển layout test/pilot, badge/nút, availability và vị trí stick.
- `forceLocalSafe(message, useEmergency)` clear ARM, đặt emergency theo yêu cầu, zero input/PWM UI và mở lại selector.
- `releaseDeadman()` zero input và gửi ngay khi nhả slider.
- `stickGeometry(zone)` đo rect và bán kính hữu dụng của joystick.
- `clampUnitVector(x, y)` giữ vector pointer trong hình tròn đơn vị.
- `applyAxisDeadzone(value, deadzone)` đưa vùng tâm về zero và rescale phần còn lại liên tục.
- `pointerVector(event, zone)` đổi tọa độ pointer thành vector joystick chuẩn hóa.
- `updateLeftStick(event)` map X→yaw, Y→throttle, clamp và redraw.
- `updateRightStick(event)` map X→roll, -Y→pitch và redraw.
- `startStick(event, side)` yêu cầu armed+pilot; riêng stick trái buộc pointer bắt đầu gần knob để throttle không nhảy; capture pointer rồi update.
- `moveStick(event, side)` chỉ xử lý đúng pointer id đang capture.
- `releaseStick(event, side)` nhả capture state; stick trái tự center yaw nhưng giữ throttle, stick phải center roll/pitch; redraw và gửi ngay.

## Link và safety gates

- `espConnected()` yêu cầu writer tồn tại và ACK control mới dưới 300 ms.
- `stmConnected()` yêu cầu ESP online, status mới dưới 300 ms và không có UART_LINK_LOST.
- `canArm()` yêu cầu cả hai link, STM32 DISARMED, mọi input zero và emergency clear.
- `keyboardThrottleEnabled()` chỉ cho phím lên/xuống khi pilot, armed, link tốt và STM32 thực sự ARMED.
- `isTextEntryTarget(target)` chặn hotkey khi focus input/textarea/select/contenteditable.

## Event/timer flow quan trọng

- Đổi test↔pilot luôn `forceLocalSafe`, gửi DISARM trước đổi mode rồi gửi state mới.
- Đổi all↔single motor hoặc motor index luôn zero và gửi một packet trước lẫn sau thay đổi.
- ARM cần giữ nút 1 giây và qua `canArm`; DISARM zero không bật emergency; STOP bật emergency.
- `pagehide` và tab bị ẩn khi armed đều zero input và gửi emergency.
- Timer 25 ms gửi control đều đặn. Timer 100 ms cập nhật link UI và tự safe/e-stop nếu ESP hoặc STM32 mất khi armed.
