# ICM20948 driver - function flow

## Luồng truy cập

```text
API sensor -> select register bank -> SPI CS low/transfer/CS high
AK09916 API -> cấu hình ICM auxiliary-I2C master -> shadow registers -> SPI read
```

## Khởi tạo và motion sensor

- `ICM20948_Init(dev, hspi, ncs_port, ncs_pin)` validate handle, đặt default state/timeout; đọc WHO_AM_I tham khảo; dừng toàn bộ AUX-I2C slave trước reset để tránh kẹt bus; reset chip, chọn clock, wake sensor, đặt range accel/gyro mặc định rồi về bank 0.
- `ICM20948_SetTimeout(dev, timeout_ms)` cập nhật timeout HAL nếu handle tồn tại.
- `ICM20948_ReadWhoAmI(dev, who_am_i)` đọc register identity qua API bank-aware.
- `ICM20948_SetAccelRange(dev, range)` read-modify-write ACCEL_CONFIG, bật DLPF và cập nhật cached range chỉ khi write thành công.
- `ICM20948_SetGyroRange(dev, range)` tương tự cho GYRO_CONFIG_1.
- `ICM20948_ConfigureMotion(dev, config)` validate range/divider/DLPF; ghi gyro divider/config và accel divider 12-bit/config; cập nhật cached sensitivity rồi trở về bank 0.
- `ICM20948_IsRawDataReady(dev, ready)` đọc INT_STATUS_1 và tách bit data-ready.
- `ICM20948_ReadRaw(dev, data)` burst-read 14 byte accel+gyro+temperature, ghép từng cặp big-endian thành int16.
- `ICM20948_ReadScaled(dev, data)` gọi ReadRaw, lấy sensitivity theo cached range, đổi accel sang g, gyro sang deg/s và nhiệt độ sang °C.

## Magnetometer AK09916

- `ICM20948_InitMagnetometer(dev, wia2)` bật internal I2C master, cấu hình clock/ODR; đọc và kiểm tra WIA2; reset AK09916, đặt continuous 100 Hz, readback mode rồi cấu hình SLV0 shadow stream 9 byte.
- `ICM20948_ReadMagRaw(dev, mag)` đọc 9-byte shadow tối đa ba lần; ghép XYZ little-endian của AK09916; từ chối overflow hoặc vector toàn zero, delay 1 ms giữa retry.
- `ICM20948_ReadMagDebug(dev, debug)` đọc tập register ICM ở bank 0/3 và shadow; sau đó dùng SLV4 one-shot để phân biệt lỗi stream SLV0 với lỗi AUX bus/device.
- `ICM20948_MagReadRegister(...)` wrapper đọc một byte qua `MagReadRegisters`.
- `ICM20948_MagReadRegisters(dev, reg, data, len)` cấu hình SLV0 read address/register/length, chờ transaction rồi đọc shadow bank 0.
- `ICM20948_MagWriteRegister(dev, reg, value)` cấu hình SLV0 write, chờ, rồi disable SLV0 transaction.
- `ICM20948_MagDebugReadSlave4(...)` lưu/tắt SLV0, clear status cũ, trigger SLV4 one-shot, poll EN tối đa 50 ms, chụp master status/data/control, tắt SLV4 và restore SLV0.
- `ICM20948_MagConfigureContinuousRead(dev)` tắt delay-shadow cũ, trỏ SLV0 tới AK09916 ST1 và enable stream 9 byte liên tục.

## Register/SPI helpers

- `ICM20948_ReadRegister(dev, bank, reg, value)` wrapper một byte cho `ReadRegisters`.
- `ICM20948_ReadRegisters(dev, bank, reg, data, len)` validate, chọn bank rồi burst-read raw.
- `ICM20948_WriteRegister(dev, bank, reg, value)` validate, chọn bank rồi raw-write.
- `ICM20948_SelectBank(dev, bank)` luôn ghi BANK_SEL, kể cả cache cho rằng đúng bank, để tự phục hồi missed write; thành công mới cập nhật cache.
- `ICM20948_ReadRawRegister(dev, reg, data, len)` dựng SPI command read+dummies, kéo CS low, transmit-receive, nhả CS và copy RX bỏ byte đầu khi HAL OK.
- `ICM20948_WriteRawRegister(dev, reg, value)` dựng hai byte write, bọc HAL transmit bằng CS low/high và map status.
- `ICM20948_FromHalStatus(status)` map HAL_OK sang driver OK, mọi lỗi HAL sang ERROR_SPI.
- `ICM20948_BankValue(bank)` dịch bank index vào nibble cao BANK_SEL.
- `ICM20948_ToInt16(msb, lsb)` ghép hai byte big-endian của ICM.
- `ICM20948_AccelSensitivity(range)` trả LSB/g tương ứng ±2/4/8/16g.
- `ICM20948_GyroSensitivity(range)` trả LSB/(deg/s) tương ứng ±250/500/1000/2000 dps.

Mọi chuỗi cấu hình trả ngay lỗi đầu tiên; caller không nên dùng dữ liệu/cached range nếu init/config chưa trả `ICM20948_OK`.
