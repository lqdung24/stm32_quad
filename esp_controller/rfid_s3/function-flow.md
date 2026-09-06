# RFID ESP32-S3 - function flow

## Luồng tổng thể

`app_main` mở USB console, tạo ba semaphore/mutex, cấu hình RC522 qua SPI và chạy polling task của thư viện. Callback lifecycle lưu con trỏ card đang active. Vòng CLI đọc một dòng và dispatch `scan`, `read`, `write`, `key` hoặc `clear`.

```text
RC522 polling --on_picc_state_changed--> s_picc + s_card_seen
USB line --process_command--> command_* --s_rc522_io_lock--> MIFARE API
```

## `main/main.c`

- `reset_keys()` đặt Key A mặc định `FF FF FF FF FF FF` cho mọi sector trong RAM.
- `hex_nibble(c)` đổi một ký tự hex không phân biệt hoa/thường thành 0..15; ký tự sai trả -1.
- `parse_hex(text, out, length)` yêu cầu đúng `length*2` ký tự, ghép từng cặp nibble vào byte; fail ngay khi độ dài/ký tự sai.
- `active_picc()` đọc `s_picc` dưới mutex. Nếu chưa có card, in lời nhắc và chờ binary semaphore tối đa 10 giây, rồi đọc lại con trỏ card.
- `print_picc(picc)` in UID nối liền dạng hex, SAK và tên loại card.
- `data_block(picc, block, sector_index)` kiểm tra card tương thích MIFARE Classic, lấy descriptor và suy ra sector, từ chối block ngoài số sector. Lưu ý: kiểm tra cấm manufacturer block/sector trailer hiện đang bị comment nên hàm hiện cho phép các block nhạy cảm này.
- `command_scan()` lấy card active; timeout thì báo lỗi, có card thì gọi `print_picc`.
- `parse_block(text, block)` parse số thập phân bằng `strtoul`, yêu cầu dùng hết chuỗi và nằm trong `uint8_t`.
- `command_rw(write, block_text, hex)` parse block và data 16 byte nếu write; chờ card; lấy mutex phần cứng tối đa 2 giây; kiểm tra lại card; validate block/sector; authenticate bằng key sector; write nếu yêu cầu; luôn read lại; với write thì so sánh byte để verify; in dữ liệu hoặc lỗi; cuối flow deauthenticate nếu đã auth và luôn trả mutex.
- `command_key(type, sector_text, hex)` parse Key A/B sáu byte và sector cụ thể hoặc `all`; cập nhật `s_keys` trong RAM, không ghi key vào card.
- `on_picc_state_changed(...)` chạy từ polling task khi mutex RC522 đã được giữ. Khi card ACTIVE, lưu con trỏ và signal `s_card_seen`; khi card rời active state, xóa con trỏ và drain signal cũ để lần chờ sau không nhận nhầm.
- `process_command(line)` tokenize không phá state toàn cục bằng `strtok_r`; dispatch `scan`, `read`, `write`, `key`, `clear`; command khác in usage.
- `app_main()` init console; tạo mutex state, semaphore card và mutex I/O; reset key; giảm log polling; tạo/cài SPI driver; truyền chính mutex I/O vào RC522 config; đăng ký callback, start scanner; sau đó lặp đọc dòng, xử lý và in prompt.

## `main/usb_console.c`

- `usb_console_init()` cài USB Serial/JTAG với TX 2048 và RX 512 byte.
- `usb_console_write(text)` kiểm tra null, gửi toàn chuỗi với timeout 1 giây, chỉ thành công khi ghi đủ.
- `usb_console_printf(format, ...)` format vào buffer 512 byte, từ chối output lỗi/truncated rồi gọi `usb_console_write`.
- `usb_console_readline(line, capacity)` đọc từng byte; bỏ dòng trống và ký tự control, echo ký tự in được, xử lý backspace, đánh dấu overflow nhưng vẫn drain đến newline; trả chuỗi null-terminated cùng `ESP_OK` hoặc `ESP_ERR_INVALID_SIZE`.

## Đồng bộ quan trọng

- `s_picc_lock` chỉ bảo vệ con trỏ lifecycle `s_picc`.
- `s_card_seen` chỉ báo sự kiện xuất hiện card.
- `s_rc522_io_lock` là cùng mutex với polling task của thư viện; mọi auth/read/write CLI phải giữ mutex này.
