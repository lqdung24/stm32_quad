# Phân tích telemetry — 07/10/2026, khoảng 14:26–14:27

Nguồn: `drone-telemetry-2026-10-07T07-27-11-856Z.csv`, 2.092 mẫu, 41,850 giây, từ **14:26:29.982 đến 14:27:11.832 giờ Việt Nam (UTC+7)**. Tên file dùng UTC và thời điểm tải xuống; thời điểm bắt đầu ghi nằm trong CSV.

## Các phát hiện chính

1. **P1, cần xác minh điều kiện thử — Sai lệch tốc độ góc và đầu ra PID tăng cùng mức ga.** Lần 1, RMS sai số roll tăng từ 0,0425 rad/s trong giây đầu lên 0,5557 rad/s trong phần giây cuối; pitch tăng 0,2207 → 0,5193 rad/s. Lần 2, roll 0,0391 → 0,4667 rad/s; pitch 0,2237 → 0,5104 rad/s. Đầu ra motor trung bình tăng khoảng 1230 → 1466 trên thang tương đương 1000..2000. Nếu bay tự do, đây là dấu hiệu cần xử lý trước khi công nhận tune ổn định. Nếu drone được giữ/cố định hoặc chưa rời đất, phản lực bên ngoài làm thay đổi ý nghĩa sai số. Kiểm tra bằng log cùng điều kiện, cùng mức ga và chuyển động mục tiêu có kiểm soát; không kết luận riêng P hoặc D quá lớn từ các số này.

2. **P1, cần xác minh điều kiện thử — Yaw không bám tốc độ mục tiêu bằng 0.** Lần 1, góc yaw ghi được đi từ -0,08° đến đỉnh +20,06°; tốc độ yaw có mẫu đạt +3,285 rad/s (~188°/s). Lần 2, yaw từ +1,50° xuống -49,69° trước khi dừng, thay đổi -51,19°; tốc độ thấp nhất -1,291 rad/s (~-74°/s). PID yaw vẫn tạo hiệu chỉnh và không chạm giới hạn ±150. Có thể liên quan cơ khí, chiều quay/cánh, lực cản ngoài, cân bằng lực đẩy hoặc phản hồi gyro; CSV không xác định nguyên nhân. Yaw Mahony6 là tích phân gyro, không phải hướng tuyệt đối; không có heading hold. Tuy vậy, sai số tốc độ yaw hiện diện trong gyro, không chỉ trong góc tích phân. Cần đối chiếu video/điều kiện thử, xác minh motor và cánh, rồi kiểm tra phản hồi yaw trong bố trí thử có kiểm soát.

3. **P1 nếu không chủ động dừng — Cả hai lần kết thúc bằng FAILSAFE.** Mẫu FAILSAFE đầu tiên xuất hiện lúc 14:26:38.217 và 14:27:07.471; từ những mẫu đó, cả bốn lệnh motor đều bằng 1000, actuator inactive, PID bằng 0. Không có SYSTEM_STATUS/error_flags hoặc gói CONTROL trong file nên không phân biệt được e-stop chủ động, command timeout, lỗi sample giữa hai lần truyền hay chuyển trạng thái không an toàn. Không có sequence gap telemetry cũng không chứng minh đường uplink CONTROL hoạt động bình thường. Xác minh bằng lịch sử nút STOP/Esc và log error_flags/command age.

4. **P2 — D vẫn có đóng góp đáng kể nhưng chưa thể kết luận D quá lớn.** Nếu firmware dùng Kp=45 và integral limit=60 như source hiện tại, có 8 mẫu roll và 7 mẫu pitch mà |PID − 45×error| > 60,05. Như vậy D có đóng góp ngoài giới hạn I tại các mẫu ấy. CSV chỉ ghi tổng PID, không ghi riêng P/I/D hay dt vòng điều khiển; dữ liệu 50 Hz không đủ tái dựng derivative của vòng IMU/PID khoảng 1 kHz. Cần ghi P/I/D, gyro và dt ở tốc độ cao hơn để phân biệt nhiễu/rung với đáp ứng chuyển động. PX4 cũng mô tả đánh đổi: D tăng damping nhưng khuếch đại nhiễu, D thấp có thể tăng overshoot; không chuyển trực tiếp gain PX4 sang firmware này. [Hướng dẫn PID của PX4](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter).

## Hai lần motor hoạt động

Các thống kê chỉ dùng mẫu `actuators_active=true` và `output_sample_matched=true`; số liệu sau khi FAILSAFE không trộn vào đánh giá PID.

| Chỉ số | Lần 1 | Lần 2 |
|---|---:|---:|
| Mẫu motor active đầu tiên | 14:26:33.320 | 14:27:02.567 |
| Mẫu motor active cuối cùng | 14:26:38.201 | 14:27:07.443 |
| Mẫu FAILSAFE đầu tiên | 14:26:38.217 | 14:27:07.471 |
| Từ active đầu đến FAILSAFE | 4,897 s | 4,904 s |
| Số mẫu active | 245 | 245 |
| RMS sai số tốc độ roll | 0,2949 rad/s | 0,3664 rad/s |
| RMS sai số tốc độ pitch | 0,3322 rad/s | 0,3986 rad/s |
| RMS sai số tốc độ yaw | 0,3570 rad/s | 0,4041 rad/s |
| RMS PID roll | 27,29 | 38,40 |
| RMS PID pitch | 33,63 | 39,96 |
| RMS PID yaw | 12,67 | 16,39 |
| Roll đo được | -4,52..+0,79° | -3,86..+1,37° |
| Pitch đo được | +1,79..+4,09° | +2,43..+4,58° |
| Lệnh motor thấp/cao nhất | 1174 / 1689 | 1176 / 1658 |
| Chênh lệch motor lớn nhất trong một bank | 450 | 455 |

Góc roll/pitch khá nhỏ trong lúc motor active, nhưng tốc độ góc dao động và đầu ra motor biến thiên mạnh. Max pitch 26,27° và max |gyro pitch| 4,554 rad/s của toàn file đều nằm ngoài đoạn motor active, không dùng chúng để mô tả mức mất ổn định trước FAILSAFE.

## So với log ngày 06/10

| Chỉ số, chỉ đoạn active | 06/10 | 07/10, gộp 2 lần | Thay đổi |
|---|---:|---:|---:|
| RMS sai số roll, rad/s | 0,3606 | 0,3326 | giảm 7,8% |
| RMS sai số pitch, rad/s | 0,4135 | 0,3669 | giảm 11,3% |
| RMS sai số yaw, rad/s | 0,2920 | 0,3813 | tăng 30,6% |
| RMS PID roll | 43,33 | 33,31 | giảm 23,1% |
| RMS PID pitch | 49,20 | 36,93 | giảm 24,9% |
| Đỉnh trị tuyệt đối PID pitch | 200,00 | 127,27 | không chạm ±200 trong log mới |
| Lệnh motor trung bình lớn nhất | 1610 | 1466 | mức ga thử thấp hơn |

Đây là so sánh mô tả, chưa phải thử nghiệm chứng minh hiệu quả giảm Kd: mức ga thấp hơn, điều kiện cơ khí chưa xác nhận, và gain đã flash không có trong CSV. Source hiện tại có roll/pitch **45 / 20 / 0,6**, yaw **35 / 10 / 0**, D cutoff **20 Hz**, outer Angle gain **4/s** tại `stm32cube/Components/DroneControl/Src/drone_control.c:131`. Báo cáo hôm qua giả định roll/pitch Kd=0,8.

## Những điểm đã kiểm tra

- Một session 42578; sequence 747..2838 liên tục, không mất sequence hoặc lặp sample ID trong bản ghi.
- Tất cả 2.092 mẫu báo attitude_valid=true; không suy ra rằng không từng có sample invalid giữa các lần truyền.
- 490 mẫu active đều có liên kết IMU/PID/motor; delay tới driver submission: 466 mẫu 1 ms, 24 mẫu 0 ms. Không phải đo thời điểm DMA hoàn tất hay phản ứng ESC/RPM.
- Khoảng cách timestamp STM của telemetry 18..22 ms, chủ yếu 20 ms; không phải dt vòng PID.
- Error bằng setpoint − gyro ở cả ba trục, đúng đơn vị rad/s.
- Setpoint roll/pitch khớp outer quaternion Angle với stick trung tính trong sai số dưới 0,00084 rad/s. Đây là suy luận từ quan hệ tín hiệu vì CSV không có mode/command flags.
- Nghịch đảo mixer Quad-X từ bốn lệnh motor khớp tổng PID trong 0,45 đơn vị, phù hợp lượng tử hóa. Không thấy correction scaling hoặc endpoint motor trong các mẫu active; vẫn không loại trừ sự kiện giữa các mẫu hay giới hạn lực đẩy thực tế.
- Không mẫu active nào chạm clamp roll/pitch ±200 hoặc yaw ±150.
- Khi FAILSAFE, telemetry chuyển sang lệnh dừng. Chưa kiểm chứng latency watchdog hay đầu ra chân phần cứng.

## Giới hạn và bước tiếp theo

**Bổ sung từ quan sát của người dùng:** drone thực tế nghiêng mạnh về một bên, nhưng người dùng không thấy hiệu chỉnh rõ. Chưa xác định hướng/góc nghiêng và thời điểm trước hay sau STOP/FAILSAFE. Nếu nghiêng mạnh xảy ra trong đoạn motor active, mức roll/pitch chỉ khoảng ±5° trong CSV không thể được dùng làm bằng chứng drone giữ góc tốt; cần ưu tiên đối chiếu góc thực tế với ước lượng IMU.

Log vẫn có bằng chứng PID tạo lệnh: lúc **14:26:38.090**, roll=-4,52°, roll target=+0,315 rad/s, gyro roll=-0,747 rad/s, PID roll=+95,45; bốn lệnh motor lần lượt 1642/1481/1310/1430. Theo motor order trong tài liệu, trung bình bên trái là 1561,5, bên phải 1370, chênh 191,5 đơn vị, khớp 2×PID roll trong lượng tử hóa. Điều này xác nhận hiệu chỉnh đã đi tới bank lệnh ghi được; chưa xác nhận motor vật lý đúng thứ tự, ESC phản ứng đúng hoặc torque thực tế đủ/đúng chiều.

Kiểm tra phân biệt đầu tiên: tháo cánh, giữ DISARMED, đặt thân ở các góc roll/pitch đã biết (ví dụ 0°, ±15°, ±30°), so sánh góc telemetry và trục thay đổi. Nếu sai ngay khi motor dừng, kiểm tra chiều lắp IMU, mapping trục và hiệu chuẩn; nếu đúng khi dừng nhưng sai lúc motor chạy, cần thêm dữ liệu accelerometer/gyro và quan sát đồng bộ để kiểm tra ảnh hưởng rung/chuyển động. Nếu góc đo đúng mà thân không hồi về, chuyển trọng tâm sang thứ tự motor, chiều hiệu chỉnh và đáp ứng lực đẩy rồi mới xác định gain. Chưa kết luận lỗi estimator hoặc gain từ quan sát chưa đồng bộ.

Chưa xác nhận drone bay tự do/giữ tay/cố định/chạy dưới đất, gain thực tế đã flash, hành động dừng chủ động, thứ tự/chiều quay motor thực tế, chiều lắp IMU, RPM, điện áp pin và rung accelerometer. Theo tài liệu, M1 front-left CW, M2 rear-left CCW, M3 front-right CCW, M4 rear-right CW; chưa đối chiếu được với phần cứng.

Ưu tiên xác định điều kiện thử và lý do FAILSAFE, kiểm tra yaw, rồi thu log cùng mức ga với P/I/D và dt riêng. Chưa đủ căn cứ đề xuất một bộ gain tối ưu hoặc giảm D tiếp. Trạng thái: **phân tích log hoàn tất; chưa xác nhận flight-ready**. Không sửa firmware, gain hay file CubeMX; không chạy build/test firmware vì không có thay đổi mã. Đã đồng bộ hai ô Kd trong `.context/domains/control-safety.md` từ 0,8 về 0,6 theo source hiện tại, giữ nguyên các chỉnh sửa có sẵn khác.

Biểu đồ: [telemetry_review.png](telemetry_review.png), trục thời gian theo STM; đường đỏ là mẫu FAILSAFE đầu tiên, vùng đỏ là dữ liệu sau khi dừng.

SHA-256 file CSV: `4dbdf64e70aed50dba1e1f413016497092024a0b66ad86290df4f62f17fe2aec`.
