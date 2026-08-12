# ICM20948 TODO

- [ ] Thêm API chọn full-scale cho accelerometer và gyroscope.
- [ ] Áp dụng range đã chọn đồng nhất cho trục X/Y/Z (ICM20948 không đặt range riêng từng trục).
- [ ] Cập nhật hệ số đổi raw -> `g` và `deg/s` theo range, rồi log lại range đang dùng, viết thành hàm trong component chứ không chỉ trả mỗi raw
