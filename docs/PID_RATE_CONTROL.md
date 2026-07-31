# Rate PID đã triển khai

Cập nhật: 2026-07-30

## 1. Trạng thái hiện tại

Firmware hiện có **ba bộ PID điều khiển tốc độ góc thân** độc lập:

- roll rate;
- pitch rate;
- yaw rate.

Đây là **inner rate loop** dùng cho chế độ điều khiển kiểu Acro/rate. Nó chưa
phải PID cascade hoàn chỉnh vì chưa có outer angle loop.

Điểm đặc biệt quan trọng:

> PID hiện đã nhận command, đọc gyro BODY FRD và tính correction, nhưng
> correction chỉ được giữ trong `RateControlDebug.output[]`. Nó **chưa được
> đưa vào Quad-X mixer và chưa làm thay đổi PWM motor**.

Bốn motor hiện vẫn nhận cùng một collective throttle trong
`apply_throttle()`. Vì vậy phần hiện tại phù hợp để review logic và bench test,
chưa phải flight control hoàn chỉnh.

Các file chính:

- `Components/DroneControl/Inc/rate_control.h`: kiểu dữ liệu và API của PID.
- `Components/DroneControl/Src/rate_control.c`: thuật toán PID.
- `Components/DroneControl/Src/drone_control.c`: gain, command, arming,
  failsafe và điểm gọi PID.
- `Components/App/Src/app.c`: chuyển gyro sang BODY FRD `rad/s`, tính `dt` và
  cấp sample cho PID.
- `Components/App/Src/app_rtos.c`: task flight chạy chu kỳ danh định 10 ms.
- `Tests/rate_control/test_rate_control.c`: unit test chạy trên host.

## 2. Luồng tín hiệu

```text
ESP32 CONTROL_COMMAND
  roll/pitch/yaw: -1000 .. +1000
            |
            v
RateControl_SetCommand()
  clamp và chuẩn hóa về -1 .. +1
            |
            v
target body rate
  roll/pitch: -200 .. +200 deg/s
  yaw:        -150 .. +150 deg/s
  lưu và xử lý bằng rad/s
            |
            +-------------------------------+
                                            |
ICM20948 raw gyro                           |
            |                               |
            v                               |
trừ gyro bias                               |
            |                               |
            v                               |
sensor frame -> BODY FRD                    |
            |                               |
            v                               v
measured body rate (rad/s) ----------> rate error
                                      target - measured
                                            |
                                            v
                            P + I - D(measurement)
                                            |
                                            v
                            correction giới hạn theo trục
                                            |
                                            v
                              RateControlDebug.output[]
                                            |
                                            X
                                  chưa có Quad-X mixer
```

Quy ước thân đang dùng là BODY FRD:

- `+X`: forward;
- `+Y`: right;
- `+Z`: down;
- positive roll: cánh phải đi xuống;
- positive pitch: mũi đi lên;
- positive yaw: quay theo chiều kim đồng hồ khi nhìn từ trên.

Việc ánh xạ cảm biến hiện là:

```text
body_x = -sensor_x
body_y =  sensor_y
body_z = -sensor_z
```

## 3. Signal ledger

| Giá trị                   | Đơn vị/range                                                 | Nguồn                           | Nơi sử dụng                |
| --------------------------- | --------------------------------------------------------------- | -------------------------------- | ----------------------------- |
| `command->roll/pitch/yaw` | số nguyên`-1000..1000`                                      | control packet                   | `RateControl_SetCommand()`  |
| `normalized`              | không thứ nguyên`-1..1`                                    | command đã clamp               | tính target rate             |
| roll/pitch target           | `rad/s`, tối đa `±200 deg/s` = khoảng `±3.491 rad/s` | stick mapping                    | rate error                    |
| yaw target                  | `rad/s`, tối đa `±150 deg/s` = khoảng `±2.618 rad/s` | stick mapping                    | rate error                    |
| gyro raw                    | ICM20948 count                                                  | SPI                              | trừ bias và đổi đơn vị |
| gyro body                   | BODY FRD`rad/s`                                               | `Attitude_GyroRawToBodyRadS()` | `RateControl_Update()`      |
| `dt_s`                    | giây                                                           | chênh lệch`HAL_GetTick()`    | I và D                       |
| `integral[]`              | logical mixer-correction unit                                   | tích phân sai số              | tổng PID                     |
| `filtered_derivative[]`   | `rad/s²`                                                     | đạo hàm measurement qua LPF   | D term                        |
| `output[]`                | logical mixer-correction unit, chưa phải µs                  | PID                              | debug; mixer tương lai      |
| throttle                    | logical`0..1000`, tạm clamp tối đa 500                     | control packet                   | PWM collective                |
| PWM hiện tại              | `1000..1500 µs` do test clamp                                | `apply_throttle()`             | cả bốn motor bằng nhau     |

`output[]` được gọi là normalized mixer correction trong comment hiện tại,
nhưng nó không nằm trong miền `-1..1`. Hiểu chính xác hơn, đây là logical
mixer-correction unit trên thang command khoảng `0..1000`; quy ước này cần
được đóng thành contract rõ ràng khi triển khai mixer.

## 4. Timing ledger

| Hạng mục                  | Timing hiện tại                                              |
| --------------------------- | -------------------------------------------------------------- |
| FreeRTOS flight task        | chu kỳ danh định 10 ms, priority High                       |
| Đọc gyro/attitude         | được gate ở 10 ms, xấp xỉ 100 Hz                         |
| Rate PID                    | chạy khi có sample gyro hợp lệ, armed và throttle khác 0 |
| `dt`                      | `elapsed_ms * 0.001f`, độ phân giải 1 ms                 |
| Miền`dt` PID chấp nhận | `0.001..0.050 s`                                             |
| D-term LPF roll/pitch       | 20 Hz                                                          |
| D-term yaw                  | tắt vì`kd=0` và cutoff 0                                  |
| Control-link timeout        | 300 ms                                                         |
| Quad-X mixer                | chưa có                                                      |
| Motor update theo PID       | chưa có                                                      |

Task dùng `osDelayUntil()` nên lịch chạy là lịch tuyệt đối. Tuy nhiên sample
IMU hiện vẫn được polling, chưa dùng data-ready interrupt và chưa cấu hình ODR
thành một contract rõ ràng với rate loop.

## 5. Cấu trúc dữ liệu

### 5.1 Trục điều khiển

```c
typedef enum
{
  RATE_CONTROL_ROLL = 0,
  RATE_CONTROL_PITCH = 1,
  RATE_CONTROL_YAW = 2
} RateControlAxis;
```

Mọi mảng PID có đúng ba phần tử và dùng enum này làm index. Cách này tránh
phải tạo ba biến riêng lặp lại cho roll, pitch và yaw.

### 5.2 Cấu hình một PID

```c
typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral_limit;
  float output_limit;
  float derivative_cutoff_hz;
} RatePidConfig;
```

Ý nghĩa:

- `kp`: gain tỉ lệ theo rate error.
- `ki`: gain tích phân; đã bao gồm phép nhân `dt_s` trong thuật toán.
- `kd`: gain đạo hàm theo measured rate.
- `integral_limit`: giới hạn riêng cho trạng thái I.
- `output_limit`: giới hạn correction cuối cùng của trục.
- `derivative_cutoff_hz`: cutoff của low-pass filter bậc một trên đạo hàm.

Nếu gọi đơn vị output là `u` thì:

```text
kp: u / (rad/s)
ki: u / rad
kd: u / (rad/s²)
integral_limit: u
output_limit: u
```

### 5.3 Cấu hình cả ba trục

```c
typedef struct
{
  RatePidConfig pid[RATE_CONTROL_AXIS_COUNT];
  float maximum_rate_rad_s[RATE_CONTROL_AXIS_COUNT];
} RateControlConfig;
```

`maximum_rate_rad_s[]` quyết định stick full-scale tương ứng với tốc độ góc
mục tiêu bao nhiêu.

### 5.4 Dữ liệu quan sát

```c
typedef struct
{
  float target_rad_s[RATE_CONTROL_AXIS_COUNT];
  float measured_rad_s[RATE_CONTROL_AXIS_COUNT];
  float output[RATE_CONTROL_AXIS_COUNT];
} RateControlDebug;
```

Struct này cho phép telemetry/test đọc:

- target;
- measurement;
- correction cuối cùng.

Hiện chưa tách riêng P term, I term, D term và saturation flag. Nên bổ sung các
giá trị đó trước khi tune trên test rig.

### 5.5 Trạng thái runtime

```c
typedef struct
{
  RateControlConfig config;
  RateControlDebug debug;
  float integral[RATE_CONTROL_AXIS_COUNT];
  float previous_measurement[RATE_CONTROL_AXIS_COUNT];
  float filtered_derivative[RATE_CONTROL_AXIS_COUNT];
  bool derivative_initialized[RATE_CONTROL_AXIS_COUNT];
  bool initialized;
} RateControl;
```

Trong đó:

- `config`: bản copy cấu hình, không phụ thuộc lifetime của biến config bên
  ngoài.
- `debug`: target, measurement và output gần nhất.
- `integral`: trạng thái tích phân của từng trục.
- `previous_measurement`: sample trước để tính đạo hàm.
- `filtered_derivative`: trạng thái LPF của D term.
- `derivative_initialized`: ngăn đạo hàm nhảy ở sample đầu tiên.
- `initialized`: guard chống dùng controller trước khi init thành công.

`RateControl` được nhúng trực tiếp trong `DroneControlContext`, nên toàn bộ
state điều khiển thuộc về một context:

```c
typedef struct
{
  UART_HandleTypeDef *uart;
  MotorPwm_Handle_t motors;
  RateControl rate_control;
  /* Các trường protocol, state machine và failsafe khác. */
} DroneControlContext;
```

## 6. Gain đang cấu hình

Đây là nguyên văn cấu hình hiện tại:

```c
static const RateControlConfig rate_control_config = {
    .pid = {
        [RATE_CONTROL_ROLL] = {
            .kp = 45.0f,
            .ki = 20.0f,
            .kd = 0.8f,
            .integral_limit = 60.0f,
            .output_limit = 200.0f,
            .derivative_cutoff_hz = 20.0f,
        },
        [RATE_CONTROL_PITCH] = {
            .kp = 45.0f,
            .ki = 20.0f,
            .kd = 0.8f,
            .integral_limit = 60.0f,
            .output_limit = 200.0f,
            .derivative_cutoff_hz = 20.0f,
        },
        [RATE_CONTROL_YAW] = {
            .kp = 35.0f,
            .ki = 10.0f,
            .kd = 0.0f,
            .integral_limit = 50.0f,
            .output_limit = 150.0f,
            .derivative_cutoff_hz = 0.0f,
        },
    },
    .maximum_rate_rad_s = {
        [RATE_CONTROL_ROLL] = CONTROL_ROLL_PITCH_MAX_RATE_RAD_S,
        [RATE_CONTROL_PITCH] = CONTROL_ROLL_PITCH_MAX_RATE_RAD_S,
        [RATE_CONTROL_YAW] = CONTROL_YAW_MAX_RATE_RAD_S,
    },
};
```

Với:

```c
#define CONTROL_DEG_TO_RAD                   0.01745329252f
#define CONTROL_ROLL_PITCH_MAX_RATE_RAD_S   (200.0f * CONTROL_DEG_TO_RAD)
#define CONTROL_YAW_MAX_RATE_RAD_S           (150.0f * CONTROL_DEG_TO_RAD)
```

Các gain này chỉ là giá trị khởi đầu để bench test. Không thể kết luận gain
đúng nếu chưa biết quán tính frame, motor, propeller, ESC, tần số actuator và
chưa tune trên airframe thật.

## 7. API và trách nhiệm

| Hàm                               | Trách nhiệm                                       |
| ---------------------------------- | --------------------------------------------------- |
| `RateControl_Init()`             | validate config, xóa state và copy config         |
| `RateControl_Reset()`            | xóa target/debug và toàn bộ state I/D           |
| `RateControl_SetCommand()`       | map command`-1000..1000` sang target `rad/s`    |
| `RateControl_Update()`           | validate`dt`/measurement rồi update cả ba trục |
| `update_axis()`                  | thực hiện thuật toán PID của một trục        |
| `RateControl_GetDebug()`         | snapshot target, measurement và output             |
| `DroneControl_UpdateBodyRates()` | safety gate giữa gyro và PID                      |

## 8. Các hàm lõi — trích dẫn đầy đủ

Các đoạn dưới đây là bản chép nguyên hàm từ source tại thời điểm tài liệu được
cập nhật.

### 8.1 Khởi tạo

```c
bool RateControl_Init(RateControl *control,
                      const RateControlConfig *config)
{
  if ((control == NULL) || !config_valid(config))
  {
    return false;
  }

  memset(control, 0, sizeof(*control));
  control->config = *config;
  control->initialized = true;
  return true;
}
```

`config_valid()` yêu cầu tất cả gain/limit/rate đều finite; gain không âm,
`output_limit > 0` và maximum rate lớn hơn 0.

### 8.2 Reset state

```c
void RateControl_Reset(RateControl *control)
{
  if ((control == NULL) || !control->initialized)
  {
    return;
  }

  memset(&control->debug, 0, sizeof(control->debug));
  clear_dynamic_state(control);
}
```

Hàm phụ được gọi bên trong:

```c
static void clear_dynamic_state(RateControl *control)
{
  memset(control->integral, 0, sizeof(control->integral));
  memset(control->previous_measurement, 0,
         sizeof(control->previous_measurement));
  memset(control->filtered_derivative, 0,
         sizeof(control->filtered_derivative));
  memset(control->derivative_initialized, 0,
         sizeof(control->derivative_initialized));
  memset(control->debug.output, 0, sizeof(control->debug.output));
}
```

### 8.3 Map stick sang target rate

```c
void RateControl_SetCommand(RateControl *control,
                            int16_t roll,
                            int16_t pitch,
                            int16_t yaw)
{
  const int16_t command[RATE_CONTROL_AXIS_COUNT] = {
      roll,
      pitch,
      yaw,
  };
  uint8_t axis;

  if ((control == NULL) || !control->initialized)
  {
    return;
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    const float normalized =
        clampf((float)command[axis],
               -(float)RATE_CONTROL_COMMAND_LIMIT,
               (float)RATE_CONTROL_COMMAND_LIMIT) /
        (float)RATE_CONTROL_COMMAND_LIMIT;
    control->debug.target_rad_s[axis] =
        normalized * control->config.maximum_rate_rad_s[axis];
  }
}
```

Công thức:

```text
normalized = clamp(command, -1000, 1000) / 1000
target_rad_s = normalized * maximum_rate_rad_s
```

Ví dụ roll command `+500` tạo target `+100 deg/s`, tương đương khoảng
`+1.745 rad/s`.

### 8.4 Update ba trục và kiểm tra input

```c
bool RateControl_Update(RateControl *control,
                        const float measured_rad_s[RATE_CONTROL_AXIS_COUNT],
                        float dt_s)
{
  uint8_t axis;

  if ((control == NULL) ||
      (measured_rad_s == NULL) ||
      !control->initialized ||
      !isfinite(dt_s) ||
      (dt_s < RATE_CONTROL_MIN_DT_S) ||
      (dt_s > RATE_CONTROL_MAX_DT_S))
  {
    if ((control != NULL) && control->initialized)
    {
      clear_dynamic_state(control);
    }
    return false;
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    if (!isfinite(measured_rad_s[axis]))
    {
      clear_dynamic_state(control);
      return false;
    }
  }

  for (axis = 0U; axis < RATE_CONTROL_AXIS_COUNT; ++axis)
  {
    control->debug.measured_rad_s[axis] = measured_rad_s[axis];
    control->debug.output[axis] =
        update_axis(control, axis, measured_rad_s[axis], dt_s);
  }
  return true;
}
```

Nếu `dt` hoặc gyro không hợp lệ, hàm trả `false` và xóa state động để tránh
giữ integral/D history không an toàn.

### 8.5 Thuật toán PID một trục

```c
static float update_axis(RateControl *control,
                         uint8_t axis,
                         float measurement,
                         float dt_s)
{
  const RatePidConfig *pid = &control->config.pid[axis];
  const float error = control->debug.target_rad_s[axis] - measurement;
  float derivative = 0.0f;
  float derivative_term;
  float candidate_integral;
  float candidate_output;
  float output;

  if (control->derivative_initialized[axis])
  {
    const float raw_derivative =
        (measurement - control->previous_measurement[axis]) / dt_s;
    if (pid->derivative_cutoff_hz > 0.0f)
    {
      const float rc =
          1.0f / (RATE_CONTROL_TWO_PI * pid->derivative_cutoff_hz);
      const float alpha = dt_s / (rc + dt_s);
      control->filtered_derivative[axis] +=
          alpha * (raw_derivative - control->filtered_derivative[axis]);
      derivative = control->filtered_derivative[axis];
    }
    else
    {
      derivative = raw_derivative;
      control->filtered_derivative[axis] = raw_derivative;
    }
  }
  else
  {
    control->derivative_initialized[axis] = true;
  }
  control->previous_measurement[axis] = measurement;

  /* Derivative-on-measurement avoids a kick when the pilot moves the stick. */
  derivative_term = -pid->kd * derivative;
  candidate_integral =
      clampf(control->integral[axis] + (pid->ki * error * dt_s),
             -pid->integral_limit,
             pid->integral_limit);
  candidate_output =
      (pid->kp * error) + candidate_integral + derivative_term;

  /*
   * Conditional integration: do not wind the integrator farther into an
   * output limit. Integration in the direction that leaves saturation is
   * still allowed.
   */
  if (!(((candidate_output > pid->output_limit) && (error > 0.0f)) ||
        ((candidate_output < -pid->output_limit) && (error < 0.0f))))
  {
    control->integral[axis] = candidate_integral;
  }

  output = (pid->kp * error) +
           control->integral[axis] +
           derivative_term;
  return clampf(output, -pid->output_limit, pid->output_limit);
}
```

Thuật toán tương ứng:

```text
error[k] = target[k] - measurement[k]

raw_derivative[k] =
    (measurement[k] - measurement[k-1]) / dt

filtered_derivative[k] =
    filtered_derivative[k-1]
    + alpha * (raw_derivative[k] - filtered_derivative[k-1])

alpha = dt / (1 / (2*pi*cutoff_hz) + dt)

P = kp * error
I_candidate = clamp(I_previous + ki * error * dt, ±integral_limit)
D = -kd * filtered_derivative

output_candidate = P + I_candidate + D
output = clamp(P + accepted_I + D, ±output_limit)
```

Đạo hàm được lấy trên measurement và mang dấu âm. Khi pilot đổi setpoint đột
ngột, D term không phản ứng trực tiếp với bước nhảy setpoint, nhờ đó tránh
derivative kick.

Conditional integration chỉ chặn I khi output đã vượt giới hạn và error đang
đẩy nó lún sâu hơn vào saturation. Nếu error có chiều kéo output ra khỏi
saturation thì tích phân vẫn được phép cập nhật.

### 8.6 Safety gate từ flight code vào PID

```c
void DroneControl_UpdateBodyRates(float roll_rad_s,
                                  float pitch_rad_s,
                                  float yaw_rad_s,
                                  float dt_s)
{
  const float measured_rad_s[RATE_CONTROL_AXIS_COUNT] = {
      roll_rad_s,
      pitch_rad_s,
      yaw_rad_s,
  };

  if (!control.initialized ||
      (control.state != DRONE_STATE_ARMED) ||
      (control.applied_throttle == 0U))
  {
    RateControl_Reset(&control.rate_control);
    return;
  }

  if (!RateControl_Update(&control.rate_control, measured_rad_s, dt_s))
  {
    RateControl_Reset(&control.rate_control);
  }
}
```

PID không tích lũy state khi:

- controller chưa init;
- drone chưa armed;
- throttle đang bằng 0;
- sample gyro hoặc `dt` bị PID từ chối.

Ngoài ra đường disarm/failsafe cũng reset PID:

```c
static void disarm_output(void)
{
  MotorPwm_Disarm(&control.motors);
  RateControl_Reset(&control.rate_control);
  control.applied_throttle = 0U;
}
```

## 9. Các điểm tích hợp

### 9.1 Command packet cập nhật target

Sau khi packet đã qua decode, CRC, range, session và sequence validation:

```c
control.requested_throttle = command->throttle;
RateControl_SetCommand(&control.rate_control,
                       command->roll,
                       command->pitch,
                       command->yaw);
```

Setpoint được giữ giữa các packet. Nếu không có packet hợp lệ trong 300 ms,
link timeout gọi failsafe, disarm motor và reset PID.

### 9.2 Gyro cập nhật measurement

Đoạn tích hợp trong `App_UpdateAttitude()`:

```c
dt_s = (float)elapsed_ms * 0.001f;
if (dt_s > 0.05f)
{
  dt_s = 0.05f;
}

gyro_sensor_raw.x = icm20948_raw.gyro.x;
gyro_sensor_raw.y = icm20948_raw.gyro.y;
gyro_sensor_raw.z = icm20948_raw.gyro.z;
gyro_bias_sensor_raw.x = icm20948_gyro_bias.x;
gyro_bias_sensor_raw.y = icm20948_gyro_bias.y;
gyro_bias_sensor_raw.z = icm20948_gyro_bias.z;
gyro_body_rad_s = Attitude_GyroRawToBodyRadS(gyro_sensor_raw,
                                              gyro_bias_sensor_raw,
                                              131.0f);
DroneControl_UpdateBodyRates(gyro_body_rad_s.x,
                             gyro_body_rad_s.y,
                             gyro_body_rad_s.z,
                             dt_s);
```

`131.0 LSB/(deg/s)` khớp với gyro full-scale ±250 dps đang cấu hình. Hàm
attitude trừ bias, ánh xạ sensor frame sang BODY FRD, rồi đổi `deg/s` sang
`rad/s`.

Rate loop được đặt trước điều kiện `mahony9.initialized`, vì rate PID chỉ cần
gyro và vẫn có thể hoạt động khi magnetometer hoặc absolute attitude chưa sẵn
sàng.

### 9.3 Output chưa vào motor

Đây là logic motor hiện tại:

```c
const uint16_t pulse_us = (uint16_t)(
    CONTROL_ESC_MIN_PULSE_US +
    (((uint32_t)applied *
      (CONTROL_ESC_MAX_PULSE_US - CONTROL_ESC_MIN_PULSE_US)) /
     1000U));
for (i = 0U; i < MOTOR_PWM_MOTOR_COUNT; ++i)
{
  pulses[i] = pulse_us;
}
```

Không có `RateControlDebug.output[]` trong phép tính này. Mọi motor nhận cùng
`pulse_us`.

## 10. Cơ chế an toàn đã có

- Input config phải finite và nằm trong miền hợp lệ.
- Command được clamp trước khi map sang target rate.
- Measurement và `dt` phải finite.
- PID chỉ chấp nhận `dt` từ 1 ms đến 50 ms.
- D term sample đầu tiên bằng 0, tránh derivative spike lúc khởi động.
- D term roll/pitch có LPF bậc một 20 Hz.
- Integral có hard limit.
- Conditional integration chống windup tại giới hạn output theo trục.
- Output mỗi trục có hard limit.
- Reset state khi disarm, failsafe, throttle 0 hoặc PID input invalid.
- PID không có đường ghi thẳng PWM; motor vẫn qua arming/failsafe gate duy
  nhất.
- Không sửa file CubeMX sinh tự động để triển khai module PID.

## 11. Findings và giới hạn cần xử lý

### P1 — Loop stall dài đang bị che bởi phép clamp `dt`

Bằng chứng:

- `App_UpdateAttitude()` ép `dt_s > 0.05f` về đúng `0.05f`.
- `RateControl_Update()` chỉ từ chối khi `dt_s > 0.05f`.

Hệ quả: nếu flight task hoặc SPI bị block 100 ms, 500 ms hoặc lâu hơn, PID vẫn
nhận `dt=50 ms` và chạy thay vì phát hiện stale sample/reset. Điều này mâu
thuẫn với mục tiêu “reject stalls”.

Khuyến nghị: kiểm tra elapsed time gốc trước khi clamp. Khi vượt ngưỡng, reset
PID và đi vào sensor-invalid/failsafe policy; không biến stall dài thành sample
hợp lệ.

Test cần có: feed elapsed time 51 ms, 100 ms và 500 ms qua đúng integration
path; xác nhận PID không update và motor đi về trạng thái an toàn đã chọn.

### P1 — Chưa có mixer và actuator saturation feedback

Bằng chứng: output PID chỉ lưu trong debug; `apply_throttle()` ghi cùng một
pulse cho cả bốn motor.

Hệ quả: rate PID hiện không thể ổn định thân drone. Khi thêm mixer, giới hạn
output từng trục hiện tại cũng chưa đủ để xử lý saturation tổng hợp của
throttle + roll + pitch + yaw.

Khuyến nghị: chỉ làm Quad-X mixer sau khi xác nhận motor order, vị trí, chiều
CW/CCW và dấu gyro. Thiết kế desaturation có priority rõ ràng; sau đó nối
actuator saturation trở lại anti-windup.

Test cần có: pure roll/pitch/yaw allocation, motor mapping thực tế, giới hạn
cao/thấp, throttle shift và sign test trên rig không cánh.

### P1 — Chưa có PID cascade

Bằng chứng: command được map trực tiếp thành body-rate target. Góc Euler từ
Mahony chưa được dùng để sinh desired rate.

Hệ quả: chỉ có rate/Acro behavior; drone không tự giữ roll/pitch angle theo
stick và không có yaw heading hold.

Khuyến nghị: tune inner rate loop trước. Sau khi rate loop ổn định, thêm outer
angle controller chạy chậm hơn và giới hạn output của outer loop thành desired
body rate.

Test cần có: xác nhận dataflow
`angle target -> desired rate -> rate PID -> mixer`, yaw wrapping và chuyển mode
không giật.

### P2 — Sensor timing chưa được khóa với rate loop

Bằng chứng: flight task polling mỗi 10 ms; chưa dùng ICM data-ready và chưa có
contract rõ ràng cho gyro ODR/sample divider.

Hệ quả: chưa chứng minh được mỗi lần PID update tương ứng đúng một sample mới;
jitter và sample age chưa được đo.

Khuyến nghị: cấu hình ODR/filter rõ ràng, thêm data-ready hoặc sample sequence,
và đo timestamp/jitter trên phần cứng.

Test cần có: logic-analyzer/GPIO timing trace và telemetry histogram của
`dt`, task runtime và sample age.

### P2 — Debug chưa đủ để tune

Bằng chứng: debug chỉ có target, measured và output.

Hệ quả: khó phân biệt rung do P, I windup, D noise hay mixer saturation.

Khuyến nghị: thêm P/I/D term, raw/filtered derivative, saturation flag và
measured `dt` vào telemetry trước khi tune.

## 12. Unit test hiện có

Host tests đang kiểm tra:

- mapping và clamp command;
- dấu output P;
- derivative-on-measurement có dấu chống lại bước tăng measured rate;
- integral limit;
- output limit;
- reset;
- từ chối `dt=100 ms`.

Chạy bằng:

```sh
make -C Tests/rate_control clean test
```

Khoảng trống test:

- integration test từ gyro mapping đến PID;
- NaN/Inf riêng cho từng trục;
- biên chính xác `dt=1 ms` và `dt=50 ms`;
- anti-windup khi error đổi chiều;
- timing stall qua `App_UpdateAttitude()`;
- mixer và actuator saturation.

## 13. Những thông tin phần cứng chưa được xác nhận

Trước khi nối PID vào motor phải xác nhận:

- vị trí vật lý Motor 1–4;
- mapping Motor 1–4 sang TIM channel/pin;
- chiều quay CW/CCW từng motor;
- loại và chiều propeller;
- ESC protocol và tần số update hỗ trợ;
- dấu BODY FRD của gyro bằng thao tác xoay thực tế;
- frame, motor, propeller, pin và khối lượng để tune gain.

## 14. Readiness

Trạng thái hiện tại: **code-review ready và bench-test ready khi tháo cánh**.

Chưa đạt:

- restrained-rig ready, vì PID chưa nối mixer và timing stall chưa được xử lý;
- flight ready, vì chưa có Quad-X mixer, desaturation, hardware sign test và
  tuning thực tế.
