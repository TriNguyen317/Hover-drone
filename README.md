# PID kín cho drone INAV qua Raspberry Pi

Project này chạy hai vòng ngoài trên Raspberry Pi:

```text
GPS -> PID vị trí North/East -> tilt roll/pitch -> MSP RC -> INAV ANGLE/rate PID -> motor
ToF/barometer -> PID cao độ -> throttle --------^ 
```

INAV vẫn chịu trách nhiệm cho vòng attitude/rate nhanh. Không bật `NAV POSHOLD`
hoặc `NAV ALTHOLD` trong mission này, vì chúng sẽ xung đột với hai PID chạy trên
Pi.

Chu trình tự động:

```text
PREFLIGHT -> CALIBRATE -> WAIT_TAKEOVER -> ARM -> TAKEOFF
          -> ALTITUDE_HOVER 10 s (GPS PID OFF)
          -> GPS_ACQUIRE -> GPS_HOLD -> LAND
          -> ground confirm -> DISARM -> COMPLETE
```

Chế độ thử cao độ riêng:

```text
PREFLIGHT -> CALIBRATE -> WAIT_TAKEOVER -> verify mask 24 -> auto ARM
          -> TAKEOFF -> ALTITUDE_HOVER 10 s -> LAND -> auto DISARM
```

Trong chế độ này Pi chỉ override throttle và ARM. Roll/pitch/yaw đi trực tiếp từ
receiver vật lý; INAV giữ toàn bộ attitude/rate PID. GPS và compass không bắt
buộc, không được đọc và PID GPS không bao giờ được bật. Các cột GPS trong CSV để
trống.

Trong `TAKEOFF` và `ALTITUDE_HOVER`, Pi chỉ đóng vòng cao độ; roll/pitch luôn ở
`rc_mid`. GPS vẫn được đọc, ghi log và kiểm tra chất lượng nhưng chưa tham gia
điều khiển. Sau đúng pha hover 10 giây, PID North/East được reset theo vị trí
hiện tại rồi mới kích hoạt để quay về/neo tại tọa độ GPS đã ghi lúc ground.

## Thành phần

- `flight_controller/msp.py`: transport MSPv1 có checksum, timeout và retry.
- `flight_controller/inav.py`: decode GPS, rangefinder/barometer, attitude, mode,
  RC và RPM bốn motor qua MSPv2.
- `flight_controller/pid.py`: PID có anti-windup, derivative-on-measurement và lọc D.
- `flight_controller/mission.py`: state machine, safety gates, CSV log.
- `flight_controller/simulator.py`: plant mô phỏng có nhiễu ngang.
- `config.example.json`: toàn bộ gain, giới hạn và mission parameters.
- `docs/inav_cli_setup.txt`: mẫu cấu hình AUX/override cho INAV 9.x.
- `tests/`: unit test và full simulated mission.

## Điều kiện phần cứng

- Raspberry Pi nối UART MSP với flight controller INAV: Pi TX -> FC RX, Pi RX <- FC
  TX, chung GND, logic 3.3 V.
- Full GPS mission yêu cầu GPS, compass, gyro, accelerometer và cảm biến cao độ
  là `HW_SENSOR_OK`. Altitude-only/bench chỉ yêu cầu gyro, accelerometer và cảm
  biến cao độ.
- Mặc định dùng rangefinder qua `MSP_SONAR_ALTITUDE`; có thể chọn `barometer`
  trong config.
- Receiver vật lý luôn còn kết nối. Một switch riêng kích hoạt `MSP RC OVERRIDE`;
  switch này tuyệt đối không nằm trong `msp_override_channels`.
- RTH và failsafe phải ở switch/kênh vật lý riêng và đã được test trước.

## Cấu hình INAV

Đọc file `docs/inav_cli_setup.txt`, sau đó đối chiếu với kết quả CLI `aux` của
chính flight controller. Ví dụ trong file dùng:

- CH5/AUX1: ARM do Pi điều khiển;
- CH6/AUX2: ANGLE do Pi điều khiển ở full mission, hoặc do switch vật lý giữ ON
  trong altitude-only;
- CH8/AUX4: switch takeover vật lý;
- full GPS mission dùng mask `63` (CH1..CH6);
- altitude-only và bench tháo cánh dùng mask `24` (chỉ throttle + CH5 ARM).
  Mask dùng thứ tự nội bộ AERT của INAV: throttle bit 3 (`8`) + ARM/AUX1 bit 4
  (`16`), kể cả khi receiver map là AETR.

Không paste số slot/mask nếu cấu hình kênh của drone khác. Sau khi `save`, kiểm
tra lại `aux` và `get msp_override_channels`.

## Cài trên Raspberry Pi

```bash
cd controller
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -v
```

Nếu `/dev/serial0` bị từ chối quyền, thêm user vào nhóm sở hữu UART (thường là
`dialout`) rồi đăng nhập lại. Không chạy INAV Configurator, telemetry monitor hay
process MSP khác trên cùng UART trong lúc chạy controller.

### Reboot INAV và kiểm tra kết nối receiver

Chỉ chạy khi drone ở mặt đất, đã tháo cánh, INAV đang DISARMED và CH8 takeover
đang OFF. Script không ARM, không gửi MSP RC và không điều khiển motor:

```bash
python3 reset_inav_and_check_rx.py --config config.real.json --no-bind-rp4td
```

Đây là lệnh chỉ reboot FC. Nhập đúng `RESET DISARMED FC`. Script gửi normal `MSP_REBOOT`, chờ INAV kết nối
lại rồi in các kênh RC và trạng thái FAILSAFE trong 5 giây. Đây là reboot FC và
kiểm tra link receiver, không phải factory reset cấu hình INAV và cũng không tự
bind/factory-reset receiver. Reset/bind receiver phụ thuộc đúng model và protocol
(ví dụ ELRS/CRSF, SBUS, FlySky hoặc mLRS).

Với RadioMaster RP4TD ExpressLRS/CRSF, có thể reboot FC rồi yêu cầu INAV gửi
`bind_rx` tới receiver mà không power-cycle ba lần:

```bash
python3 reset_inav_and_check_rx.py --config config.real.json
```

Bind RP4TD được bật mặc định. Nhập đúng `RESET FC AND BIND RP4TD`, chuẩn bị trang ExpressLRS Lua trên tay cầm,
rồi làm theo prompt. Cách này cần firmware RP4TD hỗ trợ CRSF RX_BIND; nếu firmware
3.3.1 gốc không chuyển sang LED nháy kép thì nâng cả RX/TX lên cùng major ELRS
3.4 trở lên hoặc dùng cùng Binding Phrase qua WebUI.

## Chạy

Mô phỏng toàn bộ mission, không cần hardware:

```bash
python3 run_controller.py --simulate
```

Mô phỏng riêng nhiệm vụ giữ cao độ, không kích hoạt PID GPS:

```bash
python3 run_controller.py --simulate --altitude-only
```

Preflight/check-only trên FC thật; lệnh này không gửi raw RC, ARM hay motor:

```bash
python3 run_controller.py --config config.example.json
```

Bay thật chỉ sau các bước bench/flight-test bên dưới:

```bash
python3 run_controller.py --config config.example.json --fly
```

### Test auto-ARM khi đã tháo toàn bộ cánh

Trước hết đặt `msp_override_channels = 24`, bật ANGLE vật lý, để ARM vật lý OFF
và takeover CH8 OFF. Sau đó chạy:

```bash
python3 run_controller.py --config config.real.json --bench-arm-test --fly
```

Chương trình không yêu cầu nhập chuỗi xác nhận, chờ bật CH8, tự kiểm tra throttle
bị override nhưng roll/pitch/yaw không bị override, tự ARM ở throttle `rc_min`
trong 3 giây, ghi cao độ/RPM rồi tự DISARM. Không dùng
`--altitude-only --fly` cho lần tháo cánh vì drone không tăng cao và sẽ timeout
ở TAKEOFF.

Sau khi CH8 bật, controller gửi frame SAFE với ARM OFF/throttle LOW trong
`safety.override_settle_s` (mặc định 3 giây) để MSP override thoát trạng thái
recovery/failsafe rồi mới probe mask và ARM.

### Bay thử giữ cao độ

Chỉ sau khi bench test hoàn tất và đã kiểm tra bay tay/ANGLE/failsafe:

```bash
python3 run_controller.py --config config.real.json --altitude-only --fly
```

Giữ roll/pitch/yaw ở giữa và bật CH8. Pi tự ARM,
cất cánh đến `target_altitude_m`, hover trong `altitude_hover_time_s`, hạ cánh và
tự DISARM. Trong toàn bộ nhiệm vụ này `position_pid_enabled=0`.

Sau calibration, chương trình chuyển thẳng sang chờ switch takeover CH8
vật lý, không yêu cầu gõ xác nhận. Vẫn cần `--fly` để bay thật; các trường
`required_confirmation`, `altitude_only_confirmation`, `bench_confirmation`
trong config cũ không còn được sử dụng. Khi có lỗi giữa không trung, chương trình dừng stream MSP và không tự
disarm; pilot phải chuyển đúng trình tự takeover/ARM được mô tả ở phần test bên
dưới rồi dùng RC/RTH để hạ cánh.

CSV được ghi vào `logs/`, gồm state, GPS hiện tại (`gps_lat`, `gps_lon`), cao độ
AGL/thô, North/East error, attitude, RC output, các thành phần PID, chất lượng
GPS, `motor_1_output` đến `motor_4_output` và `motor_1_rpm` đến `motor_4_rpm`.
Console phân biệt `OUT[M1=...]` là lệnh output INAV gửi ESC qua `MSP_MOTOR`, còn
`RPM[M1=...]` là tốc độ đo qua `MSP2_INAV_ESC_RPM`; hai nguồn được đọc mặc định
2 Hz trong khi PID chạy 10 Hz. CSV còn ghi `active_modes` và một dòng `event`
cuối khi abort/manual takeover để giữ lại nguyên nhân kết thúc.

Khi `height_source=rangefinder`, `vertical_speed_mps` được tính và lọc từ chính
rangefinder để cùng nguồn với PID cao độ; `inav_vertical_speed_mps` vẫn được ghi
riêng để chẩn đoán barometer/estimator nhưng không còn gây false abort ở sát đất.
Trong altitude-only/bench, các trường GPS/North/East để trống và console hiển
thị `GPS=disabled`.

`MSP_SET_RAW_RC` được gửi theo kiểu write-only giống một RC stream; controller
không chờ ACK cho từng frame throttle/ARM. Các lệnh đọc API, sensor, attitude,
height, mode và RPM vẫn bắt buộc có phản hồi vì đây là PID kín. Khi vừa mở UART,
controller tự thử lại handshake trong `serial.startup_timeout_s` (mặc định 5 s)
trước khi kết luận đường telemetry không hoạt động.

Các tham số serial liên quan:

- `request_timeout_s`: timeout cho từng yêu cầu telemetry;
- `retries`: số lần thử lại từng yêu cầu;
- `startup_timeout_s`: tổng thời gian chờ handshake đầu tiên;
- `startup_retry_delay_s`: khoảng nghỉ giữa các lần handshake.

Trong `logging`:

- `motor_rpm_hz`: tần số lấy mẫu RPM, cho phép 0.5–5 Hz;
- `require_motor_rpm`: mặc định `true`; preflight từ chối ARM nếu ESC telemetry
  không trả đủ bốn motor. Chỉ đặt `false` khi cố ý cho phép cột RPM để trống.

Trong takeoff, I-term cao độ bị khóa cho đến khi rangefinder xác nhận độ cao vượt
`liftoff_detect_margin_m` liên tục trong `liftoff_confirm_time_s`. Nếu chưa phát
hiện liftoff trước `liftoff_timeout_s`, controller abort thay vì tiếp tục tăng ga.
Ở full-GPS thông thường, PID X/Y giữ tọa độ ground ngay sau liftoff với giới hạn nghiêng
`takeoff_position_max_tilt_deg`, tiếp tục hoạt động trong 10 giây hover đầu tiên.
Với `optical_flow.mode=relative`, INAV hợp nhất cảm biến và cung cấp full local
pose; Pi lấy mốc X/Y tại lần đầu bật position control, chạy PID ngoài rồi gửi
AERT. Pi không tự tích phân raw flow hoặc fusion GPS lần thứ hai.
Altitude-only không có GPS vẫn chỉ giữ attitude; muốn giữ X/Y trong nhà phải có
optical flow hoặc nguồn định vị ngang tương đương.

## Calibration ground

Drone phải đứng yên và gần level. Controller lấy 40 mẫu (mặc định) rồi lưu:

- trung bình roll/pitch và circular mean yaw;
- median GPS làm vị trí neo;
- median rangefinder/barometer làm cao độ ground; riêng relative mode dùng
  median INAV local Z và lưu thêm khoảng cách rangefinder để kiểm tra tầm đo;
- độ lệch chuẩn cao độ và RMS GPS để từ chối calibration không ổn định.

Toàn bộ mẫu calibration được lưu vào `logs/calibration_YYYYMMDD_HHMMSS.csv`.
Các dòng `SAMPLE` chứa từng lần đọc roll/pitch/yaw, độ cao, GPS, fix, số vệ
tinh và HDOP. Dòng `REFERENCE` cuối file chứa mốc được tính, `height_std_m`,
`position_rms_m` và tên phương pháp tổng hợp. Roll và pitch dùng `mean`; GPS
và độ cao dùng `median`; yaw dùng `circular_mean`.

Yaw hiện tại dùng để đổi lệnh North/East sang trục forward/right của drone.
Roll/pitch ground là reference kiểm tra tư thế, không được cộng thành RC trim;
INAV ANGLE vẫn dùng calibration level của flight controller.

## Trình tự test bắt buộc

1. **Tháo toàn bộ cánh**: chạy check-only; kiểm tra sensor, channel map, ARM OFF.
2. Vẫn tháo cánh: đặt mask `24`, chạy `--bench-arm-test --fly`, xác nhận ARM,
   RPM, throttle thấp và auto-DISARM.
3. Gắn cánh, bay thủ công: kiểm tra ANGLE, motor direction, FC orientation,
   failsafe và RTH.
4. Tune PID cao độ ở dây giữ/khung test an toàn; bắt đầu với I và D thấp.
5. Thử neo thấp ngoài trời, vùng trống, GPS tốt; giới hạn target altitude và tilt.
6. Chỉ tăng gain/độ cao sau khi đọc CSV và không có saturation/dao động.

Trong altitude-only, ARM vật lý thường đang OFF vì Pi override CH5. Nếu cần lấy
quyền điều khiển khi còn trên không, bật ARM vật lý ON trước rồi mới tắt CH8;
tắt CH8 khi ARM vật lý đang OFF sẽ disarm ngay. Kênh FAILSAFE/RTH khẩn cấp phải
độc lập và không nằm trong mask override.

`config.example.json` là điểm khởi đầu cho simulator, không phải gain đã chứng
nhận cho airframe thật. Bắt buộc đo `throttle_hover`, xác nhận `roll_sign`,
`pitch_sign`, `pwm_per_degree`, rangefinder envelope và tune lại gain trên drone.

## Tuning nhanh

Vòng X/Y hiện dùng POSITION_P_VELOCITY_PI:
`v_set = -position_velocity_kp * position`, giới hạn vector bằng
`horizontal_speed_limit_mps`; PI vận tốc tạo góc nghiêng với
`velocity_tilt_kp` (deg/(m/s)), `velocity_tilt_ki` (deg/m).
Mặc định tương ứng 0.5 /s, 0.5 m/s, 5.0 và 0.3; đây là gain mô phỏng,
cần kiểm chứng trên airframe. Vận tốc North/East tính từ GNSS ground speed
và course over ground trong MSP_RAW_GPS, không dùng yaw làm hướng chuyển động.
MSP_RAW_GPS không cung cấp tuổi mẫu GNSS; vòng này chưa phát hiện được mọi
trường hợp GNSS giữ dữ liệu cũ dù MSP vẫn trả lời.

Các config `position_north_pid`/`position_east_pid` cũ được đọc để tương thích
nhưng không còn điều khiển X/Y. Cột `gps_*_error_m` vẫn là sai số vị trí;
`gps_*_p/i/d/output_deg` giờ mô tả vòng PI vận tốc (D bằng 0).
Cột `velocity_*_setpoint_mps`, `velocity_*_mps`, `velocity_*_error_mps`
ghi vận tốc mục tiêu, đo được và sai số. Giới hạn góc theo phase vẫn áp dụng.
GPS sai vị trí vẫn có thể gây lệnh quay về sai; cấu trúc này không cải thiện
độ chính xác tuyệt đối của cảm biến.

- `throttle_hover`: RC value giữ cao khi INAV ở ANGLE; sai giá trị này làm I term
  phải bù quá nhiều.
- `altitude_pid.kp`: tăng đến khi phản hồi rõ nhưng chưa dao động; thêm I để bỏ
  steady-state error; D chỉ thêm vừa đủ để giảm overshoot và rất nhạy với noise.
- PID vị trí có output là độ nghiêng. Tune `kp` trước, giới hạn
  `max_tilt_deg` thấp; thêm D để hãm drift, I rất nhỏ vì GPS chậm/nhiễu.
- Nếu correction chạy ngược chiều, sửa `roll_sign`/`pitch_sign`; không cố chữa
  bằng gain âm.

CSV ghi đầy đủ vòng PID GPS cho cả hai trục: error, P/I/D và output trước giới
hạn trong các cột `gps_north_*`/`gps_east_*`. Các cột
`tilt_north_limited_deg`, `tilt_east_limited_deg` là lệnh sau giới hạn vector;
`forward_tilt_deg`, `right_tilt_deg` là lệnh sau khi đổi sang hệ trục thân theo
yaw. `roll_pwm` và `pitch_pwm` là lệnh cuối gửi tới INAV. Khi PID vị trí chưa
bật hoặc chạy altitude-only, các cột PID GPS để trống.

`gps_calib_lat`/`gps_calib_lon` lưu tọa độ median lấy khi calibration và được
lặp lại trên mỗi dòng để mỗi mẫu log tự chứa đủ mốc tham chiếu.
`gps_measurement_lat`/`gps_measurement_lon` là tọa độ đọc trong từng vòng điều
khiển. `gps_lat`/`gps_lon` vẫn được giữ làm tên tương thích với log cũ. GPS có
thể cập nhật chậm hơn vòng điều khiển nên nhiều dòng liên tiếp có thể chứa cùng
một tọa độ đo.

Các thời gian liên quan nằm trong `control`:

- `altitude_hover_time_s`: mặc định 10 giây; full-GPS tiếp tục giữ X/Y trong
  pha này, còn altitude-only không chạy PID GPS;
- `gps_acquire_timeout_s`: thời gian tối đa để đạt lại tọa độ ground;
- `gps_hold_time_s`: thời gian neo GPS sau khi vị trí đã ổn định.

## Giới hạn

- MSP synchronous và Linux không phải real-time; vòng Pi chỉ nên là vòng ngoài
  5–20 Hz. INAV phải giữ vòng attitude/rate. Config mặc định giới hạn mỗi MSP
  request ở 0.18 s và abort nếu khoảng điều khiển vượt 0.5 s.
- GPS thường không đủ chính xác để neo trong phạm vi centimet. Mặc định chấp
  nhận 1.8 m và từ chối khi lỗi vượt 8 m.
- Rangefinder phải nhìn được nền và còn trong envelope suốt takeoff/landing.
- Controller không thay thế geofence, pilot, RTH, RC failsafe hoặc quy trình an
  toàn bay thực tế.
