# DRONE_ALT_HOLD_1M

Project nâng cấp trực tiếp từ `testZone/drone_pilot/drone_pilot.ino`, giữ mô
hình một file cho mỗi firmware:

- `drone_alt_hold_1m.ino`: toàn bộ flight controller trên drone.
- `sender_1m/sender_1m.ino`: toàn bộ trạm phát ESP-NOW.

Mục tiêu là tự cất cánh đến và giữ **0,50 m tính từ đáy drone tới mặt đất**,
đồng thời bám vị trí ngang tại điểm hoàn tất calibration ban đầu chính xác nhất
có thể bằng optical flow. Không có vùng dung sai vị trí ngang 50 cm. Đây là
firmware thử nghiệm đã qua
kiểm tra biên dịch, chưa phải một hệ thống được chứng nhận hay đã tune trên mọi
khung drone.

## Tài liệu kỹ thuật và kiểm thử

- [Kiến trúc flight controller](docs/FLIGHT_CONTROL_ARCHITECTURE.md)
- [Phân tích sensor fusion](docs/SENSOR_FUSION_ANALYSIS.md)
- [Chuẩn hóa flow/position hold theo độ cao](docs/POSITION_HOLD_HEIGHT_NORMALIZATION.md)
- [Các vấn đề bay và bằng chứng gốc](docs/FLIGHT_ISSUES.md)
- [Thiết kế cất cánh/hạ cánh](docs/TAKEOFF_LANDING_DESIGN.md)
- [Thiết kế cất cánh trên mặt nghiêng](docs/TILTED_TAKEOFF_DESIGN.md)
- [Audit robustness](docs/ROBUSTNESS_AUDIT.md)
- [Audit toàn hệ thống](docs/FINAL_SYSTEM_AUDIT.md)
- [Sổ tay thông số](docs/PARAMETER_HANDBOOK.md)
- [Kế hoạch kiểm thử/tuning lặp lại](docs/TEST_PLAN.md)

`PARAMETER_HANDBOOK.md` là nguồn tra cứu giá trị/symbol hiện hành;
`TEST_PLAN.md` là cổng bắt buộc trước mọi thử nghiệm phần cứng. Trạng thái task
phần mềm không tự chứng minh một airframe đã an toàn hoặc tune xong.

## Phần cứng và chân nối

| Thiết bị | ESP32 |
|---|---|
| ESC M1 (front-left, CW) | GPIO26 |
| ESC M2 (front-right, CCW) | GPIO13 |
| ESC M3 (back-right, CW) | GPIO14 |
| ESC M4 (back-left, CCW) | GPIO27 |
| MPU6050 SDA / SCL | GPIO32 / GPIO33 |
| BMP388 SDA / SCL | GPIO21 / GPIO22 |
| OLED SSD1306 128×64 SDA / SCL | GPIO21 / GPIO22 (dùng chung bus với BMP388) |
| MTF01P TX → ESP32 RX | GPIO19 |
| MTF01P RX ← ESP32 TX | GPIO23 |

MTF01P phải được đặt ở protocol **Micolink**, baud 115200. Tất cả thiết bị phải
chung GND. Firmware nhắm tới ESP32 classic, board Arduino `ESP32 Dev Module`.

Thư viện cần có: `Adafruit BMP3XX`, `Adafruit Unified Sensor`, `Adafruit GFX`
và `Adafruit SSD1306`. OLED địa chỉ mặc định `0x3C`, được cập nhật 4 Hz trong
task riêng khi DISARMED/ARMED_IDLE; lúc bay không truyền I2C lên OLED để không
chặn đường BMP388/MTF. OLED và BMP388 dùng mutex chung; BMP chờ bus tối đa 2 ms,
OLED bỏ một lần refresh nếu bus đang bận.

### Năm thông số test trên OLED

OLED dành 5 dòng đầu cho flight data và 2 dòng cuối để nói rõ nguyên nhân cùng
hành động hiện tại. Vì vậy khi hiện `FAIL`/`SAFE`, không cần đoán lỗi chỉ từ tên
mode.

| Dòng OLED | Ý nghĩa |
|---|---|
| `HOLD L T:1430` | Mode, link (`L` tốt/`X` mất) và throttle µs |
| `Z:0.48 SP:0.50` | Khoảng hở đáy drone–mặt đất sau khi trừ 0,12 m và setpoint |
| `VZ:+0.02 SRC:F` | Vận tốc đứng và nguồn (`F` fusion, `M` MTF, `B` BMP, `C` coast) |
| `F:+0.01/-0.02 Q:120` | Flow forward/right, m/s và quality |
| `P:+0.03/-0.01 H` | Drift forward/right, mét; `H` là flow hold đang có dữ liệu |
| `WHY:MTF LOST/STALE` | Nguyên nhân trực tiếp khiến mode đổi |
| `ACT:SLOW LAND` | Firmware đang làm gì hoặc người vận hành cần làm gì |

OLED chỉ là kênh quan sát nhanh. Gói telemetry v5 mang dữ liệu chi tiết hơn,
nhưng sender `STATUS` chỉ in một tập con. Các phép thử cần yaw/rates, output/I
của controller, bias, rung/clip, độ cao riêng từng sensor hoặc counter radio
phải dùng packet decoder/diagnostic build có định danh theo
`docs/TEST_PLAN.md`; không được suy ra các trường chưa in.

## Thiết kế điều khiển

Luồng cao độ:

```text
MTF01P → CRC/freshness/status/strength/range gate → median/EMA → height/Vz ┐
                                                                            ├→ observer fusion
BMP388 → P0 tại mặt đất → IIR → height/Vz → align drift chậm ───────────────┘
       → position P → acceleration-limited velocity PI → two-way PWM slew
MTF01P flow X/Y → status/quality/height gate → zero-bias
               → scale từng sample theo range hiện tại sang m/s → median/EMA
               → tích phân từ mốc calib → sai số vị trí theo mét
               → velocity PI → pitch/roll SP
MPU6050 → attitude/rate controller ← pitch/roll SP → mixer saturation → 4 ESC
```

- MTF01P là nguồn chính và được bù nghiêng theo pitch/roll thời gian thực. Vận
  tốc MTF dùng hồi quy 15 mẫu rồi qua low-pass để không biến noise khoảng cách
  thành xung ga.
- Khi cả hai cảm biến tốt, BMP388 đóng góp 10% cho cao độ và 30% cho vận tốc;
  offset barometer chỉ học trên sample mới với hằng số thời gian 8 s. Nếu hai
  nguồn lệch quá 0,45 m, BMP bị loại khỏi fusion thay vì kéo sai MTF. Mất cả hai
  ngắn dưới 600 ms dùng trạng thái `COAST`; mất MTF quá 800 ms trong TAKEOFF/HOLD
  thì chuyển sang hạ cánh bằng BMP nếu còn tốt.
- Khi nhận TAKEOFF, base throttle bắt đầu từ lệnh hiện tại (bình thường là
  `1050 us` ở ARMED_IDLE) và tăng liên tục `40 us/s`. Mốc
  `TAKEOFF_START_US = 1350 us` chỉ phân tách `SPOOL_UP` với
  `GROUND_TRANSITION`, không còn là bước nhảy lệnh. Drone chỉ chuyển sang
  closed-loop sau khi điều kiện rời đất tồn tại liên tục 0,15 s; nếu vẫn chưa
  rời đất, ramp dừng ở hover thay vì tích lũy PI/setpoint trên nền. Setpoint sau
  khi xác nhận rời đất tăng tối đa 0,12 m/s; velocity demand bị giới hạn
  +0,20/-0,12 m/s và được slew bằng 0,20/0,35 m/s² để tránh bay vọt.
- Cất cánh trên mặt nghiêng nhẹ chỉ được hỗ trợ khi IMU đã calibrate trên mặt
  phẳng chuẩn rồi nghiêng fixture tại cùng vị trí ngang trước ARM; không chuyển
  drone sang một điểm ngang mới vì mốc position đã khóa tại calib. TAKEOFF chấp nhận
  tối đa 8° mỗi trục, giữ attitude đã capture trong `SPOOL_UP`, rồi slew về trim
  3°/s trong `GROUND_TRANSITION`. Trước liftoff, từng motor bị chặn không quá
  base+50 us và không dùng angle boost; sau xác nhận liftoff, output từng motor
  crossfade sang mixer bình thường trong 0,50 s. Vi phạm liên tục 0,10 s ở 12°
  hoặc 45°/s sẽ abort qua motor ramp của Task 3.
- Optical flow chỉ được dùng khi status `0x01`, quality ≥ 40, range hợp lệ và
  cảm biến cách nền ít nhất 50 mm. Raw flow được trừ zero-bias, nhân vertical
  range đồng bộ của chính sample để đổi từ `cm/s @ 1 m` sang `m/s`, sau đó mới
  lọc median/EMA và map thành vận tốc `forward/right`. Thứ tự này giữ state bộ
  lọc ở đơn vị SI khi độ cao thay đổi. Deadband từng trục là
  `max(1,5 cm/s, 3σ)` từ noise đo lúc calibration, giới hạn ở 12 cm/s. Vòng giữ
  vị trí chỉ bắt đầu khi drone cao ít nhất 0,10 m, flow tốt liên tục 0,20 s và
  chỉ yêu cầu góc nghiêng tối đa 3,5°. Lệnh góc đổi tối đa 8°/s; quality vừa chạm
  ngưỡng được 40% thẩm quyền và đạt 100% tại quality 70. Đây là mapping hiện có,
  chưa phải mapping đã được xác nhận bằng dữ liệu bay.
- Mốc ngang `(0,0)` được khóa một lần sau khi calibration altitude/flow hoàn
  tất và được giữ cho cả phiên cấp nguồn. Odometry vẫn chạy khi DISARMED, vì vậy
  không được nhấc/di chuyển drone sau calib nếu muốn đúng mốc ban đầu. Khi flow
  stale/kém chất lượng, position hold được tắt và pitch/roll trở về trim nhưng
  mốc calib cùng vị trí ước lượng cuối không bị xóa. Khi flow tốt lại liên tục
  0,20 s, controller trở lại mốc cũ, không âm thầm tạo mốc mới.
- Mọi đường hạ cánh có điều khiển (`LAND`, mất link/range, timeout, quá cao)
  đều giảm setpoint 0,08 m/s ở trên 0,40 m. Từ 0,40 m trở xuống chỉ còn 0,04 m/s.
  PWM landing được slew cả hai chiều: tăng tối đa 55 µs/s, giảm 50 µs/s và gần
  đất giảm 35 µs/s. Ga landing bị chặn ở `min(hover + 50, 1470) us`, nên một
  spike vận tốc không thể bật tức thời lên vùng 1500 us.
- Khi MTF fresh báo khoảng hở dưới 0,10 m, landing latch
  `TOUCHDOWN_DETECTION`: bỏ quyền controller/mixer và giảm riêng từng motor về
  1050 us, tuyệt đối không tăng motor nào. Cao độ/vận tốc ổn định liên tục
  0,40 s hoặc timeout 2 s chuyển sang `MOTOR_RAMP_DOWN`; bốn output sau đó nội
  suy về 1000 us trong 1 s rồi DISARM. Nếu mất cả hai cảm biến trước touchdown,
  base throttle hạ hở vòng 25 µs/s và khóa sau 8 s.
- MTF đo từ sensor tới nền. Firmware trừ cố định
  `MTF_SENSOR_TO_DRONE_BOTTOM_M = 0.12 m`, nên target 0,50 m là khoảng cách từ
  đáy drone tới nền; raw range MTF tại target sẽ xấp xỉ 0,62 m khi drone phẳng.

## State machine an toàn

```text
DISARMED --ARM + pre-arm 2 s--> ARMED_IDLE
  --TAKEOFF--> SPOOL_UP --> GROUND_TRANSITION
  --rời đất xác nhận 0,15 s--> ATTITUDE_TRANSFER 0,50 s + CONTROLLED_ASCENT
  --> ALTITUDE_CAPTURE --> ALT_HOLD

TAKEOFF/ALT_HOLD --LAND hoặc failsafe-->
  CONTROLLED_DESCENT --> GROUND_APPROACH --> TOUCHDOWN_DETECTION
  --> MOTOR_RAMP_DOWN --> DISARMED

Abort trước rời đất ---------------------> MOTOR_RAMP_DOWN --> DISARMED
Bất kỳ state nào --KILL/sai số góc/lỗi MPU--> TRIPPED (chỉ RESET mới mở khóa)
```

Tên bên trái vẫn là `FlightMode` công khai; các tên dài là transition phase nội
bộ. Sender in telemetry theo dạng `[MODE/PHASE]` để thấy chính xác bước đang chạy.

Firmware không tự ARM. Pre-arm bắt buộc có link hợp lệ, IMU đã calibrate,
BMP388 + MTF01P range/flow còn fresh, mốc position đã khởi tạo, drone gần mặt
đất, nằm ngang, không rung/clip và không
chuyển động. `ARMED_IDLE` tự DISARM sau 15 s nếu không nhận TAKEOFF.

Các đường failsafe chính:

| Sự cố | Hành động |
|---|---|
| Mất link > 3 s khi bay | Hạ cánh, không tự resume |
| Mất MTF01P > 800 ms | Hạ cánh bằng BMP388 nếu còn tốt |
| Mất cả MTF và BMP | Hạ throttle hở vòng, khóa sau 8 s |
| Cao quá 1,60 m | Hạ cánh |
| Không xác nhận rời đất sau 4 s trong `GROUND_TRANSITION` | Hủy takeoff và ramp motor xuống |
| Takeoff quá 20 s | Hạ cánh |
| Chuyến bay quá 60 s | Hạ cánh |
| Sai số pitch/roll > 45° trong 0,25 s | Safety trip |
| Mất MPU6050 kéo dài | Safety trip |

### Giải thích `WHY` và `ACT` trên OLED

| OLED `WHY` | Sai ở đâu | Kiểm tra trước tiên |
|---|---|---|
| `LINK LOST` | Không nhận heartbeat sender quá 3 s | Nguồn sender, MAC, channel, PMK/LMK |
| `MTF LOST/STALE` | Không có frame MTF mới quá 800 ms | UART 19/23, Micolink 115200, dây/GND |
| `FLOW NOT READY` / `POS ORIGIN WAIT` | Flow chưa fresh hoặc control chưa khóa được mốc calib | Nền/ánh sáng/quality/age; giữ nguyên vị trí sau calib |
| `OVER ALT 1.60M` | Cao độ đã vượt trần an toàn | Offset 12 cm, dấu/range MTF, tune altitude |
| `NO LIFT` | `GROUND_TRANSITION` hết 4 s mà chưa xác nhận rời đất | Pin, cánh, chiều motor, hover throttle |
| `TAKEOFF TIMEOUT` | Không đạt điều kiện HOLD trong 20 s | Gain altitude, rung và chất lượng range |
| `FLIGHT TIMEOUT` | Chuyến bay vượt 60 s | Đây là giới hạn phiên test, không phải sensor hỏng |
| `NO ALT SENSOR` | Mất cả nguồn cao độ khi đang hạ | MTF và BMP388; firmware hạ ga hở vòng |
| `LAND TIMEOUT` | Hạ cánh chưa xác nhận được trong 24 s | MTF gần đất, `VZ`, offset và motor còn đẩy |
| `ATTITUDE ERROR` | Sai số pitch/roll >45° quá 0,25 s | Chiều motor/cánh, mixer sign, PID attitude |
| `ATTITUDE INVALID` | Estimator attitude/rate tạo NaN hoặc Inf | MPU, nguồn, I2C và rung/clip |
| `MPU LOST` | Đọc MPU6050 lỗi liên tục | Dây SDA/SCL 32/33, nguồn và địa chỉ `0x68` |
| `KILL COMMAND` | Sender đã gửi lệnh `9`/`KILL YES` | Phải reset board trước khi chạy lại |

`ACT:SLOW LAND` nghĩa firmware đang hạ có điều khiển; `ACT:RESET REQUIRED`
nghĩa đã vào `TRIPPED` và không thể mở khóa bằng lệnh; `ACT:CHECK THEN 1=ARM`
nghĩa đang DISARMED và phải xử lý nguyên nhân `WHY` trước khi ARM.

Landing timeout là 24 s để đủ thời gian đi qua profile 0,08/0,04 m/s;
takeoff timeout là 20 s để chứa profile lên chậm 0,12 m/s. Nhánh no-rise trên
đất vẫn giữ 4 s, nên timeout dài hơn không làm drone giữ ga ground lâu hơn.
Mô hình danh định từ 0,50 m tới xác nhận/ramp disarm cần khoảng 10,15 s.
`KILL`, lỗi MPU hoặc sai số attitude nghiêm trọng vẫn dùng `TRIPPED` và
không đi qua profile hạ chậm, vì đây là nhánh dừng khẩn cấp.

Gói điều khiển ESP-NOW là unicast mã hóa bằng PMK/LMK, binary 12 byte có magic,
version, sequence và CRC-16/CCITT. Chỉ gói đúng peer + đúng CRC mới refresh watchdog link. Sender gửi
lặp action trong 1,2 s với cùng sequence; drone chỉ thi hành một lần. Sender
khởi động chỉ phát heartbeat và không mang lệnh ARM cũ.

Telemetry đang ở version 5, dài 248 byte và có transition phase, vận tốc
flow, vị trí tích phân, góc hiệu chỉnh position hold, quality/age cùng chẩn đoán
timing Task 2. Hai sketch phải được nạp cùng phiên bản.

### Robustness được đưa vào ở Task 5

- Checkpoint Task 5 dùng `cfg=0xA41F0501`, Task 6 dùng `0xA41F0601`; build
  bù CG/profile êm hiện hành phải báo `cfg=0xA41F1001`. Mailbox lệnh giữa
  ESP-NOW task và control
  dùng publish/claim atomic; action đang chờ không bị action thường khác ghi
  đè, còn `KILL` luôn có quyền ưu tiên. Action bị bận mailbox sẽ được lần gửi
  lặp cùng sequence thử lại.
- Sender đánh dấu một gói là pending trước khi gọi `esp_now_send()`. Nếu callback
  chậm quá 100 ms, sender in `callback gui cham` và chờ callback; nó không tự
  xóa pending để gửi chồng gói, vì callback cũ có thể xóa nhầm trạng thái gói
  mới. Nếu callback thực sự mất, watchdog mất link của drone vẫn là fallback.
- Control chỉ nhận altitude snapshot sau một lần đọc seqlock coherent. Bốn lần
  va chạm liên tiếp giữ bản coherent cuối; quá 100 ms không có publication mới
  thì snapshot bị đánh dấu stale. NaN/Inf ở attitude, rate, PID, tilt hoặc fused
  altitude không được đi tiếp như một measurement hợp lệ.
- ESC bắt đầu phát idle 1000 us trước khi tạo task. Nếu không tạo được AUX mutex
  hoặc OLED task, OLED bị tắt và Serial báo degraded; BMP vẫn chạy với một owner.
- Firmware hiện chỉ đo `dt`, max `dt` và overrun. Không có ngưỡng trip control
  deadline mới: stack high-water, scheduler jitter và response vật lý khi stall
  vẫn **UNKNOWN** cho tới khi có log prop-off/giá thử và chuyến bay giới hạn.

Những thay đổi này không đổi gain, fusion weight, sensor threshold, takeoff,
landing, position-hold, mixer limit hay giá trị motor của Tasks 3–4. Compile
thành công không thay thế kiểm tra phần cứng và flight test theo checklist.

### Audit toàn hệ thống Task 6

Build Task 6 cũ báo `cfg=0xA41F0601`; build 0,50 m hiện hành phải báo
`cfg=0xA41F1001`. Task 6 sửa hai lỗi fail-safe tĩnh mà không đổi
gain hoặc threshold bay:

- MPU chỉ được coi là khởi tạo thành công nếu đọc lại đủ bốn thanh ghi cấu hình
  và đúng với scale ±4 g/±500°/s mà estimator sử dụng. Lỗi đọc hoặc mismatch sẽ
  dừng ở startup trong khi ESC vẫn giữ 1000 us.
- Nếu đang LANDING/FAILSAFE, lệnh LAND lặp không reset phase hay timeout 24 s
  và không được hạ FAILSAFE thành LANDING. Một sự cố mới chỉ có thể nâng LANDING
  lên FAILSAFE mà vẫn giữ tiến trình hạ hiện tại.

Audit không tìm thấy lỗi CRITICAL còn lại bằng kiểm tra tĩnh, nhưng điều đó
không chứng minh an toàn bay. Battery failsafe, hardware kill độc lập, motor/
ESC feedback, persistent logger và redundant IMU vẫn chưa có. Timing/stack,
sensor delay/confidence, thrust/mixer, các gain và toàn bộ đáp ứng Tasks 3–4
trên phần cứng vẫn **UNKNOWN** cho tới khi chạy đúng chuỗi test bên dưới.

## Cấu hình trước khi nạp

Kiểm tra MAC ở cả hai file:

- `peerMac` trong flight controller phải là MAC của sender.
- `peerMac` trong sender phải là MAC của drone.
- `WIFI_CHANNEL` phải giống nhau.
- `espnowPmk` và `espnowLmk` phải giống từng byte. Hai khóa mẫu trong project
  chỉ phục vụ bring-up; hãy thay bằng 16 byte riêng trên cả hai firmware.

`HOVER_DEFAULT = 1430 us` là feed-forward hiện tại, nằm giữa quan sát rằng khung
đã bắt đầu nâng quanh 1400 us và leo khá mạnh ở 1500 us. Nó chưa phải giá trị
đã calibrate và không thể coi là đúng cho mọi
pin/cánh/khối lượng. Khi drone đang DISARMED, dùng `HOVER n` trên sender để đổi
base throttle trong RAM. Không tune giá trị này lần đầu bằng chuyến bay tự do.

### Bù trọng tâm lệch do pin

Cấu hình `0xA41F1001` không còn dùng angle trim để che tải lệch:
`PITCH_TRIM_DEG/ROLL_TRIM_DEG = 0/0°`. Angle trim khác 0 yêu cầu thân drone
nghiêng liên tục và tự tạo gia tốc ngang, nên không phải cách đúng để sửa CG.

Với pin lệch về phía mũi như khung hiện tại,
`CG_PITCH_COMP_US_AT_HOVER = +15 us` tạo moment feed-forward: tăng đều M1/M2
(hai motor trước), giảm đều M3/M4 (hai motor sau), không đổi PWM trung bình.
Compensation bằng 0 tại `MOTOR_MIN_US=1050`, tăng tuyến tính theo collective và
đạt 15 us tại hover; `CG_COMP_MAX_US=30` là hard bound. PID vẫn xử lý nhiễu động
và sai số còn lại, nhưng không phải chờ thân đã chúi rồi mới bắt đầu phản ứng.

`+15 us` là giá trị khởi đầu bảo thủ theo mô tả pin lệch trước, chưa phải kết quả
đo thrust/CG. Cách sửa ưu tiên vẫn là dịch pin để trọng tâm nằm tại giao điểm
bốn motor. Nếu không thể sửa cơ khí, tune compensation trên giá thử có dây giữ,
mỗi lần tối đa 2 us; dấu dương phải làm M1/M2 lớn hơn M3/M4 khi P/R và yaw gần
0. Nếu khung chúi mạnh hơn, dấu/mapping đang sai và phải KILL, không tăng gain.

`MTF_SENSOR_TO_DRONE_BOTTOM_M = 0.12f` phải đúng với khoảng cách lắp thực tế từ
mặt đo của MTF01P tới điểm thấp nhất của drone. Nếu cơ khí thay đổi, đo lại và
sửa hằng số này trước khi kiểm tra landing.

### Mapping trục optical flow

Trong `drone_alt_hold_1m.ino`, ba cấu hình sau đổi trục sensor sang hệ thân:

```cpp
#define FLOW_SWAP_XY 0
static const float FLOW_X_SIGN = +1.0f;
static const float FLOW_Y_SIGN = +1.0f;
```

Sau mapping, `+forward` phải là drone dịch về phía mũi và `+right` phải là dịch
sang phải. Không đoán mapping theo hình dáng bo mạch: tháo cánh, mở `STATUS`,
dịch drone thẳng về trước rồi sang phải. Đổi `FLOW_SWAP_XY` nếu lẫn trục, đổi
sign tương ứng nếu vận tốc ngược dấu. Chỉ thử position hold khi hai dấu đúng.

### Mốc calib và chuẩn hóa theo nhiều độ cao

`positionForwardM/positionRightM` là odometry tương đối theo hệ ngang tại thời
điểm calibration hoàn tất. Setpoint ngang luôn là `(0,0)`; controller dùng trực
tiếp sai số theo mét với `POS_HOLD_KP_PER_S = 0.40`. Không có deadband hay bán
kính cho phép 50 cm. Độ lớn velocity setpoint ngang vẫn bị giới hạn 0,35 m/s,
tilt vẫn ±3,5° và slew vẫn 8°/s để tránh đổi độ chính xác thành dao động.

Scale flow luôn dùng khoảng cách thẳng đứng từ mặt sensor tới nền ở sample hiện
tại, không dùng `ALT_TARGET_M` và không dùng clearance đã trừ offset 12 cm.
Mọi tầng sau đó dùng `m/s`, `m`, `s`, `deg`, nên cùng một bộ gain có ý nghĩa
như nhau ở các target cao độ khác nhau trong envelope đã kiểm chứng của sensor.
Chi tiết, ledger và phép thử bắt buộc nằm tại
[Chuẩn hóa flow/position hold theo độ cao](docs/POSITION_HOLD_HEIGHT_NORMALIZATION.md).

## Trình tự vận hành

1. Tháo cánh quạt, nạp hai sketch và xác nhận đúng MAC/channel.
2. Đặt drone trên mặt phẳng chuẩn, MTF hướng xuống nền có texture; bật sender
   trước rồi bật drone. Không di chuyển drone trong lúc IMU và altitude
   calibration. Nếu thử mặt nghiêng nhẹ, nghiêng fixture tại cùng vị trí ngang
   sau calibration và trước ARM; không reboot trên mặt nghiêng vì offset lúc
   boot sẽ che mất độ nghiêng thật.
3. Chờ telemetry báo `DISARMED`, `src=FUSED`, `FLOW=OK`, sensor age nhỏ, mốc
   position đã khóa và cao độ gần 0.
4. Gõ `1` (hoặc `ARM`); chờ `ARMED_IDLE`. Không có TAKEOFF trong 15 s thì
   drone tự disarm.
5. Chỉ sau các thử nghiệm bên dưới mới gõ `2` (hoặc `TAKEOFF`). Gõ `3`/`LAND`
   để kết thúc.
6. `DISARM` chỉ có hiệu lực ở ARMED_IDLE/đã ở đất. `KILL YES` là lệnh khẩn cấp
   tắt motor và khóa firmware đến khi reset.

Các lệnh chữ cũ vẫn hoạt động; mapping số giúp thao tác nhanh trên Serial
Monitor (115200 baud, gửi kèm newline):

| Số nhanh | Lệnh chữ tương đương | Tác dụng |
|---:|---|---|
| `0` | `STATUS` | In telemetry mới nhất |
| `1` | `ARM` | Bắt đầu pre-arm 2 s |
| `2` | `TAKEOFF` | Ramp ga rồi giữ 0,50 m + vị trí ngang |
| `3` | `LAND` | Hạ cánh có điều khiển |
| `4` | `DISARM` | Chỉ dùng ở ARMED_IDLE/đã ở đất |
| `5 1400` | `HOVER 1400` | Đổi hover khi DISARMED |
| `9` | `KILL YES` | Tắt khẩn cấp và khóa tới khi reset |
| `?` | `HELP` | In bảng lệnh |

`9` có tác dụng ngay như kill-switch phần mềm; không dùng nó để hạ cánh thông
thường.

### Lỗi có thể gặp và cách khắc phục trong quá trình vận hành

Theo checklist thử nghiệm, nếu quy trình bị dừng hoặc drone không vào được
một state mong muốn, cần phân loại lỗi theo nơi xuất hiện và khắc phục trước
khi bay có cánh:

- **Sai thứ tự motor/chiều quay motor/attitude sign:** kiểm tra không cánh và
  đồng bộ thứ tự motor, chiều quay, dấu của góc tiêu cực/tiêu cực trên
  telemetry. Nếu attitude sign sai, đổi sign trong controller; nếu motor chạy
  ngược chiều, đổi ESC/đổi mapping motor trong mixer hoặc đổi cặp motor lắp
  sai.
- **Telemetry, CRC, packet, peer/ESP-NOW không khớp:** kiểm tra lại `peerMac`,
  `WIFI_CHANNEL`, `espnowPmk`, `espnowLmk`, version của cả hai sketch. Lặp lại
  kiểm tra link và xóa action cũ. Sender phải khởi động trước drone và phát
  heartbeat không có lệnh ARM cũ.
- **Không vào ARMED_IDLE hoặc ARM/DISARM không thực hiện:** kiểm tra link hợp
  lệ, IMU đã calibrate, BMP388+MTF01P range/flow còn fresh, mốc position đã
  khóa, drone cách mặt đất và nằm
  ngang. Nếu `ARMED_IDLE` timeout 15 s mà không vào TAKEOFF, drone tự DISARM;
  không gõ `TAKEOFF` khi pre-arm chưa xong.
- **MTF/BMP mất hoặc stale:** nếu rút riêng từng sensor, xác nhận firmware
  chuyển đúng sang failsafe (hạ cánh bằng BMP388 nếu còn tốt; mất cả hai cảm
  biến thì base throttle hạ hở vòng và khóa sau 8 s). Rà lại `rangeAge` và
  `baroAge`, khôi phục nguồn/đặt sensor đúng protocol MTF01P.
- **Optical flow dấu sai/trục lẫn:** dùng `0`/`STATUS`, dịch drone về trước/phải
  và xác nhận `vFR` dương theo trục tương ứng. Nếu vận tốc ngược dấu, sửa
  `FLOW_SWAP_XY`, `FLOW_X_SIGN`, `FLOW_Y_SIGN` trước khi chạy position hold.
- **Quality thấp, age lớn, flow không `FLOW=OK`:** xác nhận nền có texture,
  độ cao đủ lớn hơn 50 mm, ánh sáng hợp lý, và không có rung/downwash. Gate code
  là quality 40; trong bài test ban đầu nên tìm biên thoải mái (ví dụ ≥80) thay
  vì bay sát gate. Nếu quality < 40 hoặc age lớn, position hold nhả quyền nhưng
  vẫn giữ tọa độ cuối theo mốc calib; chỉ tiếp tục thử khi flow lại tốt.
- **Takeoff không lên hoặc ramp không đúng:** tìm `HOVER_DEFAULT`, xác nhận lệnh
  bắt đầu từ output hiện tại (bình thường 1050 us), đi qua phase boundary
  `TAKEOFF_START_US = 1350 us` và tăng khoảng 40 us/s tới hover. Một frame
  `z >= 0,06 m`/Vz dương không đủ: điều kiện phải liên tục 0,15 s. Nếu tổng PWM tiếp cận trần
  hoặc mixer saturation, hạ `HOVER_DEFAULT`/giảm tải và kiểm tra khối lượng,
  cánh, pin trước khi bay.
- **Một motor tăng mạnh khi bắt đầu trên mặt nghiêng:** xác nhận IMU đã calibrate
  trên mặt phẳng trước khi chuyển drone, tilt báo cáo không quá 8°, và trước
  liftoff không motor nào vượt `throttle + 50 us`. Tilt vượt 8° phải bị từ chối;
  tilt/rate vượt 12°/45°/s liên tục 0,10 s phải abort. Không tăng các giới hạn
  này nếu chưa có log/giá thử cho thấy false abort.
- **Chúi về trước khi vừa rời đất:** trước tiên cân lại vị trí pin. Sau đó xác
  nhận startup in `CG COMP @hover P/R=+15.0/+0.0us`, tại 1050 us bốn motor vẫn
  bằng nhau và gần hover M1/M2 cao hơn M3/M4 khoảng 30 us theo từng cặp. Chỉ
  chỉnh `CG_PITCH_COMP_US_AT_HOVER` từng bước 2 us trên giá giữ; không dùng
  `PITCH_TRIM_DEG`, Kp hay Kd để che tải tĩnh.
- **Bay cao/biên độ rung/downwash:** khi tune vòng attitude và altitude trong
  bay buộc dây thấp, chỉ mới tuning `POS_HOLD_KP_PER_S`,
  `POS_HOLD_VEL_KP_DEG_PER_MPS`, `POS_HOLD_VEL_KI_DEG_PER_M` sau khi log cho
  thấy MTF/BMP/flow không timeout dưới rung và downwash.

Những lỗi trên nên được kiểm soát theo thứ tự checklist ở trên để không tiến
sang giai đoạn có cánh khi chưa có bằng chứng ổn định từ phase không cánh và
bay buộc dây thấp.

## Checklist thử nghiệm bắt buộc

Không bỏ qua thứ tự này:

1. **Không cánh:** kiểm tra thứ tự motor, chiều motor, attitude sign, telemetry,
   CRC, ARM/DISARM, ARMED_IDLE timeout và KILL.
2. **Không cánh:** rút nguồn MTF, BMP và tắt sender riêng từng trường hợp; xác
   nhận state/failsafe đúng bảng trên.
3. **Không cánh:** nâng/hạ drone bằng tay và kiểm tra dấu `z`, `vz`, tilt
   correction, spike reject và mốc mặt đất.
4. **Không cánh:** dùng `0`/`STATUS`, dịch drone về trước/phải; xác nhận
   `vFR` lần lượt dương ở đúng trục. Giữ yên phải có `FLOW=OK`, velocity gần 0,
   quality phải qua gate 40 và age nhỏ; nên đạt biên thoải mái, ví dụ ≥80, cho
   bài test ban đầu. Sau dòng `[POS] Moc calib da khoa`, dịch theo khoảng cách
   đo độc lập 0,25 m rồi 0,50 m theo từng trục và đường chéo; `STATUS` phải giữ
   cùng mốc, in đúng độ lệch `r=...m`, và gần về 0 khi đưa khung về vị trí calib.
   Lặp cùng vận tốc/khoảng dịch tại nhiều độ cao dự kiến để xác nhận kết quả
   m/s/m không đổi theo độ cao. Sửa swap/sign trước mọi thử nghiệm có cánh.
5. **Không cánh:** calibrate trên mặt phẳng, sau đó nghiêng fixture tại cùng
   vị trí ngang lần lượt tới góc đo được 0°, 3°, 5° và tối đa 8°. Nếu phải di
   chuyển khung, odometry phải theo dõi đúng đường đi và trở về gần 0 khi trả
   lại vị trí calib.
   Xác nhận capture không nhảy setpoint, target slew
   không quá 3°/s, I-term bằng 0 và mọi motor trước lift nằm trong
   `[1050, throttle+50] us`; tilt lớn hơn 8° phải chặn TAKEOFF.
6. **Giá thử có che chắn:** xác nhận takeoff đi từ output ARMED_IDLE hiện tại,
   tăng khoảng 40 us/s, không nhảy tại 1350 us và chỉ chuyển closed-loop sau
   0,15 s xác nhận liftoff. Tại 1050 us compensation phải bằng 0; khi tăng ga,
   chênh pitch phải tăng trơn tới `+15/-15 us` tại hover và mọi motor vẫn nằm
   trong giới hạn ground mixer. Với `HOVER=1430`, kiểm tra ga altitude không vượt
   1500 và ga landing không vượt trần tuyệt đối 1470. Sau touchdown latch,
   từng motor chỉ được giảm: về 1050 us rồi về 1000 us trong 1 s.
7. **Giá thử nghiêng có che chắn:** chỉ sau bài 0° thành công mới thử 3° rồi 5°.
   Xác nhận không motor nào vượt base+50 us trước lift, abort đi vào ramp giảm
   motor, và handover 0,50 s không tạo bước motor hay tăng I-term.
8. **Bay buộc dây thấp:** tune vòng attitude và altitude trước; sau đó mới tune
   `POS_HOLD_KP_PER_S`, `POS_HOLD_VEL_KP_DEG_PER_MPS` và
   `POS_HOLD_VEL_KI_DEG_PER_M` từ gain thấp. Đánh giá sai số vị trí bằng phép
   đo độc lập, không chỉ dựa vào odometry telemetry.
   Trong profile mới, altitude setpoint takeoff không tăng quá 0,12 m/s, velocity
   setpoint không vượt +0,20/-0,12 m/s; LAND giảm setpoint 0,08 m/s phía trên
   0,40 m và 0,04 m/s gần đất. Abort nếu vận tốc thực tế vượt rõ các demand này
   hoặc xuất hiện chu kỳ nảy/yo-yo tăng dần.
9. Chỉ bay tự do khi log cho thấy MTF/BMP/flow không timeout dưới rung và
   downwash.

Optical flow đo vận tốc tương đối trên nền, không phải vị trí tuyệt đối. Tích
phân lâu sẽ có drift; nền ít texture, thiếu sáng, thay đổi độ cao đột ngột và
yaw trôi (hệ thống không có compass) đều làm độ chính xác vị trí giảm.
