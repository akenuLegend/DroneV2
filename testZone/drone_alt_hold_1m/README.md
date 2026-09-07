# DRONE_ALT_HOLD_1M

Project nâng cấp trực tiếp từ `testZone/drone_pilot/drone_pilot.ino`, giữ mô
hình một file cho mỗi firmware:

- `drone_alt_hold_1m.ino`: toàn bộ flight controller trên drone.
- `sender_1m/sender_1m.ino`: toàn bộ trạm phát ESP-NOW.

Mục tiêu là tự cất cánh đến **1,00 m so với vị trí lúc khởi động** rồi giữ độ
cao. Đây là firmware thử nghiệm đã qua kiểm tra biên dịch, chưa phải một hệ
thống được chứng nhận hay đã tune trên mọi khung drone.

## Phần cứng và chân nối

| Thiết bị | ESP32 |
|---|---|
| ESC M1 (front-left, CW) | GPIO26 |
| ESC M2 (front-right, CCW) | GPIO13 |
| ESC M3 (back-right, CW) | GPIO14 |
| ESC M4 (back-left, CCW) | GPIO27 |
| MPU6050 SDA / SCL | GPIO32 / GPIO33 |
| BMP388 SDA / SCL | GPIO21 / GPIO22 |
| MTF01P TX → ESP32 RX | GPIO19 |
| MTF01P RX ← ESP32 TX | GPIO23 |

MTF01P phải được đặt ở protocol **Micolink**, baud 115200. Tất cả thiết bị phải
chung GND. Firmware nhắm tới ESP32 classic, board Arduino `ESP32 Dev Module`.

Thư viện cần có: `Adafruit BMP3XX`, `Adafruit Unified Sensor`; OLED đang tắt
mặc định để tránh tải và tranh chấp bus với BMP388.

## Thiết kế điều khiển

Luồng cao độ:

```text
MTF01P → CRC/freshness/status/strength/range gate → spike confirm
       → median-5 → EMA → hồi quy vận tốc ┐
                                          ├→ chọn nguồn → position P
BMP388 → P0 tại mặt đất → altitude/velocity┘              → velocity PI
                                                               → base PWM
MPU6050 → attitude/rate controller → mixer có saturation → 4 ESC
```

- MTF01P là nguồn chính và được bù nghiêng theo pitch/roll thời gian thực.
- BMP388 có mốc áp suất đo tại chỗ và chỉ là fallback ngắn hạn. Khi MTF mất quá
  800 ms trong TAKEOFF/HOLD, drone chuyển sang hạ cánh; link phục hồi không tự
  đưa drone trở lại HOLD.
- Khi nhận TAKEOFF, base throttle được ramp mềm từ 1050 us tới hover trong 1 s;
  sau đó setpoint cất cánh tăng tối đa 0,35 m/s. Setpoint hạ cánh giảm 0,22 m/s.
- Landing chỉ tự DISARM khi MTF còn tốt, cao dưới 0,10 m và vận tốc nhỏ liên tục
  0,8 s. Nếu mất cả hai cảm biến, base throttle hạ hở vòng và khóa sau 8 s.
- Altitude target được tính sau khi trừ khoảng cách MTF từ cảm biến tới mặt đất
  lúc khởi động, nên là **độ cao tăng thêm 1,00 m**, không phải raw range 1 m.

## State machine an toàn

```text
DISARMED --ARM + pre-arm 2 s--> ARMED_IDLE --TAKEOFF--> TAKEOFF --> ALT_HOLD
    ^                              |                       |          |
    |                              +--timeout/mất link-----+----------+
    |                                                         |
    +---------------------- LANDING / FAILSAFE <---------------+

Bất kỳ state nào --KILL/sai số góc/lỗi MPU--> TRIPPED (chỉ RESET mới mở khóa)
```

Firmware không tự ARM. Pre-arm bắt buộc có link hợp lệ, IMU đã calibrate,
BMP388 + MTF01P còn fresh, drone gần mặt đất, nằm ngang, không rung/clip và
không chuyển động. `ARMED_IDLE` tự DISARM sau 15 s nếu không nhận TAKEOFF.

Các đường failsafe chính:

| Sự cố | Hành động |
|---|---|
| Mất link > 3 s khi bay | Hạ cánh, không tự resume |
| Mất MTF01P > 800 ms | Hạ cánh bằng BMP388 nếu còn tốt |
| Mất cả MTF và BMP | Hạ throttle hở vòng, khóa sau 8 s |
| Cao quá 1,60 m | Hạ cánh |
| Không tăng > 0,15 m sau 4 s | Hủy takeoff và hạ/disarm |
| Takeoff quá 15 s | Hạ cánh |
| Chuyến bay quá 60 s | Hạ cánh |
| Sai số pitch/roll > 45° trong 0,25 s | Safety trip |
| Mất MPU6050 kéo dài | Safety trip |

Gói điều khiển ESP-NOW là unicast mã hóa bằng PMK/LMK, binary 12 byte có magic,
version, sequence và CRC-16/CCITT. Chỉ gói đúng peer + đúng CRC mới refresh watchdog link. Sender gửi
lặp action trong 1,2 s với cùng sequence; drone chỉ thi hành một lần. Sender
khởi động chỉ phát heartbeat và không mang lệnh ARM cũ.

## Cấu hình trước khi nạp

Kiểm tra MAC ở cả hai file:

- `peerMac` trong flight controller phải là MAC của sender.
- `peerMac` trong sender phải là MAC của drone.
- `WIFI_CHANNEL` phải giống nhau.
- `espnowPmk` và `espnowLmk` phải giống từng byte. Hai khóa mẫu trong project
  chỉ phục vụ bring-up; hãy thay bằng 16 byte riêng trên cả hai firmware.

`HOVER_DEFAULT = 1470 us` được lấy từ mức lift trong profile thử nghiệm cũ,
không thể coi là đúng cho mọi pin/cánh/khối lượng. Khi drone đang DISARMED, dùng
`HOVER n` trên sender để đổi base throttle trong RAM. Không được tune giá trị
này lần đầu bằng chuyến bay tự do.

## Trình tự vận hành

1. Tháo cánh quạt, nạp hai sketch và xác nhận đúng MAC/channel.
2. Đặt drone nằm ngang, MTF hướng xuống nền có texture; bật sender trước rồi
   bật drone. Không di chuyển drone trong lúc IMU và altitude calibration.
3. Chờ telemetry báo `DISARMED`, `src=MTF`, `rangeAge`/`baroAge` nhỏ và cao độ
   gần 0.
4. Gõ `ARM`; chờ `ARMED_IDLE`. Không có `TAKEOFF` trong 15 s thì drone tự disarm.
5. Chỉ sau các thử nghiệm bên dưới mới gõ `TAKEOFF`. Gõ `LAND` để kết thúc.
6. `DISARM` chỉ có hiệu lực ở ARMED_IDLE/đã ở đất. `KILL YES` là lệnh khẩn cấp
   tắt motor và khóa firmware đến khi reset.

Các lệnh sender:

```text
HELP
STATUS
HOVER 1470
ARM
TAKEOFF
LAND
DISARM
KILL YES
```

## Checklist thử nghiệm bắt buộc

Không bỏ qua thứ tự này:

1. **Không cánh:** kiểm tra thứ tự motor, chiều motor, attitude sign, telemetry,
   CRC, ARM/DISARM, ARMED_IDLE timeout và KILL.
2. **Không cánh:** rút nguồn MTF, BMP và tắt sender riêng từng trường hợp; xác
   nhận state/failsafe đúng bảng trên.
3. **Không cánh:** nâng/hạ drone bằng tay và kiểm tra dấu `z`, `vz`, tilt
   correction, spike reject và mốc mặt đất.
4. **Giá thử có che chắn:** tìm `HOVER_DEFAULT`, kiểm tra mixer saturation và
   xác nhận tổng PWM không chạm trần liên tục.
5. **Bay buộc dây thấp:** tune vòng attitude trước, sau đó mới tune
   `ALT_POS_KP`, `ALT_VEL_KP_US_PER_MPS`, `ALT_VEL_KI_US_PER_M`.
6. Chỉ bay tự do khi log cho thấy MTF/BMP không timeout dưới rung và downwash.

Project chỉ giữ độ cao; dữ liệu optical-flow X/Y của MTF01P chưa được dùng để
giữ vị trí ngang. Vì không có compass, yaw tuyệt đối vẫn có thể trôi chậm.
