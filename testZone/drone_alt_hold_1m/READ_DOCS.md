# Hướng dẫn đọc tài liệu hệ thống drone

Tài liệu này trình bày thứ tự nên đọc các file để hiểu từ yêu cầu, các lỗi bay
thực tế, thuật toán điều khiển, cách tổ chức code cho tới quy trình kiểm thử và
hoàn thiện hệ thống drone.

## 1. Tổng quan và cách vận hành

### 1.1. `README.md`

Đọc [README.md](README.md) đầu tiên để nắm được:

- Phần cứng, mapping motor và cảm biến.
- Các flight mode.
- Cách sử dụng ARM, TAKEOFF, LAND và KILL.
- Cấu hình firmware hiện tại `0xA41F1001`.
- Các giới hạn an toàn và checklist vận hành cơ bản.

### 1.2. `FLIGHT_CONTROL_ARCHITECTURE.md`

Tiếp theo, đọc
[FLIGHT_CONTROL_ARCHITECTURE.md](docs/FLIGHT_CONTROL_ARCHITECTURE.md). Đây là
tài liệu quan trọng nhất để hiểu toàn hệ thống:

- Task nào chạy trên core nào.
- Tần số IMU, control loop, altitude task và telemetry.
- Đường dữ liệu từ sensor tới motor.
- Attitude estimator, PID cascade, altitude controller và position hold.
- Mixer, state machine và các nhánh failsafe.

Luồng xử lý tổng thể:

```text
MPU6050 ──> Madgwick ──> pitch/roll/rate
                         │
MTF01P ──> range + flow ─┼─> altitude/position setpoint
BMP388 ──> barometer ────┘
                         │
                         v
     Position controller + Altitude controller
                         │
                         v
        Angle controller → Rate PID
                         │
            CG feed-forward
                         │
                         v
                   X-frame mixer
                         │
                         v
                    M1 M2 M3 M4
```

## 2. Hiểu vấn đề thực tế và lý do thiết kế

### 2.1. `FLIGHT_LOGS_ANALYSIS.md`

Đọc [FLIGHT_LOGS_ANALYSIS.md](FLIGHT_LOGS_ANALYSIS.md) để xem các lần bay lỗi
thực tế, bao gồm:

- Drone vọt cao.
- Rơi hoặc dao động yo-yo.
- Trôi ngang.
- Optical flow bị mất.
- Drone nảy trở lại khi landing.

### 2.2. `FLIGHT_ISSUES.md`

Sau đó đọc [FLIGHT_ISSUES.md](docs/FLIGHT_ISSUES.md). Tài liệu này chuyển các
hiện tượng trong log thành:

- Nguyên nhân có thể hoặc đã được xác nhận.
- Mức độ tin cậy của kết luận.
- Đoạn code và tham số liên quan.
- Thay đổi được đề xuất.
- Cách kiểm chứng kết luận.

Hai tài liệu trên giúp giải thích vì sao code hiện tại cần state machine, các
giới hạn motor, anti-windup, slew limiter và nhiều nhánh an toàn.

## 3. Đi sâu vào từng hệ thống điều khiển

### 3.1. Sensor và hợp nhất độ cao

Đọc [SENSOR_FUSION_ANALYSIS.md](docs/SENSOR_FUSION_ANALYSIS.md) để hiểu cách:

- MTF01P và BMP388 được đọc độc lập.
- Dữ liệu được lọc và hợp nhất.
- Firmware xử lý dữ liệu stale, sensor lỗi hoặc mất nguồn đo.
- Độ trễ sensor có thể ảnh hưởng đến vòng altitude như thế nào.

### 3.2. Optical flow và position hold

Đọc
[POSITION_HOLD_HEIGHT_NORMALIZATION.md](docs/POSITION_HOLD_HEIGHT_NORMALIZATION.md)
để hiểu:

- Chuyển raw optical flow thành vận tốc `m/s` theo độ cao thực tế.
- Tích phân vận tốc thành vị trí tương đối.
- Giữ mốc calibration `(0,0)` trong suốt phiên cấp nguồn.
- Chuỗi điều khiển `position error → velocity demand → pitch/roll setpoint`.
- Giới hạn của optical flow, yaw không có compass và sai số tích phân vị trí.

### 3.3. Takeoff và landing

Đọc [TAKEOFF_LANDING_DESIGN.md](docs/TAKEOFF_LANDING_DESIGN.md) để hiểu state
machine cất/hạ cánh và lý do không thể chỉ dùng một ramp ga đơn giản.

Các phase chính:

```text
ARMED_IDLE
  → SPOOL_UP
  → GROUND_TRANSITION
  → CONTROLLED_ASCENT
  → ALTITUDE_CAPTURE
  → ALT_HOLD

ALT_HOLD
  → CONTROLLED_DESCENT
  → GROUND_APPROACH
  → TOUCHDOWN_DETECTION
  → MOTOR_RAMP_DOWN
  → DISARMED
```

### 3.4. Takeoff trên mặt nghiêng và handover mixer

Đọc [TILTED_TAKEOFF_DESIGN.md](docs/TILTED_TAKEOFF_DESIGN.md) để hiểu:

- Capture attitude khi còn trên mặt đất.
- Ground-limited mixer.
- Khóa integral trước khi rời đất.
- Crossfade từ ground mixer sang mixer bay bình thường.
- Abort khi góc nghiêng hoặc tốc độ góc vượt giới hạn.

### 3.5. Đồng bộ, lỗi dữ liệu và độ bền hệ thống

Đọc [ROBUSTNESS_AUDIT.md](docs/ROBUSTNESS_AUDIT.md) để hiểu:

- Race condition giữa hai core.
- ESP-NOW mailbox và ưu tiên lệnh KILL.
- Snapshot sensor bằng seqlock.
- Cách xử lý NaN, Inf và dữ liệu stale.
- Lý do sensor, OLED và telemetry không được block control loop.

### 3.6. Audit tổng thể

Đọc [FINAL_SYSTEM_AUDIT.md](docs/FINAL_SYSTEM_AUDIT.md) để xem:

- Các lỗi đã được sửa.
- Các vấn đề được phân loại CRITICAL, HIGH, MEDIUM và LOW.
- Những giới hạn phần mềm chưa thể giải quyết.
- Những giả thuyết vẫn cần dữ liệu bench hoặc flight test.

## 4. Hiểu tham số và cách hoàn thiện drone

### 4.1. `PARAMETER_HANDBOOK.md`

Dùng [PARAMETER_HANDBOOK.md](docs/PARAMETER_HANDBOOK.md) như tài liệu tra cứu:

- Ý nghĩa từng gain, filter và giới hạn.
- Ảnh hưởng khi tăng hoặc giảm một tham số.
- Triệu chứng khi tham số quá cao hoặc quá thấp.
- Thứ tự tune thích hợp.
- Những trường telemetry cần quan sát.

Không nên đọc handbook như một danh sách giá trị cần thay đổi. Chỉ thay đổi một
nhóm tham số sau khi hệ thống phía trước nó đã được kiểm chứng.

### 4.2. `TEST_PLAN.md`

Đọc [TEST_PLAN.md](docs/TEST_PLAN.md) cuối cùng để đưa hệ thống từ trạng thái
“compile thành công” sang trạng thái “đã được kiểm chứng trên phần cứng”:

```text
Kiểm tra code
   ↓
Không cánh
   ↓
Kiểm tra sensor và mapping
   ↓
Giá thử motor có che chắn
   ↓
Bay có dây giữ
   ↓
Tune hover và bù CG
   ↓
Tune altitude
   ↓
Tune position hold
   ↓
Lặp lại takeoff/landing
   ↓
Bay tự do
```

Không được bỏ qua một giai đoạn chỉ vì firmware đã compile hoặc một lần bay
ngắn chưa xảy ra lỗi.

## 5. Thứ tự đọc code nguồn

Source of truth của flight controller là
[drone_alt_hold_1m.ino](drone_alt_hold_1m.ino). Không nên đọc tuyến tính từ dòng
đầu tới cuối. Thứ tự dưới đây giúp hiểu code theo luồng điều khiển thực tế.

### Bước 1: Điểm khởi động và lịch thực thi

1. `setup()` — khởi tạo ESC, task, sensor, ESP-NOW, PID và calibration.
2. `loop()` — lịch đọc MPU và thời điểm gọi control loop.
3. `controlTick()` — trung tâm của mỗi chu kỳ điều khiển 250 Hz.

### Bước 2: Luồng điều khiển chính

4. `runFlightControl()` — kết hợp altitude, position, attitude và mixer.
5. `altitudeThrottle()` — xử lý takeoff, altitude hold, landing và vertical PI.
6. `updatePositionHold()` — tạo pitch/roll setpoint từ optical flow và vị trí.
7. `updateTakeoffAttitudeProfile()` — điều khiển attitude trước và sau rời đất.

### Bước 3: Attitude PID và mixer

8. `cascaded_compute()` — angle error thành desired angular rate.
9. `pid_compute()` — rate PID, D-term filter và anti-windup.
10. `cgCompensationScale()`, `cgPitchCompensation()` và
    `cgRollCompensation()` — bù moment do trọng tâm lệch.
11. `motor_mix()` — mixer X-frame khi bay bình thường.
12. `motorMixGroundLimited()` — mixer giới hạn khi còn trên mặt đất.
13. `motorMixTakeoffTransfer()` — crossfade sang mixer bình thường sau liftoff.

### Bước 4: Estimator và sensor

14. `estimateAttitude()` cùng các hàm Madgwick — tính pitch, roll và angular
    rate từ MPU6050.
15. `AltitudeEstimator` và parser MTF01P — xử lý range, optical flow, BMP388 và
    fusion độ cao.
16. `altitudeTask()` — lịch xử lý MTF/BMP trên core phụ.

### Bước 5: State machine, liên lạc và quan sát

17. `handlePendingCommand()` — xử lý ARM, TAKEOFF, LAND, DISARM và KILL.
18. `startLanding()`, `updateAltitudeSafety()` và `tripSafety()` — các nhánh
    failsafe và chuyển trạng thái.
19. `telemPublish()` và `telemTask()` — tạo telemetry gửi về sender.
20. `oledTask()` — hiển thị trạng thái nhưng không tham gia điều khiển motor.

Sender tương ứng nằm tại [sender_1m/sender_1m.ino](sender_1m/sender_1m.ino).
Chỉ cần đọc sau khi đã hiểu flight controller; tập trung vào packet command,
heartbeat, action sequence, telemetry decoder và các lệnh Serial.

## 6. Tài liệu lịch sử công việc

Hai file sau nên đọc cuối cùng:

- [CODEX_TASK.md](CODEX_TASK.md): danh sách các task đã thực hiện và phạm vi của
  từng lần nâng cấp.
- [CODEX_STATE.md](CODEX_STATE.md): kết quả kiểm tra, build, vấn đề còn mở và
  hành động tiếp theo.

Đây là lịch sử phát triển và trạng thái kiểm chứng, không phải tài liệu giải
thích thuật toán chính.

## 7. Thứ tự rút gọn

Nếu chỉ cần một danh sách ngắn, hãy đọc theo thứ tự:

1. `README.md`
2. `docs/FLIGHT_CONTROL_ARCHITECTURE.md`
3. `FLIGHT_LOGS_ANALYSIS.md`
4. `docs/FLIGHT_ISSUES.md`
5. `docs/SENSOR_FUSION_ANALYSIS.md`
6. `docs/POSITION_HOLD_HEIGHT_NORMALIZATION.md`
7. `docs/TAKEOFF_LANDING_DESIGN.md`
8. `docs/TILTED_TAKEOFF_DESIGN.md`
9. `docs/ROBUSTNESS_AUDIT.md`
10. `docs/FINAL_SYSTEM_AUDIT.md`
11. `docs/PARAMETER_HANDBOOK.md`
12. `docs/TEST_PLAN.md`
13. `drone_alt_hold_1m.ino`
14. `sender_1m/sender_1m.ino`
15. `CODEX_TASK.md` và `CODEX_STATE.md`

