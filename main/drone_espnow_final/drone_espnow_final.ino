/*
 * DRONE FLIGHT CONTROLLER - ESP32 + MPU6050 + 4x ESC brushless (X-frame)
 * Uoc luong tu the: Madgwick AHRS + uoc luong bias gyro truc tuyen.
 *
 * CHAN
 *   M1(FL,CW)=26  M2(FR,CCW)=13  M3(BR,CW)=14  M4(BL,CCW)=27
 *   MPU6050 SDA=32 SCL=33 | OLED SDA=21 SCL=22 | Serial 500000
 *
 * ESP-NOW
 *   NODE A (board nay) NHAN throttle tu NODE B, va gui telemetry nguoc ve.
 *
 * AN TOAN
 *   - ESC phai DA hieu chuan san bang sketch rieng.
 *   - Tu ARM khi nam ngang va dung yen lien tuc 3 giay.
 *   - Khong co kill-switch phan mem. Muon dung phai NGAT NGUON.
 */

#define ENABLE_OLED 1

// Cong tac mem cho hieu chuan tu dong trong setup().
//   1 = chay calibrateImu() nhu binh thuong (MAC DINH, an toan).
//   0 = BO QUA hieu chuan: bias gyro = 0, offset LEVEL = 0, |a| ref = 1.0g.
//       Chi dung de kiem chung bay that. Doc canh bao in ra Serial luc khoi dong.
#define ENABLE_AUTO_CALIB 1

// Cong tac mem cho ho so ga tu dong (nang len roi dao quanh muc hover).
//   0 = ga chay theo lenh ESP-NOW tu NODE B nhu binh thuong (MAC DINH).
//   1 = drone TU lai ga theo ho so, BO QUA lenh ga thuong cua NODE B.
//       NGOAI LE - CHOT AN TOAN TU XA: NODE B gui ga <= LANDING_HOVER_US (1100)
//       thi ho so nha quyen VINH VIEN, ga tra lai cho NODE B va drone ha canh
//       theo LANDING_DESCENT_S. Muon chay lai ho so phai RESET board.
#define ENABLE_THROTTLE_PROFILE 1

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "driver/ledc.h"
#include "soc/ledc_struct.h"
#if ENABLE_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
#endif

// ============================================================================
// 1. CAU HINH
// ============================================================================

// -- Chan ESC (X-frame) --
static const int ESC_PIN_M1 = 26;   // truoc-trai, canh CW
static const int ESC_PIN_M2 = 13;   // truoc-phai, canh CCW
static const int ESC_PIN_M3 = 14;   // sau-phai,   canh CW
static const int ESC_PIN_M4 = 27;   // sau-trai,   canh CCW

// -- ESC PWM --
static const int ESC_PWM_HZ   = 200;
static const int ESC_PWM_BITS = 16;
static const int ESC_CH_M1 = 0, ESC_CH_M2 = 1, ESC_CH_M3 = 2, ESC_CH_M4 = 3;

// -- I2C --
#define MPU_SDA   32
#define MPU_SCL   33
#define MPU_FREQ  400000UL
#define MPU_ADDR  0x68

#if ENABLE_OLED
  #define OLED_SDA  21
  #define OLED_SCL  22
  #define OLED_ADDR 0x3C
  #define OLED_TASK_CORE  0
  #define OLED_TASK_STACK 4096
  #define OLED_TASK_PRIO  1
#endif

// -- ESP-NOW: nhan throttle tu NODE B qua task uu tien cao nhat --
#define WIFI_CHANNEL       1
#define ONLY_FROM_PEER     1                          // 1 = chi nhan tu MAC NODE B
#define ESPNOW_TASK_PRIO   (configMAX_PRIORITIES - 1) // 24 - tren ca task WiFi
#define ESPNOW_TASK_CORE   0
#define ESPNOW_TASK_STACK  3072
#define ESPNOW_QUEUE_LEN   4
#define ESPNOW_MSG_MAX     24

#define ESPNOW_PING_STR    "PING"    // nhip tim tu NODE B, khong phai lenh ga
#define LINK_TIMEOUT_MS    3000UL    // mat goi qua lau = dut lien lac

// -- TELEMETRY NGUOC: NODE A -> NODE B --
#define TELEM_ENABLE     1
#define TELEM_HZ         10
#define TELEM_TASK_PRIO  1           // THAP hon vong dieu khien
#define TELEM_TASK_CORE  0
#define TELEM_TASK_STACK 3072

// Nhan dang goi nhi phan; doi cau truc la phai tang TELEM_VER.
#define TELEM_MAGIC      0xD1
#define TELEM_VER        1

#define TELEM_SERIAL     0           // 1 = in them dong telemetry ra Serial

// MAC cua NODE B (ben gui).
static uint8_t peerMac[6] = {0xA0, 0xB7, 0x65, 0xF6, 0x43, 0x10};

// -- Throttle (us) --
static const int IDLE_THROTTLE = 1000;
static const int HOVER_DEFAULT = 1200;      // dung khi chua nhan duoc goi nao

// -- Dai dieu khien hai ben HOVER, tinh theo % ga --
static const float THROTTLE_BAND_PCT = 0.15f;
static const int   THROTTLE_BAND_MIN = 60;
static const int   THROTTLE_BAND_MAX = 250;
static const int   MOTOR_MIN_US      = 1050;   // san tuyet doi cho mot dong co

// Dai hien hanh, tinh lai moi chu ky boi updateThrottleAndLink() tren core 1.
static int throttleBand = 100;

// Mien hop le cho gia tri nhan qua song, ngoai mien la BO QUA.
static const int HOVER_MIN_ALLOWED = 1050;
static const int HOVER_MAX_ALLOWED = 1750;

// Toc do bo ga toi gia tri moi. 0 = tat, ap tuc thi.
static const float HOVER_SLEW_US_PER_S = 200.0f;

// -- Che do ha canh: lenh <= LANDING_HOVER_US thi bo ga cham lai, chi chieu xuong --
static const int   LANDING_HOVER_US  = 1100;
static const float LANDING_DESCENT_S = 5.0f;
static const float LANDING_SLEW_MIN  = 20.0f;

// -- Ho so ga tu dong: chi co tac dung khi ENABLE_THROTTLE_PROFILE = 1 --
static const float PROFILE_START_US     = 1300.0f;  // ga ngay luc vao ARM
static const float PROFILE_LIFT_US      = 1470.0f;  // dich cua doan nang len
static const float PROFILE_LIFT_S       = 1.0f;     // thoi gian nang len (giay)
static const float PROFILE_MIN_US       = 1470.0f;  // day cua doan dao
static const float PROFILE_MAX_US       = 1480.0f;  // dinh cua doan dao
static const float PROFILE_OSC_PERIOD_S = 8.0f;     // chu ky mot nhip len-xuong

// -- Ba muc ga: dich (core 0 ghi) -> bo dan -> gia tri thuc dieu khien --
static volatile int hoverTarget    = HOVER_DEFAULT;
static float        hoverSlewF     = (float)HOVER_DEFAULT;
static int          HOVER_THROTTLE = HOVER_DEFAULT;

#define MIN_THROTTLE  (HOVER_THROTTLE - throttleBand)
#define MAX_THROTTLE  (HOVER_THROTTLE + throttleBand)

// -- Nhip: doc IMU o SAMPLE_HZ, chay dieu khien moi IMU_OVERSAMPLE mau --
static const uint32_t CONTROL_HZ     = 250;
static const int      IMU_OVERSAMPLE = 4;
static const uint32_t SAMPLE_HZ      = CONTROL_HZ * IMU_OVERSAMPLE;   // 1000Hz
static const uint32_t SAMPLE_US      = 1000000UL / SAMPLE_HZ;

// -- Che do tune: in CSV 100Hz cho MOT truc --
#define TUNE_MODE      0
#define TUNE_AXIS_ROLL 1     // 1 = roll | 0 = pitch

#if TUNE_MODE
static const uint32_t SERIAL_US = 10000UL;    // 100Hz
#else
static const uint32_t SERIAL_US = 100000UL;   // 10Hz
#endif
static const uint32_t OLED_MS  = 250;
static const uint32_t TELEM_US = 1000000UL / TELEM_HZ;

// -- Arm/an toan --
static const float ARM_MAX_ANGLE   = 25.0f;   // do
static const float ARM_MAX_RATE    = 15.0f;   // do/s
static const float AUTO_ARM_HOLD_S = 3.0f;
static const float ARM_RAMP_S      = 0.4f;    // ga em MIN -> HOVER
static const float SAFE_ANGLE      = 45.0f;   // nghieng qua -> khoa vinh vien
static const float TRIP_RAMP_S     = 1.5f;    // ha ga ve IDLE khi khoa
static const int   SENSOR_FAIL_LIMIT = (int)(SAMPLE_HZ / 20);   // ~50ms mat cam bien

// -- Tin cay accel: ti le so voi |a| do duoc luc hieu chuan --
static const float ACC_DEV_FULL = 0.06f;   // lech duoi muc nay: tin hoan toan
static const float ACC_DEV_ZERO = 0.25f;   // lech tren muc nay: bo qua accel
static const float ACC_LPF_HZ   = 20.0f;

// -- Nguong bao clip (cam bien bao hoa) --
static const int16_t CLIP_LSB = 32000;

// -- Huong lap chip: 1 = +Y chip ve mui | 0 = +X chip ve mui --
#define MPU_MOUNT_Y_FORWARD 1

// -- Dau. GYRO_*_SIGN phai la +1; muon dao chieu dieu khien thi doi MIX_*_SIGN --
static const float GYRO_PITCH_SIGN = +1.0f;
static const float GYRO_ROLL_SIGN  = +1.0f;
static const float GYRO_YAW_SIGN   = +1.0f;
static const float MIX_PITCH_SIGN  = +1.0f;
static const float MIX_ROLL_SIGN   = +1.0f;
static const float MIX_YAW_SIGN    = +1.0f;   // da kiem chung bay that, dung doi

// -- Trim dong co (us) --
static const int TRIM_M1 = 0, TRIM_M2 = 0, TRIM_M3 = 0, TRIM_M4 = 0;

// -- Trim tu the (do): cong thang vao setpoint goc, kep o +/-TRIM_DEG_MAX --
static const float PITCH_TRIM_DEG = -0.57f;
static const float ROLL_TRIM_DEG  = -0.16f;
static const float TRIM_DEG_MAX   = 5.0f;

// -- Loc --
static const float GYRO_LPF_HZ     = 30.0f;
static const float GYRO_YAW_LPF_HZ = 30.0f;
static const float DTERM_LPF_HZ    = 25.0f; 
static const float MADGWICK_BETA   = 0.1f;
static const float MADGWICK_ZETA   = 0.05f;   // hoc bias gyro truc tuyen

// -- PID --
// ATT_US_PER_DEG = us xung dong co tren moi do nghieng (nut van do nhay).
static const float ATT_US_PER_DEG = 3.0f;

static const float PITCH_RATE_KP  = 1.5f;
static const float PITCH_RATE_KI  = 0.40f;
static const float PITCH_RATE_KD  = 0.04f;
static const float PITCH_ANGLE_KP = ATT_US_PER_DEG / PITCH_RATE_KP;

static const float ROLL_RATE_KP  = 1.5f;
static const float ROLL_RATE_KI  = 0.40f;
static const float ROLL_RATE_KD  = 0.04f;
static const float ROLL_ANGLE_KP = ATT_US_PER_DEG / ROLL_RATE_KP;

static const float YAW_RATE_KP = 2.50f;
static const float YAW_RATE_KI = 2.0f;
static const float YAW_RATE_KD = 0.015f;

// Vong goc ngoai cho yaw = giu huong. 0 = tat.
static const float YAW_ANGLE_KP = 1.0f;

static const float MAX_RATE     = 200.0f;   // gioi han desired rate (do/s)
static const float MAX_YAW_RATE = 160.0f;

// Bien output PID; updateThrottleAndLink() ghi de theo dai ga moi chu ky.
static const float RATE_OUT_LIMIT = 120.0f;

static const float RATE_I_LIMIT    = 30.0f;    // truc yaw
static const float RATE_I_LIMIT_PR = 20.0f;    // pitch/roll

// I-term relax: khoa dan tich luy khi sai so goc lon, ngung han tu nguong nay.
static const float ITERM_RELAX_DEG  = 7.0f;
static const float ITERM_RELAX_RATE = ITERM_RELAX_DEG * PITCH_ANGLE_KP;

// Chi hoc bias gyro khi drone gan dung yen.
static const float BIAS_LEARN_MAX_RATE = 30.0f;   // do/s

// Vung chet mem bac 2 cho vong goc.
static const float ANGLE_DEADBAND = 0.2f;

static const float LEVEL_OFFSET_MAX = 10.0f;   // offset lon hon => CHAN ARM

// ============================================================================
// 2. KIEU DU LIEU
// Phai khai bao truoc ham dau tien: Arduino IDE chen moi prototype vao mot vi
// tri duy nhat, ngay truoc ham dau tien cua file.
// ============================================================================

typedef enum { LPF_ORDER_1 = 1, LPF_ORDER_2 = 2 } LPFOrder;

typedef struct {
  float    tau;
  float    y1, y2;
  LPFOrder order;
} LowPassFilter;

typedef struct {
  float kp, ki, kd;
  float out_limit, i_limit;
  float integral;
  float prev_meas;
  bool  has_prev;
  float i_gate;          // 0..1 nhan vao luong tich luy khau I (0 = khoa han)
  LowPassFilter d_lpf;
} PIDController;

typedef struct {
  PIDController angle;   // vong ngoai: chi P, output la do/s mong muon
  PIDController rate;    // vong trong: PID day du, output la us
  bool          i_frozen;// khoa khau I tu ben ngoai (luc ramp ga)
} CascadedAxis;

typedef struct {
  float q0, q1, q2, q3;
  float beta, zeta;
  float bx, by, bz;      // bias gyro con du (rad/s)
} Madgwick;

typedef struct {
  float ax, ay, az;      // g,    truc chip
  float gx, gy, gz;      // do/s, truc chip
  bool  clipped;
} ImuSample;

// Tich luy IMU_OVERSAMPLE mau roi lay trung binh.
typedef struct {
  float ax, ay, az, gx, gy, gz;
  int   n, clip;
} ImuAccum;

typedef enum { FS_DISARMED, FS_ARMED, FS_TRIPPED } FlightMode;

// Mot goi tho chuyen tu callback WiFi sang task uu tien 24.
typedef struct {
  char buf[ESPNOW_MSG_MAX];   // da ket thuc bang '\0'
  int  len;
} EspNowMsg;

// Goi telemetry gui ve NODE B. Phai GIONG TUNG BYTE voi khoi cung ten trong
// node_b_sender.ino; doi truong nao cung phai tang TELEM_VER.
typedef struct __attribute__((packed)) {
  uint8_t  magic;        // 0   TELEM_MAGIC
  uint8_t  ver;          // 1   TELEM_VER
  uint8_t  mode;         // 2   0=DISARMED 1=ARMED 2=TRIPPED
  uint8_t  flags;        // 3   bit0 linkUp | 1 overrun | 2 calibrated | 3 clip | 4 espnowReady

  uint32_t ms;           // 4   millis() tren drone
  uint16_t loopHz;       // 8   nhip dieu khien do duoc
  uint16_t seq;          // 10  dem goi

  float pitch, roll, yaw;             // 12  do (yaw la heading tich luy, co troi)
  float pitchRate, rollRate, yawRate; // 24  do/s, da loc
  float outP, outR, outY;             // 36  output PID (us)
  float iPitch, iRoll, iYaw;          // 48  tich phan - luong trim dang phai bu
  float biasX, biasY, biasZ;          // 60  bias Madgwick hoc luc bay (do/s)
  float calBiasX, calBiasY, calBiasZ; // 72  bias do duoc luc hieu chuan (do/s)
  float pitchOffset, rollOffset;      // 84  offset LEVEL
  float vibX, vibY, vibZ;             // 92  bien do rung dinh trong ky (g)
  float accW;                         // 104 trong so tin cay accel trung binh

  int16_t  m1, m2, m3, m4;            // 108 xung 4 dong co (us)
  int16_t  throttle;                  // 116 ga lenh
  int16_t  hover;                     // 118 HOVER dang hieu luc
  int16_t  hoverTarget;               // 120 HOVER dang bo toi
  uint16_t clip;                      // 122 so mau bao hoa trong ky
  uint16_t rx, bad, lost;             // 124 dem goi lenh nhan duoc
  char     reason[20];                // 130 modeReason, luon co '\0'
} TelemPacket;                        // 150 byte

static_assert(sizeof(TelemPacket) == 150,
              "TelemPacket doi kich thuoc - phai sua node_b_sender.ino cho khop!");
static_assert(sizeof(TelemPacket) <= 250,
              "ESP-NOW chi cho toi da 250 byte moi goi.");

void telemTask(void *arg);

// Khai bao truoc callback ESP-NOW: chu ky doi giua core 2.x va 3.x cua Arduino.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);
#endif
void espnowTask(void *arg);

#if ENABLE_OLED
typedef struct {
  float       pitch, roll, yaw;
  float       outP, outR, outY;
  int         throttle;
  int         m1, m2, m3, m4;
  uint16_t    loopHz;
  FlightMode  mode;
  const char* reason;
} OledSnap;

void updateOled(const OledSnap *s);
void oledTask(void *arg);
#endif

// Dai dieu khien ung voi mot muc ga bat ky. Ham THUAN nen core 0 goi duoc.
static inline int throttleBandFor(int hover) {
  int b = (int)lroundf((float)hover * THROTTLE_BAND_PCT);
  if (b < THROTTLE_BAND_MIN) b = THROTTLE_BAND_MIN;
  if (b > THROTTLE_BAND_MAX) b = THROTTLE_BAND_MAX;

  // Khong de MIN tut duoi nguong khoi dong motor.
  int room = hover - MOTOR_MIN_US;
  if (b > room) b = (room > THROTTLE_BAND_MIN) ? room : THROTTLE_BAND_MIN;
  return b;
}

// ============================================================================
// 3. BIEN TOAN CUC
// ============================================================================

static LowPassFilter lpfPitchRate, lpfRollRate, lpfYawRate;
static LowPassFilter lpfAccX, lpfAccY, lpfAccZ;
static CascadedAxis  pitchAxis, rollAxis;
static PIDController yawRatePid;
static Madgwick      ahrs;
static ImuAccum      imuAcc;

static float pitch = 0.0f, roll = 0.0f, yawHeading = 0.0f;
static float pitchRate = 0.0f, rollRate = 0.0f, yawRate = 0.0f;
static float pitchOffset = 0.0f, rollOffset = 0.0f;

// Diem mo rong khi them dieu khien tu xa. Hien luon 0 = giu thang bang tai cho.
static float pitchSetpoint = 0.0f, rollSetpoint = 0.0f, yawRateSetpoint = 0.0f;

// Huong ma vong giu huong dang bam vao (do).
static float yawHold = 0.0f;

static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
static float accMagRef = 1.0f;          // |a| tinh do duoc luc hieu chuan
static bool  imuCalibrated = false;

static FlightMode  mode = FS_DISARMED;
static const char* modeReason = "chua arm";

static int   throttleCmd = IDLE_THROTTLE;
static int   curM1 = IDLE_THROTTLE, curM2 = IDLE_THROTTLE;
static int   curM3 = IDLE_THROTTLE, curM4 = IDLE_THROTTLE;
static float lastP = 0.0f, lastR = 0.0f, lastY = 0.0f;

static float armHoldT = 0.0f, armRampT = 0.0f, tripRampT = 0.0f;
static int   tripStartM1, tripStartM2, tripStartM3, tripStartM4;

static uint32_t nextSampleUs = 0, lastCtrlUs = 0, lastSerialUs = 0, lastRateUs = 0;
static uint32_t lastTelemUs  = 0;
static uint32_t ctrlCount = 0;
static uint16_t loopHz = 0;
static int      sampleTick = 0, sensorFailCount = 0;
static bool     loopOverrun = false;

// Chan doan rung: bien do rung tung truc va trong so accel trung binh.
static float    vibX = 0.0f, vibY = 0.0f, vibZ = 0.0f;
static float    accWSum = 0.0f;
static uint32_t accWCount = 0, clipCount = 0;

// -- ESP-NOW --
static QueueHandle_t     espnowQueue   = NULL;
static TaskHandle_t      espnowTaskH   = NULL;
static volatile uint32_t espnowRx      = 0;   // goi hop le, da ap dung
static volatile uint32_t espnowBad     = 0;   // goi khong doc duoc so / ngoai mien
static volatile uint32_t espnowLastMs  = 0;   // moc goi cuoi cung, ke ca nhip tim
static volatile uint32_t espnowLost    = 0;   // goi rot vi queue day
static volatile uint32_t espnowPing    = 0;   // so nhip tim da nhan
static bool              espnowReady   = false;
static bool              linkUp        = false;   // trang thai lien lac, core 1
static bool              linkEverUp    = false;

// -- Telemetry nguoc --
static volatile uint32_t telemSeq     = 0;      // seqlock giua core 1 va core 0
static TelemPacket       telemShared;           // ban chup dung chung
static uint16_t          telemCount   = 0;
static TaskHandle_t      telemTaskH   = NULL;
static volatile uint32_t telemTxOk    = 0, telemTxFail = 0;

#if ENABLE_OLED
  TwoWire I2C_OLED = TwoWire(1);
  Adafruit_SSD1306 display(128, 64, &I2C_OLED, -1);
  static bool         oledOK = false;
  static TaskHandle_t oledTaskHandle = NULL;
#endif

// ============================================================================
// 4. BO LOC THONG THAP (PT1/PT2)
// alpha tinh lai moi vong theo dt thuc nen doi nhip khong hong dac tinh loc.
// ============================================================================

static void lpf_init(LowPassFilter *f, float cutoff_hz, LPFOrder order) {
  f->order = order;
  f->tau   = 1.0f / (2.0f * PI * cutoff_hz);
  f->y1 = f->y2 = 0.0f;
}

static void lpf_reset(LowPassFilter *f) { f->y1 = f->y2 = 0.0f; }

// Nap san gia tri on dinh de tranh qua do luc khoi dong.
static void lpf_preset(LowPassFilter *f, float v) { f->y1 = f->y2 = v; }

static float lpf_update(LowPassFilter *f, float x, float dt) {
  float last = (f->order == LPF_ORDER_2) ? f->y2 : f->y1;
  if (dt <= 0.0f || isnan(x) || isinf(x)) return last;

  float alpha = constrain(dt / (dt + f->tau), 0.0f, 1.0f);
  f->y1 = alpha * x + (1.0f - alpha) * f->y1;

  if (f->order == LPF_ORDER_2) {
    f->y2 = alpha * f->y1 + (1.0f - alpha) * f->y2;
    if (isnan(f->y2) || isinf(f->y2)) f->y2 = 0.0f;
    return f->y2;
  }
  if (isnan(f->y1) || isinf(f->y1)) f->y1 = 0.0f;
  return f->y1;
}

// ============================================================================
// 5. PID
// D lay tren measurement (khong derivative kick) + anti-windup co dieu kien.
// ============================================================================

static void pid_init(PIDController *p, float kp, float ki, float kd,
                     float out_limit, float i_limit, float dterm_hz) {
  p->kp = kp; p->ki = ki; p->kd = kd;
  p->out_limit = out_limit;
  p->i_limit   = i_limit;
  p->integral  = 0.0f;
  p->prev_meas = 0.0f;
  p->has_prev  = false;
  p->i_gate    = 1.0f;
  lpf_init(&p->d_lpf, dterm_hz, LPF_ORDER_1);
}

static void pid_reset(PIDController *p) {
  p->integral  = 0.0f;
  p->prev_meas = 0.0f;
  p->has_prev  = false;
  lpf_reset(&p->d_lpf);
}

static float pid_compute(PIDController *p, float setpoint, float meas, float dt) {
  if (dt <= 0.0f) return 0.0f;
  if (isnan(meas) || isinf(meas) || isnan(setpoint)) { pid_reset(p); return 0.0f; }

  float error  = setpoint - meas;
  float p_term = p->kp * error;

  float d_term = 0.0f;
  if (p->kd != 0.0f) {
    if (p->has_prev)
      d_term = -p->kd * lpf_update(&p->d_lpf, (meas - p->prev_meas) / dt, dt);
    p->prev_meas = meas;
    p->has_prev  = true;
  }

  float pre_out = p_term + p->ki * p->integral + d_term;
  bool sat_hi = (pre_out >  p->out_limit);
  bool sat_lo = (pre_out < -p->out_limit);
  bool may_integrate = (!sat_hi && !sat_lo) ||
                       (sat_hi && error < 0.0f) || (sat_lo && error > 0.0f);
  if (may_integrate && p->i_gate > 0.0f)
    p->integral = constrain(p->integral + error * dt * p->i_gate,
                            -p->i_limit, p->i_limit);

  float out = p_term + p->ki * p->integral + d_term;
  if (isnan(out) || isinf(out)) { pid_reset(p); return 0.0f; }
  return constrain(out, -p->out_limit, p->out_limit);
}

// Vung chet mem cho vong GOC (khong dung cho vong toc do).
static inline float soft_deadband(float e, float w) {
  if (w <= 0.0f) return e;
  float e2 = e * e, w2 = w * w;
  return e * (e2 / (e2 + w2));
}

// Chuan hoa goc ve [-180, 180].
static inline float wrap180(float a) {
  if (!isfinite(a)) return 0.0f;
  if (fabsf(a) <= 180.0f) return a;
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

// ============================================================================
// 6. CASCADED PID: vong goc (P) long vong toc do goc (PID)
// ============================================================================

static void cascaded_init(CascadedAxis *c, float angle_kp,
                          float rate_kp, float rate_ki, float rate_kd) {
  pid_init(&c->angle, angle_kp, 0.0f, 0.0f, MAX_RATE, 0.0f, 100.0f);
  pid_init(&c->rate,  rate_kp, rate_ki, rate_kd,
           RATE_OUT_LIMIT, RATE_I_LIMIT_PR, DTERM_LPF_HZ);
  c->i_frozen = false;
}

static void cascaded_reset(CascadedAxis *c) {
  pid_reset(&c->angle);
  pid_reset(&c->rate);
}

// Khoa khau I khi drone chua thuc su bay (dang ramp ga).
static void cascaded_freeze_i(CascadedAxis *c, bool frozen) {
  c->i_frozen = frozen;
}

static float cascaded_compute(CascadedAxis *c, float angle_sp, float angle_meas,
                              float rate_meas, float dt) {
  float angle_err    = soft_deadband(angle_sp - angle_meas, ANGLE_DEADBAND);
  float desired_rate = pid_compute(&c->angle, angle_err, 0.0f, dt);
  desired_rate = constrain(desired_rate, -MAX_RATE, MAX_RATE);

  // i-term relax: goc lech cang lon cang it tich luy.
  float relax = 1.0f - constrain(fabsf(desired_rate) / ITERM_RELAX_RATE, 0.0f, 1.0f);
  c->rate.i_gate = c->i_frozen ? 0.0f : relax;

  return pid_compute(&c->rate, desired_rate, rate_meas, dt);
}

// ============================================================================
// 7. MADGWICK AHRS
// Quaternion + gradient descent, tham so w (0..1) la trong so tin cay accel.
// ============================================================================

static void madgwick_init(Madgwick *m, float beta, float zeta) {
  m->q0 = 1.0f; m->q1 = m->q2 = m->q3 = 0.0f;
  m->beta = beta;
  m->zeta = zeta;
  m->bx = m->by = m->bz = 0.0f;
}

// w     = trong so tin cay accel (nhan vao beta)
// wBias = 0/1, chi cho hoc bias gyro khi drone gan dung yen
static void madgwick_update(Madgwick *m, float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float w, float wBias, float dt) {
  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  gx -= m->bx; gy -= m->by; gz -= m->bz;

  float s0 = 0, s1 = 0, s2 = 0, s3 = 0;
  bool  haveGrad = false;

  if (w > 0.0f) {
    float anorm = sqrtf(ax * ax + ay * ay + az * az);
    if (anorm > 1e-6f) {
      float r = 1.0f / anorm;
      ax *= r; ay *= r; az *= r;

      float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
      float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
      float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

      s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
      s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
      s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
      s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;

      float snorm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
      if (snorm > 1e-6f) {
        r = 1.0f / snorm;
        s0 *= r; s1 *= r; s2 *= r; s3 *= r;
        haveGrad = true;
      }
    }
  }

  float beta = m->beta * w;
  float zeta = m->zeta * w * wBias;

  // Doi gradient ve sai so goc roi tich luy thanh bias gyro.
  if (haveGrad && zeta > 0.0f) {
    float ex = 2.0f * (q0 * s1 - q1 * s0 - q2 * s3 + q3 * s2);
    float ey = 2.0f * (q0 * s2 + q1 * s3 - q2 * s0 - q3 * s1);
    float ez = 2.0f * (q0 * s3 - q1 * s2 + q2 * s1 - q3 * s0);
    m->bx += zeta * ex * dt;
    m->by += zeta * ey * dt;
    m->bz += zeta * ez * dt;
    gx -= zeta * ex * dt;
    gy -= zeta * ey * dt;
    gz -= zeta * ez * dt;
  }

  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if (haveGrad) {
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

  float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qnorm < 1e-6f || isnan(qnorm)) { madgwick_init(m, m->beta, m->zeta); return; }
  float r = 1.0f / qnorm;
  m->q0 = q0 * r; m->q1 = q1 * r; m->q2 = q2 * r; m->q3 = q3 * r;
}

// Huong "len troi" trong he truc chip, suy ra tu quaternion.
static void madgwick_gravity(const Madgwick *m, float *gx, float *gy, float *gz) {
  *gx = 2.0f * (m->q1 * m->q3 - m->q0 * m->q2);
  *gy = 2.0f * (m->q0 * m->q1 + m->q2 * m->q3);
  *gz = m->q0 * m->q0 - m->q1 * m->q1 - m->q2 * m->q2 + m->q3 * m->q3;
}

// ============================================================================
// 8. IMU - doc burst 14 byte, tich luy IMU_OVERSAMPLE mau roi lay trung binh
// ============================================================================

#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT   0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

static const float ACCEL_SCALE = 8192.0f;   // LSB/g   @ +/-4g
static const float GYRO_SCALE  = 65.5f;     // LSB/dps @ +/-500 do/s

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpuRead(uint8_t reg, uint8_t *val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (bool)true) != 1) return false;
  *val = Wire.read();
  return true;
}

// Doc het 14 byte vao dem TRUOC khi ghep (C++ khong dinh trinh tu hai ve cua '|').
static bool mpuReadRaw(ImuSample *s) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL_XOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (bool)true) != 14) return false;
  if (Wire.available() < 14) return false;

  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = (uint8_t)Wire.read();

  int16_t rax = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
  int16_t ray = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
  int16_t raz = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
  //           b[6], b[7] = nhiet do, bo qua
  int16_t rgx = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
  int16_t rgy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);

  s->ax = rax / ACCEL_SCALE;
  s->ay = ray / ACCEL_SCALE;
  s->az = raz / ACCEL_SCALE;
  s->gx = rgx / GYRO_SCALE;
  s->gy = rgy / GYRO_SCALE;
  s->gz = rgz / GYRO_SCALE;

  s->clipped = (abs(rax) >= CLIP_LSB || abs(ray) >= CLIP_LSB || abs(raz) >= CLIP_LSB ||
                abs(rgx) >= CLIP_LSB || abs(rgy) >= CLIP_LSB || abs(rgz) >= CLIP_LSB);
  return true;
}

static void accumReset(ImuAccum *a) {
  a->ax = a->ay = a->az = a->gx = a->gy = a->gz = 0.0f;
  a->n = a->clip = 0;
}

static void accumAdd(ImuAccum *a, const ImuSample *s) {
  a->ax += s->ax; a->ay += s->ay; a->az += s->az;
  a->gx += s->gx; a->gy += s->gy; a->gz += s->gz;
  a->n++;
  if (s->clipped) a->clip++;
}

static bool accumMean(const ImuAccum *a, ImuSample *out) {
  if (a->n <= 0) return false;
  float r = 1.0f / (float)a->n;
  out->ax = a->ax * r; out->ay = a->ay * r; out->az = a->az * r;
  out->gx = a->gx * r; out->gy = a->gy * r; out->gz = a->gz * r;
  out->clipped = (a->clip > 0);
  return true;
}

// Doc lai thanh ghi de kiem chung cau hinh da vao chip that su.
static bool mpuInit() {
  uint8_t who = 0;
  if (!mpuRead(MPU_REG_WHO_AM_I, &who)) return false;
  if (who != 0x68 && who != 0x69 && who != 0x70 && who != 0x71 && who != 0x73)
    return false;

  if (!mpuWrite(MPU_REG_PWR_MGMT_1,   0x01)) return false;  // clock = gyro X PLL
  delay(50);
  if (!mpuWrite(MPU_REG_CONFIG,       0x03)) return false;  // DLPF 44Hz
  if (!mpuWrite(MPU_REG_GYRO_CONFIG,  0x08)) return false;  // +/-500 do/s
  if (!mpuWrite(MPU_REG_ACCEL_CONFIG, 0x08)) return false;  // +/-4g
  if (!mpuWrite(MPU_REG_SMPLRT_DIV,   0x00)) return false;  // 1kHz
  delay(50);

  uint8_t rCfg = 0, rGyro = 0, rAcc = 0, rDiv = 0;
  mpuRead(MPU_REG_CONFIG,       &rCfg);
  mpuRead(MPU_REG_GYRO_CONFIG,  &rGyro);
  mpuRead(MPU_REG_ACCEL_CONFIG, &rAcc);
  mpuRead(MPU_REG_SMPLRT_DIV,   &rDiv);
  Serial.print("[MPU] WHO=0x");  Serial.print(who,   HEX);
  Serial.print(" DLPF=0x");      Serial.print(rCfg,  HEX);
  Serial.print(" GYRO=0x");      Serial.print(rGyro, HEX);
  Serial.print(" ACCEL=0x");     Serial.print(rAcc,  HEX);
  Serial.print(" DIV=0x");       Serial.println(rDiv, HEX);
  if (rCfg != 0x03 || rGyro != 0x08 || rAcc != 0x08 || rDiv != 0x00)
    Serial.println("[MPU] LOI: cau hinh doc lai khac gia tri da ghi (mong doi 0x3/0x8/0x8/0x0)!");
  if (who != 0x68)
    Serial.println("[MPU] CANH BAO: WHO_AM_I khac 0x68 - khong phai MPU6050 nguyen ban.");
  return true;
}

// ============================================================================
// 9. HIEU CHUAN
// Do bias gyro va offset LEVEL cua accel; goc do duoc luc nay duoc dinh nghia
// la 0 do nen drone bat buoc phai dang nam ngang that.
// ============================================================================

static bool calibrateImu() {
  const int SAMPLES = 800;
  double sgx = 0, sgy = 0, sgz = 0, sax = 0, say = 0, saz = 0;
  double sgx2 = 0, sgy2 = 0, sgz2 = 0, sAbsA = 0;
  int got = 0, clip = 0;

  Serial.println("[CAL] Dat drone nam ngang, giu yen (~3s)...");
#if ENABLE_OLED
  if (oledOK) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("HIEU CHUAN");
    display.println("DAT MAT PHANG NGANG");
    display.println("GIU YEN 3 GIAY!");
    display.display();
  }
#endif

  ImuSample s;
  for (int i = 0; i < SAMPLES * 3 && got < SAMPLES; i++) {
    if (mpuReadRaw(&s)) {
      sgx += s.gx; sgy += s.gy; sgz += s.gz;
      sax += s.ax; say += s.ay; saz += s.az;
      sgx2 += (double)s.gx * s.gx;
      sgy2 += (double)s.gy * s.gy;
      sgz2 += (double)s.gz * s.gz;
      sAbsA += sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
      if (s.clipped) clip++;
      got++;
    }
    delay(3);
  }
  if (got < SAMPLES / 2) {
    Serial.println("[CAL] THAT BAI: doc cam bien loi.");
    return false;
  }

  // Do lech chuan gyro = thuoc do "co dung yen khong".
  float gsdX = sqrtf(fmaxf(0.0f, (float)(sgx2 / got - (sgx / got) * (sgx / got))));
  float gsdY = sqrtf(fmaxf(0.0f, (float)(sgy2 / got - (sgy / got) * (sgy / got))));
  float gsdZ = sqrtf(fmaxf(0.0f, (float)(sgz2 / got - (sgz / got) * (sgz / got))));
  float gsdMax = fmaxf(gsdX, fmaxf(gsdY, gsdZ));
  if (gsdMax > 3.0f) {
    Serial.print("[CAL] THAT BAI: drone KHONG dung yen (lech chuan gyro ");
    Serial.print(gsdMax, 2); Serial.println(" do/s > 3.0). Giu yen roi RESET.");
    return false;
  }
  if (clip > 0) {
    Serial.print("[CAL] THAT BAI: "); Serial.print(clip);
    Serial.println(" mau bao hoa cam bien. Kiem tra day/nguon MPU.");
    return false;
  }

  gyroBiasX = (float)(sgx / got);
  gyroBiasY = (float)(sgy / got);
  gyroBiasZ = (float)(sgz / got);

  float axm = (float)(sax / got), aym = (float)(say / got), azm = (float)(saz / got);
  float anorm = sqrtf(axm * axm + aym * aym + azm * azm);
  if (anorm < 0.5f || anorm > 1.5f) {
    Serial.print("[CAL] THAT BAI: |accel| = "); Serial.print(anorm, 3);
    Serial.println(" g - drone dang rung/di chuyen hoac accel hong.");
    return false;
  }
  accMagRef = anorm;

  // Snap quaternion ve tu the hien tai bang beta giam dan; offset LEVEL lay
  // truc tiep tu accel trung binh.
  madgwick_init(&ahrs, 2.0f, 0.0f);
  for (int i = 0; i < 400; i++) {
    ahrs.beta = 2.0f * expf(-4.0f * (float)i / 400.0f);
    madgwick_update(&ahrs, 0, 0, 0, axm, aym, azm, 1.0f, 0.0f, 0.004f);
  }
  ahrs.beta = MADGWICK_BETA;
  ahrs.zeta = MADGWICK_ZETA;

  float gvx = axm / anorm, gvy = aym / anorm, gvz = azm / anorm;
#if MPU_MOUNT_Y_FORWARD
  pitchOffset = atan2f( gvy, sqrtf(gvx * gvx + gvz * gvz)) * RAD_TO_DEG;
  rollOffset  = atan2f(-gvx, gvz) * RAD_TO_DEG;
#else
  pitchOffset = atan2f( gvx, sqrtf(gvy * gvy + gvz * gvz)) * RAD_TO_DEG;
  rollOffset  = atan2f( gvy, gvz) * RAD_TO_DEG;
#endif

  float meanAbsA = (float)(sAbsA / got);
  Serial.print("[CAL] Bias do/s  X:"); Serial.print(gyroBiasX, 3);
  Serial.print(" Y:");                 Serial.print(gyroBiasY, 3);
  Serial.print(" Z:");                 Serial.println(gyroBiasZ, 3);
  Serial.print("[CAL] Level offset  P:"); Serial.print(pitchOffset, 2);
  Serial.print(" R:");                    Serial.println(rollOffset, 2);
  Serial.print("[CAL] accel TB  ax:"); Serial.print(axm, 4);
  Serial.print(" ay:");                Serial.print(aym, 4);
  Serial.print(" az:");                Serial.print(azm, 4);
  Serial.print(" g | |a| ref:");       Serial.print(accMagRef, 4);
  Serial.print(" | TB do dai mau:");   Serial.println(meanAbsA, 4);

  if (accMagRef / meanAbsA < 0.98f)
    Serial.println("[CAL] CANH BAO: drone DA XE DICH luc hieu chuan - bias/offset khong tin duoc.");
  else if (fabsf(meanAbsA - 1.0f) > 0.10f)
    Serial.println("[CAL] CANH BAO: sai thang do accel > 10% - kiem tra ACCEL_CONFIG/ACCEL_SCALE.");

  if (fabsf(pitchOffset) > LEVEL_OFFSET_MAX || fabsf(rollOffset) > LEVEL_OFFSET_MAX)
    Serial.println("[CAL] CANH BAO: offset LEVEL > 10 do => SE KHONG TU ARM. Dat ngang that roi RESET.");
  if (fabsf(gyroBiasX) > 20.0f || fabsf(gyroBiasY) > 20.0f || fabsf(gyroBiasZ) > 20.0f)
    Serial.println("[CAL] CANH BAO: bias gyro > 20 do/s - drone bi xe dich luc hieu chuan?");

  pitch = roll = yawHeading = 0.0f;
  lpf_reset(&lpfPitchRate);
  lpf_reset(&lpfRollRate);
  lpf_reset(&lpfYawRate);
  lpf_preset(&lpfAccX, axm);
  lpf_preset(&lpfAccY, aym);
  lpf_preset(&lpfAccZ, azm);
  accumReset(&imuAcc);
  imuCalibrated = true;
  Serial.println("[CAL] Xong.");
  return true;
}

// ============================================================================
// 10. DRIVER ESC (LEDC)
// Khong dung ledcWrite()/ESP32Servo vi ledc_update_duty() cho toi bien chu ky
// PWM ke tiep. Thay bang: bat sig_out_en luc khoi tao, sau do chi ghi para_up.
// ============================================================================

// Thu thuat thanh ghi duoi day chi dung tren ESP32 classic.
#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
  #error "Khoi 10 chi dung tren ESP32 classic. Tren S2/S3/C3/C6/H2 phai dung ledc_update_duty()."
#endif

#define ESC_LEDC_MODE   LEDC_LOW_SPEED_MODE
#define ESC_LEDC_TIMER  LEDC_TIMER_0
#define ESC_PARA_UP_BIT (1UL << 4)

// duty = us * 2^BITS * HZ / 1e6. Bat buoc uint64 de khong tran.
static inline uint32_t escUsToDuty(int us) {
  return (uint32_t)(((uint64_t)us * (1ULL << ESC_PWM_BITS) * ESC_PWM_HZ) / 1000000ULL);
}

static void escInitTimer() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode      = ESC_LEDC_MODE;
  tcfg.duty_resolution = (ledc_timer_bit_t)ESC_PWM_BITS;
  tcfg.timer_num       = ESC_LEDC_TIMER;
  tcfg.freq_hz         = ESC_PWM_HZ;
  tcfg.clk_cfg         = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);
}

// Chan phai co xung 1000us truoc khi ESC duoc cap dien, neu khong ESC tu choi arm.
static void escAttach(int pin, int ch) {
  ledc_channel_config_t ccfg = {};
  ccfg.gpio_num   = pin;
  ccfg.speed_mode = ESC_LEDC_MODE;
  ccfg.channel    = (ledc_channel_t)ch;
  ccfg.intr_type  = LEDC_INTR_DISABLE;
  ccfg.timer_sel  = ESC_LEDC_TIMER;
  ccfg.duty       = escUsToDuty(IDLE_THROTTLE);
  ccfg.hpoint     = 0;
  ledc_channel_config(&ccfg);
  ledc_update_duty(ESC_LEDC_MODE, (ledc_channel_t)ch);   // lan goi DUY NHAT
}

static inline void escWriteOne(int ch, int us) {
  ledc_set_duty(ESC_LEDC_MODE, (ledc_channel_t)ch, escUsToDuty(us));
  LEDC.channel_group[ESC_LEDC_MODE].channel[ch].conf0.val |= ESC_PARA_UP_BIT;
}

// Diem DUY NHAT trong file ghi xung xuong ESC.
static void escWriteAll(int v1, int v2, int v3, int v4) {
  escWriteOne(ESC_CH_M1, v1);
  escWriteOne(ESC_CH_M2, v2);
  escWriteOne(ESC_CH_M3, v3);
  escWriteOne(ESC_CH_M4, v4);
  curM1 = v1; curM2 = v2; curM3 = v3; curM4 = v4;
}

// ============================================================================
// 11. MIXER (X-frame)
//   M1(FL)=T+P+R+Y  M2(FR)=T+P-R-Y  M3(BR)=T-P-R+Y  M4(BL)=T-P+R-Y
// ============================================================================

// Bat khi mixer phai cat bot lenh yaw; runFlightControl doc o tick sau de khoa
// khau I cua yaw.
static bool yawMixClipped = false;

static void motor_mix(float throttle, float p_ctrl, float r_ctrl, float y_ctrl,
                      int *m1, int *m2, int *m3, int *m4) {
  const int   hov   = HOVER_THROTTLE;
  const int   minT  = hov - throttleBand;
  const int   maxT  = hov + throttleBand;
  const float avail = (float)(maxT - minT);

  // (a) Uu tien tu the: pitch/roll duoc phuc vu truoc, yaw lay phan con lai.
  float p1 = +p_ctrl + r_ctrl;
  float p2 = +p_ctrl - r_ctrl;
  float p3 = -p_ctrl - r_ctrl;
  float p4 = -p_ctrl + r_ctrl;

  float pHi = fmaxf(fmaxf(p1, p2), fmaxf(p3, p4));
  float pLo = fminf(fminf(p1, p2), fminf(p3, p4));
  float spanPR = pHi - pLo;

  if (spanPR > avail && spanPR > 0.0f) {
    float k = avail / spanPR;
    p1 *= k; p2 *= k; p3 *= k; p4 *= k;
    spanPR = avail;
  }

  // (b) Yaw chi duoc lay phan dai con lai.
  float yRoom = 0.5f * (avail - spanPR);
  float y     = constrain(y_ctrl, -yRoom, yRoom);
  yawMixClipped = (fabsf(y_ctrl) - yRoom) > 0.5f;

  float f1 = p1 + y;
  float f2 = p2 - y;
  float f3 = p3 + y;
  float f4 = p4 - y;

  float hi = fmaxf(fmaxf(f1, f2), fmaxf(f3, f4));
  float lo = fminf(fminf(f1, f2), fminf(f3, f4));

  // (c) Tinh tien base vao dai hop le - hy sinh do cao de giu tu the.
  float base = throttle;
  if (base + hi > (float)maxT) base = (float)maxT - hi;
  if (base + lo < (float)minT) base = (float)minT - lo;

  *m1 = constrain((int)lroundf(base + f1) + TRIM_M1, minT, maxT);
  *m2 = constrain((int)lroundf(base + f2) + TRIM_M2, minT, maxT);
  *m3 = constrain((int)lroundf(base + f3) + TRIM_M3, minT, maxT);
  *m4 = constrain((int)lroundf(base + f4) + TRIM_M4, minT, maxT);
}

// ============================================================================
// 12. AN TOAN & TU ARM
// ============================================================================

static void resetControllers() {
  cascaded_reset(&pitchAxis);
  cascaded_reset(&rollAxis);
  pid_reset(&yawRatePid);
  yawHold = yawHeading;
}

// Dieu kien du de arm ngay o tick nay (viec dem thoi gian o updateAutoArm).
static bool armConditionsMet() {
  if (mode == FS_TRIPPED)          { modeReason = "da khoa, phai RESET"; return false; }
  if (!imuCalibrated)              { modeReason = "chua hieu chuan";     return false; }
  if (isnan(pitch) || isnan(roll)) { modeReason = "goc NaN";             return false; }
  if (fabsf(pitchOffset) > LEVEL_OFFSET_MAX || fabsf(rollOffset) > LEVEL_OFFSET_MAX) {
    modeReason = "offset LEVEL qua lon - hieu chuan lai khi nam ngang";
    return false;
  }
  if (fabsf(pitch) > ARM_MAX_ANGLE || fabsf(roll) > ARM_MAX_ANGLE) {
    modeReason = "chua nam ngang";
    return false;
  }
  if (fabsf(pitchRate) > ARM_MAX_RATE || fabsf(rollRate) > ARM_MAX_RATE ||
      fabsf(yawRate) > ARM_MAX_RATE) {
    modeReason = "con dang rung/di chuyen";
    return false;
  }
  return true;
}

// Arm khi dieu kien dung lien tuc du lau; mot tick khong dat la bo dem ve 0.
static void updateAutoArm(float dt) {
  if (!armConditionsMet()) { armHoldT = 0.0f; return; }

  float before = armHoldT;
  armHoldT  += dt;
  modeReason = "sap arm";

  int a = (int)(AUTO_ARM_HOLD_S - before);
  int b = (int)(AUTO_ARM_HOLD_S - armHoldT);
  if (b < a && b >= 0) {
    Serial.print("[ARM] Tu dong arm sau "); Serial.print(b + 1); Serial.println("s...");
  }

  if (armHoldT >= AUTO_ARM_HOLD_S) {
    resetControllers();
    armHoldT    = 0.0f;
    armRampT    = 0.0f;
    throttleCmd = MIN_THROTTLE;
    mode        = FS_ARMED;
    modeReason  = "armed";
    Serial.println("[ARM] DA ARM - dong co bat dau quay.");
  }
}

// Khoa vinh vien: khong co duong quay lai bang phan mem, phai RESET board.
static void tripSafety(const char* reason) {
  if (mode == FS_TRIPPED) return;
  mode        = FS_TRIPPED;
  modeReason  = reason;
  tripRampT   = 0.0f;
  tripStartM1 = curM1; tripStartM2 = curM2;
  tripStartM3 = curM3; tripStartM4 = curM4;
  Serial.print("\n[SAFETY] "); Serial.println(reason);
}

// ============================================================================
// 13. ESP-NOW - NHAN HOVER_THROTTLE TU NODE B
//   song -> callback WiFi (prio 23, core 0) -> queue -> espnowTask (prio 24)
// ============================================================================

// Callback chay trong task WiFi. Chi duoc phep chep va day queue.
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
#endif
#if ONLY_FROM_PEER
  if (memcmp(mac, peerMac, 6) != 0) return;
#endif
  if (len <= 0 || espnowQueue == NULL) return;

  EspNowMsg m;
  m.len = (len < ESPNOW_MSG_MAX - 1) ? len : ESPNOW_MSG_MAX - 1;
  memcpy(m.buf, data, m.len);
  m.buf[m.len] = '\0';

  // Timeout 0: khong bao gio duoc chan task WiFi. Queue day thi bo goi va dem lai.
  if (xQueueSend(espnowQueue, &m, 0) != pdTRUE) espnowLost++;
}

// Task uu tien cao nhat: doc so, bao cao, ap dung.
void espnowTask(void *arg) {
  (void)arg;
  EspNowMsg m;

  for (;;) {
    // Chan vo han o day, neu khong se bo doi task WiFi tren cung core 0.
    if (xQueueReceive(espnowQueue, &m, portMAX_DELAY) != pdTRUE) continue;

    // Can cu duy nhat de biet con lien lac hay khong - nhip tim cung tinh.
    espnowLastMs = millis();

    // Ban sao sach de in: bo ky tu khong in duoc, dung o '\0'.
    char shown[ESPNOW_MSG_MAX];
    int k = 0;
    for (int i = 0; i < m.len && k < ESPNOW_MSG_MAX - 1; i++) {
      char c = m.buf[i];
      if (c == '\0') break;
      shown[k++] = (c >= 32 && c < 127) ? c : '.';
    }
    shown[k] = '\0';

    // Nhip tim: khong in, khong dem vao bad, khong dong toi ga.
    if (strcmp(shown, ESPNOW_PING_STR) == 0) { espnowPing++; continue; }

    char *end = NULL;
    long v = strtol(m.buf, &end, 10);

    // In moi goi nhan duoc, ke ca goi bi tu choi. Gop thanh MOT chuoi roi ghi
    // mot lan de khong bi dong telemetry chen vao giua.
    char line[120];
    if (end == m.buf) {
      espnowBad++;
      snprintf(line, sizeof(line),
               "[ESPNOW] nhan \"%s\" -> BO QUA (khong doc duoc so)\n", shown);
    } else if (v < HOVER_MIN_ALLOWED || v > HOVER_MAX_ALLOWED) {
      espnowBad++;
      snprintf(line, sizeof(line),
               "[ESPNOW] nhan \"%s\" -> %ld BO QUA (ngoai mien %d..%d)\n",
               shown, v, HOVER_MIN_ALLOWED, HOVER_MAX_ALLOWED);
    } else {
      espnowRx++;
      hoverTarget = (int)v;   // ghi nguyen tu. Core 1 se bo dan toi day.
      const int nb = throttleBandFor((int)v);
      snprintf(line, sizeof(line),
               "[ESPNOW] nhan \"%s\" -> %d | dich HOVER=%d dai +/-%d MIN=%d MAX=%d%s\n",
               shown, (int)v, (int)v, nb, (int)v - nb, (int)v + nb,
               (mode == FS_ARMED) ? "  (dang bay - bo dan)" : "  (chua arm - ap ngay)");
    }
    Serial.write(line);
  }
}

// ============================================================================
// HO SO GA TU DONG (cong tac mem ENABLE_THROTTLE_PROFILE)
// Doan 1: nang thang tu PROFILE_START_US len PROFILE_LIFT_US trong
//         PROFILE_LIFT_S giay, tinh tu luc vao ARM.
// Doan 2: dao hinh sin trong [PROFILE_MIN_US, PROFILE_MAX_US] mai mai.
// KHONG co cam bien do cao tren board nay: day la vong HO, no KHONG giu duoc
// do cao. Neu muc trung binh lech so voi hover that thi drone van len hoac
// xuong deu - doan dao chi cong them mot nhip bap benh len tren cai troi do.
// ============================================================================

#if ENABLE_THROTTLE_PROFILE

static float profileT = 0.0f;        // giay ke tu luc vao ARM
static bool  profileAborted = false; // NODE B da doi quyen lai ga - MOT CHIEU

// Goi moi tick tu updateThrottleAndLink(). Tra ve true va ghi *hoverOut khi ho
// so dang lai ga; tra ve false thi ga chay theo lenh ESP-NOW nhu binh thuong.
static bool throttleProfileUpdate(float dt, float *hoverOut) {
  // CHOT AN TOAN TU XA: NODE B ra lenh ga <= LANDING_HOVER_US thi ho so nha
  // quyen VINH VIEN - giong tripSafety, khong co duong quay lai bang phan mem.
  // Tu tick nay ga chay theo NODE B, ke ca duong ha canh cham LANDING_DESCENT_S.
  // Kiem tra TRUOC ca mode: cho phep chan tu duoi dat, truoc khi drone tu ARM.
  if (!profileAborted && hoverTarget <= LANDING_HOVER_US) {
    profileAborted = true;
    Serial.print("[PROFILE] NODE B ra lenh ga ");
    Serial.print(hoverTarget);
    Serial.println(" - HO SO NHA QUYEN VINH VIEN, ga theo NODE B tu day.");
  }
  if (profileAborted) return false;

  if (mode != FS_ARMED) {         // chua bay: dat lai de lan ARM sau chay tu dau
    profileT = 0.0f;
    return false;
  }

  profileT += dt;

  float us;
  if (profileT < PROFILE_LIFT_S) {
    float k = profileT / PROFILE_LIFT_S;
    us = PROFILE_START_US + (PROFILE_LIFT_US - PROFILE_START_US) * k;
  } else {
    // Lech pha phi0 de diem noi hai doan bang dung PROFILE_LIFT_US, khong nhay buoc.
    const float mid = 0.5f * (PROFILE_MAX_US + PROFILE_MIN_US);
    const float amp = 0.5f * (PROFILE_MAX_US - PROFILE_MIN_US);
    float phi0 = (amp > 0.1f)
                 ? asinf(constrain((PROFILE_LIFT_US - mid) / amp, -1.0f, 1.0f))
                 : 0.0f;
    float w = 6.28318531f / PROFILE_OSC_PERIOD_S;
    us = mid + amp * sinf(phi0 + w * (profileT - PROFILE_LIFT_S));
  }

  *hoverOut = constrain(us, (float)HOVER_MIN_ALLOWED, (float)HOVER_MAX_ALLOWED);
  return true;
}

#endif  // ENABLE_THROTTLE_PROFILE

// Bo ga dan tu HOVER hien tai toi dich, va theo doi lien lac. Goi tu controlTick
// (core 1) truoc moi tinh toan cua chu ky.
static void updateThrottleAndLink(float dt) {
  const int target = hoverTarget;          // doc volatile MOT lan

  // Toc do ha canh chot mot lan luc lenh vua doi, khong tinh lai moi tick.
  static int   prevTarget    = HOVER_DEFAULT;
  static float landingSlew   = 0.0f;        // 0 = khong o che do ha canh
  if (target != prevTarget) {
    prevTarget  = target;
    landingSlew = 0.0f;
    if (target <= LANDING_HOVER_US && hoverSlewF > (float)target && mode == FS_ARMED) {
      float dist = hoverSlewF - (float)target;
      landingSlew = dist / LANDING_DESCENT_S;
      if (landingSlew < LANDING_SLEW_MIN)      landingSlew = LANDING_SLEW_MIN;
      if (landingSlew > HOVER_SLEW_US_PER_S)   landingSlew = HOVER_SLEW_US_PER_S;
      Serial.print("[LAND] Ha canh: "); Serial.print((int)hoverSlewF);
      Serial.print(" -> ");             Serial.print(target);
      Serial.print(" trong ");          Serial.print(dist / landingSlew, 1);
      Serial.print("s (");              Serial.print(landingSlew, 0);
      Serial.println("us/s)");
    }
  }

  // Chua arm thi ap ngay: dong co dang o IDLE, khong co gi de giat.
  if (mode == FS_DISARMED || HOVER_SLEW_US_PER_S <= 0.0f) {
    hoverSlewF = (float)target;
  } else {
    // Toc do cham chi ap cho chieu di xuong.
    float rate = (landingSlew > 0.0f && hoverSlewF > (float)target)
                 ? landingSlew : HOVER_SLEW_US_PER_S;
    float step = rate * dt;
    if (hoverSlewF < target)      hoverSlewF = fminf(hoverSlewF + step, (float)target);
    else if (hoverSlewF > target) hoverSlewF = fmaxf(hoverSlewF - step, (float)target);
  }
#if ENABLE_THROTTLE_PROFILE
  // Ho so lai ga: ghi de ket qua bo dan o tren, bo qua ca lenh tu NODE B.
  // Ghi vao hoverSlewF chu khong phai HOVER_THROTTLE, de khi tat ho so giua
  // chung thi bo dan chay tiep tu dung cho nay, khong nhay buoc.
  {
    float profUs;
    if (throttleProfileUpdate(dt, &profUs)) hoverSlewF = profUs;
  }
#endif

  HOVER_THROTTLE = (int)lroundf(hoverSlewF);

  // Dai dieu khien bam theo ga; day la diem duy nhat ghi throttleBand.
  int band = throttleBandFor(HOVER_THROTTLE);
  if (band != throttleBand) {
    throttleBand = band;
    // Bien output PID phai bang dung du dia mixer that su co.
    float lim = (float)band;
    pitchAxis.rate.out_limit = lim;
    rollAxis.rate.out_limit  = lim;
    yawRatePid.out_limit     = lim;
  }

  // Trang thai lien lac. Chi bao cao, khong dong toi che do bay.
  uint32_t age = millis() - espnowLastMs;
  bool up = (espnowLastMs != 0) && (age < LINK_TIMEOUT_MS);

  if (up != linkUp) {
    linkUp = up;
    if (up) {
      Serial.println(linkEverUp ? "[LINK] DA KET NOI LAI voi NODE B."
                                : "[LINK] Bat duoc NODE B.");
      linkEverUp = true;
    } else {
      Serial.print("[LINK] *** MAT KET NOI voi NODE B *** giu HOVER=");
      Serial.println(HOVER_THROTTLE);
    }
  }
}

static bool espnowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[ESPNOW] MAC board nay : ");
  Serial.println(WiFi.macAddress());
  Serial.printf("[ESPNOW] Nhan tu       : %02X:%02X:%02X:%02X:%02X:%02X\n",
                peerMac[0], peerMac[1], peerMac[2],
                peerMac[3], peerMac[4], peerMac[5]);

  uint8_t ch; wifi_second_chan_t sec;
  esp_wifi_get_channel(&ch, &sec);
  Serial.printf("[ESPNOW] WiFi channel  : %d (dat=%d)\n", ch, WIFI_CHANNEL);

  // Queue + task phai san sang truoc khi dang ky callback.
  espnowQueue = xQueueCreate(ESPNOW_QUEUE_LEN, sizeof(EspNowMsg));
  if (espnowQueue == NULL) {
    Serial.println("[ESPNOW] LOI: khong cap duoc queue - bay voi HOVER mac dinh.");
    return false;
  }
  if (xTaskCreatePinnedToCore(espnowTask, "espnow", ESPNOW_TASK_STACK, NULL,
                              ESPNOW_TASK_PRIO, &espnowTaskH,
                              ESPNOW_TASK_CORE) != pdPASS) {
    Serial.println("[ESPNOW] LOI: khong tao duoc task - bay voi HOVER mac dinh.");
    return false;
  }
  Serial.print("[ESPNOW] Task uu tien "); Serial.print(ESPNOW_TASK_PRIO);
  Serial.print("/");                      Serial.print(configMAX_PRIORITIES - 1);
  Serial.print(" tren core ");            Serial.print(ESPNOW_TASK_CORE);
  Serial.println(" (tren ca task WiFi).");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] LOI: esp_now_init that bai - bay voi HOVER mac dinh.");
    return false;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  // Them peer: can de gui telemetry nguoc lai.
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println("[ESPNOW] CANH BAO: khong them duoc peer (van nhan duoc du lieu).");

  Serial.print("[ESPNOW] San sang. Mien hop le ");
  Serial.print(HOVER_MIN_ALLOWED); Serial.print("..");
  Serial.print(HOVER_MAX_ALLOWED);
  Serial.print(" | mac dinh HOVER="); Serial.println(HOVER_DEFAULT);
  return true;
}

// ============================================================================
// 13B. TELEMETRY NGUOC - GUI TOAN BO SO LIEU VE NODE B
//   core 1, controlTick -> telemPublish(): chup so lieu roi di tiep ngay
//   core 0, telemTask   -> telemRead() + esp_now_send()
// Dong bo bang seqlock chu khong mutex: ben ghi (vong dieu khien) khong bao gio doi.
// ============================================================================

static inline uint16_t telemSat16(uint32_t v) {
  return (v > 65535UL) ? (uint16_t)65535 : (uint16_t)v;
}

// Goi tu controlTick (core 1) moi TELEM_US. Day cung la cho duy nhat xoa cac bo
// tich luy chan doan (rung/trong so accel/clip).
static void telemPublish() {
  const float accWAvg = accWCount ? (accWSum / (float)accWCount) : 1.0f;

  telemSeq = telemSeq + 1;             // -> le: dang ghi, dung tin
  __sync_synchronize();

  telemShared.magic = TELEM_MAGIC;
  telemShared.ver   = TELEM_VER;
  telemShared.mode  = (uint8_t)mode;
  telemShared.flags = (uint8_t)((linkUp        ? 0x01 : 0) |
                                (loopOverrun   ? 0x02 : 0) |
                                (imuCalibrated ? 0x04 : 0) |
                                (clipCount     ? 0x08 : 0) |
                                (espnowReady   ? 0x10 : 0));
  telemShared.ms     = millis();
  telemShared.loopHz = loopHz;
  telemShared.seq    = ++telemCount;

  telemShared.pitch     = pitch;
  telemShared.roll      = roll;
  telemShared.yaw       = yawHeading;
  telemShared.pitchRate = pitchRate;
  telemShared.rollRate  = rollRate;
  telemShared.yawRate   = yawRate;
  telemShared.outP      = lastP;
  telemShared.outR      = lastR;
  telemShared.outY      = lastY;

  telemShared.iPitch = pitchAxis.rate.integral;
  telemShared.iRoll  = rollAxis.rate.integral;
  telemShared.iYaw   = yawRatePid.integral;

  telemShared.biasX = ahrs.bx * RAD_TO_DEG;
  telemShared.biasY = ahrs.by * RAD_TO_DEG;
  telemShared.biasZ = ahrs.bz * RAD_TO_DEG;

  telemShared.calBiasX = gyroBiasX;
  telemShared.calBiasY = gyroBiasY;
  telemShared.calBiasZ = gyroBiasZ;

  telemShared.pitchOffset = pitchOffset;
  telemShared.rollOffset  = rollOffset;

  telemShared.vibX = vibX;
  telemShared.vibY = vibY;
  telemShared.vibZ = vibZ;
  telemShared.accW = accWAvg;

  telemShared.m1 = (int16_t)curM1;
  telemShared.m2 = (int16_t)curM2;
  telemShared.m3 = (int16_t)curM3;
  telemShared.m4 = (int16_t)curM4;

  telemShared.throttle    = (int16_t)throttleCmd;
  telemShared.hover       = (int16_t)HOVER_THROTTLE;
  telemShared.hoverTarget = (int16_t)hoverTarget;

  telemShared.clip = telemSat16(clipCount);
  telemShared.rx   = telemSat16(espnowRx);
  telemShared.bad  = telemSat16(espnowBad);
  telemShared.lost = telemSat16(espnowLost);

  strncpy(telemShared.reason, modeReason ? modeReason : "",
          sizeof(telemShared.reason) - 1);
  telemShared.reason[sizeof(telemShared.reason) - 1] = '\0';

  __sync_synchronize();
  telemSeq = telemSeq + 1;             // -> chan: ban chup da lanh lan

  vibX = vibY = vibZ = 0.0f;
  accWSum = 0.0f; accWCount = 0; clipCount = 0;
}

static bool telemRead(TelemPacket *out) {
  for (int i = 0; i < 4; i++) {
    uint32_t s1 = telemSeq;
    if (s1 & 1u) continue;             // core 1 dang ghi do
    __sync_synchronize();
    *out = telemShared;
    __sync_synchronize();
    if (telemSeq == s1) return true;   // khong bi ghi de giua chung
  }
  return false;
}

void telemTask(void *arg) {
  (void)arg;
  TelemPacket p;

  // vTaskDelayUntil de nhip khong troi theo thoi gian ham gui chay lau ngan.
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEM_HZ);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);
    if (!espnowReady) continue;
    if (!telemRead(&p)) continue;      // 4 lan deu trung ghi - bo ky nay

    // Bo qua ma khong thu lai: goi ke tiep chi cach 100ms va moi hon.
    if (esp_now_send(peerMac, (const uint8_t *)&p, sizeof(p)) == ESP_OK) telemTxOk++;
    else                                                                telemTxFail++;
  }
}

// ============================================================================
// 14. OLED - task rieng tren CORE 0
// display.display() chan ~29ms nen khong duoc nam trong vong dieu khien.
// Dong bo bang seqlock nhu khoi 13B.
// ============================================================================

#if ENABLE_OLED

static volatile uint32_t oledSeq = 0;
static OledSnap          oledShared;

static inline void oledPublish() {
  oledSeq = oledSeq + 1;
  __sync_synchronize();
  oledShared.pitch    = pitch;
  oledShared.roll     = roll;
  oledShared.yaw      = yawHeading;
  oledShared.outP     = lastP;
  oledShared.outR     = lastR;
  oledShared.outY     = lastY;
  oledShared.throttle = throttleCmd;
  oledShared.m1       = curM1;
  oledShared.m2       = curM2;
  oledShared.m3       = curM3;
  oledShared.m4       = curM4;
  oledShared.loopHz   = loopHz;
  oledShared.mode     = mode;
  oledShared.reason   = modeReason;
  __sync_synchronize();
  oledSeq = oledSeq + 1;
}

static bool oledRead(OledSnap *out) {
  for (int i = 0; i < 4; i++) {
    uint32_t s1 = oledSeq;
    if (s1 & 1u) continue;
    __sync_synchronize();
    *out = oledShared;
    __sync_synchronize();
    if (oledSeq == s1) return true;
  }
  return false;
}

void updateOled(const OledSnap *s) {
  if (!oledOK) return;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print(s->mode == FS_TRIPPED ? "SAFE" : (s->mode == FS_ARMED ? "ARM" : "IDLE"));
  display.print(" T:"); display.print(s->throttle);
  display.print(" ");   display.print(s->loopHz); display.println("Hz");

  display.print("P:");  display.print(s->pitch, 1);
  display.print(" R:"); display.print(s->roll, 1);
  display.print(" Y:"); display.println(s->yaw, 0);

  display.print("o:");  display.print(s->outP, 0);
  display.print(" ");   display.print(s->outR, 0);
  display.print(" ");   display.println(s->outY, 0);

  display.print("M1:");  display.print(s->m1);
  display.print(" M2:"); display.println(s->m2);
  display.print("M4:");  display.print(s->m4);
  display.print(" M3:"); display.println(s->m3);

  display.println(s->reason);
  display.display();
}

void oledTask(void *arg) {
  (void)arg;
  OledSnap snap;
  for (;;) {
    if (oledRead(&snap)) updateOled(&snap);
    vTaskDelay(pdMS_TO_TICKS(OLED_MS));
  }
}
#endif

// ============================================================================
// 15. CHAN DOAN SERIAL
// ============================================================================

#if TUNE_MODE
// CSV 100Hz cho MOT truc: goc | toc do goc | output PID | ga trung binh.
static void printTelemetry() {
  Serial.print(millis());
  Serial.print(',');
#if TUNE_AXIS_ROLL
  Serial.print(roll, 2);      Serial.print(',');
  Serial.print(rollRate, 1);  Serial.print(',');
  Serial.print(lastR, 1);
#else
  Serial.print(pitch, 2);     Serial.print(',');
  Serial.print(pitchRate, 1); Serial.print(',');
  Serial.print(lastP, 1);
#endif
  Serial.print(',');
  Serial.println((curM1 + curM2 + curM3 + curM4) / 4 - HOVER_THROTTLE);
}

#elif TELEM_SERIAL
// Mot dong 10Hz: I = tich phan, B = bias gyro, V = rung, W = trong so accel,
// C = so mau bao hoa, H = HOVER va trang thai lien lac.
static void printTelemetry() {
  Serial.print(mode == FS_TRIPPED ? "SAFE" : (mode == FS_ARMED ? "ARM " : "IDLE"));
  Serial.print(" T:");   Serial.print(throttleCmd);
  Serial.print(" ");     Serial.print(loopHz);   Serial.print("Hz");
  Serial.print(" | P:"); Serial.print(pitch, 1);
  Serial.print(" R:");   Serial.print(roll, 1);
  Serial.print(" Yr:");  Serial.print(yawRate, 1);
  Serial.print(" | o:"); Serial.print(lastP, 0);
  Serial.print(",");     Serial.print(lastR, 0);
  Serial.print(",");     Serial.print(lastY, 0);
  Serial.print(" | M:"); Serial.print(curM1);
  Serial.print(" ");     Serial.print(curM2);
  Serial.print(" ");     Serial.print(curM3);
  Serial.print(" ");     Serial.print(curM4);
  Serial.print(" | I:"); Serial.print(pitchAxis.rate.integral, 1);
  Serial.print(",");     Serial.print(rollAxis.rate.integral, 1);
  Serial.print(" B:");   Serial.print(ahrs.bx * RAD_TO_DEG, 2);
  Serial.print(",");     Serial.print(ahrs.by * RAD_TO_DEG, 2);

  // Doc tu ban chup telemetry chu khong tu bien tich luy (telemPublish xoa chung).
  Serial.print(" | V:"); Serial.print(telemShared.vibX, 2);
  Serial.print("/");     Serial.print(telemShared.vibY, 2);
  Serial.print("/");     Serial.print(telemShared.vibZ, 2);
  Serial.print(" W:");   Serial.print(telemShared.accW, 2);
  Serial.print(" C:");   Serial.print(telemShared.clip);
  Serial.print(" ref:"); Serial.print(accMagRef, 3);

  Serial.print(" | H:"); Serial.print(HOVER_THROTTLE);
  if (HOVER_THROTTLE != hoverTarget) {        // dang bo toi dich
    Serial.print(">"); Serial.print(hoverTarget);
  }
  Serial.print(" ");     Serial.print(linkUp ? "LINK" : "MAT-KN");
  Serial.print("/");     Serial.print(espnowRx);
  if (espnowLastMs) {   // tuoi cua goi cuoi cung, giay
    Serial.print("/"); Serial.print((millis() - espnowLastMs) / 1000); Serial.print("s");
  }
  if (espnowBad)  { Serial.print(" bad:");  Serial.print(espnowBad); }
  if (espnowLost) { Serial.print(" lost:"); Serial.print(espnowLost); }

  if (loopOverrun)      Serial.print(" [overrun]");
  if (mode != FS_ARMED) { Serial.print(" ["); Serial.print(modeReason); Serial.print("]"); }
  Serial.println();
}

#else
// TELEM_SERIAL = 0: so lieu di ve NODE B, Serial chi con cac dong su kien.
static inline void printTelemetry() {}
#endif  // TUNE_MODE / TELEM_SERIAL

// ============================================================================
// 16. SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== DRONE FC (ESP32) + ESP-NOW ===");

#if ENABLE_OLED
  I2C_OLED.begin(OLED_SDA, OLED_SCL, 400000UL);
  I2C_OLED.setTimeOut(10);
  oledOK = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Khoi dong...");
    display.display();
  } else {
    Serial.println("[WARN] Khong tim thay OLED.");
  }
#endif

  // ESC truoc cam bien: giu xung hop le cho ESC cang som cang tot.
  escInitTimer();
  escAttach(ESC_PIN_M1, ESC_CH_M1);
  escAttach(ESC_PIN_M2, ESC_CH_M2);
  escAttach(ESC_PIN_M3, ESC_CH_M3);
  escAttach(ESC_PIN_M4, ESC_CH_M4);
  escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);

  // ESP-NOW bat truoc khi hieu chuan; loi ESP-NOW khong dung chuong trinh.
  espnowReady = espnowInit();

  Wire.begin(MPU_SDA, MPU_SCL, MPU_FREQ);
  Wire.setTimeOut(5);
  if (!mpuInit()) {
    Serial.println("[FATAL] Khong tim thay MPU6050!");
    escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);
    while (true) delay(500);
  }
  Wire.setClock(MPU_FREQ);

  // Do that thoi gian mot lan doc IMU; vuot ngan sach thi phai giam IMU_OVERSAMPLE.
  {
    uint32_t tSum = 0, tMax = 0;
    int nOk = 0;
    for (int i = 0; i < 50; i++) {
      ImuSample s;
      uint32_t t0 = micros();
      bool ok = mpuReadRaw(&s);
      uint32_t d = micros() - t0;
      if (ok) { tSum += d; if (d > tMax) tMax = d; nOk++; }
      delayMicroseconds(500);
    }
    if (nOk) {
      Serial.print("[IMU] doc "); Serial.print(tSum / nOk);
      Serial.print("us TB / ");   Serial.print(tMax);
      Serial.print("us dinh | ngan sach "); Serial.print(SAMPLE_US);
      Serial.println("us");
      if (tMax * 5 > SAMPLE_US * 4)
        Serial.println("[IMU] CANH BAO: doc IMU chiem >80% chu ky - giam IMU_OVERSAMPLE.");
    }
  }

  lpf_init(&lpfPitchRate, GYRO_LPF_HZ,     LPF_ORDER_2);
  lpf_init(&lpfRollRate,  GYRO_LPF_HZ,     LPF_ORDER_2);
  lpf_init(&lpfYawRate,   GYRO_YAW_LPF_HZ, LPF_ORDER_2);
  lpf_init(&lpfAccX,      ACC_LPF_HZ,      LPF_ORDER_1);
  lpf_init(&lpfAccY,      ACC_LPF_HZ,      LPF_ORDER_1);
  lpf_init(&lpfAccZ,      ACC_LPF_HZ,      LPF_ORDER_1);

  cascaded_init(&pitchAxis, PITCH_ANGLE_KP, PITCH_RATE_KP, PITCH_RATE_KI, PITCH_RATE_KD);
  cascaded_init(&rollAxis,  ROLL_ANGLE_KP,  ROLL_RATE_KP,  ROLL_RATE_KI,  ROLL_RATE_KD);
  pid_init(&yawRatePid, YAW_RATE_KP, YAW_RATE_KI, YAW_RATE_KD,
           RATE_OUT_LIMIT, RATE_I_LIMIT, DTERM_LPF_HZ);

  madgwick_init(&ahrs, MADGWICK_BETA, MADGWICK_ZETA);
  accumReset(&imuAcc);

#if ENABLE_AUTO_CALIB
  if (!calibrateImu()) {
    Serial.println("[FATAL] Hieu chuan that bai - dung han.");
    while (true) delay(500);
  }
#else
  // Cong tac mem ENABLE_AUTO_CALIB = 0: bo qua hieu chuan.
  // Cac bien giu nguyen gia tri khoi tao: gyroBias* = 0, pitchOffset/rollOffset = 0,
  // accMagRef = 1.0f. Van phai bat co nay, neu khong canArm() se chan vinh vien.
  imuCalibrated = true;
  Serial.println("[CAL] *** DA TAT HIEU CHUAN TU DONG (ENABLE_AUTO_CALIB = 0) ***");
  Serial.println("[CAL] Bias gyro = 0 va offset LEVEL = 0. Goc 0 do bay gio la mat");
  Serial.println("[CAL] phang ngang cua CHIP, khong phai mat phang luc khoi dong.");
  Serial.println("[CAL] Madgwick se tu hoc bias (zeta) nhung mat vai giay dau.");
#endif

#if ENABLE_OLED
  if (oledOK)
    xTaskCreatePinnedToCore(oledTask, "oled", OLED_TASK_STACK, NULL,
                            OLED_TASK_PRIO, &oledTaskHandle, OLED_TASK_CORE);
#endif

  // Trim tu the: kep truoc khi dung.
  pitchSetpoint = constrain(PITCH_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  rollSetpoint  = constrain(ROLL_TRIM_DEG,  -TRIM_DEG_MAX, TRIM_DEG_MAX);
  if (pitchSetpoint != 0.0f || rollSetpoint != 0.0f) {
    Serial.print("[TRIM] Setpoint tu the  P:"); Serial.print(pitchSetpoint, 2);
    Serial.print("  R:");                       Serial.println(rollSetpoint, 2);
  }
  if (PITCH_TRIM_DEG != pitchSetpoint || ROLL_TRIM_DEG != rollSetpoint)
    Serial.println("[TRIM] CANH BAO: trim vuot +/-5 do da bi kep - do la loi co khi, khong phai trim.");

  // Telemetry chi bat sau khi hieu chuan xong.
#if TELEM_ENABLE
  if (espnowReady) {
    if (xTaskCreatePinnedToCore(telemTask, "telem", TELEM_TASK_STACK, NULL,
                                TELEM_TASK_PRIO, &telemTaskH,
                                TELEM_TASK_CORE) != pdPASS) {
      Serial.println("[TELEM] LOI: khong tao duoc task - NODE B se khong co so lieu.");
    } else {
      Serial.print("[TELEM] Gui so lieu ve NODE B "); Serial.print(TELEM_HZ);
      Serial.print("Hz, ");   Serial.print((int)sizeof(TelemPacket));
      Serial.print(" byte/goi | uu tien "); Serial.print(TELEM_TASK_PRIO);
      Serial.print(" core ");               Serial.println(TELEM_TASK_CORE);
    }
  } else {
    Serial.println("[TELEM] ESP-NOW khong san sang - khong gui duoc so lieu.");
  }
#endif
#if !TELEM_SERIAL && !TUNE_MODE
  Serial.println("[TELEM] Serial cua drone KHONG in dong so lieu nua - doc tren NODE B.");
#endif

  if (GYRO_PITCH_SIGN < 0.0f || GYRO_ROLL_SIGN < 0.0f)
    Serial.println("[WARN] GYRO_PITCH/ROLL_SIGN = -1: vong toc do chong vong goc, SE LAT.");
  if (TRIM_M1 > 60 || TRIM_M1 < -60 || TRIM_M2 > 60 || TRIM_M2 < -60 ||
      TRIM_M3 > 60 || TRIM_M3 < -60 || TRIM_M4 > 60 || TRIM_M4 < -60)
    Serial.println("[WARN] |TRIM| > 60us: phan cung co van de.");

  Serial.print("[BAND] Dai dieu khien = "); Serial.print((int)(THROTTLE_BAND_PCT * 100.0f));
  Serial.print("% cua ga, kep ");           Serial.print(THROTTLE_BAND_MIN);
  Serial.print("..");                       Serial.print(THROTTLE_BAND_MAX);
  Serial.print("us, san motor ");           Serial.print(MOTOR_MIN_US);
  Serial.println("us. Vi du:");
  for (int h = 1200; h <= 1600; h += 200) {
    int b = throttleBandFor(h);
    Serial.print("        HOVER "); Serial.print(h);
    Serial.print(" -> +/-");        Serial.print(b);
    Serial.print("us  MIN ");       Serial.print(h - b);
    Serial.print("  MAX ");         Serial.println(h + b);
  }

  Serial.print("[OK] Dieu khien "); Serial.print(CONTROL_HZ);
  Serial.print("Hz, doc IMU ");     Serial.print(SAMPLE_HZ);
  Serial.print("Hz (trung binh ");  Serial.print(IMU_OVERSAMPLE);
  Serial.println(" mau).");
  Serial.print("[OK] Se TU DONG ARM khi nam ngang (<"); Serial.print((int)ARM_MAX_ANGLE);
  Serial.print(" do) va dung yen ");                    Serial.print((int)AUTO_ARM_HOLD_S);
  Serial.println("s. TRANH XA CANH QUAT.");
  if (!espnowReady)
    Serial.println("[WARN] ESP-NOW khong hoat dong - HOVER co dinh o mac dinh.");

  uint32_t t0 = micros();
  lastCtrlUs = lastRateUs = lastTelemUs = t0;
  nextSampleUs = t0 + SAMPLE_US;
}

// ============================================================================
// 17. LOOP: lay mau IMU o SAMPLE_HZ, chay dieu khien o CONTROL_HZ
// ============================================================================

// Trong so tin cay accel: 1 khi |a| gan |a| tinh, giam tuyen tinh ve 0 khi lech xa.
static float accelWeight(float magRatio) {
  float dev = fabsf(magRatio - 1.0f);
  if (dev <= ACC_DEV_FULL) return 1.0f;
  if (dev >= ACC_DEV_ZERO) return 0.0f;
  return (ACC_DEV_ZERO - dev) / (ACC_DEV_ZERO - ACC_DEV_FULL);
}

// Do bien do rung tung truc = hieu giua mau tho va gia tri da loc thong thap.
static void trackVibration(const ImuSample *s) {
  float dx = fabsf(s->ax - lpfAccX.y1);
  float dy = fabsf(s->ay - lpfAccY.y1);
  float dz = fabsf(s->az - lpfAccZ.y1);
  if (dx > vibX) vibX = dx;
  if (dy > vibY) vibY = dy;
  if (dz > vibZ) vibZ = dz;
}

// Uoc luong tu the tu mau IMU da trung binh.
static void estimateAttitude(const ImuSample *m, float dt) {
  float gcx = m->gx - gyroBiasX;
  float gcy = m->gy - gyroBiasY;
  float gcz = m->gz - gyroBiasZ;

  float axf = lpf_update(&lpfAccX, m->ax, dt);
  float ayf = lpf_update(&lpfAccY, m->ay, dt);
  float azf = lpf_update(&lpfAccZ, m->az, dt);

  float magRatio = sqrtf(axf * axf + ayf * ayf + azf * azf) / accMagRef;
  float accW = accelWeight(magRatio);
  accWSum += accW; accWCount++;

  // Chi hoc bias gyro khi accel tin duoc hoan toan va drone gan nhu dung yen.
  float rateMax = fmaxf(fabsf(gcx), fmaxf(fabsf(gcy), fabsf(gcz)));
  float wBias = (accW >= 1.0f && rateMax < BIAS_LEARN_MAX_RATE) ? 1.0f : 0.0f;

  madgwick_update(&ahrs,
                  gcx * DEG_TO_RAD, gcy * DEG_TO_RAD, gcz * DEG_TO_RAD,
                  axf, ayf, azf, accW, wBias, dt);

  float gvx, gvy, gvz;
  madgwick_gravity(&ahrs, &gvx, &gvy, &gvz);
#if MPU_MOUNT_Y_FORWARD
  pitch = atan2f( gvy, sqrtf(gvx * gvx + gvz * gvz)) * RAD_TO_DEG - pitchOffset;
  roll  = atan2f(-gvx, gvz) * RAD_TO_DEG - rollOffset;
  float pitchRateRaw =  gcx * GYRO_PITCH_SIGN;
  float rollRateRaw  =  gcy * GYRO_ROLL_SIGN;
#else
  pitch = atan2f( gvx, sqrtf(gvy * gvy + gvz * gvz)) * RAD_TO_DEG - pitchOffset;
  roll  = atan2f( gvy, gvz) * RAD_TO_DEG - rollOffset;
  float pitchRateRaw = -gcy * GYRO_PITCH_SIGN;
  float rollRateRaw  =  gcx * GYRO_ROLL_SIGN;
#endif
  float yawRateRaw = gcz * GYRO_YAW_SIGN;

  pitchRate = lpf_update(&lpfPitchRate, pitchRateRaw, dt);
  rollRate  = lpf_update(&lpfRollRate,  rollRateRaw,  dt);
  yawRate   = lpf_update(&lpfYawRate,   yawRateRaw,   dt);

  // Huong yaw: tich phan gyro Z, khong co la ban nen van troi cham theo bias.
  yawHeading = wrap180(yawHeading + yawRate * dt);

  if (isnan(pitch) || isnan(roll)) tripSafety("Goc NaN");
  else if (mode == FS_ARMED &&
           (fabsf(pitch) > SAFE_ANGLE || fabsf(roll) > SAFE_ANGLE))
    tripSafety("Nghieng qua 45 do");
}

// Ha ga tuyen tinh ve IDLE. Chay moi tick, khong phu thuoc cam bien.
static void runTripRamp(float dt) {
  tripRampT += dt;
  float k = 1.0f - constrain(tripRampT / TRIP_RAMP_S, 0.0f, 1.0f);
  escWriteAll(IDLE_THROTTLE + (int)((tripStartM1 - IDLE_THROTTLE) * k),
              IDLE_THROTTLE + (int)((tripStartM2 - IDLE_THROTTLE) * k),
              IDLE_THROTTLE + (int)((tripStartM3 - IDLE_THROTTLE) * k),
              IDLE_THROTTLE + (int)((tripStartM4 - IDLE_THROTTLE) * k));
  resetControllers();
  lastP = lastR = lastY = 0.0f;
  throttleCmd = IDLE_THROTTLE;
}

// Ramp ga em MIN -> HOVER (PID van chay), roi chay PID day du va tron dong co.
static void runFlightControl(float dt) {
  armRampT += dt;
  float r = constrain(armRampT / ARM_RAMP_S, 0.0f, 1.0f);

  throttleCmd = MIN_THROTTLE + (int)(throttleBand * r);

  // Chi cho khau I tich luy sau khi ramp xong (luc ramp drone con nam tren san).
  bool ramping = (r < 1.0f);
  cascaded_freeze_i(&pitchAxis, ramping);
  cascaded_freeze_i(&rollAxis,  ramping);
  // yawMixClipped: mixer da cat bot lenh yaw ma PID khong tu biet -> khoa I.
  yawRatePid.i_gate = (ramping || yawMixClipped) ? 0.0f : 1.0f;

  lastP = cascaded_compute(&pitchAxis, pitchSetpoint, pitch, pitchRate, dt);
  lastR = cascaded_compute(&rollAxis,  rollSetpoint,  roll,  rollRate,  dt);

  // Giu huong: luc ramp hoac khi co lenh xoay thi moc bam theo huong hien tai.
  float yawSp = constrain(yawRateSetpoint, -MAX_YAW_RATE, MAX_YAW_RATE);
  if (YAW_ANGLE_KP > 0.0f) {
    if (ramping || fabsf(yawRateSetpoint) > 1.0f) {
      yawHold = yawHeading;
    } else {
      yawSp = constrain(YAW_ANGLE_KP * wrap180(yawHold - yawHeading),
                        -MAX_YAW_RATE, MAX_YAW_RATE);
    }
  }
  lastY = pid_compute(&yawRatePid, yawSp, yawRate, dt);

  int m1, m2, m3, m4;
  motor_mix((float)throttleCmd,
            lastP * MIX_PITCH_SIGN, lastR * MIX_ROLL_SIGN, lastY * MIX_YAW_SIGN,
            &m1, &m2, &m3, &m4);
  escWriteAll(m1, m2, m3, m4);
}

// Mot chu ky dieu khien: trung binh cac mau da tich luy -> uoc luong -> dieu khien.
static void controlTick(uint32_t nowUs) {
  float dt = (float)(nowUs - lastCtrlUs) * 1e-6f;
  lastCtrlUs = nowUs;
  dt = constrain(dt, 0.0005f, 0.05f);

  // Chot HOVER_THROTTLE cho ca chu ky nay truoc khi bat dau tinh.
  updateThrottleAndLink(dt);

  ImuSample mean;
  bool imuOk = accumMean(&imuAcc, &mean);
  clipCount += imuAcc.clip;
  accumReset(&imuAcc);

  if (imuOk) {
    ctrlCount++;
    estimateAttitude(&mean, dt);
  }

  if (mode == FS_TRIPPED) {
    runTripRamp(dt);
  } else if (mode == FS_DISARMED) {
    escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);
    lastP = lastR = lastY = 0.0f;
    if (imuOk) updateAutoArm(dt);
  } else if (imuOk) {
    runFlightControl(dt);
  }
  // armed nhung khong co mau: giu nguyen xung cuoi, cho tick sau

  if (nowUs - lastRateUs >= 1000000UL) {
    lastRateUs = nowUs;
    loopHz     = ctrlCount;
    ctrlCount  = 0;
  }

  // Chup so lieu cho NODE B. Phai dung truoc printTelemetry.
  if (nowUs - lastTelemUs >= TELEM_US) {
    lastTelemUs = nowUs;
    telemPublish();
  }

  if (nowUs - lastSerialUs >= SERIAL_US) {
    lastSerialUs = nowUs;
    printTelemetry();
  }

#if ENABLE_OLED
  oledPublish();
#endif
}

void loop() {
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) return;

  if ((int32_t)(nowUs - nextSampleUs) > (int32_t)SAMPLE_US) {
    nextSampleUs = nowUs + SAMPLE_US;      // tre qua 1 chu ky -> bo nhip lo
    loopOverrun  = true;
  } else {
    nextSampleUs += SAMPLE_US;
    loopOverrun   = false;
  }

  ImuSample s;
  if (mpuReadRaw(&s)) {
    sensorFailCount = 0;
    trackVibration(&s);
    accumAdd(&imuAcc, &s);
  } else if (++sensorFailCount >= SENSOR_FAIL_LIMIT) {
    tripSafety("Mat ket noi MPU6050");
  }

  if (++sampleTick >= IMU_OVERSAMPLE) {
    sampleTick = 0;
    controlTick(nowUs);
  }
}
