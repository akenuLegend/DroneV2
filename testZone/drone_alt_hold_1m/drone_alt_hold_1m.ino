/*
 * DRONE_ALT_HOLD_1M - ESP32 + MPU6050 + BMP388 + MTF01P + 4x ESC
 *
 * Nen tang: testZone/drone_pilot/drone_pilot.ino
 * Nang cap: cat canh co kiem soat va giu cao do 0.50 m.
 *
 * CHAN   M1(FL,CW)=26  M2(FR,CCW)=13  M3(BR,CW)=14  M4(BL,CCW)=27
 *        MPU6050 SDA=32 SCL=33 | BMP388 SDA=21 SCL=22
 *        MTF01P TX->GPIO19, RX->GPIO23 (UART1, Micolink 115200)
 *
 * AN TOAN
 *   - ESC phai da hieu chuan bang sketch rieng (main/cali).
 *   - KHONG tu ARM. Can lenh ARM roi TAKEOFF hop le tu sender da ghep MAC.
 *   - MTF01P la cao do chinh; BMP388 du phong ngan han va doi chieu.
 *   - Mat link/mat range/qua cao/qua thoi gian -> TU HA CANH, khong giu ga vo han.
 *   - Khoa an toan chi go duoc bang RESET board.
 */

#define ENABLE_OLED             1   // SSD1306 128x64 dung chung bus AUX voi BMP388
#define ENABLE_AUTO_CALIB       1

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_BMP3XX.h>
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
static const int ESC_PIN_M1 = 26, ESC_PIN_M2 = 13, ESC_PIN_M3 = 14, ESC_PIN_M4 = 27;
static const int ESC_CH_M1 = 0, ESC_CH_M2 = 1, ESC_CH_M3 = 2, ESC_CH_M4 = 3;
static const int ESC_PWM_HZ = 200, ESC_PWM_BITS = 16;
static const int ESC_MIN_US = 1000, ESC_MAX_US = 2000;

// -- I2C --
#define MPU_SDA   32
#define MPU_SCL   33
#define MPU_FREQ  400000UL
#define MPU_ADDR  0x68

// -- Cam bien cao do --
#define AUX_SDA 21
#define AUX_SCL 22
#define AUX_FREQ 400000UL
#define MTF_RX_PIN 19
#define MTF_TX_PIN 23
#define MTF_BAUD 115200UL
#define ALT_TASK_CORE 0
#define ALT_TASK_PRIO 2
#define ALT_TASK_STACK 6144
#define AUX_MUTEX_WAIT_MS 2

#if ENABLE_OLED
  #define OLED_SDA AUX_SDA
  #define OLED_SCL AUX_SCL
  #define OLED_ADDR 0x3C
  #define OLED_TASK_CORE 0
  #define OLED_TASK_STACK 4096
  #define OLED_TASK_PRIO 1
  #define OLED_MS 250
#endif

// -- ESP-NOW --
#define WIFI_CHANNEL      1
#define ONLY_FROM_PEER    1
#define ESPNOW_TASK_PRIO  (configMAX_PRIORITIES - 1)
#define ESPNOW_TASK_CORE  0
#define ESPNOW_TASK_STACK 3072
#define ESPNOW_QUEUE_LEN  4
#define ESPNOW_MSG_MAX    24
#define LINK_TIMEOUT_MS   3000UL

static uint8_t peerMac[6] = {0xA0, 0xB7, 0x65, 0xF6, 0x43, 0x10};
// Doi CA HAI khoa trong drone va sender truoc khi dung that.
static const uint8_t espnowPmk[ESP_NOW_KEY_LEN] = {
  0x44,0x52,0x4F,0x4E,0x45,0x2D,0x31,0x4D,0x2D,0x50,0x4D,0x4B,0x30,0x30,0x31,0x21
};
static const uint8_t espnowLmk[ESP_NOW_KEY_LEN] = {
  0x41,0x4C,0x54,0x2D,0x48,0x4F,0x4C,0x44,0x2D,0x4C,0x4D,0x4B,0x30,0x30,0x31,0x21
};

// -- Telemetry nguoc --
#define TELEM_ENABLE     1
#define TELEM_HZ         10
#define TELEM_TASK_PRIO  1
#define TELEM_TASK_CORE  0
#define TELEM_TASK_STACK 3072
#define TELEM_MAGIC      0xD1
#define TELEM_VER        5
#define TELEM_SERIAL     0

// ID thu cong cua manifest cau hinh flight. Tang khi layout/tham so flight doi;
// build date/time in luc khoi dong tach rieng de truy vet dung binary.
static const uint32_t FLIGHT_CONFIG_SIGNATURE = 0xA41F1001UL;

// -- Nhip --
static const uint32_t CONTROL_HZ     = 250;
static const int      IMU_OVERSAMPLE = 4;
static const uint32_t SAMPLE_HZ      = CONTROL_HZ * IMU_OVERSAMPLE;
static const uint32_t SAMPLE_US      = 1000000UL / SAMPLE_HZ;
static const uint32_t SERIAL_US      = 100000UL;
static const uint32_t TELEM_US       = 1000000UL / TELEM_HZ;

// -- Ga --
static const int   IDLE_THROTTLE      = 1000;
// Tren khung dang test, 1400 us da bat dau nang con 1500 us leo rat manh.
// 1430 us chi la feed-forward hien tai, chua duoc calibrate theo pin/tai.
static const int   HOVER_DEFAULT      = 1430;
static const int   MOTOR_MIN_US       = 1050;
static const int   HOVER_MIN_ALLOWED  = 1050;
static const int   HOVER_MAX_ALLOWED  = 1750;
static const float THROTTLE_BAND_PCT  = 0.25f;
static const int   THROTTLE_BAND_MIN  = 60;
static const int   THROTTLE_BAND_MAX  = 350;
static const float HOVER_SLEW_US_PER_S = 100.0f;

// -- Arm / an toan --
static const float ARM_MAX_ANGLE     = 25.0f;
static const float ARM_MAX_RATE      = 15.0f;
static const float ARM_CONFIRM_S     = 2.0f;
static const float SPOOL_UP_S        = 0.4f;
static const float TRIP_RAMP_S       = 1.5f;
static const float LEVEL_OFFSET_MAX  = 10.0f;
static const int   SENSOR_FAIL_LIMIT = (int)(SAMPLE_HZ / 20);

// -- Tu dong cat canh / giu 0.50 m --
// MTF nam cao hon diem thap nhat cua drone 12 cm: tru offset nay de ra clearance.
static const float MTF_SENSOR_TO_DRONE_BOTTOM_M = 0.12f;
static const float ALT_TARGET_M             = 0.50f;
static const float ALT_SETPOINT_SLEW_MPS    = 0.12f;
// TAKEOFF ramp tu output hien tai; 1350 us chi la moc vao GROUND_TRANSITION.
static const int   TAKEOFF_START_US          = 1350;
static const float TAKEOFF_RAMP_US_PER_S     = 40.0f;
static const float TAKEOFF_LIFTOFF_M         = 0.06f;
static const float TAKEOFF_LIFTOFF_VZ_MPS    = 0.10f;
static const float TAKEOFF_LIFTOFF_CONFIRM_S = 0.15f;
// Task 4: chi ap dung khi con cham dat va trong handover ngay sau lift.
static const float TAKEOFF_GROUND_MAX_TILT_DEG      = 8.0f;
static const float TAKEOFF_GROUND_ABORT_ANGLE_DEG   = 12.0f;
static const float TAKEOFF_GROUND_ABORT_RATE_DPS    = 45.0f;
static const float TAKEOFF_GROUND_ABORT_CONFIRM_S   = 0.10f;
static const float TAKEOFF_LEVEL_SLEW_DEG_PER_S     = 3.0f;
static const float TAKEOFF_GROUND_MOTOR_DELTA_US    = 50.0f;
static const float TAKEOFF_ATTITUDE_TRANSFER_S      = 0.50f;
static const float ALT_LAND_SLEW_MPS        = 0.08f;
static const float ALT_LAND_NEAR_SLEW_MPS   = 0.04f;
static const float LAND_NEAR_GROUND_M       = 0.40f;
static const float ALT_THROTTLE_UP_US_PER_S         = 90.0f;
static const float ALT_THROTTLE_DOWN_US_PER_S       = 100.0f;
static const float LAND_THROTTLE_UP_US_PER_S        = 55.0f;
static const float LAND_THROTTLE_DOWN_US_PER_S      = 50.0f;
static const float LAND_NEAR_THROTTLE_DOWN_US_PER_S = 35.0f;
static const float LAND_THROTTLE_HEADROOM_US         = 50.0f;
static const float LAND_THROTTLE_MAX_US              = 1470.0f;
static const float BLIND_THROTTLE_DOWN_US_PER_S     = 25.0f;
static const float ALT_POS_KP               = 0.70f;
static const float ALT_POS_DEADBAND_M       = 0.015f;
static const float ALT_VEL_DEADBAND_MPS     = 0.025f;
static const float ALT_VEL_MAX_UP_MPS       = 0.20f;
static const float ALT_VEL_MAX_DOWN_MPS     = 0.12f;
static const float ALT_VEL_ACCEL_UP_MPS2    = 0.20f;
static const float ALT_VEL_ACCEL_DOWN_MPS2  = 0.35f;
static const float ALT_VEL_KP_US_PER_MPS    = 80.0f;
static const float ALT_VEL_KI_US_PER_M      = 30.0f;
static const float ALT_I_LIMIT_US           = 40.0f;
static const float ALT_CORRECTION_UP_US     = 70.0f;
static const float ALT_CORRECTION_DOWN_US   = 45.0f;
static const float ALT_HOLD_TOLERANCE_M     = 0.08f;
static const float ALT_MAX_M                = 1.60f;
static const float ALT_LANDED_M             = 0.10f;
static const float ALT_LANDED_VZ_MPS        = 0.25f;
static const float ALT_LANDED_CONFIRM_S     = 0.40f;
static const float LAND_TOUCHDOWN_DOWN_US_PER_S = 250.0f;
static const float LAND_TOUCHDOWN_TIMEOUT_S = 2.0f;
static const float LAND_MOTOR_RAMP_S         = 1.0f;
static const float TAKEOFF_NO_RISE_S        = 4.0f;
static const float TAKEOFF_TIMEOUT_S        = 20.0f;
static const float MAX_FLIGHT_S             = 60.0f;
static const float LAND_TIMEOUT_S           = 24.0f;
static const uint32_t ARMED_IDLE_TIMEOUT_MS = 15000UL;
static const uint32_t RANGE_FRESH_MS        = 250;
static const uint32_t RANGE_FAILSAFE_MS     = 800;
static const uint32_t BARO_FRESH_MS         = 250;
static const uint32_t ALT_COAST_MS           = 600;
static const uint32_t BLIND_LAND_MS         = 8000;
static const uint32_t ALTITUDE_SNAPSHOT_STALE_MS = 100;

// -- Hop nhat MTF01P + BMP388 --
// MTF chinh xac hon o 1 m; BMP chi them thanh phan cham va noi tiep khi MTF mat.
static const uint8_t ALT_SOURCE_NONE         = 0;
static const uint8_t ALT_SOURCE_MTF          = 1;
static const uint8_t ALT_SOURCE_BARO         = 2;
static const uint8_t ALT_SOURCE_FUSED        = 3;
static const uint8_t ALT_SOURCE_COAST        = 4;
static const uint8_t RANGE_VELOCITY_SAMPLES = 15;
static const float RANGE_FILTER_TAU_S        = 0.10f;
static const float RANGE_VELOCITY_TAU_S      = 0.20f;
static const float FUSION_BARO_HEIGHT_WEIGHT = 0.10f;
static const float FUSION_BARO_VEL_WEIGHT    = 0.30f;
static const float FUSION_HEIGHT_TAU_MTF_S   = 0.12f;
static const float FUSION_HEIGHT_TAU_BARO_S  = 0.45f;
static const float FUSION_VELOCITY_TAU_S     = 0.18f;
static const float BARO_ALIGN_TAU_S          = 8.0f;
static const float BARO_INNOVATION_GATE_M    = 0.45f;

// -- Giu vi tri ngang bang optical flow MTF01P --
// Hai dong mapping nay PHAI duoc kiem tra bang cach day drone bang tay, khong canh.
// Sau mapping: +forward = ve mui, +right = sang phai than drone.
#define FLOW_SWAP_XY 0
static const float FLOW_X_SIGN                    = +1.0f;
static const float FLOW_Y_SIGN                    = +1.0f;
static const uint8_t FLOW_GOOD_STATUS             = 0x01;
static const uint8_t FLOW_MIN_QUALITY             = 40;
static const uint32_t FLOW_MIN_HEIGHT_MM          = 50;
static const uint32_t FLOW_FRESH_MS               = 250;
static const float FLOW_FILTER_TAU_S              = 0.12f;
static const float FLOW_REFERENCE_HEIGHT_M        = 1.0f;
static const float FLOW_CM_S_TO_M_S               = 0.01f;
static const float FLOW_DEADBAND_MPS              = 0.015f;
static const float FLOW_ZERO_SIGMA_MULTIPLIER     = 3.0f;
static const float FLOW_MAX_DEADBAND_MPS          = 0.12f;
static const float FLOW_MAX_SPEED_MPS             = 3.0f;

// Vong ngang lay error_m = 0 - position_m quanh moc calib; khong co "vung cho
// phep" 50 cm. Toc do/gain o he SI nen khong phu thuoc ALT_TARGET_M.
static const float POS_HOLD_FLOW_CONFIRM_S         = 0.20f;
static const float POS_HOLD_KP_PER_S                = 0.40f;
static const float POS_HOLD_MAX_VEL_MPS            = 0.35f;
static const float POS_HOLD_VEL_KP_DEG_PER_MPS     = 3.0f;
static const float POS_HOLD_VEL_KI_DEG_PER_M       = 0.3f;
static const float POS_HOLD_I_LIMIT_DEG            = 1.5f;
static const float POS_HOLD_MAX_TILT_DEG           = 3.5f;
static const float POS_HOLD_TILT_SLEW_DEG_PER_S    = 8.0f;
static const float POS_HOLD_MIN_ALT_M               = 0.10f;
static const float POS_HOLD_ESTIMATE_LIMIT_M        = 2.0f;
// Voi mixer hien tai: tang toc ve truoc can pitch am, sang phai can roll duong.
static const float POS_HOLD_PITCH_SIGN             = -1.0f;
static const float POS_HOLD_ROLL_SIGN              = +1.0f;

// Trip theo SAI SO goc (muc tieu - thuc te) va phai keo dai, khong phai goc tuyet doi.
static const float SAFE_ANGLE_ERR    = 45.0f;
static const float SAFE_ANGLE_HOLD_S = 0.25f;

// Rung/clip vuot nguong thi CHAN ARM, khong chi bao cao.
static const float ARM_VIB_MAX   = 0.35f;   // g, bien do dinh moi truc
static const int   ARM_CLIP_MAX  = 0;       // so mau bao hoa cho phep

// -- Huong lap chip: 1 = +Y chip ve mui --
#define MPU_MOUNT_Y_FORWARD 1

// -- Dau mixer --
static const float MIX_PITCH_SIGN = +1.0f;
static const float MIX_ROLL_SIGN  = +1.0f;
static const float MIX_YAW_SIGN   = +1.0f;

// -- Can bang tai / trim --
static const int   TRIM_M1 = 0, TRIM_M2 = 0, TRIM_M3 = 0, TRIM_M4 = 0;
// Pin lech ve mui can them moment nang cap motor truoc. Gia tri duong trong
// mixer nay tang M1/M2 va giam M3/M4. Bu ramp theo collective de khong lam cac
// motor tach nhau dot ngot luc ARM/idle; day la feed-forward moment, khong phai
// mot lenh nghieng than drone. Can xac nhan/tinh chinh tren gia thu co day giu.
static const float CG_PITCH_COMP_US_AT_HOVER = +15.0f;
static const float CG_ROLL_COMP_US_AT_HOVER  = 0.0f;
static const float CG_COMP_MAX_US            = 30.0f;
// Level target phai la 0 do. Dung angle trim de che CG lech se tu tao gia toc
// ngang; moment tinh da duoc xu ly rieng boi CG_*_COMP o mixer.
static const float PITCH_TRIM_DEG = 0.0f;
static const float ROLL_TRIM_DEG  = 0.0f;
static const float TRIM_DEG_MAX   = 5.0f;

// -- Loc --
static const float GYRO_LPF_HZ     = 30.0f;
static const float GYRO_YAW_LPF_HZ = 30.0f;
static const float DTERM_LPF_HZ    = 25.0f;
static const float ACC_LPF_HZ      = 20.0f;
static const float SETPOINT_LPF_HZ = 20.0f;   // loc setpoint truoc khi lay sai so
static const float MADGWICK_BETA   = 0.1f;
static const float MADGWICK_ZETA   = 0.05f;

// -- Tin cay accel --
static const float ACC_DEV_FULL = 0.06f;
static const float ACC_DEV_ZERO = 0.25f;
static const int16_t CLIP_LSB   = 32000;

// -- PID --
static const float ATT_US_PER_DEG = 3.0f;

static const float PITCH_RATE_KP  = 1.5f;
static const float PITCH_RATE_KI  = 0.40f;
static const float PITCH_RATE_KD  = 0.04f;
static const float PITCH_ANGLE_KP = ATT_US_PER_DEG / PITCH_RATE_KP;

static const float ROLL_RATE_KP  = 1.5f;
static const float ROLL_RATE_KI  = 0.40f;
static const float ROLL_RATE_KD  = 0.04f;
static const float ROLL_ANGLE_KP = ATT_US_PER_DEG / ROLL_RATE_KP;

static const float YAW_RATE_KP  = 2.50f;
static const float YAW_RATE_KI  = 2.0f;
static const float YAW_RATE_KD  = 0.015f;
static const float YAW_ANGLE_KP = 1.0f;

static const float MAX_RATE     = 200.0f;
static const float MAX_YAW_RATE = 160.0f;

// Gia toc goc toi da (do/s^2) cho sqrt_controller. 0 = tro ve P thuan.
static const float ANGLE_ACCEL_MAX = 200.0f;

static const float RATE_OUT_LIMIT  = 120.0f;
static const float RATE_I_LIMIT    = 30.0f;
static const float RATE_I_LIMIT_PR = 20.0f;

static const float ITERM_RELAX_DEG  = 7.0f;
static const float ITERM_RELAX_RATE = ITERM_RELAX_DEG * PITCH_ANGLE_KP;

static const float BIAS_LEARN_MAX_RATE = 30.0f;
static const float ANGLE_DEADBAND      = 0.2f;

// -- Mixer: san tham quyen yaw (ArduPilot MOT_YAW_HEADROOM/1000) --
static const float YAW_HEADROOM_PCT = 0.20f;

// -- Tuyen tinh hoa luc day (ArduPilot MOT_THST_EXPO) --
// 0.0 = tat (dong nhat, hanh vi nhu ban goc).
// Bat len ~0.65 se DOI Y NGHIA cua so ga: phai do lai diem hover truoc khi bay.
static const float THRUST_EXPO = 0.0f;

// -- Bu ga theo goc nghieng (angle boost) --
#define ANGLE_BOOST_ON 1
static const float ANGLE_BOOST_COS_MIN = 0.5f;   // tran boost = 1/0.5 = 2x

// -- Hieu chuan gyro theo hoi tu --
static const int   CAL_BATCH        = 100;    // mau moi lo
static const int   CAL_MAX_BATCH    = 40;     // toi da ~12s
static const float CAL_CONVERGE_DPS = 0.15f;  // hai lo lien tiep phai khop trong nguong nay
static const float CAL_ACC_MOVE_G   = 0.02f;  // accel doi hon muc nay -> bo lo

// -- Trang thai dai dieu khien --
static volatile int hoverTarget    = HOVER_DEFAULT;
static float        hoverSlewF     = (float)HOVER_DEFAULT;
static int          HOVER_THROTTLE = HOVER_DEFAULT;
static int          throttleBand   = 100;

// ============================================================================
// 2. KIEU DU LIEU
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
  float i_gate;
  LowPassFilter d_lpf;
} PIDController;

typedef struct {
  PIDController rate;
  LowPassFilter sp_lpf;
  float         angle_kp;
} CascadedAxis;

typedef struct {
  float q0, q1, q2, q3;
  float beta, zeta;
  float bx, by, bz;
} Madgwick;

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  bool  clipped;
} ImuSample;

typedef struct {
  float ax, ay, az, gx, gy, gz;
  int   n, clip;
} ImuAccum;

// Co bao hoa tu mixer, doc o chu ky sau de khoa khau I dung truc.
typedef struct {
  bool roll, pitch, yaw, thr_lo, thr_hi;
} MixLimits;

typedef enum : uint8_t {
  FS_DISARMED = 0,
  FS_ARMED_IDLE,
  FS_TAKEOFF,
  FS_ALT_HOLD,
  FS_LANDING,
  FS_FAILSAFE,
  FS_TRIPPED
} FlightMode;

// Subphase noi bo; giu nguyen FlightMode de khong pha giao thuc/operator flow.
typedef enum : uint8_t {
  PHASE_NONE = 0,
  PHASE_TAKEOFF_SPOOL_UP,
  PHASE_TAKEOFF_GROUND_TRANSITION,
  PHASE_TAKEOFF_CONTROLLED_ASCENT,
  PHASE_TAKEOFF_ALTITUDE_CAPTURE,
  PHASE_LAND_CONTROLLED_DESCENT,
  PHASE_LAND_GROUND_APPROACH,
  PHASE_LAND_TOUCHDOWN_DETECTION,
  PHASE_LAND_MOTOR_RAMP_DOWN
} TransitionPhase;

typedef enum : uint8_t {
  CMD_HEARTBEAT = 0,
  CMD_ARM,
  CMD_TAKEOFF,
  CMD_LAND,
  CMD_DISARM,
  CMD_KILL,
  CMD_SET_HOVER
} CommandCode;

#define CMD_MAGIC 0xC3
#define CMD_VER   1

// Goi nhi phan co CRC: packet sai/truncated khong bao gio co the bien thanh ARM.
typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  ver;
  uint8_t  command;
  uint8_t  reserved;
  uint16_t sequence;
  uint16_t targetCm;
  uint16_t hoverUs;
  uint16_t crc;
} ControlPacket;

static_assert(sizeof(ControlPacket) == 12, "ControlPacket phai la 12 byte");

typedef struct {
  uint8_t buf[ESPNOW_MSG_MAX];
  int  len;
} EspNowMsg;

// Phai giong tung byte voi khoi cung ten trong node_b_sender.ino.
typedef struct __attribute__((packed)) {
  uint8_t  magic, ver, mode, flags;
  uint32_t ms;
  uint16_t loopHz, seq;
  float    pitch, roll, yaw;
  float    pitchRate, rollRate, yawRate;
  float    outP, outR, outY;
  float    iPitch, iRoll, iYaw;
  float    biasX, biasY, biasZ;
  float    calBiasX, calBiasY, calBiasZ;
  float    pitchOffset, rollOffset;
  float    vibX, vibY, vibZ;
  float    accW;
  int16_t  m1, m2, m3, m4;
  int16_t  throttle, hover, hoverTarget;
  uint16_t clip, rx, bad, lost;
  char     reason[20];

  float    altitudeM, verticalSpeedMps, altitudeSetpointM;
  float    rangeAltitudeM, baroAltitudeM;
  int16_t  altitudeCorrectionUs;
  uint16_t rangeAgeMs, baroAgeMs;
  uint8_t  altitudeSource;       // 0=none, 1=MTF, 2=BMP, 3=fused, 4=coast
  uint8_t  rangeStrength;
  uint8_t  lastCommand;
  uint8_t  transitionPhase;

  // Optical-flow da map vao truc than va vi tri tich phan tu moc calib.
  float    flowForwardMps, flowRightMps;
  float    positionForwardM, positionRightM;
  float    positionPitchDeg, positionRollDeg;
  uint16_t flowAgeMs;
  uint8_t  flowQuality;
  uint8_t  flowReserved;

  // Telemetry v5: giu layout 248 byte cua v4, doi identity de ghep dung binary
  // position-hold da chuan hoa; van do duoc timing/sensor thay vi
  // suy doan tu ODR danh nghia. Don vi rate la 0.1Hz, duration la microsecond.
  float    altitudeVelocitySetpointMps;
  float    altitudeIntegralUs;
  uint32_t configSignature;
  uint16_t controlDtUs, controlDtMaxUs, controlOverruns;
  uint16_t mtfRateHz10, mtfGapMaxUs;
  uint16_t bmpRateHz10, bmpReadMaxUs, auxWaitMaxUs;
  uint16_t altitudeGapMaxUs, uartBacklogMax;
  uint16_t mtfBadCrc, mtfStale, bmpReadFail, auxLockMiss;
} TelemPacket;

static_assert(sizeof(TelemPacket) == 248, "TelemPacket doi kich thuoc - sua sender_1m.ino!");
static_assert(sizeof(TelemPacket) <= 250, "ESP-NOW toi da 250 byte/goi.");

typedef struct {
  float altitudeM;
  float verticalSpeedMps;
  float rangeAltitudeM;
  float baroAltitudeM;
  uint32_t rangeAgeMs;
  uint32_t baroAgeMs;
  uint8_t source;
  uint8_t rangeStrength;
  float flowForwardMps;
  float flowRightMps;
  uint32_t flowAgeMs;
  uint8_t flowQuality;
  uint32_t publishedMs;
  uint16_t mtfRateHz10;
  uint16_t mtfGapMaxUs;
  uint16_t bmpRateHz10;
  uint16_t bmpReadMaxUs;
  uint16_t auxWaitMaxUs;
  uint16_t altitudeGapMaxUs;
  uint16_t uartBacklogMax;
  uint16_t mtfBadCrc;
  uint16_t mtfStale;
  uint16_t bmpReadFail;
  uint16_t auxLockMiss;
  bool calibrated;
  bool rangeFresh;
  bool baroFresh;
  bool flowFresh;
} AltitudeSnapshot;

void telemTask(void *arg);
void espnowTask(void *arg);
void altitudeTask(void *arg);
static bool landingNearGround();

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
#else
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len);
#endif

#if ENABLE_OLED
typedef struct {
  // 5 dong flight data + 2 dong WHY/ACT de chan doan failsafe ngay tren OLED.
  float altitudeM, altitudeSetpointM, verticalSpeedMps;
  float flowForwardMps, flowRightMps;
  float positionForwardM, positionRightM;
  int throttle;
  uint8_t altitudeSource, flowQuality;
  FlightMode mode;
  bool linkUp, flowFresh;
  const char *reason;
} OledSnap;
void oledTask(void *arg);
#endif

// ============================================================================
// 3. BIEN TOAN CUC
// ============================================================================

static LowPassFilter lpfPitchRate, lpfRollRate, lpfYawRate;
static LowPassFilter lpfAccX, lpfAccY, lpfAccZ;
static CascadedAxis  pitchAxis, rollAxis;
static PIDController yawRatePid;
static Madgwick      ahrs;
static ImuAccum      imuAcc;
static MixLimits     mixLimit = {false, false, false, false, false};

static float pitch = 0.0f, roll = 0.0f, yawHeading = 0.0f;
static float pitchRate = 0.0f, rollRate = 0.0f, yawRate = 0.0f;
static float pitchOffset = 0.0f, rollOffset = 0.0f;
static float pitchSetpoint = 0.0f, rollSetpoint = 0.0f, yawRateSetpoint = 0.0f;
static float yawHold = 0.0f;

static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;
static float accMagRef = 1.0f;
static bool  imuCalibrated = false;

static FlightMode  mode = FS_DISARMED;
static const char *modeReason = "cho lenh ARM";

static int   throttleCmd = IDLE_THROTTLE;
static int   curM1 = IDLE_THROTTLE, curM2 = IDLE_THROTTLE;
static int   curM3 = IDLE_THROTTLE, curM4 = IDLE_THROTTLE;
static float lastP = 0.0f, lastR = 0.0f, lastY = 0.0f;

static float armHoldT = 0.0f, tripRampT = 0.0f, tripAngleT = 0.0f;
static float spoolRatio = 0.0f;
static int   tripStartM1, tripStartM2, tripStartM3, tripStartM4;

static uint32_t nextSampleUs = 0, lastCtrlUs = 0, lastSerialUs = 0;
static uint32_t lastRateUs = 0, lastTelemUs = 0;
static uint32_t ctrlCount = 0;
static uint16_t loopHz = 0;
static int      sampleTick = 0, sensorFailCount = 0;
static bool     loopOverrun = false;
static uint32_t controlDtUs = 0, controlDtMaxUs = 0, controlOverrunCount = 0;

static float    vibX = 0.0f, vibY = 0.0f, vibZ = 0.0f;
static float    vibHoldX = 0.0f, vibHoldY = 0.0f, vibHoldZ = 0.0f;
static float    accWSum = 0.0f;
static uint32_t accWCount = 0, clipCount = 0, clipHold = 0;

// -- ESP-NOW --
static QueueHandle_t     espnowQueue  = NULL;
static TaskHandle_t      espnowTaskH  = NULL;
static volatile uint32_t espnowRx = 0, espnowBad = 0, espnowLost = 0, espnowPing = 0;
static volatile uint32_t espnowLastMs = 0;
static bool              espnowReady = false;
static bool              linkUp = false, linkEverUp = false;
static volatile uint8_t  pendingCommand = CMD_HEARTBEAT;
static volatile uint16_t pendingTargetCm = 50;
static volatile uint16_t pendingHoverUs = HOVER_DEFAULT;
static uint16_t          lastActionSequence = 0;
static bool              haveActionSequence = false;
static uint8_t           lastActionCommand = CMD_HEARTBEAT;
static uint32_t          lastActionMs = 0;
static volatile uint8_t  lastCommand = CMD_HEARTBEAT;
static bool              armRequested = false;
static uint32_t          armRequestMs = 0;
static uint32_t          armedIdleStartMs = 0;

// -- Failsafe --
static bool  fsLanding = false;
static uint32_t blindLandStartMs = 0;
static int blindLandStartThrottle = HOVER_DEFAULT;
static float altitudeThrottleSlewF = (float)HOVER_DEFAULT;
static bool altitudeThrottleSlewLimited = false;

// -- Cao do --
static HardwareSerial MTFSerial(1);
static TwoWire I2C_AUX(1);
static Adafruit_BMP3XX bmp;
static SemaphoreHandle_t auxI2cMutex = NULL;
static TaskHandle_t altitudeTaskH = NULL;
static volatile uint32_t altitudeSeq = 0;
static AltitudeSnapshot altitudeShared = {};
static AltitudeSnapshot altitudeNow = {};
static volatile uint32_t altitudeSnapshotReadFailCount = 0;
static volatile float altitudeTiltCos = 1.0f;
static bool bmpReady = false;
static uint8_t bmpAddress = 0;
static float altitudeSetpointM = 0.0f;
static float altitudeVelocitySetpointMps = 0.0f;
static float altitudeIntegral = 0.0f;
static float altitudeCorrectionUs = 0.0f;
static float takeoffThrottleF = (float)MOTOR_MIN_US;
static float takeoffElapsedS = 0.0f, phaseElapsedS = 0.0f;
static float liftoffConfirmS = 0.0f;
static bool  takeoffLiftConfirmed = false, takeoffReachedClearance = false;
static float takeoffGroundPitchDeg = 0.0f, takeoffGroundRollDeg = 0.0f;
static float takeoffAttitudePitchSpDeg = 0.0f, takeoffAttitudeRollSpDeg = 0.0f;
static float takeoffGroundUnsafeS = 0.0f, takeoffAttitudeTransferS = 0.0f;
static bool  takeoffAttitudeProfileActive = false;
static float holdConfirmS = 0.0f;
static float landedConfirmS = 0.0f;
static uint32_t flightStartMs = 0;
static uint32_t landingStartMs = 0;
static TransitionPhase transitionPhase = PHASE_NONE;
static int landingRampStartThrottle = IDLE_THROTTLE;
static int landingRampStartM1 = IDLE_THROTTLE, landingRampStartM2 = IDLE_THROTTLE;
static int landingRampStartM3 = IDLE_THROTTLE, landingRampStartM4 = IDLE_THROTTLE;

// -- Position hold: vi tri trong he truc ngang co dinh tai moc calib ban dau. --
static float positionForwardM = 0.0f, positionRightM = 0.0f;
static float positionIntegralForward = 0.0f, positionIntegralRight = 0.0f;
static float positionPitchDeg = 0.0f, positionRollDeg = 0.0f;
static float positionYawReferenceDeg = 0.0f;
static float positionFlowConfirmS = 0.0f;
static float positionFlowLostS = 0.0f;
static bool  positionReferenceInitialized = false, positionHoldActive = false;

// -- Telemetry --
static volatile uint32_t telemSeq = 0;
static TelemPacket       telemShared;
static uint16_t          telemCount = 0;
static TaskHandle_t      telemTaskH = NULL;
static volatile uint32_t telemTxOk = 0, telemTxFail = 0;

#if ENABLE_OLED
  Adafruit_SSD1306 display(128, 64, &I2C_AUX, -1);
  static bool              oledOK = false;
  static TaskHandle_t      oledTaskHandle = NULL;
  static volatile uint32_t oledSeq = 0;
  static OledSnap          oledShared;
#endif

// ============================================================================
// 4. BO LOC THONG THAP (PT1/PT2)
// ============================================================================

static void lpf_init(LowPassFilter *f, float cutoff_hz, LPFOrder order) {
  f->order = order;
  f->tau   = 1.0f / (2.0f * PI * cutoff_hz);
  f->y1 = f->y2 = 0.0f;
}

static void lpf_reset(LowPassFilter *f) { f->y1 = f->y2 = 0.0f; }
static void lpf_preset(LowPassFilter *f, float v) { f->y1 = f->y2 = v; }

// NaN/Inf -> giu gia tri cuoi, khong tra 0 (mot cu giat lenh giua luc bay).
static float lpf_update(LowPassFilter *f, float x, float dt) {
  float last = (f->order == LPF_ORDER_2) ? f->y2 : f->y1;
  if (dt <= 0.0f || isnan(x) || isinf(x)) return last;

  float alpha = constrain(dt / (dt + f->tau), 0.0f, 1.0f);
  f->y1 = alpha * x + (1.0f - alpha) * f->y1;

  if (f->order == LPF_ORDER_2) {
    f->y2 = alpha * f->y1 + (1.0f - alpha) * f->y2;
    if (isnan(f->y2) || isinf(f->y2)) f->y2 = last;
    return f->y2;
  }
  if (isnan(f->y1) || isinf(f->y1)) f->y1 = last;
  return f->y1;
}

// ============================================================================
// 5. PID
// D lay tren measurement; chong bao hoa nhan tin hieu tu mixer qua i_gate.
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

// mixSat = mixer da cat lenh cua truc nay o chu ky truoc.
static float pid_compute(PIDController *p, float setpoint, float meas, float dt, bool mixSat) {
  if (dt <= 0.0f) return 0.0f;
  if (!isfinite(meas) || !isfinite(setpoint)) { pid_reset(p); return 0.0f; }

  float error  = setpoint - meas;
  float p_term = p->kp * error;

  float d_term = 0.0f;
  if (p->kd != 0.0f) {
    if (p->has_prev)
      d_term = -p->kd * lpf_update(&p->d_lpf, (meas - p->prev_meas) / dt, dt);
    p->prev_meas = meas;
    p->has_prev  = true;
  }

  // Chong bao hoa hai nguon: gioi han cua chinh PID, va co bao hoa tu mixer.
  float pre_out = p_term + p->ki * p->integral + d_term;
  bool sat_hi = (pre_out >  p->out_limit);
  bool sat_lo = (pre_out < -p->out_limit);
  bool own_ok = (!sat_hi && !sat_lo) || (sat_hi && error < 0.0f) || (sat_lo && error > 0.0f);

  // Mixer khong cho biet chieu bao hoa, nen chi cho khau I CO NHO LAI.
  bool mix_ok = !mixSat || (p->integral > 0.0f && error < 0.0f)
                        || (p->integral < 0.0f && error > 0.0f);

  if (own_ok && mix_ok && p->i_gate > 0.0f)
    p->integral = constrain(p->integral + error * dt * p->i_gate, -p->i_limit, p->i_limit);

  float out = p_term + p->ki * p->integral + d_term;
  if (isnan(out) || isinf(out)) { pid_reset(p); return 0.0f; }
  return constrain(out, -p->out_limit, p->out_limit);
}

static inline float soft_deadband(float e, float w) {
  if (w <= 0.0f) return e;
  float e2 = e * e, w2 = w * w;
  return e * (e2 / (e2 + w2));
}

static inline float wrap180(float a) {
  if (!isfinite(a)) return 0.0f;
  if (fabsf(a) <= 180.0f) return a;
  a = fmodf(a + 180.0f, 360.0f);
  if (a < 0.0f) a += 360.0f;
  return a - 180.0f;
}

// ============================================================================
// 6. VONG GOC: sqrt_controller
// Tuyen tinh khi sai so nho, chuyen sang can bac hai khi sai so lon, de toc do
// yeu cau luon phanh kip trong gioi han gia toc goc accel_max.
// ============================================================================

static float sqrt_controller(float error, float p, float accel_max, float dt) {
  float rate;
  if (accel_max <= 0.0f) {
    rate = error * p;
  } else if (p <= 0.0f) {
    rate = (error > 0.0f) ?  sqrtf(2.0f * accel_max * error)
         : (error < 0.0f) ? -sqrtf(2.0f * accel_max * -error) : 0.0f;
  } else {
    float lin = accel_max / (p * p);
    if (error > lin)       rate =  sqrtf(2.0f * accel_max * (error - lin * 0.5f));
    else if (error < -lin) rate = -sqrtf(2.0f * accel_max * (-error - lin * 0.5f));
    else                   rate = error * p;
  }
  if (dt > 0.0f) rate = constrain(rate, -fabsf(error) / dt, fabsf(error) / dt);
  return isfinite(rate) ? rate : 0.0f;
}

static void cascaded_init(CascadedAxis *c, float angle_kp,
                          float rate_kp, float rate_ki, float rate_kd) {
  c->angle_kp = angle_kp;
  pid_init(&c->rate, rate_kp, rate_ki, rate_kd,
           RATE_OUT_LIMIT, RATE_I_LIMIT_PR, DTERM_LPF_HZ);
  lpf_init(&c->sp_lpf, SETPOINT_LPF_HZ, LPF_ORDER_1);
}

static void cascaded_reset(CascadedAxis *c) {
  pid_reset(&c->rate);
  lpf_reset(&c->sp_lpf);
}

static float cascaded_compute(CascadedAxis *c, float angle_sp, float angle_meas,
                              float rate_meas, float dt, bool mixSat, float iGate) {
  float sp        = lpf_update(&c->sp_lpf, angle_sp, dt);
  float angle_err = soft_deadband(sp - angle_meas, ANGLE_DEADBAND);

  float desired_rate = sqrt_controller(angle_err, c->angle_kp, ANGLE_ACCEL_MAX, dt);
  desired_rate = constrain(desired_rate, -MAX_RATE, MAX_RATE);

  // Sai so goc cang lon cang it tich luy khau I.
  float relax = 1.0f - constrain(fabsf(desired_rate) / ITERM_RELAX_RATE, 0.0f, 1.0f);
  c->rate.i_gate = iGate * relax;

  return pid_compute(&c->rate, desired_rate, rate_meas, dt, mixSat);
}

// ============================================================================
// 7. MADGWICK AHRS
// ============================================================================

static void madgwick_init(Madgwick *m, float beta, float zeta) {
  m->q0 = 1.0f; m->q1 = m->q2 = m->q3 = 0.0f;
  m->beta = beta;
  m->zeta = zeta;
  m->bx = m->by = m->bz = 0.0f;
}

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
    qDot1 -= beta * s0; qDot2 -= beta * s1;
    qDot3 -= beta * s2; qDot4 -= beta * s3;
  }

  q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

  float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (!isfinite(qnorm) || qnorm < 1e-6f) { madgwick_init(m, m->beta, m->zeta); return; }
  float r = 1.0f / qnorm;
  m->q0 = q0 * r; m->q1 = q1 * r; m->q2 = q2 * r; m->q3 = q3 * r;
}

static void madgwick_gravity(const Madgwick *m, float *gx, float *gy, float *gz) {
  *gx = 2.0f * (m->q1 * m->q3 - m->q0 * m->q2);
  *gy = 2.0f * (m->q0 * m->q1 + m->q2 * m->q3);
  *gz = m->q0 * m->q0 - m->q1 * m->q1 - m->q2 * m->q2 + m->q3 * m->q3;
}

// ============================================================================
// 8. IMU
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
  int16_t rgx = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
  int16_t rgy = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)(((uint16_t)b[12] << 8) | b[13]);

  s->ax = rax / ACCEL_SCALE;  s->ay = ray / ACCEL_SCALE;  s->az = raz / ACCEL_SCALE;
  s->gx = rgx / GYRO_SCALE;   s->gy = rgy / GYRO_SCALE;   s->gz = rgz / GYRO_SCALE;

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
  if (who != 0x68 && who != 0x69 && who != 0x70 && who != 0x71 && who != 0x73) return false;

  if (!mpuWrite(MPU_REG_PWR_MGMT_1, 0x01)) return false;   // clock = gyro X PLL
  delay(50);
  if (!mpuWrite(MPU_REG_CONFIG,       0x03)) return false; // DLPF 44Hz
  if (!mpuWrite(MPU_REG_GYRO_CONFIG,  0x08)) return false; // +/-500 do/s
  if (!mpuWrite(MPU_REG_ACCEL_CONFIG, 0x08)) return false; // +/-4g
  if (!mpuWrite(MPU_REG_SMPLRT_DIV,   0x00)) return false; // 1kHz
  delay(50);

  uint8_t rCfg = 0, rGyro = 0, rAcc = 0, rDiv = 0;
  const bool readbackOk = mpuRead(MPU_REG_CONFIG, &rCfg) &&
                          mpuRead(MPU_REG_GYRO_CONFIG, &rGyro) &&
                          mpuRead(MPU_REG_ACCEL_CONFIG, &rAcc) &&
                          mpuRead(MPU_REG_SMPLRT_DIV, &rDiv);
  if (!readbackOk) {
    Serial.println("[MPU] LOI: khong doc lai duoc cau hinh!");
    return false;
  }
  Serial.printf("[MPU] WHO=0x%X DLPF=0x%X GYRO=0x%X ACCEL=0x%X DIV=0x%X\n",
                who, rCfg, rGyro, rAcc, rDiv);
  if (rCfg != 0x03 || rGyro != 0x08 || rAcc != 0x08 || rDiv != 0x00) {
    Serial.println("[MPU] LOI: cau hinh doc lai khac gia tri da ghi!");
    return false;
  }
  return true;
}

// ============================================================================
// 9. HIEU CHUAN - lay trung binh theo LO va cho hai lo lien tiep HOI TU
// Trung binh N mau co dinh van "thanh cong" khi drone dang troi cham; so hai lo
// lien tiep bat duoc dung ca do.
// ============================================================================

static bool calSampleBatch(int n, double *sg, double *sa, int *clip) {
  ImuSample s;
  int got = 0;
  sg[0] = sg[1] = sg[2] = sa[0] = sa[1] = sa[2] = 0.0;
  *clip = 0;
  for (int i = 0; i < n * 3 && got < n; i++) {
    if (mpuReadRaw(&s)) {
      sg[0] += s.gx; sg[1] += s.gy; sg[2] += s.gz;
      sa[0] += s.ax; sa[1] += s.ay; sa[2] += s.az;
      if (s.clipped) (*clip)++;
      got++;
    }
    delay(3);
  }
  if (got < n / 2) return false;
  for (int k = 0; k < 3; k++) { sg[k] /= got; sa[k] /= got; }
  return true;
}

static bool calibrateImu() {
  Serial.println("[CAL] Dat drone nam ngang, giu yen...");
#if ENABLE_OLED
  if (oledOK) {
    if (auxI2cMutex) xSemaphoreTake(auxI2cMutex, portMAX_DELAY);
    display.clearDisplay(); display.setCursor(0, 0);
    display.println("HIEU CHUAN");
    display.println("DAT MAT PHANG NGANG");
    display.println("GIU YEN!");
    display.display();
    if (auxI2cMutex) xSemaphoreGive(auxI2cMutex);
  }
#endif

  double g[3], a[3], gPrev[3] = {0, 0, 0}, aPrev[3] = {0, 0, 0};
  bool   havePrev = false;
  int    clip = 0, batch = 0;
  bool   converged = false;

  while (batch < CAL_MAX_BATCH && !converged) {
    if (!calSampleBatch(CAL_BATCH, g, a, &clip)) {
      Serial.println("[CAL] THAT BAI: doc cam bien loi.");
      return false;
    }
    if (clip > 0) {
      Serial.printf("[CAL] THAT BAI: %d mau bao hoa. Kiem tra day/nguon MPU.\n", clip);
      return false;
    }
    batch++;

    if (havePrev) {
      // Accel doi trong luc lay lo -> co nguoi dong vao, bo lo nay.
      double da = fabs(a[0] - aPrev[0]) + fabs(a[1] - aPrev[1]) + fabs(a[2] - aPrev[2]);
      double dg = fmax(fabs(g[0] - gPrev[0]), fmax(fabs(g[1] - gPrev[1]), fabs(g[2] - gPrev[2])));
      if (da < CAL_ACC_MOVE_G && dg < CAL_CONVERGE_DPS) converged = true;
      else Serial.printf("[CAL] lo %d: lech gyro %.3f do/s, accel %.4f g - chua hoi tu\n",
                         batch, dg, da);
    }
    for (int k = 0; k < 3; k++) { gPrev[k] = g[k]; aPrev[k] = a[k]; }
    havePrev = true;
  }

  if (!converged) {
    Serial.println("[CAL] THAT BAI: khong hoi tu - drone khong dung yen. Giu yen roi RESET.");
    return false;
  }

  gyroBiasX = (float)g[0]; gyroBiasY = (float)g[1]; gyroBiasZ = (float)g[2];

  float axm = (float)a[0], aym = (float)a[1], azm = (float)a[2];
  float anorm = sqrtf(axm * axm + aym * aym + azm * azm);
  if (anorm < 0.5f || anorm > 1.5f) {
    Serial.printf("[CAL] THAT BAI: |accel| = %.3f g - accel hong hoac drone dang rung.\n", anorm);
    return false;
  }
  accMagRef = anorm;

  // Snap quaternion ve tu the hien tai bang beta giam dan.
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

  Serial.printf("[CAL] Hoi tu sau %d lo | bias do/s X:%.3f Y:%.3f Z:%.3f\n",
                batch, gyroBiasX, gyroBiasY, gyroBiasZ);
  Serial.printf("[CAL] Level offset P:%.2f R:%.2f | |a| ref:%.4f\n",
                pitchOffset, rollOffset, accMagRef);

  if (fabsf(pitchOffset) > LEVEL_OFFSET_MAX || fabsf(rollOffset) > LEVEL_OFFSET_MAX)
    Serial.println("[CAL] CANH BAO: offset LEVEL qua lon => SE KHONG TU ARM.");
  if (fabsf(gyroBiasX) > 20.0f || fabsf(gyroBiasY) > 20.0f || fabsf(gyroBiasZ) > 20.0f)
    Serial.println("[CAL] CANH BAO: bias gyro > 20 do/s.");

  pitch = roll = yawHeading = 0.0f;
  lpf_reset(&lpfPitchRate); lpf_reset(&lpfRollRate); lpf_reset(&lpfYawRate);
  lpf_preset(&lpfAccX, axm); lpf_preset(&lpfAccY, aym); lpf_preset(&lpfAccZ, azm);
  accumReset(&imuAcc);
  imuCalibrated = true;
  Serial.println("[CAL] Xong.");
  return true;
}

// ============================================================================
// 10. CAO DO: MTF01P (chinh) + BMP388 (du phong ngan han)
// ============================================================================

static uint16_t crc16Ccitt(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

static inline uint32_t readLe32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline int16_t readLe16s(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

typedef struct {
  uint32_t sensorMs;
  uint32_t distanceMm;
  uint32_t receivedUs;
  uint8_t sequence;
  uint8_t strength;
  uint8_t status;
  int16_t flowVelX;
  int16_t flowVelY;
  uint8_t flowQuality;
  uint8_t flowStatus;
} MtfRangeFrame;

class MtfParser {
  enum State : uint8_t { HEAD, DEV, SYS, MSG, SEQ, LEN, PAYLOAD, CHECKSUM } state = HEAD;
  uint8_t payload[64] = {};
  uint8_t index = 0, message = 0, sequence = 0, length = 0, sum = 0;
  uint32_t lastByteUs = 0, lastSensorMs = 0;
  bool haveSensorTime = false;

public:
  uint32_t good = 0, badCrc = 0, stale = 0;

  void resetFrame() { state = HEAD; index = 0; sum = 0; }

  bool push(uint8_t b, uint32_t nowUs, MtfRangeFrame *out) {
    if (state != HEAD && (uint32_t)(nowUs - lastByteUs) > 20000UL) resetFrame();
    lastByteUs = nowUs;
    switch (state) {
      case HEAD:
        if (b == 0xEF) { sum = b; state = DEV; }
        break;
      case DEV: sum += b; state = SYS; break;
      case SYS: sum += b; state = MSG; break;
      case MSG: message = b; sum += b; state = SEQ; break;
      case SEQ: sequence = b; sum += b; state = LEN; break;
      case LEN:
        length = b; sum += b; index = 0;
        if (length > sizeof(payload)) resetFrame();
        else state = length ? PAYLOAD : CHECKSUM;
        break;
      case PAYLOAD:
        payload[index++] = b; sum += b;
        if (index >= length) state = CHECKSUM;
        break;
      case CHECKSUM: {
        const bool crcOk = (b == sum);
        const uint8_t msg = message, seq = sequence, len = length;
        resetFrame();
        if (!crcOk) { badCrc++; return false; }
        if (msg != 0x51 || len != 20) return false;

        const uint32_t sensorMs = readLe32(payload);
        bool fresh = !haveSensorTime || sensorMs > lastSensorMs;
        if (!fresh && lastSensorMs > sensorMs && lastSensorMs - sensorMs >= 5000UL)
          fresh = true; // MTF vua reboot: cho timeline moi khoi dong.
        if (!fresh) { stale++; return false; }
        haveSensorTime = true;
        lastSensorMs = sensorMs;

        out->sensorMs = sensorMs;
        out->distanceMm = readLe32(payload + 4);
        out->receivedUs = nowUs;
        out->sequence = seq;
        out->strength = payload[8];
        out->status = payload[10];
        // Payload 0x51: flow signed tai byte 12..15, quality/status tai 16..17.
        out->flowVelX = readLe16s(payload + 12);
        out->flowVelY = readLe16s(payload + 14);
        out->flowQuality = payload[16];
        out->flowStatus = payload[17];
        good++;
        return true;
      }
    }
    return false;
  }
};

typedef struct {
  float values[5];
  uint8_t count, next;
  float push(float x) {
    values[next] = x; next = (next + 1) % 5; if (count < 5) count++;
    float a[5];
    for (uint8_t i = 0; i < count; i++) a[i] = values[i];
    for (uint8_t i = 1; i < count; i++) {
      const float key = a[i]; int j = i - 1;
      while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
      a[j + 1] = key;
    }
    return a[count / 2];
  }
  void reset() { memset(values, 0, sizeof(values)); count = next = 0; }
} Median5;

class AltitudeEstimator {
  MtfParser parser;
  Median5 rangeMedian;
  float rangeEma = 0.0f, rangeVelocity = 0.0f, lastRangeMm = 0.0f;
  float rangeHistory[RANGE_VELOCITY_SAMPLES] = {};
  uint32_t rangeTime[RANGE_VELOCITY_SAMPLES] = {};
  uint8_t rangeCount = 0, rangeNext = 0, stepCount = 0;
  float stepCandidateMm = 0.0f;
  uint32_t lastRangeUs = 0, lastRangeSensorMs = 0;
  uint32_t lastBmpAttemptMs = 0, lastBmpGoodMs = 0;
  uint32_t lastCalRangeUs = 0, lastCalBmpMs = 0;

  // Flow duoc doi sang m/s theo height cua chinh sample truoc khi loc. Nhu vay
  // state median/EMA cung mot don vi vat ly khi drone thay doi cao do.
  Median5 flowMedianX, flowMedianY;
  float flowEmaX = 0.0f, flowEmaY = 0.0f;
  float flowBiasX = 0.0f, flowBiasY = 0.0f;
  float flowNoiseX = 0.0f, flowNoiseY = 0.0f;
  float flowForwardMps = 0.0f, flowRightMps = 0.0f;
  uint32_t lastFlowUs = 0, lastFlowSensorMs = 0;
  uint8_t flowQuality = 0;
  bool flowValid = false, flowFilterInitialized = false;

  float pressurePa = 0.0f, pressureZeroPa = 0.0f;
  float baroEma = 0.0f, baroVelocity = 0.0f, baroOffset = 0.0f;
  float baroHistory[12] = {};
  uint32_t baroTime[12] = {};
  uint8_t baroCount = 0, baroNext = 0;

  // Observer hop nhat: tich phan Vz de noi giua cac sample, sau do sua cham ve
  // cao do do duoc. Nho vay mat/phuc hoi mot sensor khong tao buoc nhay output.
  float fusedAltitude = 0.0f, fusedVelocity = 0.0f;
  uint32_t lastFusionUs = 0, fusionRangeSeenUs = 0, fusionBaroSeenMs = 0;
  uint32_t lastBaroAlignMs = 0;
  uint8_t fusionSource = ALT_SOURCE_NONE;
  bool fusionInitialized = false, baroAligned = false;

  uint32_t calStartMs = 0;
  double calRangeSum = 0.0, calRangeSq = 0.0, calPressureSum = 0.0;
  uint32_t calRangeN = 0, calPressureN = 0;
  double calFlowSumX = 0.0, calFlowSumY = 0.0;
  double calFlowSqX = 0.0, calFlowSqY = 0.0;
  uint32_t calFlowN = 0;
  float groundRangeM = 0.0f;
  bool calibrated = false;
  uint8_t rangeStrength = 0;

  // Chan doan do task nay so huu (chi ghi tren core 0). Max/counter la
  // monotonic de khong can reset cheo core; snapshot se saturate ve uint16_t.
  uint32_t diagWindowStartMs = 0;
  uint32_t diagMtfWindowFrames = 0, diagBmpWindowSamples = 0;
  uint32_t diagLastMtfSensorMs = 0, diagLastPollUs = 0;
  uint32_t diagMtfGapMaxUs = 0, diagBmpReadMaxUs = 0;
  uint32_t diagAuxWaitMaxUs = 0, diagAltitudeGapMaxUs = 0;
  uint32_t diagUartBacklogMax = 0, diagBmpReadFail = 0, diagAuxLockMiss = 0;
  uint16_t diagMtfRateHz10 = 0, diagBmpRateHz10 = 0;

  static uint16_t saturate16(uint32_t value) {
    return value > 65535UL ? (uint16_t)65535 : (uint16_t)value;
  }

  void noteMtfFrame(uint32_t sensorMs) {
    if (diagLastMtfSensorMs) {
      const uint32_t gap = (sensorMs - diagLastMtfSensorMs) * 1000UL;
      if (gap > diagMtfGapMaxUs) diagMtfGapMaxUs = gap;
    }
    diagLastMtfSensorMs = sensorMs;
    diagMtfWindowFrames++;
  }

  void updateDiagnosticRates(uint32_t nowMs) {
    if (!diagWindowStartMs) {
      diagWindowStartMs = nowMs;
      return;
    }
    const uint32_t elapsedMs = nowMs - diagWindowStartMs;
    if (elapsedMs < 1000UL) return;
    diagMtfRateHz10 = saturate16((diagMtfWindowFrames * 10000UL + elapsedMs / 2UL) /
                                 elapsedMs);
    diagBmpRateHz10 = saturate16((diagBmpWindowSamples * 10000UL + elapsedMs / 2UL) /
                                 elapsedMs);
    diagMtfWindowFrames = diagBmpWindowSamples = 0;
    diagWindowStartMs = nowMs;
  }

  float rangeVelocityFromHistory(float h, uint32_t sampleMs) {
    rangeHistory[rangeNext] = h; rangeTime[rangeNext] = sampleMs;
    rangeNext = (rangeNext + 1) % RANGE_VELOCITY_SAMPLES;
    if (rangeCount < RANGE_VELOCITY_SAMPLES) rangeCount++;
    if (rangeCount < 3) return 0.0f;
    const uint8_t oldest = (rangeNext + RANGE_VELOCITY_SAMPLES - rangeCount) %
                           RANGE_VELOCITY_SAMPLES;
    const uint32_t base = rangeTime[oldest];
    float mt = 0, mh = 0;
    for (uint8_t i = 0; i < rangeCount; i++) {
      const uint8_t k = (oldest + i) % RANGE_VELOCITY_SAMPLES;
      mt += (uint32_t)(rangeTime[k] - base) * 1e-3f; mh += rangeHistory[k];
    }
    mt /= rangeCount; mh /= rangeCount;
    float n = 0, d = 0;
    for (uint8_t i = 0; i < rangeCount; i++) {
      const uint8_t k = (oldest + i) % RANGE_VELOCITY_SAMPLES;
      const float t = (uint32_t)(rangeTime[k] - base) * 1e-3f - mt;
      n += t * (rangeHistory[k] - mh); d += t * t;
    }
    return d > 1e-8f ? n / d : 0.0f;
  }

  bool acceptRange(const MtfRangeFrame &f, float tiltCos) {
    if (f.status != 0x01 || f.strength < 10 || f.distanceMm < 30 || f.distanceMm > 12000)
      return false;
    if (!isfinite(tiltCos)) return false;
    const float dt = lastRangeSensorMs ? (uint32_t)(f.sensorMs - lastRangeSensorMs) * 1e-3f
                                        : 0.01f;
    if (lastRangeSensorMs && dt > 0.25f) {
      rangeMedian.reset(); rangeCount = rangeNext = 0; rangeEma = 0.0f; lastRangeMm = 0.0f;
      rangeVelocity = 0.0f;
    }
    if (lastRangeMm > 0.0f) {
      const float current = (float)f.distanceMm;
      const float allowed = 150.0f + 3000.0f * dt + 0.15f * fmaxf(lastRangeMm, current);
      if (fabsf(current - lastRangeMm) > allowed) {
        if (!stepCount || fabsf(current - stepCandidateMm) > fmaxf(80.0f, 0.15f * current)) {
          stepCandidateMm = current; stepCount = 1;
        } else stepCount++;
        if (stepCount < 3) return false;
      }
    }
    stepCount = 0;
    lastRangeMm = (float)f.distanceMm;
    const float verticalM = f.distanceMm * 0.001f * constrain(tiltCos, 0.50f, 1.0f);
    const float med = rangeMedian.push(verticalM);
    if (rangeEma == 0.0f) rangeEma = med;
    else rangeEma += (1.0f - expf(-constrain(dt, 0.001f, 0.25f) /
                                  RANGE_FILTER_TAU_S)) * (med - rangeEma);
    const float rawVz = rangeVelocityFromHistory(rangeEma, f.sensorMs);
    const float velocityAlpha = 1.0f - expf(-constrain(dt, 0.001f, 0.25f) /
                                            RANGE_VELOCITY_TAU_S);
    rangeVelocity += velocityAlpha * (constrain(rawVz, -3.0f, 3.0f) - rangeVelocity);
    lastRangeUs = f.receivedUs;
    lastRangeSensorMs = f.sensorMs;
    rangeStrength = f.strength;
    return true;
  }

  void acceptFlow(const MtfRangeFrame &f, bool rangeAccepted) {
    const uint32_t elapsedMs = millis() - calStartMs;
    flowQuality = f.flowQuality;
    const bool diagnosticOk = rangeAccepted && f.flowStatus == FLOW_GOOD_STATUS &&
                              f.flowQuality >= FLOW_MIN_QUALITY;

    // Khi drone dung yen trong calibration, lay trung binh raw lam zero-bias.
    if (!calibrated && diagnosticOk && elapsedMs >= 1000UL && elapsedMs < 4000UL) {
      calFlowSumX += f.flowVelX;
      calFlowSumY += f.flowVelY;
      calFlowSqX += (double)f.flowVelX * f.flowVelX;
      calFlowSqY += (double)f.flowVelY * f.flowVelY;
      calFlowN++;
    }

    const bool usable = calibrated && diagnosticOk &&
                        f.distanceMm >= FLOW_MIN_HEIGHT_MM;
    if (!usable) {
      flowValid = false;
      flowForwardMps = flowRightMps = 0.0f;
      return;
    }

    float dt = lastFlowSensorMs ? (float)(uint32_t)(f.sensorMs - lastFlowSensorMs) * 1e-3f
                                : 0.01f;
    if (!lastFlowSensorMs || dt <= 0.0f || dt > 0.25f) {
      flowMedianX.reset(); flowMedianY.reset();
      flowEmaX = flowEmaY = 0.0f;
      flowFilterInitialized = false;
      dt = 0.01f;
    }

    // MTF raw la cm/s tai FLOW_REFERENCE_HEIGHT_M. Scale bang vertical range
    // dong bo cua frame, doi sang m/s, roi moi median/EMA. Loc sau chuan hoa
    // tranh mang state raw cua cao do cu sang cao do moi.
    const float heightM = rangeEma > 0.0f ? rangeEma :
                          f.distanceMm * 0.001f * altitudeTiltCos;
    const float velocityScale = (heightM / FLOW_REFERENCE_HEIGHT_M) *
                                FLOW_CM_S_TO_M_S;
    const float sampleXMps = ((float)f.flowVelX - flowBiasX) * velocityScale * FLOW_X_SIGN;
    const float sampleYMps = ((float)f.flowVelY - flowBiasY) * velocityScale * FLOW_Y_SIGN;
    const float medX = flowMedianX.push(sampleXMps);
    const float medY = flowMedianY.push(sampleYMps);
    const float alpha = 1.0f - expf(-constrain(dt, 0.001f, 0.25f) / FLOW_FILTER_TAU_S);
    if (!flowFilterInitialized) {
      flowEmaX = medX; flowEmaY = medY;
      flowFilterInitialized = true;
    } else {
      flowEmaX += alpha * (medX - flowEmaX);
      flowEmaY += alpha * (medY - flowEmaY);
    }

    const float deadbandX = fminf(FLOW_MAX_DEADBAND_MPS,
      fmaxf(FLOW_DEADBAND_MPS,
            FLOW_ZERO_SIGMA_MULTIPLIER * flowNoiseX * velocityScale));
    const float deadbandY = fminf(FLOW_MAX_DEADBAND_MPS,
      fmaxf(FLOW_DEADBAND_MPS,
            FLOW_ZERO_SIGMA_MULTIPLIER * flowNoiseY * velocityScale));
    float xMps = fabsf(flowEmaX) < deadbandX ? 0.0f : flowEmaX;
    float yMps = fabsf(flowEmaY) < deadbandY ? 0.0f : flowEmaY;

#if FLOW_SWAP_XY
    const float forwardMps = yMps;
    const float rightMps = xMps;
#else
    const float forwardMps = xMps;
    const float rightMps = yMps;
#endif
    if (!isfinite(forwardMps) || !isfinite(rightMps) ||
        fabsf(forwardMps) > FLOW_MAX_SPEED_MPS || fabsf(rightMps) > FLOW_MAX_SPEED_MPS) {
      flowValid = false;
      flowForwardMps = flowRightMps = 0.0f;
      return;
    }
    flowForwardMps = forwardMps;
    flowRightMps = rightMps;
    flowQuality = f.flowQuality;
    lastFlowUs = f.receivedUs;
    lastFlowSensorMs = f.sensorMs;
    flowValid = true;
  }

  void drainMtfFrames() {
    const int backlog = MTFSerial.available();
    if (backlog > 0 && (uint32_t)backlog > diagUartBacklogMax)
      diagUartBacklogMax = (uint32_t)backlog;
    while (MTFSerial.available()) {
      MtfRangeFrame f;
      const uint32_t byteReadUs = micros();
      if (parser.push((uint8_t)MTFSerial.read(), byteReadUs, &f)) {
        noteMtfFrame(f.sensorMs);
        const bool rangeAccepted = acceptRange(f, altitudeTiltCos);
        acceptFlow(f, rangeAccepted);
      }
    }
  }

  void updateBmp(uint32_t nowMs) {
    if (!bmpReady || (uint32_t)(nowMs - lastBmpAttemptMs) < 22UL) return;
    lastBmpAttemptMs = nowMs;
    const uint32_t waitStartUs = micros();
    bool haveBus = true;
    if (auxI2cMutex)
      haveBus = xSemaphoreTake(auxI2cMutex, pdMS_TO_TICKS(AUX_MUTEX_WAIT_MS)) == pdTRUE;
    const uint32_t waitUs = micros() - waitStartUs;
    if (waitUs > diagAuxWaitMaxUs) diagAuxWaitMaxUs = waitUs;
    if (!haveBus) {
      diagAuxLockMiss++;
      return;
    }

    const uint32_t readStartUs = micros();
    const bool ok = bmp.performReading();
    const uint32_t readUs = micros() - readStartUs;
    if (readUs > diagBmpReadMaxUs) diagBmpReadMaxUs = readUs;
    if (auxI2cMutex) xSemaphoreGive(auxI2cMutex);
    if (!ok || !isfinite(bmp.pressure) || bmp.pressure < 30000.0f || bmp.pressure > 110000.0f) {
      diagBmpReadFail++;
      return;
    }
    pressurePa = bmp.pressure;
    // Day la moc hoan tat read tren host, khong tu nhan la sample-time vat ly.
    lastBmpGoodMs = millis();
    nowMs = lastBmpGoodMs;
    diagBmpWindowSamples++;
    if (!calibrated || pressureZeroPa <= 0.0f) return;
    const float rawAlt = 44330.0f * (1.0f - powf(pressurePa / pressureZeroPa, 0.19029495f));
    if (!isfinite(rawAlt)) return;
    if (baroCount == 0) baroEma = rawAlt;
    else baroEma += 0.08f * (rawAlt - baroEma);

    baroHistory[baroNext] = baroEma; baroTime[baroNext] = nowMs;
    baroNext = (baroNext + 1) % 12; if (baroCount < 12) baroCount++;
    if (baroCount == 12) {
      const uint8_t oldest = baroNext;
      const float dt = (uint32_t)(nowMs - baroTime[oldest]) * 0.001f;
      if (dt > 0.05f) {
        const float rawVz = (baroEma - baroHistory[oldest]) / dt;
        baroVelocity += 0.10f * (constrain(rawVz, -3.0f, 3.0f) - baroVelocity);
      }
    }
  }

  void updateFusion(uint32_t nowMs, uint32_t rangeAge, uint32_t baroAge,
                    bool rangeFresh, bool baroFresh, float rangeAltitude) {
    const uint32_t nowUs = micros();
    const bool newRange = lastRangeUs && lastRangeUs != fusionRangeSeenUs;
    const bool newBaro = lastBmpGoodMs && lastBmpGoodMs != fusionBaroSeenMs;

    // Can BMP vao MTF mot lan khi bat dau, sau do chi hoc drift rat cham va chi
    // tren sample moi. Ban cu hoc 1% moi 1 ms nen BMP duoi theo noise MTF trong
    // khoang 0.1 s va khong con la nguon doc lap.
    if (rangeFresh && baroFresh) {
      if (!baroAligned) {
        baroOffset = rangeAltitude - baroEma;
        baroAligned = true;
        lastBaroAlignMs = nowMs;
      } else if (newRange || newBaro) {
        const float alignDt = lastBaroAlignMs
          ? constrain((uint32_t)(nowMs - lastBaroAlignMs) * 0.001f, 0.001f, 0.25f)
          : 0.02f;
        lastBaroAlignMs = nowMs;
        const float innovation = rangeAltitude - (baroEma + baroOffset);
        if (fabsf(innovation) <= BARO_INNOVATION_GATE_M) {
          const float alpha = 1.0f - expf(-alignDt / BARO_ALIGN_TAU_S);
          baroOffset += alpha * innovation;
        }
      }
    } else if (baroFresh && !baroAligned) {
      // P0 da duoc lay luc dat yen, nen 0 la fallback hop ly neu MTF chua san sang.
      baroOffset = 0.0f;
      baroAligned = true;
    }

    const float alignedBaro = baroEma + baroOffset;
    const bool sensorsAgree = rangeFresh && baroFresh &&
                              fabsf(rangeAltitude - alignedBaro) <= BARO_INNOVATION_GATE_M;
    const uint32_t freshestAge = rangeAge < baroAge ? rangeAge : baroAge;
    const bool coast = !rangeFresh && !baroFresh && fusionInitialized &&
                       freshestAge <= ALT_COAST_MS;

    float measuredAltitude = fusedAltitude;
    float measuredVelocity = 0.0f;
    bool haveMeasurement = false;
    if (sensorsAgree) {
      measuredAltitude = (1.0f - FUSION_BARO_HEIGHT_WEIGHT) * rangeAltitude +
                         FUSION_BARO_HEIGHT_WEIGHT * alignedBaro;
      measuredVelocity = (1.0f - FUSION_BARO_VEL_WEIGHT) * rangeVelocity +
                         FUSION_BARO_VEL_WEIGHT * baroVelocity;
      fusionSource = ALT_SOURCE_FUSED;
      haveMeasurement = true;
    } else if (rangeFresh) {
      measuredAltitude = rangeAltitude;
      measuredVelocity = rangeVelocity;
      fusionSource = ALT_SOURCE_MTF;
      haveMeasurement = true;
    } else if (baroFresh) {
      measuredAltitude = alignedBaro;
      measuredVelocity = baroVelocity;
      fusionSource = ALT_SOURCE_BARO;
      haveMeasurement = true;
    } else {
      fusionSource = coast ? ALT_SOURCE_COAST : ALT_SOURCE_NONE;
    }

    float dt = lastFusionUs ? (float)(uint32_t)(nowUs - lastFusionUs) * 1e-6f : 0.001f;
    dt = constrain(dt, 0.0005f, 0.05f);
    lastFusionUs = nowUs;

    if (!fusionInitialized && haveMeasurement) {
      fusedAltitude = measuredAltitude;
      fusedVelocity = constrain(measuredVelocity, -2.0f, 2.0f);
      fusionInitialized = true;
    } else if (fusionInitialized) {
      if (haveMeasurement) {
        const float velocityTarget = constrain(measuredVelocity, -2.0f, 2.0f);
        const float velocityAlpha = 1.0f - expf(-dt / FUSION_VELOCITY_TAU_S);
        fusedVelocity += velocityAlpha * (velocityTarget - fusedVelocity);

        const float prediction = fusedAltitude + fusedVelocity * dt;
        const float heightTau = rangeFresh ? FUSION_HEIGHT_TAU_MTF_S :
                                             FUSION_HEIGHT_TAU_BARO_S;
        const float heightAlpha = 1.0f - expf(-dt / heightTau);
        fusedAltitude = prediction + heightAlpha * (measuredAltitude - prediction);
      } else if (coast) {
        fusedAltitude += fusedVelocity * dt;
        fusedVelocity *= expf(-dt / 0.35f);
      } else {
        fusedVelocity = 0.0f;
      }
      fusedAltitude = constrain(fusedAltitude, -0.30f, 12.0f);
    }

    // Invariant cuoi truoc khi publish: loi noi bo/so hoc khong duoc bien
    // thanh mot altitude hop le tren core dieu khien.
    if (!isfinite(fusedAltitude) || !isfinite(fusedVelocity)) {
      fusedAltitude = fusedVelocity = 0.0f;
      fusionSource = ALT_SOURCE_NONE;
      fusionInitialized = false;
      baroAligned = false;
    }

    if (newRange) fusionRangeSeenUs = lastRangeUs;
    if (newBaro) fusionBaroSeenMs = lastBmpGoodMs;
  }

public:
  void beginCalibration() {
    calStartMs = millis(); calRangeSum = calRangeSq = calPressureSum = 0.0;
    calRangeN = calPressureN = 0; calibrated = false;
    calFlowSumX = calFlowSumY = calFlowSqX = calFlowSqY = 0.0; calFlowN = 0;
    flowBiasX = flowBiasY = flowNoiseX = flowNoiseY = 0.0f;
    flowMedianX.reset(); flowMedianY.reset();
    flowEmaX = flowEmaY = 0.0f; flowValid = flowFilterInitialized = false;
    lastFlowUs = lastFlowSensorMs = 0;
    fusedAltitude = fusedVelocity = 0.0f;
    lastFusionUs = fusionRangeSeenUs = fusionBaroSeenMs = lastBaroAlignMs = 0;
    fusionSource = ALT_SOURCE_NONE;
    fusionInitialized = baroAligned = false;
    lastCalRangeUs = lastCalBmpMs = 0;
  }

  void poll(uint32_t nowMs) {
    const uint32_t pollUs = micros();
    if (diagLastPollUs) {
      const uint32_t gap = pollUs - diagLastPollUs;
      if (gap > diagAltitudeGapMaxUs) diagAltitudeGapMaxUs = gap;
    }
    diagLastPollUs = pollUs;

    drainMtfFrames();
    updateBmp(nowMs);
    // BMP/I2C co the chan; xa UART lan nua truoc fusion/publication.
    drainMtfFrames();
    nowMs = millis();
    updateDiagnosticRates(nowMs);

    if (!calibrated) {
      const uint32_t elapsed = nowMs - calStartMs;
      if (elapsed >= 1000UL && elapsed < 4000UL) {
        if (lastRangeUs && lastRangeUs != lastCalRangeUs &&
            (uint32_t)(micros() - lastRangeUs) < RANGE_FRESH_MS * 1000UL) {
          calRangeSum += rangeEma; calRangeSq += (double)rangeEma * rangeEma; calRangeN++;
          lastCalRangeUs = lastRangeUs;
        }
        if (pressurePa > 0.0f && lastBmpGoodMs != lastCalBmpMs) {
          calPressureSum += pressurePa; calPressureN++; lastCalBmpMs = lastBmpGoodMs;
        }
      } else if (elapsed >= 4000UL) {
        if (calRangeN >= 100 && calPressureN >= 50) {
          const float mean = (float)(calRangeSum / calRangeN);
          const float variance = fmaxf(0.0f, (float)(calRangeSq / calRangeN) - mean * mean);
          if (sqrtf(variance) <= 0.03f) {
            groundRangeM = mean;
            pressureZeroPa = (float)(calPressureSum / calPressureN);
            if (calFlowN >= 50) {
              flowBiasX = (float)(calFlowSumX / calFlowN);
              flowBiasY = (float)(calFlowSumY / calFlowN);
              const float varianceX = fmaxf(0.0f,
                (float)(calFlowSqX / calFlowN) - flowBiasX * flowBiasX);
              const float varianceY = fmaxf(0.0f,
                (float)(calFlowSqY / calFlowN) - flowBiasY * flowBiasY);
              flowNoiseX = sqrtf(varianceX);
              flowNoiseY = sqrtf(varianceY);
              Serial.printf("[FLOW] zero X=%+.2f Y=%+.2f sigma=%.2f/%.2f (%lu mau)\n",
                            flowBiasX, flowBiasY, flowNoiseX, flowNoiseY,
                            (unsigned long)calFlowN);
            } else {
              Serial.printf("[FLOW] CANH BAO: chi %lu mau zero; dung bias 0.\n",
                            (unsigned long)calFlowN);
            }
            calibrated = true; baroEma = baroVelocity = baroOffset = 0.0f;
            baroCount = baroNext = 0;
            Serial.printf("[ALT] CAL OK ground=%.3fm P0=%.2fPa\n", groundRangeM, pressureZeroPa);
          } else Serial.printf("[ALT] CAL lap lai: range sigma %.3fm\n", sqrtf(variance));
        } else Serial.printf("[ALT] CAL lap lai: MTF=%lu BMP=%lu mau\n",
                             (unsigned long)calRangeN, (unsigned long)calPressureN);
        if (!calibrated) beginCalibration();
      }
    }

    const uint32_t rangeAge = lastRangeUs ? (uint32_t)(micros() - lastRangeUs) / 1000UL : UINT32_MAX;
    const uint32_t baroAge = lastBmpGoodMs ? nowMs - lastBmpGoodMs : UINT32_MAX;
    const uint32_t flowAge = lastFlowUs ? (uint32_t)(micros() - lastFlowUs) / 1000UL : UINT32_MAX;
    const bool rangeFresh = calibrated && rangeAge <= RANGE_FRESH_MS;
    const bool baroFresh = calibrated && baroAge <= BARO_FRESH_MS && pressurePa > 0.0f;
    const bool flowFresh = calibrated && flowValid && flowAge <= FLOW_FRESH_MS;
    // rangeEma van la khoang cach sensor-ground de scale flow. Cao do dieu khien
    // la clearance cua than drone sau khi tru dung offset lap MTF 12 cm.
    const float rangeAlt = rangeEma - MTF_SENSOR_TO_DRONE_BOTTOM_M;
    if (calibrated)
      updateFusion(nowMs, rangeAge, baroAge, rangeFresh, baroFresh, rangeAlt);

    AltitudeSnapshot s = {};
    s.calibrated = calibrated; s.rangeFresh = rangeFresh; s.baroFresh = baroFresh;
    s.publishedMs = nowMs;
    s.rangeAltitudeM = rangeAlt; s.baroAltitudeM = baroEma + baroOffset;
    s.rangeAgeMs = rangeAge; s.baroAgeMs = baroAge; s.rangeStrength = rangeStrength;
    s.flowForwardMps = flowForwardMps; s.flowRightMps = flowRightMps;
    s.flowAgeMs = flowAge; s.flowQuality = flowQuality; s.flowFresh = flowFresh;
    s.mtfRateHz10 = diagMtfRateHz10;
    s.mtfGapMaxUs = saturate16(diagMtfGapMaxUs);
    s.bmpRateHz10 = diagBmpRateHz10;
    s.bmpReadMaxUs = saturate16(diagBmpReadMaxUs);
    s.auxWaitMaxUs = saturate16(diagAuxWaitMaxUs);
    s.altitudeGapMaxUs = saturate16(diagAltitudeGapMaxUs);
    s.uartBacklogMax = saturate16(diagUartBacklogMax);
    s.mtfBadCrc = saturate16(parser.badCrc);
    s.mtfStale = saturate16(parser.stale);
    s.bmpReadFail = saturate16(diagBmpReadFail);
    s.auxLockMiss = saturate16(diagAuxLockMiss);
    if (fusionInitialized && fusionSource != ALT_SOURCE_NONE) {
      s.altitudeM = fusedAltitude;
      s.verticalSpeedMps = fusedVelocity;
      s.source = fusionSource;
    }

    altitudeSeq = altitudeSeq + 1; __sync_synchronize();
    altitudeShared = s;
    __sync_synchronize(); altitudeSeq = altitudeSeq + 1;
  }
};

static AltitudeEstimator altitudeEstimator;

static bool altitudeRead(AltitudeSnapshot *out) {
  for (uint8_t tries = 0; tries < 4; tries++) {
    const uint32_t s = altitudeSeq; if (s & 1U) continue;
    AltitudeSnapshot candidate;
    __sync_synchronize(); candidate = altitudeShared; __sync_synchronize();
    if (altitudeSeq == s) { *out = candidate; return true; }
  }
  return false;
}

static bool sanitizeAltitudeSnapshot(AltitudeSnapshot *s) {
  bool valid = true;
  if (!isfinite(s->rangeAltitudeM)) {
    s->rangeAltitudeM = 0.0f;
    s->rangeFresh = false; s->rangeAgeMs = UINT32_MAX;
    valid = false;
  }
  if (!isfinite(s->baroAltitudeM)) {
    s->baroAltitudeM = 0.0f;
    s->baroFresh = false; s->baroAgeMs = UINT32_MAX;
    valid = false;
  }
  if (!isfinite(s->flowForwardMps) || !isfinite(s->flowRightMps)) {
    s->flowForwardMps = s->flowRightMps = 0.0f;
    s->flowFresh = false; s->flowAgeMs = UINT32_MAX;
    valid = false;
  }
  if (s->source > ALT_SOURCE_COAST || !isfinite(s->altitudeM) ||
      !isfinite(s->verticalSpeedMps) || s->altitudeM < -0.30f ||
      s->altitudeM > 12.0f) {
    s->altitudeM = s->verticalSpeedMps = 0.0f;
    s->source = ALT_SOURCE_NONE;
    s->rangeFresh = s->baroFresh = s->flowFresh = false;
    s->rangeAgeMs = s->baroAgeMs = s->flowAgeMs = UINT32_MAX;
    valid = false;
  }
  return valid;
}

void altitudeTask(void *arg) {
  (void)arg;
  Serial.printf("[SCHED] altitude core=%d prio=%u\n", xPortGetCoreID(),
                (unsigned)uxTaskPriorityGet(NULL));
  altitudeEstimator.beginCalibration();
  for (;;) { altitudeEstimator.poll(millis()); vTaskDelay(pdMS_TO_TICKS(1)); }
}

static bool beginBmp388() {
  const uint8_t addresses[] = {0x77, 0x76};
  for (uint8_t address : addresses) {
    I2C_AUX.beginTransmission(address);
    if (I2C_AUX.endTransmission() != 0) continue;
    if (!bmp.begin_I2C(address, &I2C_AUX)) continue;
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    bmpAddress = address;
    return true;
  }
  return false;
}

// ============================================================================
// 11. DRIVER ESC (LEDC) + TUYEN TINH HOA LUC DAY
// Khong dung ledcWrite(): ledc_update_duty() cho toi bien chu ky PWM ke tiep.
// Thay bang: bat sig_out_en luc khoi tao, sau do chi ghi para_up.
// ============================================================================

#if defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32S3) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
  #error "Khoi 10 chi dung tren ESP32 classic."
#endif

#define ESC_LEDC_MODE   LEDC_LOW_SPEED_MODE
#define ESC_LEDC_TIMER  LEDC_TIMER_0
#define ESC_PARA_UP_BIT (1UL << 4)

// Dao nguoc duong cong luc day: luc day ~ (1-e)*pwm + e*pwm^2.
// e = 0 -> dong nhat.
static inline int escLinearize(int us) {
  if (THRUST_EXPO <= 0.0f) return us;
  const float e = THRUST_EXPO;
  float thrust = constrain((float)(us - ESC_MIN_US) / (float)(ESC_MAX_US - ESC_MIN_US),
                           0.0f, 1.0f);
  float act = ((e - 1.0f) + sqrtf((1.0f - e) * (1.0f - e) + 4.0f * e * thrust)) / (2.0f * e);
  act = constrain(act, 0.0f, 1.0f);
  return ESC_MIN_US + (int)lroundf(act * (float)(ESC_MAX_US - ESC_MIN_US));
}

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
  ledc_update_duty(ESC_LEDC_MODE, (ledc_channel_t)ch);
}

static inline void escWriteOne(int ch, int us) {
  ledc_set_duty(ESC_LEDC_MODE, (ledc_channel_t)ch, escUsToDuty(us));
  LEDC.channel_group[ESC_LEDC_MODE].channel[ch].conf0.val |= ESC_PARA_UP_BIT;
}

// Diem DUY NHAT trong file ghi xung xuong ESC.
static void escWriteAll(int v1, int v2, int v3, int v4) {
  curM1 = v1; curM2 = v2; curM3 = v3; curM4 = v4;
  escWriteOne(ESC_CH_M1, escLinearize(v1));
  escWriteOne(ESC_CH_M2, escLinearize(v2));
  escWriteOne(ESC_CH_M3, escLinearize(v3));
  escWriteOne(ESC_CH_M4, escLinearize(v4));
}

// ============================================================================
// 11. MIXER (X-frame)
//   M1(FL)=T+P+R+Y  M2(FR)=T+P-R-Y  M3(BR)=T-P-R+Y  M4(BL)=T-P+R-Y
// Thu tu uu tien: pitch/roll -> yaw (co san toi thieu) -> throttle.
// Moi lan cat deu bat co trong mixLimit de PID khoa khau I dung truc o tick sau.
// ============================================================================

static inline int throttleBandFor(int hover) {
  int b = (int)lroundf((float)hover * THROTTLE_BAND_PCT);
  b = constrain(b, THROTTLE_BAND_MIN, THROTTLE_BAND_MAX);
  int room = hover - MOTOR_MIN_US;
  if (b > room) b = (room > THROTTLE_BAND_MIN) ? room : THROTTLE_BAND_MIN;
  return b;
}

// Moment feed-forward do CG lech phai tang dan cung luc nang. O MOTOR_MIN,
// compensation bang 0 de bon ESC khoi dong deu; tai hover no dat gia tri cau
// hinh. Khong extrapolate qua hover va luon kep boi hard safety bound.
static float cgCompensationScale(float throttle) {
  const float span = fmaxf(1.0f, (float)HOVER_THROTTLE - (float)MOTOR_MIN_US);
  return constrain((throttle - (float)MOTOR_MIN_US) / span, 0.0f, 1.0f);
}

static float cgPitchCompensation(float throttle) {
  return constrain(CG_PITCH_COMP_US_AT_HOVER, -CG_COMP_MAX_US, CG_COMP_MAX_US) *
         cgCompensationScale(throttle);
}

static float cgRollCompensation(float throttle) {
  return constrain(CG_ROLL_COMP_US_AT_HOVER, -CG_COMP_MAX_US, CG_COMP_MAX_US) *
         cgCompensationScale(throttle);
}

static void motor_mix(float throttle, float p_ctrl, float r_ctrl, float y_ctrl,
                      int *m1, int *m2, int *m3, int *m4) {
  // Altitude controller phai co quyen ha base toi MOTOR_MIN_US de LAND.
  const int   minT  = MOTOR_MIN_US;
  const int   maxT  = min(ESC_MAX_US, HOVER_THROTTLE + throttleBand);
  const float avail = (float)(maxT - minT);

  mixLimit.roll = mixLimit.pitch = mixLimit.yaw = false;
  mixLimit.thr_lo = mixLimit.thr_hi = false;

  // (a) Yaw duoc giu truoc mot san tham quyen toi thieu.
  const float yFloor = avail * 0.5f * YAW_HEADROOM_PCT;
  const float yWant  = fabsf(y_ctrl);
  const float yKeep  = fminf(yWant, yFloor);

  // (b) Pitch/roll duoc phuc vu trong phan dai con lai.
  float p1 = +p_ctrl + r_ctrl;
  float p2 = +p_ctrl - r_ctrl;
  float p3 = -p_ctrl - r_ctrl;
  float p4 = -p_ctrl + r_ctrl;

  float pHi = fmaxf(fmaxf(p1, p2), fmaxf(p3, p4));
  float pLo = fminf(fminf(p1, p2), fminf(p3, p4));
  float spanPR   = pHi - pLo;
  float prBudget = avail - 2.0f * yKeep;
  if (prBudget < 0.0f) prBudget = 0.0f;

  if (spanPR > prBudget && spanPR > 0.0f) {
    float k = prBudget / spanPR;
    p1 *= k; p2 *= k; p3 *= k; p4 *= k;
    spanPR = prBudget;
    mixLimit.roll = mixLimit.pitch = true;
  }

  // (c) Yaw lay het phan dai con thua sau pitch/roll.
  float yRoom = fmaxf(0.5f * (avail - spanPR), yKeep);
  float y     = constrain(y_ctrl, -yRoom, yRoom);
  if (yWant - yRoom > 0.5f) mixLimit.yaw = true;

  float f1 = p1 + y, f2 = p2 - y, f3 = p3 + y, f4 = p4 - y;

  float hi = fmaxf(fmaxf(f1, f2), fmaxf(f3, f4));
  float lo = fminf(fminf(f1, f2), fminf(f3, f4));

  // (d) Tinh tien base vao dai hop le - hy sinh do cao de giu tu the.
  float base = throttle;
  if (base + hi > (float)maxT) { base = (float)maxT - hi; mixLimit.thr_hi = true; }
  if (base + lo < (float)minT) { base = (float)minT - lo; mixLimit.thr_lo = true; }

  *m1 = constrain((int)lroundf(base + f1) + TRIM_M1, minT, maxT);
  *m2 = constrain((int)lroundf(base + f2) + TRIM_M2, minT, maxT);
  *m3 = constrain((int)lroundf(base + f3) + TRIM_M3, minT, maxT);
  *m4 = constrain((int)lroundf(base + f4) + TRIM_M4, minT, maxT);
}

// Mixer rieng cho khung con bi san rang buoc: khong day base len de cuu motor
// phia thap, va khong motor nao vuot base takeoff + delta. Hy sinh authority
// tren san de tranh mot goc drone spool nhanh hon ramp collective.
static void motorMixGroundLimited(float throttle, float p_ctrl, float r_ctrl, float y_ctrl,
                                  int *m1, int *m2, int *m3, int *m4) {
  const int minT = MOTOR_MIN_US;
  const int normalMaxT = min(ESC_MAX_US, HOVER_THROTTLE + throttleBand);
  const int top = constrain((int)lroundf(throttle + TAKEOFF_GROUND_MOTOR_DELTA_US),
                            minT, normalMaxT);

  float f1 = +p_ctrl + r_ctrl + y_ctrl;
  float f2 = +p_ctrl - r_ctrl - y_ctrl;
  float f3 = -p_ctrl - r_ctrl + y_ctrl;
  float f4 = -p_ctrl + r_ctrl - y_ctrl;
  const float peak = fmaxf(fmaxf(fabsf(f1), fabsf(f2)),
                           fmaxf(fabsf(f3), fabsf(f4)));
  const float scale = peak > TAKEOFF_GROUND_MOTOR_DELTA_US
                    ? TAKEOFF_GROUND_MOTOR_DELTA_US / peak : 1.0f;
  f1 *= scale; f2 *= scale; f3 *= scale; f4 *= scale;

  const int r1 = (int)lroundf(throttle + f1) + TRIM_M1;
  const int r2 = (int)lroundf(throttle + f2) + TRIM_M2;
  const int r3 = (int)lroundf(throttle + f3) + TRIM_M3;
  const int r4 = (int)lroundf(throttle + f4) + TRIM_M4;
  *m1 = constrain(r1, minT, top); *m2 = constrain(r2, minT, top);
  *m3 = constrain(r3, minT, top); *m4 = constrain(r4, minT, top);

  const bool clipped = scale < 0.999f || r1 < minT || r1 > top ||
                       r2 < minT || r2 > top || r3 < minT || r3 > top ||
                       r4 < minT || r4 > top;
  mixLimit.roll = mixLimit.pitch = mixLimit.yaw = clipped;
  mixLimit.thr_lo = r1 < minT || r2 < minT || r3 < minT || r4 < minT;
  mixLimit.thr_hi = r1 > top || r2 > top || r3 > top || r4 > top;
}

// Bu ga theo goc nghieng: thanh phan thang dung cua luc day giam theo cos(tilt).
static inline float angleBoost(float thr_us) {
#if ANGLE_BOOST_ON
  float ct = cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD);
  if (!isfinite(ct)) return thr_us;
  float inv   = constrain(10.0f * ct, 0.0f, 1.0f);       // gan lat ngang -> thoi boost
  float boost = 1.0f / constrain(ct, ANGLE_BOOST_COS_MIN, 1.0f);
  return (float)IDLE_THROTTLE + (thr_us - (float)IDLE_THROTTLE) * inv * boost;
#else
  return thr_us;
#endif
}

// Handover per-motor tu mixer ground (khong angle boost) sang mixer airborne.
// Alpha=0 trung dung output ground; alpha=1 trung dung output normal.
static void motorMixTakeoffTransfer(float throttle, float p_ctrl, float r_ctrl, float y_ctrl,
                                    float alpha, int *m1, int *m2, int *m3, int *m4) {
  int g1, g2, g3, g4, n1, n2, n3, n4;
  motorMixGroundLimited(throttle, p_ctrl, r_ctrl, y_ctrl, &g1, &g2, &g3, &g4);
  const MixLimits groundLimits = mixLimit;
  motor_mix(angleBoost(throttle), p_ctrl, r_ctrl, y_ctrl, &n1, &n2, &n3, &n4);
  const MixLimits normalLimits = mixLimit;
  alpha = constrain(alpha, 0.0f, 1.0f);
  *m1 = (int)lroundf((float)g1 + ((float)n1 - (float)g1) * alpha);
  *m2 = (int)lroundf((float)g2 + ((float)n2 - (float)g2) * alpha);
  *m3 = (int)lroundf((float)g3 + ((float)n3 - (float)g3) * alpha);
  *m4 = (int)lroundf((float)g4 + ((float)n4 - (float)g4) * alpha);
  mixLimit.roll = groundLimits.roll || normalLimits.roll;
  mixLimit.pitch = groundLimits.pitch || normalLimits.pitch;
  mixLimit.yaw = groundLimits.yaw || normalLimits.yaw;
  mixLimit.thr_lo = groundLimits.thr_lo || normalLimits.thr_lo;
  mixLimit.thr_hi = groundLimits.thr_hi || normalLimits.thr_hi;
}

// ============================================================================
// 12. AN TOAN, ARM, FAILSAFE
// ============================================================================

// Reset controller demand only. The position estimate remains relative to the
// post-calibration origin for the whole powered session.
static void resetHorizontalController() {
  positionIntegralForward = positionIntegralRight = 0.0f;
  positionPitchDeg = positionRollDeg = 0.0f;
  positionFlowConfirmS = 0.0f;
  positionFlowLostS = 0.0f;
  positionHoldActive = false;
  pitchSetpoint = constrain(PITCH_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  rollSetpoint = constrain(ROLL_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
}

static void resetHorizontalReference() {
  positionForwardM = positionRightM = 0.0f;
  positionYawReferenceDeg = yawHeading;
  positionReferenceInitialized = false;
  resetHorizontalController();
}

static void resetControllers() {
  cascaded_reset(&pitchAxis);
  cascaded_reset(&rollAxis);
  pid_reset(&yawRatePid);
  yawHold = yawHeading;
  altitudeVelocitySetpointMps = 0.0f;
  altitudeIntegral = 0.0f;
  altitudeCorrectionUs = 0.0f;
  altitudeThrottleSlewLimited = false;
  resetHorizontalController();
}

static bool airborne() {
  return mode == FS_TAKEOFF || mode == FS_ALT_HOLD ||
         mode == FS_LANDING || mode == FS_FAILSAFE;
}

static const char *transitionPhaseName(TransitionPhase phase) {
  switch (phase) {
    case PHASE_TAKEOFF_SPOOL_UP:           return "SPOOL_UP";
    case PHASE_TAKEOFF_GROUND_TRANSITION:  return "GROUND_TRANSITION";
    case PHASE_TAKEOFF_CONTROLLED_ASCENT:  return "CONTROLLED_ASCENT";
    case PHASE_TAKEOFF_ALTITUDE_CAPTURE:   return "ALTITUDE_CAPTURE";
    case PHASE_LAND_CONTROLLED_DESCENT:    return "CONTROLLED_DESCENT";
    case PHASE_LAND_GROUND_APPROACH:       return "GROUND_APPROACH";
    case PHASE_LAND_TOUCHDOWN_DETECTION:   return "TOUCHDOWN_DETECTION";
    case PHASE_LAND_MOTOR_RAMP_DOWN:       return "MOTOR_RAMP_DOWN";
    default:                               return "NONE";
  }
}

static void setTransitionPhase(TransitionPhase next) {
  if (transitionPhase == next) return;
  transitionPhase = next;
  phaseElapsedS = 0.0f;
  Serial.printf("[PHASE] %s\n", transitionPhaseName(next));
}

static void captureLandingRampStart() {
  landingRampStartThrottle = throttleCmd;
  landingRampStartM1 = curM1; landingRampStartM2 = curM2;
  landingRampStartM3 = curM3; landingRampStartM4 = curM4;
}

static void beginTouchdownDetection() {
  if (transitionPhase == PHASE_LAND_TOUCHDOWN_DETECTION ||
      transitionPhase == PHASE_LAND_MOTOR_RAMP_DOWN) return;
  captureLandingRampStart();
  landedConfirmS = 0.0f;
  setTransitionPhase(PHASE_LAND_TOUCHDOWN_DETECTION);
}

static void beginLandingMotorRamp() {
  if (transitionPhase == PHASE_LAND_MOTOR_RAMP_DOWN) return;
  captureLandingRampStart();
  setTransitionPhase(PHASE_LAND_MOTOR_RAMP_DOWN);
}

static void tripSafety(const char *reason) {
  if (mode == FS_TRIPPED) return;
  mode        = FS_TRIPPED;
  transitionPhase = PHASE_NONE;
  takeoffAttitudeProfileActive = false;
  modeReason  = reason;
  tripRampT   = 0.0f;
  tripStartM1 = curM1; tripStartM2 = curM2;
  tripStartM3 = curM3; tripStartM4 = curM4;
  Serial.printf("\n[SAFETY] %s\n", reason);
}

static bool armConditionsMet() {
  if (mode == FS_TRIPPED)          { modeReason = "da khoa, phai RESET"; return false; }
  if (mode != FS_DISARMED)         { modeReason = "khong o DISARMED";    return false; }
  if (!espnowReady || !linkUp)     { modeReason = "chua co link sender"; return false; }
  if (!imuCalibrated)              { modeReason = "chua hieu chuan";     return false; }
  if (!altitudeNow.calibrated)     { modeReason = "cao do chua CAL";     return false; }
  if (!altitudeNow.rangeFresh)     { modeReason = "MTF01P khong san sang"; return false; }
  if (!altitudeNow.baroFresh)      { modeReason = "BMP388 khong san sang"; return false; }
  if (!altitudeNow.flowFresh)      { modeReason = "flow khong san sang"; return false; }
  if (!positionReferenceInitialized) {
    modeReason = "moc vi tri chua san sang"; return false;
  }
  if (!isfinite(positionForwardM) || !isfinite(positionRightM)) {
    modeReason = "vi tri khong huu han"; return false;
  }
  if (!isfinite(altitudeNow.altitudeM) ||
      !isfinite(altitudeNow.rangeAltitudeM) ||
      !isfinite(altitudeNow.baroAltitudeM)) {
    modeReason = "cao do khong huu han"; return false;
  }
  if (fabsf(altitudeNow.altitudeM) > 0.12f) {
    modeReason = "cao do dat chua ve 0"; return false;
  }
  if (!isfinite(pitch) || !isfinite(roll) || !isfinite(pitchRate) ||
      !isfinite(rollRate) || !isfinite(yawRate) || !isfinite(yawHeading)) {
    modeReason = "attitude khong huu han"; return false;
  }
  if (!isfinite(pitchOffset) || !isfinite(rollOffset) ||
      !isfinite(vibHoldX) || !isfinite(vibHoldY) || !isfinite(vibHoldZ)) {
    modeReason = "pre-arm khong huu han"; return false;
  }
  if (fabsf(pitchOffset) > LEVEL_OFFSET_MAX || fabsf(rollOffset) > LEVEL_OFFSET_MAX) {
    modeReason = "offset LEVEL qua lon"; return false;
  }
  if (fabsf(pitch) > ARM_MAX_ANGLE || fabsf(roll) > ARM_MAX_ANGLE) {
    modeReason = "chua nam ngang"; return false;
  }
  if (fabsf(pitchRate) > ARM_MAX_RATE || fabsf(rollRate) > ARM_MAX_RATE ||
      fabsf(yawRate) > ARM_MAX_RATE) {
    modeReason = "con dang rung/di chuyen"; return false;
  }
  // Rung/clip chan ARM: bat loi co khi truoc khi roi dat.
  if (vibHoldX > ARM_VIB_MAX || vibHoldY > ARM_VIB_MAX || vibHoldZ > ARM_VIB_MAX) {
    modeReason = "rung qua nguong"; return false;
  }
  if (clipHold > (uint32_t)ARM_CLIP_MAX) { modeReason = "accel bao hoa"; return false; }
  return true;
}

static void updateAutoArm(float dt) {
  if (!armRequested) { armHoldT = 0.0f; return; }
  if ((uint32_t)(millis() - armRequestMs) > 10000UL) {
    armRequested = false; armHoldT = 0.0f; modeReason = "ARM timeout"; return;
  }
  if (!armConditionsMet()) { armHoldT = 0.0f; return; }

  float before = armHoldT;
  armHoldT  += dt;
  modeReason = "dang xac nhan ARM";

  int a = (int)(ARM_CONFIRM_S - before);
  int b = (int)(ARM_CONFIRM_S - armHoldT);
  if (b < a && b >= 0) Serial.printf("[ARM] Tu dong arm sau %ds...\n", b + 1);

  if (armHoldT >= ARM_CONFIRM_S) {
    resetControllers();
    armRequested = false;
    armHoldT    = 0.0f;
    spoolRatio  = 0.0f;
    tripAngleT  = 0.0f;
    throttleCmd = IDLE_THROTTLE;
    mode        = FS_ARMED_IDLE;
    transitionPhase = PHASE_NONE;
    takeoffAttitudeProfileActive = false;
    modeReason  = "ARMED IDLE - cho TAKEOFF";
    armedIdleStartMs = millis();
    Serial.println("[ARM] ARMED IDLE - motor spool toi 1050us, cho TAKEOFF.");
  }
}

static void disarmNow(const char *reason) {
  mode = FS_DISARMED; modeReason = reason; armRequested = false;
  spoolRatio = armHoldT = 0.0f; altitudeSetpointM = 0.0f;
  transitionPhase = PHASE_NONE; phaseElapsedS = 0.0f;
  takeoffElapsedS = liftoffConfirmS = landedConfirmS = 0.0f;
  takeoffLiftConfirmed = takeoffReachedClearance = false;
  takeoffGroundUnsafeS = takeoffAttitudeTransferS = 0.0f;
  takeoffAttitudeProfileActive = false;
  altitudeThrottleSlewF = (float)IDLE_THROTTLE;
  resetControllers();
  escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);
}

static void startLanding(bool failsafe, const char *reason) {
  // LAND lap lai khong duoc reset phase/timeout, va LAND thuong khong duoc ha
  // cap mot FAILSAFE dang hoat dong. Chi cho phep nang mot LANDING len FAILSAFE.
  if (mode == FS_LANDING || mode == FS_FAILSAFE) {
    if (failsafe && mode == FS_LANDING) {
      mode = FS_FAILSAFE;
      modeReason = reason;
      fsLanding = true;
      Serial.printf("[FAILSAFE] %s (giu nguyen phase landing)\n", reason);
    }
    return;
  }
  if (!airborne()) return;
  const FlightMode priorMode = mode;
  takeoffAttitudeProfileActive = false;
  mode = failsafe ? FS_FAILSAFE : FS_LANDING;
  modeReason = reason; fsLanding = failsafe;
  altitudeSetpointM = fmaxf(altitudeNow.altitudeM, 0.0f);
  altitudeVelocitySetpointMps = 0.0f;
  altitudeIntegral = altitudeCorrectionUs = 0.0f;
  // Giu dung output hien tai; clamp ve hover o desired path va di qua slew,
  // khong tao buoc giam ngay tai bien mode.
  altitudeThrottleSlewF = (float)throttleCmd;
  altitudeThrottleSlewLimited = false;
  landedConfirmS = 0.0f; blindLandStartMs = 0;
  landingStartMs = millis();
  if (priorMode == FS_TAKEOFF && !takeoffLiftConfirmed) {
    // Abort khi chua roi dat: khong chay altitude controller nhu dang airborne.
    beginLandingMotorRamp();
  } else if (altitudeNow.rangeFresh && altitudeNow.rangeAltitudeM <= ALT_LANDED_M) {
    beginTouchdownDetection();
  } else {
    setTransitionPhase(landingNearGround() ? PHASE_LAND_GROUND_APPROACH :
                                             PHASE_LAND_CONTROLLED_DESCENT);
  }
  Serial.printf("[%s] %s\n", failsafe ? "FAILSAFE" : "LAND", reason);
}

// ============================================================================
// 13. ESP-NOW
//   song -> callback WiFi (prio 23, core 0) -> queue -> espnowTask (prio 24)
// ============================================================================

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

  // Timeout 0: khong bao gio duoc chan task WiFi.
  if (xQueueSend(espnowQueue, &m, 0) != pdTRUE) espnowLost = espnowLost + 1;
}

void espnowTask(void *arg) {
  (void)arg;
  EspNowMsg m;

  for (;;) {
    if (xQueueReceive(espnowQueue, &m, portMAX_DELAY) != pdTRUE) continue;
    if (m.len != (int)sizeof(ControlPacket)) { espnowBad = espnowBad + 1; continue; }
    ControlPacket p;
    memcpy(&p, m.buf, sizeof(p));
    const uint16_t expected = crc16Ccitt((const uint8_t *)&p, sizeof(p) - sizeof(p.crc));
    if (p.magic != CMD_MAGIC || p.ver != CMD_VER || p.crc != expected ||
        p.command > CMD_SET_HOVER) {
      espnowBad = espnowBad + 1;
      continue;
    }

    // Chi packet dung magic/version/CRC moi duoc refresh watchdog link.
    espnowLastMs = millis();
    if (p.command == CMD_HEARTBEAT) { espnowPing = espnowPing + 1; continue; }
    const uint32_t now = millis();
    if (haveActionSequence && p.sequence == lastActionSequence &&
        p.command == lastActionCommand && (uint32_t)(now - lastActionMs) < 2000UL) continue;
    bool published = false;
    if (p.command == CMD_KILL) {
      // KILL duoc phep thay the moi action chua claim; no khong co payload phu.
      __atomic_store_n(&pendingCommand, (uint8_t)CMD_KILL, __ATOMIC_RELEASE);
      published = true;
    } else if (__atomic_load_n(&pendingCommand, __ATOMIC_ACQUIRE) == CMD_HEARTBEAT) {
      // Single producer ghi payload truoc, roi CAS release moi cong bo command.
      // Neu control/producer doi mailbox trong luc nay, packet lap lai se thu lai.
      pendingTargetCm = p.targetCm;
      pendingHoverUs = p.hoverUs;
      uint8_t expected = CMD_HEARTBEAT;
      published = __atomic_compare_exchange_n(&pendingCommand, &expected, p.command,
                                               false, __ATOMIC_RELEASE,
                                               __ATOMIC_RELAXED);
    }
    if (published) {
      haveActionSequence = true;
      lastActionSequence = p.sequence;
      lastActionCommand = p.command;
      lastActionMs = now;
      lastCommand = p.command;
    }
    espnowRx = espnowRx + 1;
  }
}

static bool espnowInit() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.printf("[ESPNOW] MAC board nay: %s\n", WiFi.macAddress().c_str());
  Serial.printf("[ESPNOW] Nhan tu      : %02X:%02X:%02X:%02X:%02X:%02X\n",
                peerMac[0], peerMac[1], peerMac[2], peerMac[3], peerMac[4], peerMac[5]);

  espnowQueue = xQueueCreate(ESPNOW_QUEUE_LEN, sizeof(EspNowMsg));
  if (espnowQueue == NULL) {
    Serial.println("[ESPNOW] LOI: khong cap duoc queue.");
    return false;
  }
  if (xTaskCreatePinnedToCore(espnowTask, "espnow", ESPNOW_TASK_STACK, NULL,
                              ESPNOW_TASK_PRIO, &espnowTaskH, ESPNOW_TASK_CORE) != pdPASS) {
    Serial.println("[ESPNOW] LOI: khong tao duoc task.");
    return false;
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESPNOW] LOI: esp_now_init that bai.");
    return false;
  }
  if (esp_now_set_pmk(espnowPmk) != ESP_OK) {
    Serial.println("[ESPNOW] LOI: khong dat duoc PMK.");
    return false;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, peerMac, 6);
  peer.channel = WIFI_CHANNEL;
  memcpy(peer.lmk, espnowLmk, ESP_NOW_KEY_LEN);
  peer.encrypt = true;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESPNOW] CANH BAO: khong them duoc peer.");
    return false;
  }

  Serial.printf("[ESPNOW] San sang. Protocol nhi phan v%d + CRC | hover %d..%d\n",
                CMD_VER, HOVER_MIN_ALLOWED, HOVER_MAX_ALLOWED);
  return true;
}

// Xu ly command o core dieu khien. Task WiFi chi validate + ghi mailbox.
static void handlePendingCommand() {
  // Doc payload trong khi mailbox con occupied. Producer khong duoc sua payload
  // cua non-KILL cho den khi exchange claim xong; KILL khong dung payload nay.
  const uint8_t preview = __atomic_load_n(&pendingCommand, __ATOMIC_ACQUIRE);
  if (preview == CMD_HEARTBEAT) return;
  const uint16_t commandHoverUs = pendingHoverUs;
  // Claim + clear cung mot thao tac. Command producer publish sau exchange van
  // con trong mailbox, ke ca KILL, thay vi bi control xoa nham.
  const uint8_t command = __atomic_exchange_n(&pendingCommand,
                                               (uint8_t)CMD_HEARTBEAT,
                                               __ATOMIC_ACQUIRE);
  if (command == CMD_HEARTBEAT) return;

  switch (command) {
    case CMD_ARM:
      if (mode == FS_DISARMED) {
        armRequested = true; armRequestMs = millis(); armHoldT = 0.0f;
        Serial.println("[CMD] ARM - bat dau kiem tra pre-arm 2 giay.");
      }
      break;
    case CMD_TAKEOFF:
      if (mode == FS_ARMED_IDLE && altitudeNow.rangeFresh && altitudeNow.baroFresh &&
          altitudeNow.flowFresh && positionReferenceInitialized && linkUp &&
          fabsf(altitudeNow.altitudeM) <= 0.12f &&
          fabsf(pitch) <= ARM_MAX_ANGLE && fabsf(roll) <= ARM_MAX_ANGLE &&
          fabsf(pitchRate) <= ARM_MAX_RATE && fabsf(rollRate) <= ARM_MAX_RATE &&
          fabsf(yawRate) <= ARM_MAX_RATE) {
        if (HOVER_THROTTLE <= TAKEOFF_START_US) {
          Serial.printf("[CMD] TAKEOFF bi tu choi: hover %dus phai > moc ground %dus.\n",
                        HOVER_THROTTLE, TAKEOFF_START_US);
          break;
        }
        if (fmaxf(fabsf(pitch), fabsf(roll)) > TAKEOFF_GROUND_MAX_TILT_DEG) {
          Serial.printf("[CMD] TAKEOFF bi tu choi: mat nghieng P/R %.1f/%.1f vuot %.1f deg.\n",
                        pitch, roll, TAKEOFF_GROUND_MAX_TILT_DEG);
          break;
        }
        resetControllers();
        takeoffGroundPitchDeg = pitch; takeoffGroundRollDeg = roll;
        takeoffAttitudePitchSpDeg = pitch; takeoffAttitudeRollSpDeg = roll;
        pitchSetpoint = pitch; rollSetpoint = roll;
        lpf_preset(&pitchAxis.sp_lpf, pitchSetpoint);
        lpf_preset(&rollAxis.sp_lpf, rollSetpoint);
        takeoffGroundUnsafeS = takeoffAttitudeTransferS = 0.0f;
        takeoffAttitudeProfileActive = true;
        spoolRatio = 0.0f;
        // Bat dau tu output ARMED_IDLE that, khong nhay thang 1050 -> 1350 us.
        takeoffThrottleF = (float)constrain(throttleCmd, IDLE_THROTTLE, HOVER_THROTTLE);
        altitudeThrottleSlewF = takeoffThrottleF;
        altitudeSetpointM = fmaxf(0.0f, altitudeNow.altitudeM);
        takeoffElapsedS = phaseElapsedS = liftoffConfirmS = 0.0f;
        holdConfirmS = landedConfirmS = 0.0f;
        takeoffLiftConfirmed = takeoffReachedClearance = false;
        flightStartMs = millis(); fsLanding = false;
        mode = FS_TAKEOFF; modeReason = "tu cat canh den 0.50m";
        setTransitionPhase(takeoffThrottleF < (float)TAKEOFF_START_US
                           ? PHASE_TAKEOFF_SPOOL_UP
                           : PHASE_TAKEOFF_GROUND_TRANSITION);
        Serial.printf("[CMD] TAKEOFF -> ramp lien tuc tu %.0fus qua moc %dus, %.0fus/s.\n",
                      takeoffThrottleF, TAKEOFF_START_US, TAKEOFF_RAMP_US_PER_S);
        Serial.printf("[TAKEOFF] Capture attitude P/R %.2f/%.2f deg; ground motor <= base+%.0fus.\n",
                      takeoffGroundPitchDeg, takeoffGroundRollDeg,
                      TAKEOFF_GROUND_MOTOR_DELTA_US);
      } else Serial.println("[CMD] TAKEOFF bi tu choi: chua ARMED IDLE/range/link.");
      break;
    case CMD_LAND:
      startLanding(false, "lenh LAND tu sender");
      break;
    case CMD_DISARM:
      if (mode == FS_ARMED_IDLE || mode == FS_DISARMED) disarmNow("DISARM tu sender");
      else Serial.println("[CMD] DISARM bi tu choi khi dang bay; dung LAND hoac KILL.");
      break;
    case CMD_KILL:
      tripSafety("KILL khan cap tu sender");
      tripStartM1 = tripStartM2 = tripStartM3 = tripStartM4 = IDLE_THROTTLE;
      break;
    case CMD_SET_HOVER:
      if (mode == FS_DISARMED && commandHoverUs >= HOVER_MIN_ALLOWED &&
          commandHoverUs <= HOVER_MAX_ALLOWED) {
        hoverTarget = commandHoverUs;
        Serial.printf("[CMD] hover base=%u us\n", (unsigned)commandHoverUs);
      } else Serial.println("[CMD] HOVER bi tu choi: chi doi hop le khi DISARMED.");
      break;
    default: break;
  }
}

// Cap nhat link va base hover. Link mat khi airborne chi duoc phep dan den LAND.
static void updateThrottleAndLink(float dt) {
  uint32_t age = millis() - espnowLastMs;
  bool up = (espnowLastMs != 0) && (age < LINK_TIMEOUT_MS);
  if (up != linkUp) {
    linkUp = up;
    if (up) {
      Serial.println(linkEverUp ? "[LINK] DA KET NOI LAI." : "[LINK] Bat duoc NODE B.");
      linkEverUp = true;
    } else {
      Serial.println("[LINK] *** MAT KET NOI voi NODE B ***");
    }
  }

  handlePendingCommand();
  if (!linkUp) {
    armRequested = false;
    if (mode == FS_ARMED_IDLE) disarmNow("mat link khi ARMED IDLE");
    else if (mode == FS_TAKEOFF || mode == FS_ALT_HOLD)
      startLanding(true, "mat link sender");
  }
  if (mode == FS_ARMED_IDLE &&
      (uint32_t)(millis() - armedIdleStartMs) > ARMED_IDLE_TIMEOUT_MS)
    disarmNow("ARMED IDLE timeout");

  const float step = HOVER_SLEW_US_PER_S * dt;
  if (hoverSlewF < hoverTarget) hoverSlewF = fminf(hoverSlewF + step, (float)hoverTarget);
  else if (hoverSlewF > hoverTarget) hoverSlewF = fmaxf(hoverSlewF - step, (float)hoverTarget);

  HOVER_THROTTLE = (int)lroundf(hoverSlewF);

  int band = throttleBandFor(HOVER_THROTTLE);
  if (band != throttleBand) {
    throttleBand = band;
    float lim = (float)band;
    pitchAxis.rate.out_limit = lim;
    rollAxis.rate.out_limit  = lim;
    yawRatePid.out_limit     = lim;
  }
}

// ============================================================================
// 14. TELEMETRY - seqlock giua core 1 (ghi) va core 0 (doc)
// ============================================================================

static inline uint16_t telemSat16(uint32_t v) {
  return (v > 65535UL) ? (uint16_t)65535 : (uint16_t)v;
}

static void telemPublish() {
  const float accWAvg = accWCount ? (accWSum / (float)accWCount) : 1.0f;

  telemSeq = telemSeq + 1;                 // -> le: dang ghi
  __sync_synchronize();

  telemShared.magic = TELEM_MAGIC;
  telemShared.ver   = TELEM_VER;
  telemShared.mode  = (uint8_t)mode;
  telemShared.flags = (uint8_t)((linkUp        ? 0x01 : 0) |
                                (loopOverrun   ? 0x02 : 0) |
                                (imuCalibrated ? 0x04 : 0) |
                                (clipCount     ? 0x08 : 0) |
                                (espnowReady   ? 0x10 : 0) |
                                (altitudeNow.flowFresh ? 0x20 : 0));
  telemShared.ms     = millis();
  telemShared.loopHz = loopHz;
  telemShared.seq    = ++telemCount;

  telemShared.pitch = pitch; telemShared.roll = roll; telemShared.yaw = yawHeading;
  telemShared.pitchRate = pitchRate; telemShared.rollRate = rollRate; telemShared.yawRate = yawRate;
  telemShared.outP = lastP; telemShared.outR = lastR; telemShared.outY = lastY;

  telemShared.iPitch = pitchAxis.rate.integral;
  telemShared.iRoll  = rollAxis.rate.integral;
  telemShared.iYaw   = yawRatePid.integral;

  telemShared.biasX = ahrs.bx * RAD_TO_DEG;
  telemShared.biasY = ahrs.by * RAD_TO_DEG;
  telemShared.biasZ = ahrs.bz * RAD_TO_DEG;

  telemShared.calBiasX = gyroBiasX; telemShared.calBiasY = gyroBiasY; telemShared.calBiasZ = gyroBiasZ;
  telemShared.pitchOffset = pitchOffset; telemShared.rollOffset = rollOffset;

  telemShared.vibX = vibX; telemShared.vibY = vibY; telemShared.vibZ = vibZ;
  telemShared.accW = accWAvg;

  telemShared.m1 = (int16_t)curM1; telemShared.m2 = (int16_t)curM2;
  telemShared.m3 = (int16_t)curM3; telemShared.m4 = (int16_t)curM4;

  telemShared.throttle    = (int16_t)throttleCmd;
  telemShared.hover       = (int16_t)HOVER_THROTTLE;
  telemShared.hoverTarget = (int16_t)hoverTarget;

  telemShared.clip = telemSat16(clipCount);
  telemShared.rx   = telemSat16(espnowRx);
  telemShared.bad  = telemSat16(espnowBad);
  telemShared.lost = telemSat16(espnowLost);

  strncpy(telemShared.reason, modeReason ? modeReason : "", sizeof(telemShared.reason) - 1);
  telemShared.reason[sizeof(telemShared.reason) - 1] = '\0';

  telemShared.altitudeM = altitudeNow.altitudeM;
  telemShared.verticalSpeedMps = altitudeNow.verticalSpeedMps;
  telemShared.altitudeSetpointM = altitudeSetpointM;
  telemShared.rangeAltitudeM = altitudeNow.rangeAltitudeM;
  telemShared.baroAltitudeM = altitudeNow.baroAltitudeM;
  telemShared.altitudeCorrectionUs = (int16_t)lroundf(altitudeCorrectionUs);
  telemShared.rangeAgeMs = telemSat16(altitudeNow.rangeAgeMs);
  telemShared.baroAgeMs = telemSat16(altitudeNow.baroAgeMs);
  telemShared.altitudeSource = altitudeNow.source;
  telemShared.rangeStrength = altitudeNow.rangeStrength;
  telemShared.lastCommand = lastCommand;
  telemShared.transitionPhase = (uint8_t)transitionPhase;
  telemShared.flowForwardMps = altitudeNow.flowForwardMps;
  telemShared.flowRightMps = altitudeNow.flowRightMps;
  telemShared.positionForwardM = positionForwardM;
  telemShared.positionRightM = positionRightM;
  telemShared.positionPitchDeg = positionPitchDeg;
  telemShared.positionRollDeg = positionRollDeg;
  telemShared.flowAgeMs = telemSat16(altitudeNow.flowAgeMs);
  telemShared.flowQuality = altitudeNow.flowQuality;
  telemShared.flowReserved = 0;
  telemShared.altitudeVelocitySetpointMps = altitudeVelocitySetpointMps;
  telemShared.altitudeIntegralUs = altitudeIntegral;
  telemShared.configSignature = FLIGHT_CONFIG_SIGNATURE;
  telemShared.controlDtUs = telemSat16(controlDtUs);
  telemShared.controlDtMaxUs = telemSat16(controlDtMaxUs);
  telemShared.controlOverruns = telemSat16(controlOverrunCount);
  telemShared.mtfRateHz10 = altitudeNow.mtfRateHz10;
  telemShared.mtfGapMaxUs = altitudeNow.mtfGapMaxUs;
  telemShared.bmpRateHz10 = altitudeNow.bmpRateHz10;
  telemShared.bmpReadMaxUs = altitudeNow.bmpReadMaxUs;
  telemShared.auxWaitMaxUs = altitudeNow.auxWaitMaxUs;
  telemShared.altitudeGapMaxUs = altitudeNow.altitudeGapMaxUs;
  telemShared.uartBacklogMax = altitudeNow.uartBacklogMax;
  telemShared.mtfBadCrc = altitudeNow.mtfBadCrc;
  telemShared.mtfStale = altitudeNow.mtfStale;
  telemShared.bmpReadFail = altitudeNow.bmpReadFail;
  telemShared.auxLockMiss = altitudeNow.auxLockMiss;

  __sync_synchronize();
  telemSeq = telemSeq + 1;                 // -> chan: ban chup lanh lan

  // Giu lai ban cuoi cho armConditionsMet truoc khi xoa bo tich luy.
  vibHoldX = vibX; vibHoldY = vibY; vibHoldZ = vibZ;
  clipHold = clipCount;

  vibX = vibY = vibZ = 0.0f;
  accWSum = 0.0f; accWCount = 0; clipCount = 0;
}

static bool telemRead(TelemPacket *out) {
  for (int i = 0; i < 4; i++) {
    uint32_t s1 = telemSeq;
    if (s1 & 1u) continue;
    __sync_synchronize();
    *out = telemShared;
    __sync_synchronize();
    if (telemSeq == s1) return true;
  }
  return false;
}

void telemTask(void *arg) {
  (void)arg;
  TelemPacket p;
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEM_HZ);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);
    if (!espnowReady) continue;
    if (!telemRead(&p)) continue;
    if (esp_now_send(peerMac, (const uint8_t *)&p, sizeof(p)) == ESP_OK) telemTxOk = telemTxOk + 1;
    else                                                                 telemTxFail = telemTxFail + 1;
  }
}

// ============================================================================
// 15. OLED - task rieng tren core 0 (display() chan ~29ms)
// ============================================================================

#if ENABLE_OLED

static const char *flightModeName(FlightMode m) {
  switch (m) {
    case FS_ARMED_IDLE: return "IDLE";
    case FS_TAKEOFF: return "TKOF";
    case FS_ALT_HOLD: return "HOLD";
    case FS_LANDING: return "LAND";
    case FS_FAILSAFE: return "FAIL";
    case FS_TRIPPED: return "SAFE";
    default: return "DISA";
  }
}

// Rut gon modeReason thanh ma <= 20 ky tu de biet dung nguyen nhan tren OLED.
static const char *oledReasonName(const char *reason) {
  if (!reason) return "UNKNOWN";
  if (strstr(reason, "mat link"))             return "LINK LOST";
  if (strstr(reason, "MTF01P mat"))           return "MTF LOST/STALE";
  if (strstr(reason, "vuot tran cao"))        return "OVER ALT 1.60M";
  if (strstr(reason, "qua thoi gian bay"))    return "FLIGHT TIMEOUT";
  if (strstr(reason, "khong phat hien"))      return "NO LIFT";
  if (strstr(reason, "takeoff timeout"))      return "TAKEOFF TIMEOUT";
  if (strstr(reason, "blind landing"))        return "NO ALT SENSOR";
  if (strstr(reason, "landing timeout"))      return "LAND TIMEOUT";
  if (strstr(reason, "KILL"))                 return "KILL COMMAND";
  if (strstr(reason, "Sai so goc"))           return "ATTITUDE ERROR";
  if (strstr(reason, "Goc NaN"))              return "ATTITUDE NAN";
  if (strstr(reason, "Attitude khong"))       return "ATTITUDE INVALID";
  if (strstr(reason, "MPU6050"))              return "MPU LOST";
  if (strstr(reason, "BMP388"))               return "BMP NOT READY";
  if (strstr(reason, "MTF01P khong"))         return "MTF NOT READY";
  if (strstr(reason, "flow khong"))           return "FLOW NOT READY";
  if (strstr(reason, "moc vi tri"))           return "POS ORIGIN WAIT";
  if (strstr(reason, "vi tri khong"))         return "POSITION INVALID";
  if (strstr(reason, "cao do chua CAL"))      return "ALT NOT CAL";
  if (strstr(reason, "chua co link"))         return "NO SENDER LINK";
  if (strstr(reason, "chua hieu chuan"))      return "IMU NOT CAL";
  if (strstr(reason, "cao do dat"))           return "GROUND NOT ZERO";
  if (strstr(reason, "nam ngang") || strstr(reason, "LEVEL")) return "NOT LEVEL";
  if (strstr(reason, "rung") || strstr(reason, "di chuyen")) return "MOVING/VIBRATION";
  if (strstr(reason, "bao hoa"))              return "ACCEL CLIPPED";
  if (strstr(reason, "ARM timeout"))           return "ARM TIMEOUT";
  if (strstr(reason, "da khoa"))               return "SAFETY LOCKED";
  if (strstr(reason, "failsafe landed"))       return "FAILSAFE LANDED";
  if (strstr(reason, "landed"))                return "LANDED";
  if (strstr(reason, "lenh LAND"))             return "LAND COMMAND";
  if (strstr(reason, "ARMED IDLE"))            return "WAIT TAKEOFF";
  if (strstr(reason, "xac nhan ARM"))          return "PREARM CHECK";
  if (strstr(reason, "tu cat canh"))           return "TAKEOFF";
  if (strstr(reason, "ALT HOLD"))              return "NORMAL";
  if (strstr(reason, "cho lenh ARM"))          return "WAIT ARM";
  return "SEE TERMINAL";
}

static const char *oledActionName(FlightMode mode) {
  switch (mode) {
    case FS_FAILSAFE:  return "SLOW LAND";
    case FS_TRIPPED:   return "RESET REQUIRED";
    case FS_LANDING:   return "SLOW LAND";
    case FS_TAKEOFF:   return "CLIMB";
    case FS_ALT_HOLD:  return "HOLD";
    case FS_ARMED_IDLE:return "2=TAKEOFF";
    default:           return "CHECK THEN 1=ARM";
  }
}

static inline void oledPublish() {
  oledSeq = oledSeq + 1;
  __sync_synchronize();
  oledShared.altitudeM = altitudeNow.altitudeM;
  oledShared.altitudeSetpointM = altitudeSetpointM;
  oledShared.verticalSpeedMps = altitudeNow.verticalSpeedMps;
  oledShared.flowForwardMps = altitudeNow.flowForwardMps;
  oledShared.flowRightMps = altitudeNow.flowRightMps;
  oledShared.positionForwardM = positionForwardM;
  oledShared.positionRightM = positionRightM;
  oledShared.throttle = throttleCmd;
  oledShared.altitudeSource = altitudeNow.source;
  oledShared.flowQuality = altitudeNow.flowQuality;
  oledShared.mode = mode;
  oledShared.linkUp = linkUp;
  oledShared.flowFresh = altitudeNow.flowFresh;
  oledShared.reason = modeReason;
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

void oledTask(void *arg) {
  (void)arg;
  if (!auxI2cMutex) {
    oledOK = false;
    Serial.println("[OLED] Tat task: AUX mutex khong ton tai.");
    vTaskDelete(NULL);
    return;
  }
  Serial.printf("[SCHED] oled core=%d prio=%u\n", xPortGetCoreID(),
                (unsigned)uxTaskPriorityGet(NULL));
  OledSnap s;
  for (;;) {
    if (oledRead(&s) && oledOK &&
        (s.mode == FS_DISARMED || s.mode == FS_ARMED_IDLE)) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);

      // 1) Mode/link/throttle: phat hien failsafe va theo doi ramp 1350 -> hover.
      display.print(flightModeName(s.mode));
      display.print(s.linkUp ? " L " : " X ");
      display.print("T:"); display.println(s.throttle);

      // 2) Cao do hien tai / setpoint.
      display.print("Z:"); display.print(s.altitudeM, 2);
      display.print(" SP:"); display.println(s.altitudeSetpointM, 2);

      // 3) Van toc dung + nguon fused/MTF/BMP/coast/none.
      display.print("VZ:"); display.print(s.verticalSpeedMps, 2);
      display.print(" SRC:");
      display.println(s.altitudeSource == ALT_SOURCE_FUSED ? "F" :
                      s.altitudeSource == ALT_SOURCE_MTF ? "M" :
                      s.altitudeSource == ALT_SOURCE_BARO ? "B" :
                      s.altitudeSource == ALT_SOURCE_COAST ? "C" : "-");

      // 4) Van toc flow forward/right va quality.
      display.print("F:"); display.print(s.flowForwardMps, 2);
      display.print("/"); display.print(s.flowRightMps, 2);
      display.print(" Q:"); display.println(s.flowQuality);

      // 5) Vi tri drift forward/right; H=hold dang khoa, -=flow khong dung duoc.
      display.print("P:"); display.print(s.positionForwardM, 2);
      display.print("/"); display.print(s.positionRightM, 2);
      display.println(s.flowFresh ? " H" : " -");

      // 6-7) Nguyen nhan + hanh dong: FAIL/SAFE khong con la ma mo ho.
      display.print("WHY:"); display.println(oledReasonName(s.reason));
      display.print("ACT:"); display.print(oledActionName(s.mode));
      if (xSemaphoreTake(auxI2cMutex, 0) == pdTRUE) {
        display.display();
        xSemaphoreGive(auxI2cMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(OLED_MS));
  }
}
#endif

// ============================================================================
// 16. CHAN DOAN SERIAL
// ============================================================================

#if TELEM_SERIAL
static void printTelemetry() {
  Serial.printf("%s T:%d %dHz | P:%.1f R:%.1f Yr:%.1f | o:%.0f,%.0f,%.0f | M:%d %d %d %d",
                (mode != FS_DISARMED && mode != FS_TRIPPED) ? "ARM " : "IDLE",
                throttleCmd, loopHz, pitch, roll, yawRate,
                lastP, lastR, lastY, curM1, curM2, curM3, curM4);
  Serial.printf(" | I:%.1f,%.1f B:%.2f,%.2f | V:%.2f/%.2f/%.2f C:%u",
                pitchAxis.rate.integral, rollAxis.rate.integral,
                ahrs.bx * RAD_TO_DEG, ahrs.by * RAD_TO_DEG,
                telemShared.vibX, telemShared.vibY, telemShared.vibZ, telemShared.clip);
  Serial.printf(" | H:%d %s/%u", HOVER_THROTTLE, linkUp ? "LINK" : "MAT-KN", espnowRx);
  if (fsLanding)    Serial.print(" [FAILSAFE-LAND]");
  if (loopOverrun)  Serial.print(" [overrun]");
  if (!airborne()) Serial.printf(" [%s]", modeReason);
  Serial.println();
}
#else
static inline void printTelemetry() {}
#endif

// ============================================================================
// 17. SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== DRONE_ALT_HOLD_1M (ESP32) ===");
  Serial.printf("[BUILD] %s %s cfg=0x%08lX telem=v%u/%uB\n",
                __DATE__, __TIME__, (unsigned long)FLIGHT_CONFIG_SIGNATURE,
                (unsigned)TELEM_VER, (unsigned)sizeof(TelemPacket));
  Serial.printf("[SCHED] setup/loop core=%d prio=%u\n", xPortGetCoreID(),
                (unsigned)uxTaskPriorityGet(NULL));

  // Tao xung idle truoc moi duong allocation/task co the dung han. Neu khoi tao
  // sau do that bai, ESC van nhan 1000 us thay vi de chan floating/khong xung.
  escInitTimer();
  escAttach(ESC_PIN_M1, ESC_CH_M1);
  escAttach(ESC_PIN_M2, ESC_CH_M2);
  escAttach(ESC_PIN_M3, ESC_CH_M3);
  escAttach(ESC_PIN_M4, ESC_CH_M4);
  escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);

  auxI2cMutex = xSemaphoreCreateMutex();
  if (!auxI2cMutex)
    Serial.println("[AUX] CANH BAO: khong tao duoc mutex; BMP tiep tuc, OLED bi tat.");
  I2C_AUX.begin(AUX_SDA, AUX_SCL, AUX_FREQ);
  I2C_AUX.setTimeOut(10);
  bmpReady = beginBmp388();
  Serial.printf("[BMP388] %s%s\n", bmpReady ? "OK" : "KHONG TIM THAY",
                bmpReady ? (bmpAddress == 0x77 ? " @0x77" : " @0x76") : "");
  MTFSerial.begin(MTF_BAUD, SERIAL_8N1, MTF_RX_PIN, MTF_TX_PIN);
  Serial.println("[MTF01P] UART1 115200, protocol phai la Micolink.");

#if ENABLE_OLED
  oledOK = auxI2cMutex && display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Khoi dong...");
    display.display();
  } else if (auxI2cMutex) {
    Serial.println("[WARN] Khong tim thay OLED.");
  } else {
    Serial.println("[OLED] Da tat vi AUX mutex khong san sang.");
  }
#endif

  if (xTaskCreatePinnedToCore(altitudeTask, "altitude", ALT_TASK_STACK, NULL,
                              ALT_TASK_PRIO, &altitudeTaskH, ALT_TASK_CORE) != pdPASS) {
    Serial.println("[FATAL] Khong tao duoc altitude task.");
    while (true) delay(500);
  }

  espnowReady = espnowInit();

  Wire.begin(MPU_SDA, MPU_SCL, MPU_FREQ);
  Wire.setTimeOut(5);
  if (!mpuInit()) {
    Serial.println("[FATAL] Khong tim thay MPU6050!");
    escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);
    while (true) delay(500);
  }
  Wire.setClock(MPU_FREQ);

  // Ngan sach thoi gian doc IMU: vuot 80% chu ky thi phai giam IMU_OVERSAMPLE.
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
      Serial.printf("[IMU] doc %luus TB / %luus dinh | ngan sach %luus\n",
                    (unsigned long)(tSum / nOk), (unsigned long)tMax, (unsigned long)SAMPLE_US);
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
  imuCalibrated = true;
  Serial.println("[CAL] *** DA TAT HIEU CHUAN (bias=0, offset=0) ***");
#endif

#if ENABLE_OLED
  if (oledOK &&
      xTaskCreatePinnedToCore(oledTask, "oled", OLED_TASK_STACK, NULL,
                              OLED_TASK_PRIO, &oledTaskHandle,
                              OLED_TASK_CORE) != pdPASS) {
    oledTaskHandle = NULL;
    oledOK = false;
    Serial.println("[OLED] CANH BAO: khong tao duoc task; OLED da tat.");
  }
#endif

  pitchSetpoint = constrain(PITCH_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  rollSetpoint  = constrain(ROLL_TRIM_DEG,  -TRIM_DEG_MAX, TRIM_DEG_MAX);
  if (PITCH_TRIM_DEG != pitchSetpoint || ROLL_TRIM_DEG != rollSetpoint)
    Serial.println("[TRIM] CANH BAO: trim vuot gioi han da bi kep - do la loi co khi.");

#if TELEM_ENABLE
  if (espnowReady) {
    if (xTaskCreatePinnedToCore(telemTask, "telem", TELEM_TASK_STACK, NULL,
                                TELEM_TASK_PRIO, &telemTaskH, TELEM_TASK_CORE) != pdPASS)
      Serial.println("[TELEM] LOI: khong tao duoc task.");
    else
      Serial.printf("[TELEM] Gui %dHz, %d byte/goi\n", TELEM_HZ, (int)sizeof(TelemPacket));
  }
#endif

  Serial.printf("[CFG] Dieu khien %luHz | IMU %luHz (TB %d mau)\n",
                (unsigned long)CONTROL_HZ, (unsigned long)SAMPLE_HZ, IMU_OVERSAMPLE);
  Serial.printf("[CFG] sqrt_ctrl accel_max=%.0f do/s^2 | thrust_expo=%.2f | angle_boost=%d\n",
                ANGLE_ACCEL_MAX, THRUST_EXPO, ANGLE_BOOST_ON);
  Serial.printf("[CFG] yaw headroom %.0f%% | trip khi sai so goc >%.0f do trong %.2fs\n",
                YAW_HEADROOM_PCT * 100.0f, SAFE_ANGLE_ERR, SAFE_ANGLE_HOLD_S);
  Serial.printf("[CFG] CG COMP @hover P/R=%+.1f/%+.1fus | angle trim P/R=%+.2f/%+.2fdeg\n",
                CG_PITCH_COMP_US_AT_HOVER, CG_ROLL_COMP_US_AT_HOVER,
                PITCH_TRIM_DEG, ROLL_TRIM_DEG);
  Serial.printf("[CFG] TILT TAKEOFF: admit %.0fdeg | abort %.0fdeg/%.0fdps %.2fs | "
                "level %.1fdeg/s | motor base+%.0fus | transfer %.2fs\n",
                TAKEOFF_GROUND_MAX_TILT_DEG, TAKEOFF_GROUND_ABORT_ANGLE_DEG,
                TAKEOFF_GROUND_ABORT_RATE_DPS, TAKEOFF_GROUND_ABORT_CONFIRM_S,
                TAKEOFF_LEVEL_SLEW_DEG_PER_S, TAKEOFF_GROUND_MOTOR_DELTA_US,
                TAKEOFF_ATTITUDE_TRANSFER_S);
  Serial.printf("[CFG] LAND: %.2fm/s, duoi %.0fcm -> %.2fm/s + ga giam %.0fus/s; timeout %.0fs.\n",
                ALT_LAND_SLEW_MPS, LAND_NEAR_GROUND_M * 100.0f,
                ALT_LAND_NEAR_SLEW_MPS, LAND_NEAR_THROTTLE_DOWN_US_PER_S,
                LAND_TIMEOUT_S);
  Serial.printf("[CFG] VERTICAL: takeoff sp %.2fm/s, vz up/down %.2f/%.2fm/s, accel %.2f/%.2fm/s2.\n",
                ALT_SETPOINT_SLEW_MPS, ALT_VEL_MAX_UP_MPS, ALT_VEL_MAX_DOWN_MPS,
                ALT_VEL_ACCEL_UP_MPS2, ALT_VEL_ACCEL_DOWN_MPS2);
  Serial.printf("[OK] Khong tu ARM. Gui ARM, doi %.0fs pre-arm, roi TAKEOFF. Muc cao %.2fm.\n",
                ARM_CONFIRM_S, ALT_TARGET_M);

  uint32_t t0 = micros();
  lastCtrlUs = lastRateUs = lastTelemUs = lastSerialUs = t0;
  nextSampleUs = t0 + SAMPLE_US;
}

// ============================================================================
// 18. VONG DIEU KHIEN
// ============================================================================

static float accelWeight(float magRatio) {
  float dev = fabsf(magRatio - 1.0f);
  if (dev <= ACC_DEV_FULL) return 1.0f;
  if (dev >= ACC_DEV_ZERO) return 0.0f;
  return (ACC_DEV_ZERO - dev) / (ACC_DEV_ZERO - ACC_DEV_FULL);
}

static void trackVibration(const ImuSample *s) {
  float dx = fabsf(s->ax - lpfAccX.y1);
  float dy = fabsf(s->ay - lpfAccY.y1);
  float dz = fabsf(s->az - lpfAccZ.y1);
  if (dx > vibX) vibX = dx;
  if (dy > vibY) vibY = dy;
  if (dz > vibZ) vibZ = dz;
}

// Trip khi SAI SO goc (muc tieu - thuc te) vuot nguong va KEO DAI.
// Goc tuyet doi se bao nham ngay khi them dieu khien nghieng tu NODE B.
static void checkAttitudeError(float dt) {
  if (!airborne()) { tripAngleT = 0.0f; return; }

  float errP = fabsf(pitchSetpoint - pitch);
  float errR = fabsf(rollSetpoint - roll);
  if (fmaxf(errP, errR) > SAFE_ANGLE_ERR) {
    tripAngleT += dt;
    if (tripAngleT >= SAFE_ANGLE_HOLD_S) tripSafety("Sai so goc qua lon - mat dieu khien");
  } else {
    tripAngleT = 0.0f;
  }
}

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

  // Chi hoc bias gyro khi accel tin duoc va drone gan dung yen.
  float rateMax = fmaxf(fabsf(gcx), fmaxf(fabsf(gcy), fabsf(gcz)));
  float wBias = (accW >= 1.0f && rateMax < BIAS_LEARN_MAX_RATE) ? 1.0f : 0.0f;

  madgwick_update(&ahrs, gcx * DEG_TO_RAD, gcy * DEG_TO_RAD, gcz * DEG_TO_RAD,
                  axf, ayf, azf, accW, wBias, dt);

  float gvx, gvy, gvz;
  madgwick_gravity(&ahrs, &gvx, &gvy, &gvz);
#if MPU_MOUNT_Y_FORWARD
  pitch = atan2f( gvy, sqrtf(gvx * gvx + gvz * gvz)) * RAD_TO_DEG - pitchOffset;
  roll  = atan2f(-gvx, gvz) * RAD_TO_DEG - rollOffset;
  float pitchRateRaw = gcx;
  float rollRateRaw  = gcy;
#else
  pitch = atan2f( gvx, sqrtf(gvy * gvy + gvz * gvz)) * RAD_TO_DEG - pitchOffset;
  roll  = atan2f( gvy, gvz) * RAD_TO_DEG - rollOffset;
  float pitchRateRaw = -gcy;
  float rollRateRaw  =  gcx;
#endif

  pitchRate = lpf_update(&lpfPitchRate, pitchRateRaw, dt);
  rollRate  = lpf_update(&lpfRollRate,  rollRateRaw,  dt);
  yawRate   = lpf_update(&lpfYawRate,   gcz,          dt);

  // Khong co la ban: huong yaw se troi cham theo bias con du.
  yawHeading = wrap180(yawHeading + yawRate * dt);

  if (!isfinite(pitch) || !isfinite(roll) || !isfinite(pitchRate) ||
      !isfinite(rollRate) || !isfinite(yawRate) || !isfinite(yawHeading))
    tripSafety("Attitude khong huu han");
  else checkAttitudeError(dt);
}

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

static void runArmedIdle(float dt) {
  spoolRatio = fminf(spoolRatio + dt / SPOOL_UP_S, 1.0f);
  throttleCmd = IDLE_THROTTLE + (int)lroundf((MOTOR_MIN_US - IDLE_THROTTLE) * spoolRatio);
  escWriteAll(throttleCmd, throttleCmd, throttleCmd, throttleCmd);
  resetControllers();
  lastP = lastR = lastY = 0.0f;
}

static void updateAltitudeSafety(float dt) {
  (void)dt;
  if (!airborne()) return;

  if ((mode == FS_TAKEOFF || mode == FS_ALT_HOLD) &&
      altitudeNow.rangeAgeMs > RANGE_FAILSAFE_MS)
    startLanding(true, "MTF01P mat/qua cu");

  if ((mode == FS_TAKEOFF || mode == FS_ALT_HOLD) &&
      altitudeNow.source && altitudeNow.altitudeM > ALT_MAX_M)
    startLanding(true, "vuot tran cao 1.60m");

  if ((mode == FS_TAKEOFF || mode == FS_ALT_HOLD) && flightStartMs &&
      (uint32_t)(millis() - flightStartMs) > (uint32_t)(MAX_FLIGHT_S * 1000.0f))
    startLanding(true, "qua thoi gian bay toi da");

  if ((mode == FS_LANDING || mode == FS_FAILSAFE) && landingStartMs &&
      (uint32_t)(millis() - landingStartMs) > (uint32_t)(LAND_TIMEOUT_S * 1000.0f))
    tripSafety("landing timeout");
}

// Moc (0,0) duoc tao mot lan tai vi tri sau khi altitude/flow calibration xong.
// Odometry tiep tuc theo moc nay ca khi DISARMED; vi vay khong di chuyen drone
// sau calib neu muon diem calib la diem giu. Mat flow chi dong bang estimate,
// khong duoc phep am tham tao moc moi.
static void updateHorizontalOdometry(float dt) {
  if (!altitudeNow.calibrated) {
    if (positionReferenceInitialized) resetHorizontalReference();
    return;
  }
  if (!positionReferenceInitialized) {
    positionForwardM = positionRightM = 0.0f;
    positionYawReferenceDeg = yawHeading;
    positionReferenceInitialized = true;
    Serial.printf("[POS] Moc calib da khoa, yaw=%.1f deg, flow=%s.\n",
                  positionYawReferenceDeg,
                  altitudeNow.flowFresh ? "OK" : "CHO");
    return;
  }
  if (!altitudeNow.flowFresh) return;

  const float yawDelta = wrap180(yawHeading - positionYawReferenceDeg) * DEG_TO_RAD;
  const float cy = cosf(yawDelta), sy = sinf(yawDelta);
  const float velocityRefForward = cy * altitudeNow.flowForwardMps -
                                   sy * altitudeNow.flowRightMps;
  const float velocityRefRight = sy * altitudeNow.flowForwardMps +
                                 cy * altitudeNow.flowRightMps;
  positionForwardM += velocityRefForward * dt;
  positionRightM += velocityRefRight * dt;

  // Numerical guard only; 2 m is not a mission envelope. Preserve direction
  // rather than clamping each axis into a square.
  const float estimateRadius = hypotf(positionForwardM, positionRightM);
  if (!isfinite(estimateRadius)) {
    resetHorizontalReference();
  } else if (estimateRadius > POS_HOLD_ESTIMATE_LIMIT_M) {
    const float scale = POS_HOLD_ESTIMATE_LIMIT_M / estimateRadius;
    positionForwardM *= scale;
    positionRightM *= scale;
  }
}

// Optical flow da la m/s bat bien theo target cao do. Vong
// position(m) -> velocity(m/s) -> tilt tao setpoint nho cho PID attitude.
static void updatePositionHold(float dt, bool suppressHold) {
  const float trimPitch = constrain(PITCH_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  const float trimRoll = constrain(ROLL_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  const float tiltStep = POS_HOLD_TILT_SLEW_DEG_PER_S * dt;

  const bool flowOk = !suppressHold && altitudeNow.flowFresh && altitudeNow.rangeFresh &&
                      positionReferenceInitialized &&
                      altitudeNow.altitudeM >= POS_HOLD_MIN_ALT_M;
  if (!flowOk) {
    positionFlowLostS += dt;
    // Cho phep coasting 0.35s giu nguyen goc phanh truoc khi reset hoan toan
    if (!suppressHold && positionHoldActive && positionFlowLostS < 0.35f) {
      pitchSetpoint = constrain(trimPitch + positionPitchDeg,
                                -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
      rollSetpoint = constrain(trimRoll + positionRollDeg,
                               -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
      return;
    }
    // Mat flow that su: giu estimate theo moc calib, nhung xoa demand controller
    // va slew goc ve trim. Khi flow lai tot, confirm 0.20s truoc khi tro lai moc cu.
    positionIntegralForward = positionIntegralRight = 0.0f;
    positionFlowConfirmS = 0.0f;
    positionHoldActive = false;
    positionPitchDeg += constrain(-positionPitchDeg, -tiltStep, tiltStep);
    positionRollDeg += constrain(-positionRollDeg, -tiltStep, tiltStep);
    pitchSetpoint = constrain(trimPitch + positionPitchDeg,
                              -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
    rollSetpoint = constrain(trimRoll + positionRollDeg,
                             -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
    return;
  }
  positionFlowLostS = 0.0f;

  // Khong khoa vi tri ngay tren frame flow dau tien sau mot dot mat du lieu.
  positionFlowConfirmS = fminf(positionFlowConfirmS + dt, POS_HOLD_FLOW_CONFIRM_S);
  if (!positionHoldActive) {
    positionPitchDeg += constrain(-positionPitchDeg, -tiltStep, tiltStep);
    positionRollDeg += constrain(-positionRollDeg, -tiltStep, tiltStep);
    pitchSetpoint = constrain(trimPitch + positionPitchDeg,
                              -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
    rollSetpoint = constrain(trimRoll + positionRollDeg,
                             -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
    if (positionFlowConfirmS < POS_HOLD_FLOW_CONFIRM_S) return;
    positionIntegralForward = positionIntegralRight = 0.0f;
    positionPitchDeg = positionRollDeg = 0.0f;
    positionHoldActive = true;
  }

  const float yawDelta = wrap180(yawHeading - positionYawReferenceDeg) * DEG_TO_RAD;
  const float cy = cosf(yawDelta), sy = sinf(yawDelta);

  // Sai so bang met nhan gain 1/s thanh velocity m/s. Khong chia cho 0.50 m:
  // 50 cm la target cao do, khong phai dung sai vi tri ngang.
  float desiredRefForward = -POS_HOLD_KP_PER_S * positionForwardM;
  float desiredRefRight = -POS_HOLD_KP_PER_S * positionRightM;
  const float desiredMagnitude = hypotf(desiredRefForward, desiredRefRight);
  if (desiredMagnitude > POS_HOLD_MAX_VEL_MPS) {
    const float velocityScale = POS_HOLD_MAX_VEL_MPS / desiredMagnitude;
    desiredRefForward *= velocityScale;
    desiredRefRight *= velocityScale;
  }
  // Truc co dinh -> body de so sanh voi velocity flow hien tai.
  const float desiredForward = cy * desiredRefForward + sy * desiredRefRight;
  const float desiredRight = -sy * desiredRefForward + cy * desiredRefRight;
  const float errorForward = desiredForward - altitudeNow.flowForwardMps;
  const float errorRight = desiredRight - altitudeNow.flowRightMps;

  float rawForward = POS_HOLD_VEL_KP_DEG_PER_MPS * errorForward + positionIntegralForward;
  float rawRight = POS_HOLD_VEL_KP_DEG_PER_MPS * errorRight + positionIntegralRight;
  // Anti-windup: chi tich phan khi chua cham tran hoac sai so dang go I-term ve 0.
  if (fabsf(rawForward) < POS_HOLD_MAX_TILT_DEG || rawForward * errorForward < 0.0f)
    positionIntegralForward = constrain(positionIntegralForward +
      POS_HOLD_VEL_KI_DEG_PER_M * errorForward * dt,
      -POS_HOLD_I_LIMIT_DEG, POS_HOLD_I_LIMIT_DEG);
  if (fabsf(rawRight) < POS_HOLD_MAX_TILT_DEG || rawRight * errorRight < 0.0f)
    positionIntegralRight = constrain(positionIntegralRight +
      POS_HOLD_VEL_KI_DEG_PER_M * errorRight * dt,
      -POS_HOLD_I_LIMIT_DEG, POS_HOLD_I_LIMIT_DEG);

  rawForward = POS_HOLD_VEL_KP_DEG_PER_MPS * errorForward + positionIntegralForward;
  rawRight = POS_HOLD_VEL_KP_DEG_PER_MPS * errorRight + positionIntegralRight;
  // Quality vua qua nguong dat 40% tham quyen; quality >=70 dat 100%.
  const float flowConfidence = constrain(
    0.40f + ((float)altitudeNow.flowQuality - (float)FLOW_MIN_QUALITY) / 50.0f,
    0.35f, 1.0f);
  const float targetPitch = POS_HOLD_PITCH_SIGN * flowConfidence *
    constrain(rawForward, -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
  const float targetRoll = POS_HOLD_ROLL_SIGN * flowConfidence *
    constrain(rawRight, -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
  positionPitchDeg += constrain(targetPitch - positionPitchDeg, -tiltStep, tiltStep);
  positionRollDeg += constrain(targetRoll - positionRollDeg, -tiltStep, tiltStep);
  pitchSetpoint = constrain(trimPitch + positionPitchDeg,
                            -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
  rollSetpoint = constrain(trimRoll + positionRollDeg,
                           -POS_HOLD_MAX_TILT_DEG, POS_HOLD_MAX_TILT_DEG);
}

static float moveToward(float value, float target, float maxStep) {
  return value + constrain(target - value, -maxStep, maxStep);
}

// Reference capture/leveling rieng cho TAKEOFF. Estimator reference phai duoc
// calibrate tren mat phang truoc; pitchOffset/rollOffset khong tach duoc mount
// bias khoi do nghieng cua mat dat neu boot truc tiep tren mat nghieng.
static bool updateTakeoffAttitudeProfile(float dt) {
  if (mode != FS_TAKEOFF || !takeoffAttitudeProfileActive) return true;

  const bool beforeLift = !takeoffLiftConfirmed;
  if (beforeLift) {
    const bool unsafeAngle = fmaxf(fabsf(pitch), fabsf(roll)) >
                             TAKEOFF_GROUND_ABORT_ANGLE_DEG;
    const bool unsafeRate = fmaxf(fabsf(pitchRate), fabsf(rollRate)) >
                            TAKEOFF_GROUND_ABORT_RATE_DPS;
    takeoffGroundUnsafeS = (unsafeAngle || unsafeRate)
                         ? takeoffGroundUnsafeS + dt : 0.0f;
    if (takeoffGroundUnsafeS >= TAKEOFF_GROUND_ABORT_CONFIRM_S) {
      startLanding(true, unsafeAngle ? "ground tilt vuot gioi han"
                                    : "ground attitude rate qua lon");
      return false;
    }
  } else {
    takeoffGroundUnsafeS = 0.0f;
    takeoffAttitudeTransferS = fminf(TAKEOFF_ATTITUDE_TRANSFER_S,
                                     takeoffAttitudeTransferS + dt);
  }

  const float trimPitch = constrain(PITCH_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  const float trimRoll = constrain(ROLL_TRIM_DEG, -TRIM_DEG_MAX, TRIM_DEG_MAX);
  if (transitionPhase == PHASE_TAKEOFF_SPOOL_UP && beforeLift) {
    takeoffAttitudePitchSpDeg = takeoffGroundPitchDeg;
    takeoffAttitudeRollSpDeg = takeoffGroundRollDeg;
  } else {
    const float step = TAKEOFF_LEVEL_SLEW_DEG_PER_S * dt;
    takeoffAttitudePitchSpDeg = moveToward(takeoffAttitudePitchSpDeg, trimPitch, step);
    takeoffAttitudeRollSpDeg = moveToward(takeoffAttitudeRollSpDeg, trimRoll, step);
  }
  pitchSetpoint = takeoffAttitudePitchSpDeg;
  rollSetpoint = takeoffAttitudeRollSpDeg;

  if (!beforeLift && takeoffAttitudeTransferS >= TAKEOFF_ATTITUDE_TRANSFER_S &&
      fabsf(takeoffAttitudePitchSpDeg - trimPitch) < 0.01f &&
      fabsf(takeoffAttitudeRollSpDeg - trimRoll) < 0.01f) {
    takeoffAttitudeProfileActive = false;
    Serial.println("[TAKEOFF] Attitude transfer complete; normal authority.");
  }
  return true;
}

static bool landingNearGround() {
  // Uu tien clearance MTF da tru 12 cm; khi mat MTF dung BMP trong cua so fallback.
  if (altitudeNow.rangeFresh) return altitudeNow.rangeAltitudeM <= LAND_NEAR_GROUND_M;
  if (altitudeNow.source) return altitudeNow.altitudeM <= LAND_NEAR_GROUND_M;
  return false;
}

static bool landingModeActive() {
  return mode == FS_LANDING || mode == FS_FAILSAFE;
}

static void updateLandingPhaseFromAltitude() {
  if (!landingModeActive() || transitionPhase == PHASE_LAND_TOUCHDOWN_DETECTION ||
      transitionPhase == PHASE_LAND_MOTOR_RAMP_DOWN) return;
  if (altitudeNow.rangeFresh && altitudeNow.rangeAltitudeM <= ALT_LANDED_M) {
    // Latch: khong quay lai controller cao do neu frame sau cho thay drone nay len.
    beginTouchdownDetection();
  } else if (transitionPhase != PHASE_LAND_GROUND_APPROACH && landingNearGround()) {
    setTransitionPhase(PHASE_LAND_GROUND_APPROACH);
  } else if (transitionPhase == PHASE_NONE) {
    setTransitionPhase(PHASE_LAND_CONTROLLED_DESCENT);
  }
}

static int touchdownMotorCommand(int startUs, float elapsedS) {
  const int reduced = startUs - (int)lroundf(LAND_TOUCHDOWN_DOWN_US_PER_S * elapsedS);
  // Neu mixer da dua mot motor duoi MOTOR_MIN, tuyet doi khong tang no tro lai.
  return startUs > MOTOR_MIN_US ? max(MOTOR_MIN_US, reduced) : startUs;
}

static void runTouchdownDetection(float dt) {
  phaseElapsedS += dt;
  throttleCmd = touchdownMotorCommand(landingRampStartThrottle, phaseElapsedS);
  const int m1 = touchdownMotorCommand(landingRampStartM1, phaseElapsedS);
  const int m2 = touchdownMotorCommand(landingRampStartM2, phaseElapsedS);
  const int m3 = touchdownMotorCommand(landingRampStartM3, phaseElapsedS);
  const int m4 = touchdownMotorCommand(landingRampStartM4, phaseElapsedS);
  escWriteAll(m1, m2, m3, m4);
  resetControllers();
  lastP = lastR = lastY = 0.0f;

  const bool stable = altitudeNow.rangeFresh &&
                      altitudeNow.rangeAltitudeM <= ALT_LANDED_M &&
                      fabsf(altitudeNow.verticalSpeedMps) <= ALT_LANDED_VZ_MPS;
  landedConfirmS = stable ? landedConfirmS + dt : 0.0f;
  if (landedConfirmS >= ALT_LANDED_CONFIRM_S ||
      phaseElapsedS >= LAND_TOUCHDOWN_TIMEOUT_S)
    beginLandingMotorRamp();
}

static void runLandingMotorRamp(float dt) {
  phaseElapsedS += dt;
  const float k = 1.0f - constrain(phaseElapsedS / LAND_MOTOR_RAMP_S, 0.0f, 1.0f);
  throttleCmd = IDLE_THROTTLE +
                (int)lroundf((landingRampStartThrottle - IDLE_THROTTLE) * k);
  escWriteAll(IDLE_THROTTLE + (int)lroundf((landingRampStartM1 - IDLE_THROTTLE) * k),
              IDLE_THROTTLE + (int)lroundf((landingRampStartM2 - IDLE_THROTTLE) * k),
              IDLE_THROTTLE + (int)lroundf((landingRampStartM3 - IDLE_THROTTLE) * k),
              IDLE_THROTTLE + (int)lroundf((landingRampStartM4 - IDLE_THROTTLE) * k));
  resetControllers();
  lastP = lastR = lastY = 0.0f;
  if (phaseElapsedS >= LAND_MOTOR_RAMP_S)
    disarmNow(fsLanding ? "failsafe landed" : "landed");
}

static int altitudeThrottle(float dt) {
  if (mode == FS_TAKEOFF) {
    takeoffElapsedS += dt;
    phaseElapsedS += dt;
    const bool liftPredicate = altitudeNow.source != ALT_SOURCE_NONE &&
                               (altitudeNow.altitudeM >= TAKEOFF_LIFTOFF_M ||
                                (altitudeNow.altitudeM >= 0.025f &&
                                 altitudeNow.verticalSpeedMps >= TAKEOFF_LIFTOFF_VZ_MPS));
    liftoffConfirmS = liftPredicate ? liftoffConfirmS + dt : 0.0f;

    if (!takeoffLiftConfirmed) {
      if (liftoffConfirmS >= TAKEOFF_LIFTOFF_CONFIRM_S) {
        takeoffLiftConfirmed = true;
        takeoffAttitudeTransferS = 0.0f;
        spoolRatio = 1.0f;
        altitudeSetpointM = fmaxf(0.0f, altitudeNow.altitudeM);
        altitudeVelocitySetpointMps = 0.0f;
        altitudeIntegral = altitudeCorrectionUs = 0.0f;
        altitudeThrottleSlewF = takeoffThrottleF;
        altitudeThrottleSlewLimited = false;
        setTransitionPhase(PHASE_TAKEOFF_CONTROLLED_ASCENT);
        Serial.printf("[TAKEOFF] Xac nhan roi dat %.2fs tai %.0fus.\n",
                      liftoffConfirmS, takeoffThrottleF);
      } else {
        // SPOOL_UP va GROUND_TRANSITION dung cung slew tu output hien tai.
        // Dat hover ma chua lift thi giu output va PI=0, khong tich setpoint 0.50m.
        takeoffThrottleF = fminf((float)HOVER_THROTTLE,
                                takeoffThrottleF + TAKEOFF_RAMP_US_PER_S * dt);
        if (transitionPhase == PHASE_TAKEOFF_SPOOL_UP &&
            takeoffThrottleF >= (float)TAKEOFF_START_US)
          setTransitionPhase(PHASE_TAKEOFF_GROUND_TRANSITION);
        const float span = fmaxf(1.0f, (float)HOVER_THROTTLE - (float)MOTOR_MIN_US);
        spoolRatio = constrain((takeoffThrottleF - (float)MOTOR_MIN_US) / span, 0.0f, 1.0f);
        altitudeSetpointM = fmaxf(0.0f, altitudeNow.altitudeM);
        altitudeVelocitySetpointMps = 0.0f;
        altitudeIntegral = altitudeCorrectionUs = 0.0f;
        altitudeThrottleSlewF = takeoffThrottleF;
        altitudeThrottleSlewLimited = false;

        if (transitionPhase == PHASE_TAKEOFF_GROUND_TRANSITION &&
            phaseElapsedS >= TAKEOFF_NO_RISE_S)
          startLanding(true, "khong phat hien roi dat");
        else if (takeoffElapsedS >= TAKEOFF_TIMEOUT_S)
          startLanding(true, "takeoff timeout");
        return (int)lroundf(takeoffThrottleF);
      }
    }
  }

  if (mode == FS_TAKEOFF) {
    // CONTROLLED_ASCENT dung trajectory hien co; ALTITUDE_CAPTURE chi doi phase,
    // khong tao buoc setpoint/velocity moi.
    altitudeSetpointM = fminf(ALT_TARGET_M, altitudeSetpointM + ALT_SETPOINT_SLEW_MPS * dt);
    if (altitudeNow.altitudeM >= 0.20f) takeoffReachedClearance = true;
    if (transitionPhase == PHASE_TAKEOFF_CONTROLLED_ASCENT &&
        (altitudeSetpointM >= ALT_TARGET_M - ALT_HOLD_TOLERANCE_M ||
         altitudeNow.altitudeM >= ALT_TARGET_M - ALT_HOLD_TOLERANCE_M))
      setTransitionPhase(PHASE_TAKEOFF_ALTITUDE_CAPTURE);

    if (takeoffReachedClearance && altitudeNow.source && altitudeNow.altitudeM < 0.15f)
      startLanding(true, "roi xuong khi cat canh");
    else if (takeoffElapsedS >= TAKEOFF_TIMEOUT_S)
      startLanding(true, "takeoff timeout");

    if (mode == FS_TAKEOFF &&
        fabsf(ALT_TARGET_M - altitudeNow.altitudeM) < ALT_HOLD_TOLERANCE_M &&
        fabsf(altitudeNow.verticalSpeedMps) < 0.15f) {
      holdConfirmS += dt;
      if (holdConfirmS >= 0.6f) {
        mode = FS_ALT_HOLD; modeReason = "ALT HOLD 0.50m";
        transitionPhase = PHASE_NONE;
        phaseElapsedS = 0.0f;
        altitudeSetpointM = ALT_TARGET_M;
      }
    } else holdConfirmS = 0.0f;
  } else if (mode == FS_LANDING || mode == FS_FAILSAFE) {
    phaseElapsedS += dt;
    const float landRate = transitionPhase == PHASE_LAND_GROUND_APPROACH
                           ? ALT_LAND_NEAR_SLEW_MPS : ALT_LAND_SLEW_MPS;
    altitudeSetpointM = fmaxf(0.0f, altitudeSetpointM - landRate * dt);
  } else {
    altitudeSetpointM = ALT_TARGET_M;
  }

  if (altitudeNow.source == ALT_SOURCE_NONE) {
    // Giua ALT_COAST_MS va RANGE_FAILSAFE_MS, giu nguyen ga; safety se chuyen
    // sang LAND neu MTF that su mat qua 800 ms. Khong dap ga theo moi khe packet.
    if (mode != FS_LANDING && mode != FS_FAILSAFE) return throttleCmd;
    if (!blindLandStartMs) {
      blindLandStartMs = millis();
      blindLandStartThrottle = throttleCmd;
    }
    const float elapsed = (millis() - blindLandStartMs) * 0.001f;
    throttleCmd = max(MOTOR_MIN_US,
                      (int)lroundf(blindLandStartThrottle -
                                   BLIND_THROTTLE_DOWN_US_PER_S * elapsed));
    altitudeThrottleSlewF = (float)throttleCmd;
    altitudeThrottleSlewLimited = true;
    if ((uint32_t)(millis() - blindLandStartMs) >= BLIND_LAND_MS)
      tripSafety("blind landing timeout");
    return throttleCmd;
  }
  blindLandStartMs = 0;

  const float posError = soft_deadband(altitudeSetpointM - altitudeNow.altitudeM,
                                       ALT_POS_DEADBAND_M);
  const float rawVelocitySp = constrain(ALT_POS_KP * posError,
                                        -ALT_VEL_MAX_DOWN_MPS, ALT_VEL_MAX_UP_MPS);
  const float velocityStep = (rawVelocitySp >= altitudeVelocitySetpointMps
                              ? ALT_VEL_ACCEL_UP_MPS2 : ALT_VEL_ACCEL_DOWN_MPS2) * dt;
  altitudeVelocitySetpointMps += constrain(rawVelocitySp - altitudeVelocitySetpointMps,
                                           -velocityStep, velocityStep);
  const float velocityError = soft_deadband(
    altitudeVelocitySetpointMps - altitudeNow.verticalSpeedMps, ALT_VEL_DEADBAND_MPS);
  // Co slew cua tick truoc cung la bao hoa actuator; neu bo qua, I-term se tich
  // luy trong luc PWM dang bi gioi han toc do va lai gay vot ga sau do.
  const bool saturated = mixLimit.thr_hi || mixLimit.thr_lo ||
                         altitudeThrottleSlewLimited;
  const float pre = ALT_VEL_KP_US_PER_MPS * velocityError + altitudeIntegral;
  const bool ownHigh = pre >= ALT_CORRECTION_UP_US;
  const bool ownLow = pre <= -ALT_CORRECTION_DOWN_US;
  if ((!saturated && !ownHigh && !ownLow) || (ownHigh && velocityError < 0.0f) ||
      (ownLow && velocityError > 0.0f) ||
      (saturated && altitudeIntegral * velocityError < 0.0f)) {
    altitudeIntegral += ALT_VEL_KI_US_PER_M * velocityError * dt;
    altitudeIntegral = constrain(altitudeIntegral, -ALT_I_LIMIT_US, ALT_I_LIMIT_US);
  }
  altitudeCorrectionUs = constrain(ALT_VEL_KP_US_PER_MPS * velocityError + altitudeIntegral,
                                   -ALT_CORRECTION_DOWN_US, ALT_CORRECTION_UP_US);

  const bool landing = mode == FS_LANDING || mode == FS_FAILSAFE;
  float desiredThrottle = (float)HOVER_THROTTLE + altitudeCorrectionUs;
  if (landing) {
    desiredThrottle = fminf(desiredThrottle,
                            fminf((float)HOVER_THROTTLE + LAND_THROTTLE_HEADROOM_US,
                                  LAND_THROTTLE_MAX_US));
  } else {
    // Trong TAKEOFF / ALT_HOLD khi da tren 0.40m, khong cat ga qua sau duoi HOVER
    // tranh tinh trang phanh dot ngot lam hut luc nang roi tu do.
    if (altitudeNow.altitudeM >= 0.40f)
      desiredThrottle = fmaxf(desiredThrottle, (float)HOVER_THROTTLE - 35.0f);
  }
  desiredThrottle = constrain(desiredThrottle, (float)MOTOR_MIN_US,
                              (float)HOVER_MAX_ALLOWED);

  // Slew hai chieu. Landing van co quyen tang nhe de ham roi, nhung khong con
  // nhay tuc thoi len ga cao; tran tuyet doi landing mac dinh la 1470 us.
  const float upRate = landing ? LAND_THROTTLE_UP_US_PER_S : ALT_THROTTLE_UP_US_PER_S;
  const float downRate = landing
    ? (landingNearGround() ? LAND_NEAR_THROTTLE_DOWN_US_PER_S
                           : LAND_THROTTLE_DOWN_US_PER_S)
    : ALT_THROTTLE_DOWN_US_PER_S;
  const float maxStep = (desiredThrottle >= altitudeThrottleSlewF ? upRate : downRate) * dt;
  const float throttleDelta = desiredThrottle - altitudeThrottleSlewF;
  altitudeThrottleSlewLimited = fabsf(throttleDelta) > maxStep + 0.01f;
  altitudeThrottleSlewF += constrain(throttleDelta, -maxStep, maxStep);
  altitudeThrottleSlewF = constrain(altitudeThrottleSlewF, (float)MOTOR_MIN_US,
                                    (float)HOVER_MAX_ALLOWED);
  return (int)lroundf(altitudeThrottleSlewF);
}

static void runFlightControl(float dt) {
  if (landingModeActive()) {
    updateLandingPhaseFromAltitude();
    if (transitionPhase == PHASE_LAND_TOUCHDOWN_DETECTION) {
      runTouchdownDetection(dt);
      return;
    }
    if (transitionPhase == PHASE_LAND_MOTOR_RAMP_DOWN) {
      runLandingMotorRamp(dt);
      return;
    }
  }

  throttleCmd = altitudeThrottle(dt);
  if (!airborne()) {
    escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);
    lastP = lastR = lastY = 0.0f;
    return;
  }
  // altitudeThrottle() co the kich abort pre-lift ngay trong tick nay.
  if (landingModeActive() && transitionPhase == PHASE_LAND_MOTOR_RAMP_DOWN) {
    runLandingMotorRamp(dt);
    return;
  }

  const bool groundTransition = mode == FS_TAKEOFF && !takeoffLiftConfirmed;
  const bool profileWasActive = mode == FS_TAKEOFF && takeoffAttitudeProfileActive;
  updatePositionHold(dt, groundTransition || landingModeActive() || profileWasActive);
  if (!updateTakeoffAttitudeProfile(dt)) {
    if (landingModeActive() && transitionPhase == PHASE_LAND_MOTOR_RAMP_DOWN)
      runLandingMotorRamp(dt);
    return;
  }
  const bool attitudeTransfer = mode == FS_TAKEOFF && takeoffLiftConfirmed &&
                                takeoffAttitudeTransferS < TAKEOFF_ATTITUDE_TRANSFER_S;
  const float iGate = (groundTransition || attitudeTransfer) ? 0.0f : 1.0f;

  lastP = cascaded_compute(&pitchAxis, pitchSetpoint, pitch, pitchRate, dt,
                           mixLimit.pitch, iGate);
  lastR = cascaded_compute(&rollAxis,  rollSetpoint,  roll,  rollRate,  dt,
                           mixLimit.roll,  iGate);

  // Giu huong: dang spool hoac co lenh xoay thi moc bam theo huong hien tai.
  float yawSp = constrain(yawRateSetpoint, -MAX_YAW_RATE, MAX_YAW_RATE);
  if (YAW_ANGLE_KP > 0.0f) {
    if (groundTransition || fabsf(yawRateSetpoint) > 1.0f) {
      yawHold = yawHeading;
    } else {
      yawSp = constrain(sqrt_controller(wrap180(yawHold - yawHeading),
                                        YAW_ANGLE_KP, ANGLE_ACCEL_MAX, dt),
                        -MAX_YAW_RATE, MAX_YAW_RATE);
    }
  }
  yawRatePid.i_gate = iGate;
  lastY = pid_compute(&yawRatePid, yawSp, yawRate, dt, mixLimit.yaw);

  int m1, m2, m3, m4;
  // PID xu ly sai so dong; feed-forward xu ly moment tai tinh do pin/CG lech.
  // Tach hai thanh phan tranh dung angle trim (gay gia toc ngang) hoac tang
  // Kp/Kd (gay rung) de chua mot tai gan nhu hang dinh.
  const float pMix = lastP * MIX_PITCH_SIGN + cgPitchCompensation((float)throttleCmd);
  const float rMix = lastR * MIX_ROLL_SIGN + cgRollCompensation((float)throttleCmd);
  const float yMix = lastY * MIX_YAW_SIGN;
  if (groundTransition) {
    motorMixGroundLimited((float)throttleCmd, pMix, rMix, yMix, &m1, &m2, &m3, &m4);
  } else if (attitudeTransfer) {
    const float alpha = takeoffAttitudeTransferS / TAKEOFF_ATTITUDE_TRANSFER_S;
    motorMixTakeoffTransfer((float)throttleCmd, pMix, rMix, yMix, alpha,
                            &m1, &m2, &m3, &m4);
  } else {
    motor_mix(angleBoost((float)throttleCmd), pMix, rMix, yMix, &m1, &m2, &m3, &m4);
  }
  escWriteAll(m1, m2, m3, m4);
}

// Mot chu ky dieu khien. Thu tu ArduPilot: uoc luong -> dieu khien -> XUAT MOTOR,
// moi viec phu (telemetry/serial/OLED) day xuong sau cung.
static void controlTick(uint32_t nowUs) {
  const uint32_t rawDtUs = nowUs - lastCtrlUs;
  controlDtUs = rawDtUs;
  if (rawDtUs > controlDtMaxUs) controlDtMaxUs = rawDtUs;
  float dt = (float)rawDtUs * 1e-6f;
  lastCtrlUs = nowUs;
  dt = constrain(dt, 0.0005f, 0.05f);

  if (!altitudeRead(&altitudeNow))
    __atomic_add_fetch(&altitudeSnapshotReadFailCount, 1UL, __ATOMIC_RELAXED);
  sanitizeAltitudeSnapshot(&altitudeNow);
  if (!altitudeNow.publishedMs ||
      (uint32_t)(millis() - altitudeNow.publishedMs) > ALTITUDE_SNAPSHOT_STALE_MS) {
    altitudeNow.source = ALT_SOURCE_NONE;
    altitudeNow.rangeFresh = altitudeNow.baroFresh = altitudeNow.flowFresh = false;
    altitudeNow.rangeAgeMs = altitudeNow.baroAgeMs = altitudeNow.flowAgeMs = UINT32_MAX;
  }
  updateThrottleAndLink(dt);

  ImuSample mean;
  bool imuOk = accumMean(&imuAcc, &mean);
  clipCount += imuAcc.clip;
  accumReset(&imuAcc);

  if (imuOk) {
    ctrlCount++;
    estimateAttitude(&mean, dt);
    altitudeTiltCos = (isfinite(pitch) && isfinite(roll))
                    ? cosf(pitch * DEG_TO_RAD) * cosf(roll * DEG_TO_RAD)
                    : 1.0f;
  }

  // Odometry chay doc lap flight mode de moc ngang la vi tri calib, khong phai
  // frame flow dau tien sau khi da roi dat.
  updateHorizontalOdometry(dt);

  updateAltitudeSafety(dt);

  if (mode == FS_TRIPPED) {
    runTripRamp(dt);
  } else if (mode == FS_DISARMED) {
    escWriteAll(IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE, IDLE_THROTTLE);
    lastP = lastR = lastY = 0.0f;
    if (imuOk) updateAutoArm(dt);
  } else if (mode == FS_ARMED_IDLE) {
    runArmedIdle(dt);
  } else if (imuOk && airborne()) {
    runFlightControl(dt);
  }

  if (nowUs - lastRateUs >= 1000000UL) {
    lastRateUs = nowUs;
    loopHz     = ctrlCount;
    ctrlCount  = 0;
  }
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
    controlOverrunCount++;
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
