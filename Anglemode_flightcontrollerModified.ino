//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR 
//OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE 
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.


#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <DShotRMT.h>

HardwareSerial mav(2);             // CRSF receiver on UART2: RX=16, TX=17
Adafruit_BMP085 bmp180;

// CRSF Constants
#define CRSF_SYNC_BYTE    0xC8  // receiver address
#define CRSF_SYNC_BYTE_FC 0xEE  // flight controller address (ELRS sends this)
#define CRSF_MAX_FRAME_SIZE 64

uint8_t crsfBuffer[CRSF_MAX_FRAME_SIZE];
uint8_t crsfIndex = 0;

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw,AccXCalibration,AccYCalibration,AccZCalibration;

// Measured resting angle of the IMU on the frame - auto-set during startup
float rollLevelTrim  = 0.0f;
float pitchLevelTrim = 0.0f;

// DShot motor objects - pinout confirmed via ESC_PWM_Test
DShotRMT escFR(GPIO_NUM_33, DSHOT300, false); // M1 Front-Right
DShotRMT escBR(GPIO_NUM_32, DSHOT300, false); // M2 Rear-Right
DShotRMT escBL(GPIO_NUM_25, DSHOT300, false); // M3 Rear-Left
DShotRMT escFL(GPIO_NUM_26, DSHOT300, false); // M4 Front-Left

float PAngleRoll=2; float PAnglePitch=PAngleRoll;   // 2
float IAngleRoll=0.2; float IAnglePitch=IAngleRoll; // 0.2
float DAngleRoll=0.05; float DAnglePitch=DAngleRoll; // 0.001 // indoor


// dont touch 
float PRateRoll = 1.1; // 1.3 indoor
float IRateRoll = 1.5;  // 1.5
float DRateRoll = 0.0005; //0.0005 indoor

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 0.3f;
float DRateYaw = 0.005f;

float PAngleYaw = 2.0f;
float IAngleYaw = 0.1f;
float DAngleYaw = 0.0f;

float yawAngle = 0.0f;       // integrated heading (deg)
float yawAngleTarget = 0.0f; // desired heading (deg)

uint32_t LoopTimer;
float t = 0.004f;      // minimum loop period (used for loop-rate enforcement only)
uint32_t prevLoopMicros = 0; // for measuring actual dt

// PWM-to-DShot converter (1000-2000 us -> 120-2000 DShot throttle)
// Starts at 120 so motors spin immediately from the bottom of the stick.
int pwmToDshot(float pwm) {
  if (pwm < 1000) pwm = 1000;
  if (pwm > 2000) pwm = 2000;
  return map((int)pwm, 1000, 2000, 120, 2000);
}

volatile int ReceiverValue[8] = {1500,1500,1000,1500,1000,1000,1000,1000}; // 8 CRSF channels mapped to 1000-2000 us
// [0]=Roll(CH1) [1]=Pitch(CH2) [2]=Throttle(CH3) [3]=Yaw(CH4)
// [4]=L2(CH5)   [5]=L1(CH6)   [6]=R1(CH7)       [7]=R2(CH8)

volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
volatile float KalmanGainPitch;
volatile float KalmanGainRoll;

int ThrottleIdle = 1140;   // minimum spin speed during flight
int ThrottleLanding = 1100; // gentle landing idle (below this = disarm)
int ThrottleCutOff = 1000; // sent to ESCs when disarmed

const int ArmThrottleMax = 1100;     // allow arming even if stick minimum is not exactly 1000
const int MinArmedDshot = 150;       // minimum DShot value that reliably spins most motors

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PrevErrorAngleYaw = 0, PrevItermAngleYaw = 0;
volatile float PIDReturn[] = {0, 0, 0};

//Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
volatile float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
volatile float Kalman1DOutput[]={0,0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;


float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

bool bmp180Available = false;
float bmpTemperatureC = 0.0f;
int32_t bmpPressurePa = 0;
float bmpRawAltitudeM = 0.0f;
float bmpAbsoluteAltitudeM = 0.0f;
float bmpRelativeAltitudeM = 0.0f;
float bmpVerticalSpeedMps = 0.0f;
float bmpAltitudeOffsetM = 0.0f;
uint32_t lastBmpUpdateMs = 0;

bool altitudeHoldEnabled = false;
float altitudeHoldTargetM = 0.0f;
float altitudeHoldBaseThrottle = 1400.0f;
float altitudeHoldIntegrator = 0.0f;
float altitudeHoldPrevErrorM = 0.0f;
float altitudeHoldThrottleCorrection = 0.0f;
bool bmpReferenceReady = false;
bool bmpNewDataReady = false; // set true each time BMP delivers a fresh sample

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Arm/disarm state - controlled by CH5 switch (ReceiverValue[4])
// Arm:   CH5 > 1500  AND  throttle < 1050 (safety: arm only at low throttle)
// Disarm: CH5 < 1500  (instant, at any throttle)
bool isArmed = false;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState + (t*KalmanInput);
  KalmanUncertainty=KalmanUncertainty + (t*t*4*4); //here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3); //std deviation of error is 3 deg
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}



// Decode a complete CRSF frame and populate ReceiverValue[]
void processCRSFFrame(uint8_t *frame, uint8_t size) {
  uint8_t type = frame[2];
  // RC Channels Packed frame (type 0x16), payload = 22 bytes
  if (type == 0x16 && size >= 26) {
    const uint8_t *payload = &frame[3];
    for (int i = 0; i < 8; i++) {
      uint16_t bitOfs = i * 11;
      uint16_t byteOfs = bitOfs / 8;
      uint8_t  bitRem  = bitOfs % 8;
      uint32_t val = ((uint32_t)payload[byteOfs] |
                     ((uint32_t)payload[byteOfs + 1] << 8) |
                     ((uint32_t)payload[byteOfs + 2] << 16)) >> bitRem;
      uint16_t crsfVal = val & 0x7FF;
      // Map CRSF 11-bit range (172-1811) to PWM range (1000-2000 us)
      ReceiverValue[i] = map((long)crsfVal, 172, 1811, 1000, 2000);
    }
  }
}

// Parse incoming bytes and assemble CRSF frames
void parseCRSF(uint8_t byteIn) {
  static bool    receiving = false;
  static uint8_t length    = 0;

  if (!receiving) {
    if (byteIn == CRSF_SYNC_BYTE || byteIn == CRSF_SYNC_BYTE_FC) {
      receiving = true;
      crsfIndex = 0;
      crsfBuffer[crsfIndex++] = byteIn;
    }
    return;
  }

  crsfBuffer[crsfIndex++] = byteIn;

  if (crsfIndex == 2) {
    length = byteIn;
    if (length > CRSF_MAX_FRAME_SIZE - 2) {
      receiving = false;
      return;
    }
  }

  if (crsfIndex >= (uint8_t)(length + 2)) {
    processCRSFFrame(crsfBuffer, crsfIndex);
    receiving = false;
    crsfIndex = 0;
  }
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29; //*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;
}

void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm)
{
  float Pterm = P * Error;
  float Iterm = PrevIterm +( I * (Error + PrevError) * (t/2));
  if (Iterm > 400)
  {
    Iterm = 400;
  }
  else if (Iterm < -400)
  {
  Iterm = -400;
  }
  float Dterm = D *( (Error - PrevError)/t);
  float PIDOutput = Pterm + Iterm + Dterm;
  if (PIDOutput > 400)
  {
    PIDOutput = 400;
  }
  else if (PIDOutput < -400)
  {
    PIDOutput = -400;
  }
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// ═══════════════════════════════════════════════════════════
// ALTITUDE HOLD - set to 1 to enable, 0 to disable
// Requires BMP180. Activated by CH6 switch (ReceiverValue[5] > 1500)
// ═══════════════════════════════════════════════════════════
#define ALT_HOLD_ENABLE 1

void updateBMP180(uint32_t nowMs)
{
  if (!bmp180Available) {
    return;
  }

  const uint32_t samplePeriodMs = 100;
  if (nowMs - lastBmpUpdateMs < samplePeriodMs) {
    return;
  }

  float previousAltitude = bmpAbsoluteAltitudeM;
  uint32_t previousUpdateMs = lastBmpUpdateMs;
  lastBmpUpdateMs = nowMs;

  // BMP180 needs 100kHz for reliable reads; restore 400kHz for MPU6050 afterwards
  Wire.setClock(100000);
  bmpRawAltitudeM = bmp180.readAltitude();
  Wire.setClock(400000);
  bmpNewDataReady = true; // signal altitude hold to update

  // Heavier LPF on altitude — BMP180 in ULTRALOWPOWER is very noisy
  if (previousUpdateMs == 0) {
    bmpAbsoluteAltitudeM = bmpRawAltitudeM;
  } else {
    bmpAbsoluteAltitudeM = (0.95f * bmpAbsoluteAltitudeM) + (0.05f * bmpRawAltitudeM);
  }

  if (previousUpdateMs != 0) {
    float deltaTime = (nowMs - previousUpdateMs) / 1000.0f;
    if (deltaTime > 0.0f) {
      float rawVerticalSpeed = (bmpAbsoluteAltitudeM - previousAltitude) / deltaTime;
      bmpVerticalSpeedMps = (0.9f * bmpVerticalSpeedMps) + (0.1f * rawVerticalSpeed); // heavy LPF on vspeed
    }
  }

  if (!isArmed && ReceiverValue[2] < 1050) {
    bmpAltitudeOffsetM = (0.98f * bmpAltitudeOffsetM) + (0.02f * bmpAbsoluteAltitudeM);
  }

  bmpRelativeAltitudeM = bmpAbsoluteAltitudeM - bmpAltitudeOffsetM;
}

void resetAltitudeHold()
{
  altitudeHoldEnabled = false;
  altitudeHoldTargetM = bmpRelativeAltitudeM;
  altitudeHoldBaseThrottle = constrain(ReceiverValue[2], 1250, 1700);
  altitudeHoldIntegrator = 0.0f;
  altitudeHoldPrevErrorM = 0.0f;
  altitudeHoldThrottleCorrection = 0.0f;
}

void calibrateBMP180Reference()
{
  if (!bmp180Available) {
    return;
  }

  const int baselineSamples = 32;
  float altitudeSum = 0.0f;

  for (int i = 0; i < baselineSamples; i++) {
    altitudeSum += bmp180.readAltitude();
    delay(25);
  }

  bmpRawAltitudeM = altitudeSum / baselineSamples;
  bmpAbsoluteAltitudeM = bmpRawAltitudeM;
  bmpAltitudeOffsetM = bmpAbsoluteAltitudeM;
  bmpRelativeAltitudeM = 0.0f;
  bmpVerticalSpeedMps = 0.0f;
  altitudeHoldTargetM = 0.0f;
  altitudeHoldIntegrator = 0.0f;
  altitudeHoldPrevErrorM = 0.0f;
  altitudeHoldThrottleCorrection = 0.0f;
  bmpReferenceReady = true;
  lastBmpUpdateMs = millis();
}

void updateAltitudeHold(float dt)
{
#if ALT_HOLD_ENABLE == 0
  (void)dt;
  if (altitudeHoldEnabled) {
    resetAltitudeHold();
  }
  return;
#endif

  bool altitudeSwitchOn = ReceiverValue[5] > 1500;
  bool canHoldAltitude = bmp180Available && bmpReferenceReady && isArmed && ReceiverValue[2] >= 1200;

  if (!altitudeSwitchOn || !canHoldAltitude) {
    resetAltitudeHold();
    return;
  }

  if (!altitudeHoldEnabled) {
    altitudeHoldEnabled = true;
    altitudeHoldTargetM = bmpRelativeAltitudeM;
    altitudeHoldBaseThrottle = constrain(ReceiverValue[2], 1250, 1700);
    altitudeHoldIntegrator = 0.0f;
    altitudeHoldPrevErrorM = 0.0f;
    altitudeHoldThrottleCorrection = 0.0f;
  }

  // Gate entire altitude PID on fresh BMP data (10 Hz) — running it at 250 Hz
  // winds up the integrator 25x too fast and causes throttle spikes.
  if (!bmpNewDataReady) {
    InputThrottle = altitudeHoldBaseThrottle + altitudeHoldThrottleCorrection;
    return;
  }
  bmpNewDataReady = false;

  float stickOffset = ReceiverValue[2] - altitudeHoldBaseThrottle;
  if (fabsf(stickOffset) > 35.0f) {
    altitudeHoldTargetM += stickOffset * 0.0012f * dt * 250.0f;
  }

  float altitudeErrorM = altitudeHoldTargetM - bmpRelativeAltitudeM;
  altitudeHoldIntegrator += altitudeErrorM * dt;
  altitudeHoldIntegrator = constrain(altitudeHoldIntegrator, -80.0f, 80.0f);
  float altitudeErrorRate = (altitudeErrorM - altitudeHoldPrevErrorM) / dt;
  altitudeHoldPrevErrorM = altitudeErrorM;

  const float altitudeKp = 5.0f;  // was 140 — reduced for noisy BMP180
  const float altitudeKi = 4.0f;   // was 18  — reduced, integrates at 10Hz now
  const float altitudeKd = 0.0f;
  const float climbDamping = 20.0f; // was 55

  altitudeHoldThrottleCorrection =
    (altitudeKp * altitudeErrorM) +
    (altitudeKi * altitudeHoldIntegrator) +
    (altitudeKd * altitudeErrorRate) -
    (climbDamping * bmpVerticalSpeedMps);

  altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, -220.0f, 220.0f);
  InputThrottle = altitudeHoldBaseThrottle + altitudeHoldThrottleCorrection;
}

// ═══════════════════════════════════════════════════════════
// MOTOR TEST  -  set to 1 to run, 0 for normal flight
// Spins each motor for 2 s at low throttle then stops.
// Watch which motor spins to confirm orientation.
// REMOVE PROPS BEFORE RUNNING!
// ═══════════════════════════════════════════════════════════
#define MOTOR_TEST_ENABLE 0

// ═══════════════════════════════════════════════════════════
// MANUAL THROTTLE (NO RECEIVER) - set to 1 for testing without CRSF
// Use Serial commands: + to increase, - to decrease, s to stop
// REMOVE PROPS BEFORE RUNNING!
// ═══════════════════════════════════════════════════════════
#define MANUAL_THROTTLE_MODE 0

int manualThrottle = 1000; // starts disarmed (< 1050 = DShot 0). Use '+' to increase.

void handleManualThrottle() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == '+') {
      manualThrottle += 50;
      if (manualThrottle > 1800) manualThrottle = 1800;
    }
    else if (cmd == '-') {
      manualThrottle -= 50;
      if (manualThrottle < 1050) manualThrottle = 1050;
    }
    else if (cmd == 's') {
      manualThrottle = 1000;  // below 1050 = disarm
    }
  }
  ReceiverValue[2] = manualThrottle;
  ReceiverValue[0] = 1500; // neutral roll
  ReceiverValue[1] = 1500; // neutral pitch
  ReceiverValue[3] = 1500; // neutral yaw
}

void runMotorTest()
{
  const int TEST_THROTTLE = 100; // DShot value (0-2000), ~5% - enough to spin, low risk
  const int SPIN_MS       = 2000;
  const int PAUSE_MS      = 1000;

  // Helper: stop all
  auto stopAll = [&]() {
    for (int i = 0; i < 100; i++) {
      escFR.sendThrottle(0);
      escBR.sendThrottle(0);
      escBL.sendThrottle(0);
      escFL.sendThrottle(0);
      delay(3);
    }
  };

  // Helper: run one motor for SPIN_MS
  auto spinMotor = [&](const char* name, int m) {
    unsigned long start = millis();
    while (millis() - start < (unsigned long)SPIN_MS) {
      escFR.sendThrottle(m == 1 ? TEST_THROTTLE : 0);
      escBR.sendThrottle(m == 2 ? TEST_THROTTLE : 0);
      escBL.sendThrottle(m == 3 ? TEST_THROTTLE : 0);
      escFL.sendThrottle(m == 4 ? TEST_THROTTLE : 0);
      delay(3);
    }
    stopAll();
    delay(PAUSE_MS);
  };
  delay(3000); // safety pause

  spinMotor("M1 Front-Right (pin 32) CCW", 1);
  spinMotor("M2 Rear-Right  (pin 26) CW ", 2);
  spinMotor("M3 Rear-Left   (pin 33) CCW", 3);
  spinMotor("M4 Front-Left  (pin 25) CW ", 4);
  while (true) { delay(1000); } // halt - require reboot after test
}

void setup(void) {
  
Serial.begin(115200);

#if MOTOR_TEST_ENABLE
  // ESCs must be initialised before test
  escFR.begin(); escBL.begin(); escFL.begin(); escBR.begin();
  delay(300);
  for (int i = 0; i < 400; i++) { // arm
    escFR.sendThrottle(0); escBL.sendThrottle(0);
    escFL.sendThrottle(0); escBR.sendThrottle(0);
    delay(3);
  }
  runMotorTest(); // never returns
#endif


int led_time=100;
 pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);


  mav.begin(921600, SERIAL_8N1, 16, 17);      // CRSF receiver on UART2: RX=16, TX=17
  delay(100);
  
  Wire.setClock(100000);  // BMP180 requires 100kHz for reliable init
  Wire.begin();
  delay(250);

  bmp180Available = bmp180.begin(BMP085_ULTRALOWPOWER);
  if (bmp180Available) {
    calibrateBMP180Reference();
  }

  Wire.setClock(400000);  // switch back to 400kHz for MPU6050

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ── Live IMU calibration FIRST (before ESC arming) - keep drone flat & still ──
  // MUST be before ESC begin() so ESC beeps don't vibrate the IMU during sampling
  {
    const int CAL_SAMPLES = 2000;
    double sumGX=0, sumGY=0, sumGZ=0;
    double sumAX=0, sumAY=0, sumAZ=0;
    for (int i = 0; i < CAL_SAMPLES; i++) {
      Wire.beginTransmission(0x68);
      Wire.write(0x1B); Wire.write(0x8);
      Wire.endTransmission();
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission();
      Wire.requestFrom(0x68, 6);
      sumGX += (int16_t)(Wire.read()<<8 | Wire.read());
      sumGY += (int16_t)(Wire.read()<<8 | Wire.read());
      sumGZ += (int16_t)(Wire.read()<<8 | Wire.read());

      Wire.beginTransmission(0x68);
      Wire.write(0x1C); Wire.write(0x10);
      Wire.endTransmission();
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission();
      Wire.requestFrom(0x68, 6);
      sumAX += (int16_t)(Wire.read()<<8 | Wire.read());
      sumAY += (int16_t)(Wire.read()<<8 | Wire.read());
      sumAZ += (int16_t)(Wire.read()<<8 | Wire.read());
      delay(1);
    }
    RateCalibrationRoll  = (sumGX / CAL_SAMPLES) / 65.5f;
    RateCalibrationPitch = (sumGY / CAL_SAMPLES) / 65.5f;
    RateCalibrationYaw   = (sumGZ / CAL_SAMPLES) / 65.5f;

    // Compute resting angle from raw averaged accelerometer (before any offset subtraction)
    // This captures the true IMU mount tilt on the frame
    float rawAX = (float)(sumAX / CAL_SAMPLES) / 4096.0f;
    float rawAY = (float)(sumAY / CAL_SAMPLES) / 4096.0f;
    float rawAZ = (float)(sumAZ / CAL_SAMPLES) / 4096.0f;
    rollLevelTrim  = atan(rawAY / sqrt(rawAX*rawAX + rawAZ*rawAZ)) * 57.29f;
    pitchLevelTrim = -atan(rawAX / sqrt(rawAY*rawAY + rawAZ*rawAZ)) * 57.29f;

    // Accel calibration: zero out the resting readings so AngleRoll/Pitch = 0 when level
    AccXCalibration = rawAX;
    AccYCalibration = rawAY;
    AccZCalibration = rawAZ - 1.0f; // remove 1g
  }

  // Complementary filter initialised to 0 - correct because accel calibration
  // already zeroes out the mounting tilt, so calibrated AngleRoll/Pitch = 0 at rest.

  // ── DShot ESC init (AFTER calibration to avoid vibration corruption) ──
  escFR.begin();
  escBL.begin();
  escFL.begin();
  escBR.begin();
  delay(300); // allow RMT to stabilize

  // ── ESC arming: send zero throttle for ~1.2 s ──
  for (int i = 0; i < 400; i++) {
    escFR.sendThrottle(0);
    escBL.sendThrottle(0);
    escFL.sendThrottle(0);
    escBR.sendThrottle(0);
    delay(3);
  }

  // ── Wait for ESCs to complete initialization ──
  // Keep sending DShot 0 during the wait - a bare delay() stops all frames
  // and causes the ESCs to disarm, which is why motors would not spin.
  {
    unsigned long waitStart = millis();
    while (millis() - waitStart < 5000) {
      escFR.sendThrottle(0);
      escBL.sendThrottle(0);
      escFL.sendThrottle(0);
      escBR.sendThrottle(0);
      delay(3);
    }
  }
  
#if MANUAL_THROTTLE_MODE
#endif

  digitalWrite(15, LOW);
  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);

prevLoopMicros = micros();
LoopTimer = micros();

}

void loop(void) {
  // Measure actual elapsed time for this iteration
  uint32_t nowMicros = micros();
  uint32_t nowMs = millis();
  float dt = (nowMicros - prevLoopMicros) / 1000000.0f;
  if (dt <= 0.0f || dt > 0.05f) dt = 0.004f; // sanity clamp: reject 0 or >50ms
  prevLoopMicros = nowMicros;

  updateBMP180(nowMs);

#if MANUAL_THROTTLE_MODE
  // Manual throttle mode - use Serial commands instead of CRSF receiver
  handleManualThrottle();
#else
  // Read and parse incoming CRSF data
  static uint32_t crsfByteCount = 0;
  while (mav.available()) {
    crsfByteCount++;
    parseCRSF((uint8_t)mav.read());
  }
#endif

  //enter your loop code here
Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(); 
  Wire.requestFrom(0x68,6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();                                                   
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX=(float)AccXLSB/4096;
  AccY=(float)AccYLSB/4096;
  AccZ=(float)AccZLSB/4096;


RateRoll -= RateCalibrationRoll;
RatePitch -= RateCalibrationPitch;
RateYaw -= RateCalibrationYaw;

AccX -= AccXCalibration ;
AccY -= AccYCalibration ;
AccZ -= AccZCalibration;

  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*57.29;
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*57.29;

// // Inlined Kalman Filter computation in the loop
// KalmanAngleRoll += dt * RateRoll;
// KalmanUncertaintyAngleRoll += dt * dt * 16;
// KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 9);
// KalmanAngleRoll += KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
// KalmanUncertaintyAngleRoll *= (1 - KalmanGainRoll);

// // Inlined Kalman Filter computation for Pitch
// KalmanAnglePitch += dt * RatePitch;
// KalmanUncertaintyAnglePitch += dt * dt * 16;
// KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + 9);
// KalmanAnglePitch += KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
// KalmanUncertaintyAnglePitch *= (1 - KalmanGainPitch);

// KalmanAngleRoll  = constrain(KalmanAngleRoll,  -20.0f, 20.0f);
// KalmanAnglePitch = constrain(KalmanAnglePitch, -20.0f, 20.0f);


  // Complementary filter - higher tau = more gyro weight = smoother, less accel noise
  float tau = 6.0f;
  float alpha = tau / (tau + dt);
  complementaryAngleRoll  = alpha * (complementaryAngleRoll  + RateRoll  * dt) + (1.0f - alpha) * AngleRoll;
  complementaryAnglePitch = alpha * (complementaryAnglePitch + RatePitch * dt) + (1.0f - alpha) * AnglePitch;
// Clamping complementary filter roll angle to ±20 degrees
complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);



DesiredAngleRoll  = 0.1f*(ReceiverValue[0]-1500);
DesiredAnglePitch = 0.1f*(ReceiverValue[1]-1500);
InputThrottle=ReceiverValue[2];
// Yaw: stick moves desired heading target; stick centered = hold target
yawAngle += RateYaw * dt;
float rawYawStick = 0.15f * (ReceiverValue[3] - 1500);
if (fabsf(rawYawStick) >= 3.0f) {
  yawAngleTarget += (-rawYawStick) * dt; // rotate target at stick rate
}
// Outer angle PID for yaw (same structure as roll/pitch)
float ErrorAngleYaw = yawAngleTarget - yawAngle;
float PtermAngleYaw = PAngleYaw * ErrorAngleYaw;
float ItermAngleYaw = PrevItermAngleYaw + (IAngleYaw * (ErrorAngleYaw + PrevErrorAngleYaw) * (dt / 2));
ItermAngleYaw = constrain(ItermAngleYaw, -400.0f, 400.0f);
float DtermAngleYaw = DAngleYaw * ((ErrorAngleYaw - PrevErrorAngleYaw) / dt);
float PIDOutputAngleYaw = PtermAngleYaw + ItermAngleYaw + DtermAngleYaw;
PIDOutputAngleYaw = constrain(PIDOutputAngleYaw, -400.0f, 400.0f);
DesiredRateYaw = PIDOutputAngleYaw;
PrevErrorAngleYaw = ErrorAngleYaw;
PrevItermAngleYaw = ItermAngleYaw;


// Inlined PID equation for Roll
ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
PtermRoll = PAngleRoll * ErrorAngleRoll;
ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (dt / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / dt);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
DesiredRateRoll = PIDOutputRoll;
PrevErrorAngleRoll = ErrorAngleRoll;
PrevItermAngleRoll = ItermRoll;

ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
PtermPitch = PAnglePitch * ErrorAnglePitch;
ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (dt / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / dt);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
DesiredRatePitch = PIDOutputPitch;
PrevErrorAnglePitch = ErrorAnglePitch;
PrevItermAnglePitch = ItermPitch;

// Compute errors
ErrorRateRoll = DesiredRateRoll - RateRoll;
ErrorRatePitch = DesiredRatePitch - RatePitch;
ErrorRateYaw = DesiredRateYaw - RateYaw;

// Roll Axis PID
PtermRoll = PRateRoll * ErrorRateRoll;
ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (dt / 2));
ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / dt);
PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

// Update output and previous values for Roll
InputRoll = PIDOutputRoll;
PrevErrorRateRoll = ErrorRateRoll;
PrevItermRateRoll = ItermRoll;

// Pitch Axis PID
PtermPitch = PRatePitch * ErrorRatePitch;
ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (dt / 2));
ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / dt);
PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

// Update output and previous values for Pitch
InputPitch = PIDOutputPitch;
PrevErrorRatePitch = ErrorRatePitch;
PrevItermRatePitch = ItermPitch;

// Yaw Axis PID
PtermYaw = PRateYaw * ErrorRateYaw;
ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (dt / 2));
ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);
DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / dt);
PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);


// Update output and previous values for Yaw
InputYaw = PIDOutputYaw;
PrevErrorRateYaw = ErrorRateYaw;
PrevItermRateYaw = ItermYaw;

updateAltitudeHold(dt);


  if (InputThrottle > 1800)
  {
    InputThrottle = 1800;
  }

  
  MotorInput1 =  (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 =  (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 =  (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left  - counter clockwise
  MotorInput4 =  (InputThrottle + InputRoll - InputPitch + InputYaw); //front left - clockwise

  


  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


// int ThrottleIdle = 1150;
// int ThrottleCutOff = 1000;
  if (MotorInput1 < ThrottleIdle)
  {
    MotorInput1 = ThrottleIdle;
  }
  if (MotorInput2 < ThrottleIdle)
  {
    MotorInput2 = ThrottleIdle;
  }
  if (MotorInput3 < ThrottleIdle)
  {
    MotorInput3 = ThrottleIdle;
  }
  if (MotorInput4 < ThrottleIdle)
  {
    MotorInput4 = ThrottleIdle;
  }

  // ── Arm / Disarm via CH5 switch ──
  // Arm:   CH5 > 1500  AND throttle at bottom (safety)
  // Disarm: CH5 < 1500  (instant)
  if (ReceiverValue[4] > 1500 && ReceiverValue[2] < ArmThrottleMax && !isArmed) {
    isArmed = true;
  }
  if (ReceiverValue[4] < 1500 && isArmed && ReceiverValue[2] < 1200) {
    // Only disarm when throttle is low - prevents accidental disarm in flight
    isArmed = false;
  }

  // ── Motor output ──
  if (!isArmed) {
    // Disarmed: motors off, reset all PID state
    PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
    PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
    PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;
    PrevItermAngleRoll=0; PrevItermAnglePitch=0;
    PrevErrorAngleYaw=0;  PrevItermAngleYaw=0;
    yawAngle=0.0f; yawAngleTarget=0.0f;
    escFR.sendThrottle(0);
    escBR.sendThrottle(0);
    escBL.sendThrottle(0);
    escFL.sendThrottle(0);
  }
  else
  {
    // Armed: full PID active from the moment of arming, at all throttle levels.
    // ThrottleIdle floor on MotorInput1-4 prevents motor stall at low stick.
    escFR.sendThrottle(pwmToDshot(MotorInput1)); // pin 33 - front right  CCW
    escBR.sendThrottle(pwmToDshot(MotorInput2)); // pin 32 - rear  right  CW
    escBL.sendThrottle(pwmToDshot(MotorInput3)); // pin 25 - rear  left   CCW
    escFL.sendThrottle(pwmToDshot(MotorInput4)); // pin 26 - front left   CW
  }

// DEBUG - altitude monitor (disabled for flight - Serial.printf delays DShot timing)
  // static uint32_t lastPrint = 0;
  // if (millis() - lastPrint > 250) {
  //   Serial.printf("ALT_RAW:%.2f ALT_ABS:%.2f ALT_REL:%.2f ALT_TGT:%.2f VZ:%.3f COR:%.1f AH:%d CH6:%d\n",
  //     bmpRawAltitudeM, bmpAbsoluteAltitudeM, bmpRelativeAltitudeM,
  //     altitudeHoldTargetM, bmpVerticalSpeedMps, altitudeHoldThrottleCorrection,
  //     altitudeHoldEnabled, ReceiverValue[5]);
  //   lastPrint = millis();
  // }



//Receiver signals - uncomment only for debugging, Serial.print delays DShot timing
  // Serial.print(ReceiverValue[0]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[1]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[2]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[3]);
  // Serial.print(" ");
  // Serial.print(ReceiverValue[4]);
  // Serial.print(" - ");
  // Serial.print(ReceiverValue[5]);
  // Serial.print(" - ");
  // Serial.println("");

// //Motor PWMs in us
  // Serial.print("MotVals-");
  // Serial.print(MotorInput1);
  // Serial.print("  ");
  // Serial.print(MotorInput2);
  // Serial.print("  ");
  // Serial.print(MotorInput3);
  // Serial.print("  ");
  // Serial.print(MotorInput4);
  // Serial.println(" ");

// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");

// // //IMU values
  // Serial.print("Acc values: ");
  // Serial.print("AccX:");
  // Serial.print(AccX);
  // Serial.print("  ");
  // Serial.print("AccY:");
  // Serial.print(AccY);
  // Serial.print("  ");
  // Serial.print("AccZ:");
  // Serial.print(AccZ);
  // Serial.print(" -- ");
  // Print the gyroscope values
  // Serial.print("Gyro values: ");
  // Serial.print(RateRoll);
  // Serial.print("  ");
  // Serial.print(RatePitch);
  // Serial.print("  ");
  // Serial.print(RateYaw);
  // Serial.print("  ");
  // Serial.print(" -- ");

//PID outputs
// Serial.print("PID O/P ");
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");

//Angles from MPU
  // Serial.print("AngleRoll:");
  // Serial.print(AngleRoll);
  // //serial.print("  ");
  //   Serial.print("AnglePitch:");
  // Serial.print(AnglePitch);

  // Serial.print("KalmanAngleRoll:");
  // Serial.print(KalmanAngleRoll);
  // //serial.print("  ");
  //   Serial.print("KalmanAnglePitch:");
  // Serial.print(KalmanAnglePitch);

  // Serial.print("ComplementaryAngleRoll: ");
  // Serial.print(complementaryAngleRoll);
  // Serial.print("ComplementaryAnglePitch: ");
  // Serial.print(complementaryAnglePitch);

  // Serial.println(" ");  

  //  serial plotter comparison
  // Serial.print(KalmanAngleRoll);
  // Serial.print(" ");
  // Serial.print(KalmanAnglePitch);
  // Serial.print(" ");
  // Serial.print(complementaryAngleRoll);
  // Serial.print(" ");
  // Serial.println(complementaryAnglePitch);

  // Enforce loop rate (250 Hz = 4ms period)
  while (micros() - LoopTimer < (t*1000000)) {
    // busy wait
  }
  LoopTimer = micros();
}


