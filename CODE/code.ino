#include <Arduino.h>
#include <vl53l4cd_class.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>
#include <Adafruit_BMP085.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
MPU6050 mpu;
Adafruit_BMP085 bmp;

#define DEV_I2C Wire
#define XSHUT_PIN 7
VL53L4CD tof(&Wire, XSHUT_PIN);

const int MQ7_PIN = A0;

uint16_t lastDistanceMM = 0;

struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float temp;
  float pressure;
  float mq7;
  float scan;
  uint16_t distance;
};

#define SERVO_MIN 150
#define SERVO_MAX 600

int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return SERVO_MIN + (long)(SERVO_MAX - SERVO_MIN) * angle / 180;
}

void setServoRaw(int ch, int angle) {
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

void initSensors() {
  Wire.begin();
  mpu.initialize();
  bmp.begin();

  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(10);
  digitalWrite(XSHUT_PIN, HIGH);
  delay(10);

  Wire.begin();
  Wire.setClock(100000);

  tof.InitSensor(0x52);
  tof.VL53L4CD_SensorInit();
  tof.VL53L4CD_SetRangeTiming(80, 0);
  tof.VL53L4CD_StartRanging();
}

int CH[4][3] = {
  {0,1,2},
  {3,4,5},
  {6,7,8},
  {9,10,11}
};

int OFFSET[4][3] = {
  {80,140,90},
  {70,125,85},
  {90,150,100},
  {80,55,90}
};

int DIR[4][3] = {
  {1,1,1},
  {1,1,1},
  {1,1,1},
  {-1,-1,1}
};

float J0[4] = {0,0,0,0};
float J1[4] = {0,0,0,0};
float J2[4] = {0,0,0,0};

const int SCAN_CH = 12;
float scanAngle = 0;
int scanDir = 1;
unsigned long lastScanUpdate = 0;
const unsigned long SCAN_INTERVAL = 40;
const float SCAN_STEP = 1.2;

void updateScanner() {
  unsigned long now = millis();
  if (now - lastScanUpdate < SCAN_INTERVAL) return;
  lastScanUpdate = now;

  scanAngle += scanDir * SCAN_STEP;

  if (scanAngle > 180) { scanAngle = 180; scanDir = -1; }
  if (scanAngle < 0)   { scanAngle = 0;   scanDir = 1; }

  setServoRaw(SCAN_CH, (int)scanAngle);
}

void setJoint(int leg, int joint, float angle) {
  int raw = OFFSET[leg][joint] + DIR[leg][joint] * angle;
  raw = constrain(raw, 0, 180);
  setServoRaw(CH[leg][joint], raw);
}

void applyLeg(int leg) {
  setJoint(leg, 0, J0[leg]);
  setJoint(leg, 1, J1[leg]);
  setJoint(leg, 2, J2[leg]);
}

float ease(float t) {
  return t * t * (3 - 2 * t);
}

void smoothLeg(int leg, float t0, float t1, float t2, int ms) {
  float s0 = J0[leg], s1 = J1[leg], s2 = J2[leg];
  int steps = 60;
  int dt = ms / steps;

  for (int i = 0; i <= steps; i++) {
    float k = ease((float)i / steps);
    J0[leg] = s0 + (t0 - s0) * k;
    J1[leg] = s1 + (t1 - s1) * k;
    J2[leg] = s2 + (t2 - s2) * k;
    applyLeg(leg);
    updateScanner();
    delay(dt);
  }
}

void poseStand() {
  for (int l = 0; l < 4; l++) {
    J0[l] = 0;
    J1[l] = 0;
    J2[l] = 0;
    applyLeg(l);
  }
}

void allServos90() {
  for (int leg = 0; leg < 4; leg++)
    for (int joint = 0; joint < 3; joint++)
      setServoRaw(CH[leg][joint], 90);
  setServoRaw(SCAN_CH, 90);
}

void liftLeg(int l)            { smoothLeg(l, J0[l], -10, -15, 220); }
void swingLeg(int l, float d)  { smoothLeg(l, 14*d, -10, -15, 220); }
void lowerLeg(int l)           { smoothLeg(l, J0[l], 0, 0, 220); }
void pushLeg(int l, float d)   { smoothLeg(l, -14*d, 0, 0, 220); }

void tripodStepDir(float d0, float d1, float d2, float d3) {
  liftLeg(0); liftLeg(3);
  swingLeg(0,d0); swingLeg(3,d3);
  lowerLeg(0); lowerLeg(3);
  pushLeg(0,d0); pushLeg(3,d3);

  liftLeg(1); liftLeg(2);
  swingLeg(1,d1); swingLeg(2,d2);
  lowerLeg(1); lowerLeg(2);
  pushLeg(1,d1); pushLeg(2,d2);
}

void gaitForward()   { tripodStepDir(1,1,1,1); }
void gaitBackward()  { tripodStepDir(-1,-1,-1,-1); }
void gaitTurnLeft()  { tripodStepDir(-1,1,-1,1); }
void gaitTurnRight() { tripodStepDir(1,-1,1,-1); }

uint16_t readTOF() {
  uint8_t ready = 0;
  VL53L4CD_Result_t result;

  if (tof.VL53L4CD_CheckForDataReady(&ready) != 0) return lastDistanceMM;
  if (ready) {
    if (tof.VL53L4CD_GetResult(&result) == 0) {
      tof.VL53L4CD_ClearInterrupt();
      lastDistanceMM = result.distance_mm;
    }
  }
  return lastDistanceMM;
}

SensorData readSensors() {
  SensorData d;

  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  const float ACC_SCALE = 16384.0;
  const float GYRO_SCALE = 131.0;

  d.ax = ax / ACC_SCALE;
  d.ay = ay / ACC_SCALE;
  d.az = az / ACC_SCALE;

  d.gx = gx / GYRO_SCALE;
  d.gy = gy / GYRO_SCALE;
  d.gz = gz / GYRO_SCALE;

  d.temp = bmp.readTemperature();
  d.pressure = bmp.readPressure();
  d.mq7 = analogRead(MQ7_PIN);
  d.distance = readTOF();
  d.scan = scanAngle;

  return d;
}

String makeJSON(SensorData d) {
  String j = "{";
  j += "\"ax\":" + String(d.ax) + ",";
  j += "\"ay\":" + String(d.ay) + ",";
  j += "\"az\":" + String(d.az) + ",";
  j += "\"gx\":" + String(d.gx) + ",";
  j += "\"gy\":" + String(d.gy) + ",";
  j += "\"gz\":" + String(d.gz) + ",";
  j += "\"temp\":" + String(d.temp) + ",";
  j += "\"pressure\":" + String(d.pressure) + ",";
  j += "\"mq7\":" + String(d.mq7) + ",";
  j += "\"angle\":" + String(d.scan) + ",";    
  j += "\"distance\":" + String(d.distance);
  j += "}";
  return j;
}


bool streaming = false;
char currentCommand = 'S';

void readCommand() {
  if (!Serial.available()) return;
  char c = toupper(Serial.read());

  if (c == 'X') { streaming = true; return; }
  if (c == 'Y') { streaming = false; return; }

  if (c=='F'||c=='B'||c=='L'||c=='R'||c=='S'||c=='H')
    currentCommand = c;
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);

  poseStand();
  setServoRaw(SCAN_CH, 0);
  lastScanUpdate = millis();

  initSensors();
}

void loop() {
  readCommand();
  updateScanner();

  switch (currentCommand) {
    case 'F': gaitForward(); break;
    case 'B': gaitBackward(); break;
    case 'L': gaitTurnLeft(); break;
    case 'R': gaitTurnRight(); break;
    case 'H': allServos90(); delay(300); break;
    case 'S': poseStand(); delay(150); break;
  }

  if (streaming) {
    SensorData d = readSensors();
    String j = makeJSON(d);
    Serial.println(j);
    delay(80);
  }
}
