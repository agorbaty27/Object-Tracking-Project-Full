/*
  servo_controller.ino  —  6-servo PCA9685 driver
  Serial format:  "s1,s2,s3,s4,s5,s6\n"   (all 0-180°)
  Shutdown:       "999,999,999,999,999,999\n"  → all servos center

  PCA9685 channel map:
    CH 0  — S1  Pan          (0-180°)
    CH 1  — S2  Tilt 1       (main tilt)
    CH 2  — S3  Elbow tilt   (135° upward range)
    CH 3  — S4  Wrist tilt   (90° downward range)
    CH 4  — S5  Claw rotate  (0-180°)
    CH 5  — S6  Claw grip    (0=closed, 90=open)
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // default 0x40

#define SERVO_FREQ   50
#define PULSE_MIN   102    // ~0.5ms  = 0°
#define PULSE_MAX   512    // ~2.5ms  = 180°

#define CH_PAN      0
#define CH_TILT1    1
#define CH_ELBOW    2
#define CH_WRIST    3
#define CH_CLAW_ROT 4
#define CH_CLAW_GRP 5

#define NUM_SERVOS  6

// Default center positions (degrees) for each servo
const int CENTER[NUM_SERVOS] = { 90, 90, 90, 90, 90, 0 };

int angleToPulse(int angle) {
  return map(constrain(angle, 0, 180), 0, 180, PULSE_MIN, PULSE_MAX);
}

void setServo(uint8_t ch, int angle) {
  pwm.setPWM(ch, 0, angleToPulse(angle));
}

void centerAll() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    setServo(i, CENTER[i]);
  }
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  centerAll();
  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // Parse up to 6 comma-separated values
  int vals[NUM_SERVOS];
  int count = 0;
  int start = 0;

  for (int i = 0; i <= line.length() && count < NUM_SERVOS; i++) {
    if (i == line.length() || line[i] == ',') {
      vals[count++] = line.substring(start, i).toInt();
      start = i + 1;
    }
  }

  if (count < NUM_SERVOS) return; // incomplete packet, ignore

  // Shutdown command
  if (vals[0] == 999) {
    centerAll();
    return;
  }

  // Drive each channel
  setServo(CH_PAN,      vals[0]);
  setServo(CH_TILT1,    180 - vals[1]);
  setServo(CH_ELBOW,    vals[2]);
  setServo(CH_WRIST,    180 - vals[3]);
  setServo(CH_CLAW_ROT, vals[4]);
  setServo(CH_CLAW_GRP, vals[5]);
}
