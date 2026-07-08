#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// ================= PID =================
volatile float Kp = 32 , Ki = 0.0, Kd = 2.2;
float derivative = 0.0;
float integral = 0.0;
float error, lastError, output;
unsigned long prevMillis = 0;

// ============== SPEED ==================
volatile int baseSpeed = 255;
int leftSpeed, rightSpeed;
const float setPoint = 4;

// ============== INPUT ==================
const int toggle = 17;

const int minDist = 2;
const int maxDist = 45;

float right, left;

// =========== ULTRASONIC ================
struct UltrasonicSensor {
  int trigPin;
  int echoPin;
  int currentDist;
  int prevDist;

  void initializePins() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
  }
};

UltrasonicSensor rightUltra, leftUltra, frontRightUltra, frontLeftUltra;

// ============ MOTOR ====================
const int EN = 23;

const int L_LPWM = 26;
const int L_RPWM = 25;

const int R_LPWM = 19;
const int R_RPWM = 18;

// ============= PWM =====================
#define PWM_FREQ 20000
#define PWM_RES 8

// ======================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("SKYT");

  rightUltra.trigPin = 4;
  rightUltra.echoPin = 22;
  rightUltra.initializePins();

  frontRightUltra.trigPin = 16;
  frontRightUltra.echoPin = 21;
  frontRightUltra.initializePins();

  leftUltra.trigPin = 27;
  leftUltra.echoPin = 34;
  leftUltra.initializePins();

  frontLeftUltra.trigPin = 14;
  frontLeftUltra.echoPin = 35;
  frontLeftUltra.initializePins();

  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

  pinMode(toggle, INPUT_PULLUP);

  xTaskCreatePinnedToCore(
    BT_TuningTask,
    "BT_Task",
    4000,
    NULL,
    1,
    NULL,
    0
  );
}

// ========== BLUETOOTH TUNING ===========
void BT_TuningTask(void *pvParameters) {
  for (;;) {
    if (SerialBT.available()) {
      char type = SerialBT.read();        // P, I, D, S
      float val = SerialBT.parseFloat();

      if (type == 'P') {
        Kp = val;
        SerialBT.print("Kp = ");
        SerialBT.println(Kp);
      }
      else if (type == 'I') {
        Ki = val;
        SerialBT.print("Ki = ");
        SerialBT.println(Ki);
      }
      else if (type == 'D') {
        Kd = val;
        SerialBT.print("Kd = ");
        SerialBT.println(Kd);
      }
      else if (type == 'S') {
        baseSpeed = (int)val;
        SerialBT.print("BaseSpeed = ");
        SerialBT.println(baseSpeed);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// ================= LOOP ================
void loop() {

  if (!digitalRead(toggle)) digitalWrite(EN, HIGH);
  else digitalWrite(EN, LOW);

  readUltrasonic(rightUltra);
  readUltrasonic(leftUltra);
  readUltrasonic(frontRightUltra);
  readUltrasonic(frontLeftUltra);

  float rightDist = rightUltra.currentDist;
  float leftDist = leftUltra.currentDist;
  float frontRightDist = frontRightUltra.currentDist;
  float frontLeftDist = frontLeftUltra.currentDist;

  Serial.print(rightDist); Serial.print(", ");
  Serial.print(leftDist); Serial.print(", ");
  Serial.print(frontLeftDist); Serial.print(", ");
  Serial.println(frontRightDist);

  // Error calculation (weighted)
  error = (leftDist - rightDist) * 0 +
          (frontLeftDist - frontRightDist) * 1;

  output = PID(error);

  if (rightDist <= 4.5) {
    rightSpeed = baseSpeed - 50;
    leftSpeed = baseSpeed;
  }
  else if (leftDist <= 4.5) {
    rightSpeed = baseSpeed;
    leftSpeed = baseSpeed - 50;
  }
  else {
    leftSpeed = baseSpeed + output;
    rightSpeed = baseSpeed - output;
  }

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  ledcWrite(L_LPWM, leftSpeed);
  ledcWrite(L_RPWM, 0);

  ledcWrite(R_LPWM, rightSpeed);
  ledcWrite(R_RPWM, 0);
}

// =========== ULTRASONIC READ ===========
void readUltrasonic(UltrasonicSensor &s) {
  digitalWrite(s.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trigPin, LOW);

  long duration = pulseIn(s.echoPin, HIGH, 2300);
  float rawDist = (duration == 0) ? maxDist : (duration * 0.0343 / 2);

  s.prevDist = rawDist;
  s.currentDist = rawDist;
}

// ================= PID =================
float PID(float e) {
  unsigned long currentMillis = millis();
  float dt = (currentMillis - prevMillis) / 1000.0;
  if (dt <= 0) dt = 0.001;

  integral += e * dt;
  integral = constrain(integral, -100, 100); // anti-windup

  derivative = (e - lastError) / dt;

  lastError = e;
  prevMillis = currentMillis;

  return (Kp * e) + (Ki * integral) + (Kd * derivative);
}
