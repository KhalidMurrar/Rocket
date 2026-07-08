
// PID
//volatile float Kp = 10.2 , Kd = 1.65; //200
volatile float Kp = 6.85 , Kd = 1.25; //170
float derivative = 0.0;
float error, lastError, output;
unsigned long prevMillis;

//volatile int baseSpeed = 200;
volatile int baseSpeed = 170;
int leftSpeed, rightSpeed;
const float setPoint = 4;

const int toggle = 17;

const int minDist = 2;
const int maxDist = 45;

float right, left;

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

const int EN = 23;

const int L_LPWM = 26;
const int L_RPWM = 25;

const int R_LPWM = 19;
const int R_RPWM = 18;

// PWM
#define PWM_FREQ 20000
#define PWM_RES 8

void setup() {
  Serial.begin(115200);

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
1230.`
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  ledcAttach(L_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(L_LPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_RPWM, PWM_FREQ, PWM_RES);
  ledcAttach(R_LPWM, PWM_FREQ, PWM_RES);

  pinMode(toggle, INPUT_PULLUP);
}

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

  Serial.print(rightDist);
  Serial.print(", ");
  Serial.print(leftDist);
  Serial.print(", ");
  Serial.print(frontLeftDist);
  Serial.print(", ");
  Serial.println(frontRightDist);

  if (leftDist <= frontLeftDist) {
    left = leftDist;
  } else left = frontLeftDist;

  if (rightDist <= frontRightDist) right = rightDist;
  else right = frontRightDist;

  Serial.println(Kd);

 error = (leftDist - rightDist) * 0.4 + (frontLeftDist - frontRightDist) * 0.6;
  //};
  //error = left - right;
  output = PID(error);

   if (rightDist <= 4.5) {
    rightSpeed = baseSpeed - 50;
    leftSpeed = baseSpeed;
    //Serial.println("close right");
  } else if (leftDist <= 4.5) {
    rightSpeed = baseSpeed;
    leftSpeed = baseSpeed - 50;
    //Serial.println("close left");
  } else {
    leftSpeed = baseSpeed + output;
    rightSpeed = baseSpeed - output;
    //Serial.println("mid");
  }
  //leftSpeed = baseSpeed + output;
  //rightSpeed = baseSpeed - output;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  ledcWrite(L_LPWM, leftSpeed);
  ledcWrite(L_RPWM, 0);

  ledcWrite(R_LPWM, rightSpeed);
  ledcWrite(R_RPWM, 0);
}

void readUltrasonic(UltrasonicSensor &s) {
  digitalWrite(s.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(s.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(s.trigPin, LOW);

  long duration = pulseIn(s.echoPin, HIGH, 2300);
  float rawDist = (duration == 0) ? maxDist : (duration * 0.0343 / 2);

  //float distance = (rawDist * 0.7) + (s.prevDist * 0.3);

  s.prevDist = rawDist;
  s.currentDist = rawDist;
}

float PID(float e) {
  unsigned long currentMillis = millis();
  float dt = (currentMillis - prevMillis) / 1000.0;

  derivative = (error - lastError) / dt;

  lastError = e;
  prevMillis = currentMillis;

  return Kp * e + Kd * derivative;
}
