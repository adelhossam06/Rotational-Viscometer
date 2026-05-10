#include <PID_v1.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define MOTOR_PWM 6
#define MOTOR_IN1 7
#define MOTOR_IN2 5
#define ENCODER_A 2
#define CURRENT_PIN A0

#define PULSES_PER_REV 20
#define KT 0.05
#define RADIUS 0.02
#define HEIGHT 0.05
#define MAX_CURRENT 4.0

const float ACS712_SENSITIVITY = 0.185f;
const float ADC_VREF = 5.0f;
const float ADC_COUNTS = 1023.0f;
const float CURRENT_FILTER_ALPHA = 0.15f;

volatile long pulses = 0;
double actualRPM = 0, targetRPM = 0, motorPWM = 0;
float currentAmps = 0;
unsigned long lastTime = 0;
float currentZeroV = 2.5f;
float currentFiltered = 0.0f;

float viscosityResults[4] = {0, 0, 0, 0};
float airBaselines[] = {0.085, 0.085, 0.085, 0.085};

double Kp = 0.8, Ki = 1.2, Kd = 0.05;
PID myPID(&actualRPM, &motorPWM, &targetRPM, Kp, Ki, Kd, DIRECT);

void countEncoder() { pulses++; }

float readCurrentInstantA() {
  const int N = 40;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(CURRENT_PIN);
    delayMicroseconds(200);
  }
  float raw = (float)sum / (float)N;
  float voltage = (raw / ADC_COUNTS) * ADC_VREF;
  float amps = (voltage - currentZeroV) / ACS712_SENSITIVITY;
  currentFiltered = (1.0f - CURRENT_FILTER_ALPHA) * currentFiltered + CURRENT_FILTER_ALPHA * amps;
  return fabs(currentFiltered);
}

void calibrateACS712Zero() {
  analogWrite(MOTOR_PWM, 0);
  lcd.clear(); lcd.print("Calibrating...");
  long sum = 0;
  for (int i = 0; i < 700; i++) {
    sum += analogRead(CURRENT_PIN);
    delay(2);
  }
  currentZeroV = ((float)sum / 700.0 / ADC_COUNTS) * ADC_VREF;
  lcd.clear(); lcd.print("Zero Set!"); delay(1000);
}

void calculateRPM() {
  if (millis() - lastTime >= 500) {
    noInterrupts();
    long p = pulses; pulses = 0;
    interrupts();
    actualRPM = (p * 120.0) / PULSES_PER_REV;
    lastTime = millis();
  }
}

void runControlLoop() {
  calculateRPM();
  currentAmps = readCurrentInstantA();
  if (currentAmps > MAX_CURRENT) {
    analogWrite(MOTOR_PWM, 0);
    lcd.clear(); lcd.print("OVERCURRENT!");
    while (1) {}
  }
  myPID.Compute();
  motorPWM = constrain(motorPWM, 0, 255);
  analogWrite(MOTOR_PWM, (int)motorPWM);
}

float calculateFinalViscosity(float avgRPM, float avgAmps, float currentBaseline) {
  if (avgRPM < 10) return 0.0f;
  float netAmps = avgAmps - currentBaseline;
  if (netAmps < 0.005) netAmps = 0.001;
  float omega = (2.0f * 3.1416f * RADIUS * avgRPM) / 60.0f;
  float torque = KT * netAmps;
  float gap = 0.01;
  float areaFactor = 2.0f * 3.1416f * RADIUS * RADIUS * HEIGHT;
  float taw = torque / areaFactor;
  float gamma = (omega * RADIUS) / gap;

  Serial.print(gamma);
  Serial.print(",");
  Serial.println(taw);

  return torque * gap / (areaFactor * omega);
}

void setup() {
  Serial.begin(9600);
  lcd.init(); lcd.backlight();
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT); pinMode(MOTOR_IN2, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), countEncoder, RISING);
  calibrateACS712Zero();
  digitalWrite(MOTOR_IN1, HIGH); digitalWrite(MOTOR_IN2, LOW);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(20, 255);
}

void loop() {
  int testSpeeds[] = {80, 120, 160, 200};

  for (int i = 0; i < 4; i++) {
    targetRPM = testSpeeds[i];
    float currentBaseline = airBaselines[i];

    lcd.clear(); lcd.print("Target: "); lcd.print(targetRPM);

    unsigned long stepTimer = millis();
    unsigned long lastPrint = millis();
    float sumRPM = 0, sumAmps = 0;
    int count = 0;

    while (millis() - stepTimer < 6000) {
      runControlLoop();
      if (millis() - stepTimer > 2000 && millis() - lastPrint >= 500) {
        sumRPM += actualRPM;
        sumAmps += currentAmps;
        count++;
        lastPrint = millis();
      }
    }

    if (count > 0) {
      float finalRPM = sumRPM / count;
      float finalAmps = 2.3 * sumAmps / count;
      float viscosity = calculateFinalViscosity(finalRPM, finalAmps, currentBaseline);

      viscosityResults[i] = viscosity;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("R:"); lcd.print(finalRPM, 0);
      lcd.print(" A:"); lcd.print(finalAmps, 3);
      lcd.setCursor(0, 1);
      lcd.print("Visc: "); lcd.print(viscosity, 4);

      delay(4000);
    }
  }

  targetRPM = 0;
  analogWrite(MOTOR_PWM, 0);

  float vStart = viscosityResults[0];
  float vEnd = viscosityResults[3];

  lcd.clear();
  lcd.setCursor(0, 0);

  float diff = (vEnd - vStart) / vStart;

  if (abs(diff) < 0.15) {
    lcd.print("Type: Newtonian");
    Serial.println("Fluid Type: Newtonian");
  } else if (vEnd < vStart) {
    lcd.print("Non-Newt: Pseudo");
    lcd.setCursor(0, 1);
    lcd.print("Shear-thinning");
    Serial.println("Fluid Type: Non-Newtonian (Pseudoplastic)");
  } else {
    lcd.print("Non-Newt: Dilat");
    lcd.setCursor(0, 1);
    lcd.print("Shear-thickening");
    Serial.println("Fluid Type: Non-Newtonian (Dilatant)");
  }

  while (1) {}
}
