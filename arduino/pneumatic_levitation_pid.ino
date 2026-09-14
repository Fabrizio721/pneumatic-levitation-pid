#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// -----------------------------------------------------
// Pneumatic Levitation System - PID Controller
// Arduino Nano
// -----------------------------------------------------

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ---------------- Timing ----------------
unsigned long previousControlTime = 0;
unsigned long previousLCDTime = 0;

const unsigned long CONTROL_PERIOD_MS = 10;
const unsigned long LCD_PERIOD_MS = 250;

// ---------------- HC-SR04 ----------------
const int trigPin = 2;
const int echoPin = 3;

const float MAX_HEIGHT_CM = 30.0;

// ---------------- Potentiometers ----------------
const int setpointPotPin = A0;
const int kpPotPin = A1;
const int kiPotPin = A2;
const int kdPotPin = A3;

// ---------------- L298N Motor Driver ----------------
const int pwmPin = 6;   // ENA
const int IN1 = 7;
const int IN2 = 8;

// ---------------- PID Variables ----------------
double kp = 0.0;
double ki = 0.0;
double kd = 0.0;

double setpoint = 0.0;

double error = 0.0;
double previousError = 0.0;
double integralTerm = 0.0;
double derivativeTerm = 0.0;

double controlOutput = 0.0;
double ballHeight = 0.0;


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(9600);

  // HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  digitalWrite(trigPin, LOW);

  // Potentiometers
  pinMode(setpointPotPin, INPUT);
  pinMode(kpPotPin, INPUT);
  pinMode(kiPotPin, INPUT);
  pinMode(kdPotPin, INPUT);

  // L298N
  pinMode(pwmPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Fixed fan rotation direction
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  analogWrite(pwmPin, 0);

  // LCD
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("PID LEVITATION");

  lcd.setCursor(0, 1);
  lcd.print("SYSTEM READY");

  delay(1500);
  lcd.clear();

  previousControlTime = millis();
  previousLCDTime = millis();
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  unsigned long currentTime = millis();

  // ---------------------------------------------------
  // Control loop every 10 ms
  // ---------------------------------------------------

  if (currentTime - previousControlTime >= CONTROL_PERIOD_MS) {

    previousControlTime = currentTime;

    // ===============================================
    // Read potentiometers
    // ===============================================

    int setpointReading = analogRead(setpointPotPin);
    int kpReading = analogRead(kpPotPin);
    int kiReading = analogRead(kiPotPin);
    int kdReading = analogRead(kdPotPin);

    // Setpoint: 0 - 30 cm
    setpoint =
        ((double)setpointReading / 1023.0) * MAX_HEIGHT_CM;

    // Experimental PID adjustment ranges
    kp =
        ((double)kpReading / 1023.0) * 30.0;

    ki =
        ((double)kiReading / 1023.0) * 0.50;

    kd =
        ((double)kdReading / 1023.0) * 100.0;


    // ===============================================
    // HC-SR04 Distance Measurement
    // ===============================================

    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);

    digitalWrite(trigPin, LOW);

    unsigned long duration =
        pulseIn(echoPin, HIGH, 30000);

    if (duration == 0) {

      ballHeight = 0.0;

    } else {

      // Distance between sensor and ball
      double sensorDistance = duration / 58.2;

      // Sensor is located at the top of the tube
      ballHeight =
          MAX_HEIGHT_CM - sensorDistance;

      // Safety limits
      if (ballHeight < 0.0)
        ballHeight = 0.0;

      if (ballHeight > MAX_HEIGHT_CM)
        ballHeight = MAX_HEIGHT_CM;
    }


    // ===============================================
    // PID Controller
    // ===============================================

    error = setpoint - ballHeight;

    derivativeTerm =
        error - previousError;

    integralTerm += ki * error;


    // ===============================================
    // Integral Anti-Windup
    // ===============================================

    if (integralTerm > 255)
      integralTerm = 255;

    if (integralTerm < 0)
      integralTerm = 0;


    // ===============================================
    // PID Control Output
    // ===============================================

    controlOutput =
        (kp * error)
        + integralTerm
        + (kd * derivativeTerm);


    // ===============================================
    // PWM Saturation
    // ===============================================

    if (controlOutput > 255)
      controlOutput = 255;

    if (controlOutput < 0)
      controlOutput = 0;


    // ===============================================
    // Apply PWM to Fan
    // ===============================================

    analogWrite(
        pwmPin,
        (int)controlOutput
    );


    // ===============================================
    // Serial Data Output
    // ===============================================

    Serial.print("Setpoint:");
    Serial.print(setpoint);

    Serial.print(" ");

    Serial.print("Height:");
    Serial.print(ballHeight);

    Serial.print(" ");

    Serial.print("PWM:");
    Serial.print(controlOutput);

    Serial.print(" ");

    Serial.print("Kp:");
    Serial.print(kp);

    Serial.print(" ");

    Serial.print("Ki:");
    Serial.print(ki);

    Serial.print(" ");

    Serial.print("Kd:");
    Serial.println(kd);


    previousError = error;
  }


  // ---------------------------------------------------
  // LCD update
  // ---------------------------------------------------

  if (currentTime - previousLCDTime >= LCD_PERIOD_MS) {

    previousLCDTime = currentTime;

    lcd.setCursor(0, 0);

    lcd.print("SP:");
    lcd.print((int)setpoint);

    lcd.print(" H:");
    lcd.print((int)ballHeight);

    lcd.print("   ");

    lcd.setCursor(0, 1);

    lcd.print("P:");
    lcd.print((int)kp);

    lcd.print(" I:");
    lcd.print(ki, 2);

    lcd.print(" D:");
    lcd.print((int)kd);

    lcd.print(" ");
  }
}
