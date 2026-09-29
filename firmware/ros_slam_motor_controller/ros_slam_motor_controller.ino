// Ros Slam Robot - Arduino UNO differential-drive controller
// Serial input:  V <left_mps> <right_mps>
// Serial output: E <left_ticks> <right_ticks>

#include <Arduino.h>

constexpr uint8_t LEFT_PWM = 5;
constexpr uint8_t LEFT_IN1 = 7;
constexpr uint8_t LEFT_IN2 = 8;
constexpr uint8_t RIGHT_PWM = 6;
constexpr uint8_t RIGHT_IN1 = 9;
constexpr uint8_t RIGHT_IN2 = 10;
constexpr uint8_t LEFT_ENCODER = 2;
constexpr uint8_t RIGHT_ENCODER = 3;

constexpr float MAX_WHEEL_SPEED = 0.45F;
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;
constexpr unsigned long REPORT_INTERVAL_MS = 50;

volatile long left_ticks = 0;
volatile long right_ticks = 0;
volatile int8_t left_direction = 1;
volatile int8_t right_direction = 1;
unsigned long last_command_ms = 0;
unsigned long last_report_ms = 0;
String input;

void onLeftEncoder() { left_ticks += left_direction; }
void onRightEncoder() { right_ticks += right_direction; }

void driveMotor(uint8_t pwm_pin, uint8_t in1, uint8_t in2, float velocity) {
  velocity = constrain(velocity, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
  const bool forward = velocity >= 0.0F;
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);
  const int pwm = static_cast<int>(255.0F * abs(velocity) / MAX_WHEEL_SPEED);
  analogWrite(pwm_pin, pwm);
}

void stopMotors() {
  analogWrite(LEFT_PWM, 0);
  analogWrite(RIGHT_PWM, 0);
}

void parseCommand(const String &line) {
  if (!line.startsWith("V ")) return;
  const int separator = line.indexOf(' ', 2);
  if (separator < 0) return;
  const float left = line.substring(2, separator).toFloat();
  const float right = line.substring(separator + 1).toFloat();
  noInterrupts();
  left_direction = left < 0.0F ? -1 : 1;
  right_direction = right < 0.0F ? -1 : 1;
  interrupts();
  driveMotor(LEFT_PWM, LEFT_IN1, LEFT_IN2, left);
  driveMotor(RIGHT_PWM, RIGHT_IN1, RIGHT_IN2, right);
  last_command_ms = millis();
}

void setup() {
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), onLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), onRightEncoder, RISING);
  Serial.begin(115200);
  stopMotors();
}

void loop() {
  while (Serial.available()) {
    const char character = Serial.read();
    if (character == '\n') {
      parseCommand(input);
      input = "";
    } else if (character != '\r' && input.length() < 63) {
      input += character;
    }
  }

  const unsigned long now = millis();
  if (now - last_command_ms > COMMAND_TIMEOUT_MS) stopMotors();

  if (now - last_report_ms >= REPORT_INTERVAL_MS) {
    noInterrupts();
    const long left = left_ticks;
    const long right = right_ticks;
    interrupts();
    Serial.print("E ");
    Serial.print(left);
    Serial.print(' ');
    Serial.println(right);
    last_report_ms = now;
  }
}
