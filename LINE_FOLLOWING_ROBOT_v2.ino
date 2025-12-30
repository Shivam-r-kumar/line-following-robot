#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN 5
#define NUM_LEDS 5
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// Motor driver pins (L293D or L298N)
#define ena_f 14
#define ena_b 27
#define enb_f 23
#define enb_b 19
#define speed_a 13  // RIGHT MOTOR PWM
#define speed_b 12  // LEFT MOTOR PWM

const byte no_of_sensor = 5;
const byte sensor_pin[no_of_sensor] = { 34, 35, 32, 33, 25 };
const byte sensor_weight[no_of_sensor] = { 0, 1, 2, 3, 4 };

float basespeed = 200;
float last_error = 0, integral = 0;
float kp = 100, ki = 0, kd = 0;

// timeout logic
unsigned long lastline_time = 0;
unsigned long timeout_duration = 1500;

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();

  for (byte i = 0; i < no_of_sensor; i++)
    pinMode(sensor_pin[i], INPUT);

  pinMode(ena_f, OUTPUT);
  pinMode(ena_b, OUTPUT);
  pinMode(enb_f, OUTPUT);
  pinMode(enb_b, OUTPUT);

  pinMode(speed_a, OUTPUT);
  pinMode(speed_b, OUTPUT);

  Serial.println("analogWrite Ready");
}

void loop() {
  float pos = position();
  position_indicator(pos);

  static unsigned long lastPID = 0;

  if (pos == -1 && millis() - lastline_time > timeout_duration) {
    stop();
  } else if (pos != -1) {
    lastline_time = millis();

    if (millis() - lastPID >= 5) {
      lastPID = millis();
      pid_controller(pos);
    }
  }
  // for debugging
  Serial.println("position - ");
  Serial.print(pos);
  for(int i = 0 ; i <no_of_sensor ; i++){
    Serial.println("sensor");
    Serial.print(i);
    Serial.print(" - ");
    int val = digitalRead(sensor_pin[i]);
    Serial.print(val);
  }
}

void pid_controller(float pos) {
  float error = 2 - pos;
  float d = error - last_error;


  if (error == 0 || pos == -1) {
    integral = 0;
  } else {
    integral += error;
    integral = constrain(integral, -100, 100);
  }

  

  last_error = error;

  float pid = (kp * error) + (ki * integral) + (kd * d);

  float left = basespeed + pid;
  float right = basespeed - pid;

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  Serial.print("PID = ");
  Serial.print(pid);
  Serial.print(" | I = ");
  Serial.print(integral);
  Serial.print(" | L = ");
  Serial.print(left);
  Serial.print(" | R = ");
  Serial.println(right);

  motor_control(right, left);

  delay(1);
}


void motor_control(float left, float right) {

  // LEFT MOTOR
  if (left >= 0) {
    digitalWrite(ena_f, HIGH);
    digitalWrite(ena_b, LOW);
  } else {
    digitalWrite(ena_f, LOW);
    digitalWrite(ena_b, HIGH);
  }
  analogWrite(speed_b, abs(left));

  // RIGHT MOTOR
  if (right >= 0) {
    digitalWrite(enb_f, HIGH);
    digitalWrite(enb_b, LOW);
  } else {
    digitalWrite(enb_f, LOW);
    digitalWrite(enb_b, HIGH);
  }
  analogWrite(speed_a, abs(right));
}

void stop() {
  digitalWrite(ena_f, LOW);
  digitalWrite(ena_b, LOW);
  digitalWrite(enb_f, LOW);
  digitalWrite(enb_b, LOW);

  analogWrite(speed_a, 0);
  analogWrite(speed_b, 0);
}

float position() {
  int num = 0;
  int den = 0;

  for (byte i = 0; i < no_of_sensor; i++) {
    if (digitalRead(sensor_pin[i])) {
      num += sensor_weight[i];
      den++;
    }
  }
  return (den > 0) ? ((float)num / den) : -1;
}

void position_indicator(float pos) {
  float pos_indicator = (pos >= 0) ? (4 - pos) : -1;

  strip.clear();
  if (pos_indicator >= 0) {
    int lowerLED = floor(pos_indicator);
    int upperLED = ceil(pos_indicator);
    float frac = pos_indicator - lowerLED;

    if (lowerLED >= 0 && lowerLED < NUM_LEDS)
      strip.setPixelColor(lowerLED, strip.Color(255 * (1 - frac), 0, 0));

    if (upperLED >= 0 && upperLED < NUM_LEDS && upperLED != lowerLED)
      strip.setPixelColor(upperLED, strip.Color(255 * frac, 0, 0));
  }
  strip.show();
}
