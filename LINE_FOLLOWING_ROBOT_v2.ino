// ---------------- MOTOR PINS ----------------
#define Rforward 19
#define Rbackward 21
#define Lforward 22
#define Lbackward 23

#define Rspeed 17
#define Lspeed 5


// ---------------- SENSOR CONFIG ----------------
#define num_of_sensor 7

int sensorPin[num_of_sensor] = {34, 35, 32, 33, 25, 26, 27};

int weight[7] = { -3, -2, -1, 0, 1, 2, 3 };


// ---------------- DELAY PARAMETERS ----------------
#define turn_delay 10
#define u_turn_delay 50
#define stop_timer 30


// ---------------- MOTOR SPEED VARIABLES ----------------
uint8_t max_speed = 250;

int left_motor_speed = 200;
int right_motor_speed = 200;

uint8_t turn_speed = 150;


// ---------------- PID VARIABLES ----------------
int kp = 70;
int kd = 10;

int pid;
int error;
int last_error;

bool turning_state = false;
uint8_t turn_value = 0;

uint8_t sensor_sum = 0;


// ---------------- SETUP ----------------
void setup() {

  Serial.begin(115200);

  pinMode(Lforward, OUTPUT);
  pinMode(Lbackward, OUTPUT);
  pinMode(Rforward, OUTPUT);
  pinMode(Rbackward, OUTPUT);

  pinMode(Rspeed, OUTPUT);
  pinMode(Lspeed, OUTPUT);

  for (int i = 0; i < num_of_sensor; i++) {
    pinMode(sensorPin[i], INPUT);
  }
}


// ---------------- LOOP ----------------
void loop() {
  Line_Follow();
}


// ---------------- SENSOR READING ----------------
void read_sensor() {

  error = 0;
  sensor_sum = 0;

  for (int i = 0; i < num_of_sensor; i++) {

    if (digitalRead(sensorPin[i]) == 1) {

      error = error + weight[i];
      sensor_sum++;
    }
  }
  Serial.println("------------------------------ ");
  Serial.print("error -- ");
  Serial.println(error);
  Serial.print("sensor_sum -- ");
  Serial.println(sensor_sum);

  if (error == 0 && sensor_sum == 0)
    error = last_error;
}


// ---------------- PID LINE FOLLOW ----------------
void Line_Follow() {

  read_sensor();
  turn_detection();

  if (turning_state)
    return;

  pid = error * kp + kd * (error - last_error);

  last_error = error;

  Serial.print("pid -- ");
  Serial.println(pid);

  int right_motor = right_motor_speed - pid;
  int left_motor = left_motor_speed + pid;

  motor(left_motor, right_motor);
}


// ---------------- TURN DETECTION ----------------
void turn_detection() {

  if (digitalRead(sensorPin[6]) == 1 && digitalRead(sensorPin[0]) == 0)
    turn_value = 1;

  if (digitalRead(sensorPin[0]) == 1 && digitalRead(sensorPin[6]) == 0)
    turn_value = 2;


  if (sensor_sum == 0 && !turning_state) {

    if (turn_value == 1) {

      delay(turn_delay);
      motor(-turn_speed, turn_speed);
      turning_state = true;
    }

    else if (turn_value == 2) {

      delay(turn_delay);
      motor(turn_speed, -turn_speed);
      turning_state = true;
    }

    else if (turn_value == 0) {

      delay(u_turn_delay);
      motor(turn_speed, -turn_speed);
      turning_state = true;
    }
  }


  if (turning_state) {

    if (digitalRead(sensorPin[2]) == 0 || digitalRead(sensorPin[3]) == 0 || digitalRead(sensorPin[4]) == 0) {

      turning_state = false;
      turn_value = 0;
    }

    return;
  }


  else if (sensor_sum == 7) {

    delay(stop_timer);

    read_sensor();

    if (sensor_sum == 7) {

      while (sensor_sum == 7) {

        read_sensor();
        motor(0, 0);
      }
    }

    else if (sensor_sum == 0) {

      turn_value = 2;
      turning_state = true;
    }
  }
}


// ---------------- MOTOR FUNCTION ----------------
void motor(int left, int right) {

  if (right > 0) {
    digitalWrite(Rforward, HIGH);
    digitalWrite(Rbackward, LOW);
  }
  else {
    right = -right;
    digitalWrite(Rforward, LOW);
    digitalWrite(Rbackward, HIGH);
  }


  if (left > 0) {
    digitalWrite(Lforward, HIGH);
    digitalWrite(Lbackward, LOW);
  }
  else {
    left = -left;
    digitalWrite(Lforward, LOW);
    digitalWrite(Lbackward, HIGH);
  }


  if (left > max_speed) left = max_speed;
  if (right > max_speed) right = max_speed;


  analogWrite(Lspeed, left);
  analogWrite(Rspeed, right);
}
