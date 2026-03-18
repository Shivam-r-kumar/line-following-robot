# 🤖 ESP32 PID Line Follower Robot (7 Sensor)

This project is an **advanced PID-based Line Follower Robot** built using **ESP32** and **7 IR sensors**.  
The robot follows a black line using a **PD control algorithm** and can automatically detect **left turns, right turns, U-turns, and stop conditions**.

This project is suitable for **robotics competitions, autonomous navigation experiments, and embedded systems learning**.

---

# 🧠 Working Principle

The robot uses **7 IR sensors** to detect the line position.

Each sensor has a **weight value** representing its position.

| Sensor | Weight |
|------|------|
| S1 | -3 |
| S2 | -2 |
| S3 | -1 |
| S4 | 0 |
| S5 | 1 |
| S6 | 2 |
| S7 | 3 |

### Step 1 — Sensor Reading
Sensors detect the black line and output **HIGH / LOW signals**.

### Step 2 — Error Calculation
The robot calculates the **error** using the weighted sensor values.

```
error = sum(weight[i] of active sensors)
```

### Step 3 — PD Control
The robot calculates PID correction using **Proportional and Derivative terms**.

```
PID = (error * Kp) + (error - last_error) * Kd
```

### Step 4 — Motor Speed Adjustment

```
Left Motor Speed  = BaseSpeed + PID
Right Motor Speed = BaseSpeed - PID
```

This keeps the robot centered on the line.

---

# 🔄 Turn Detection

The robot detects turns using the **edge sensors**.

### Right Turn
If the **rightmost sensor detects line**:

```
S7 = 1 and S1 = 0
```

Robot performs a **right turn**.

### Left Turn
If the **leftmost sensor detects line**:

```
S1 = 1 and S7 = 0
```

Robot performs a **left turn**.

### U-Turn
If **no sensor detects the line**:

```
sensor_sum = 0
```

Robot performs a **U-turn** to recover the path.

### Stop Condition
If **all sensors detect line**:

```
sensor_sum = 7
```

Robot stops until the line disappears.

---

# 🧰 Components Used

- ESP32 Development Board
- 7 × IR Line Sensors
- 2 × DC Motors
- L298N / L293D Motor Driver
- Robot chassis
- Wheels
- Li-ion battery pack
- Jumper wires

---

# 🔌 Pin Configuration

## Motor Pins

| Motor | Forward | Backward | PWM |
|------|------|------|------|
| LEFT | 22 | 23 | 5 |
| RIGHT | 19 | 21 | 17 |

---

## Sensor Pins

| Sensor | ESP32 Pin |
|------|------|
| S1 | 34 |
| S2 | 35 |
| S3 | 32 |
| S4 | 33 |
| S5 | 25 |
| S6 | 26 |
| S7 | 27 |

---

# ⚙ Adjustable Parameters

These values can be tuned for better performance.

```cpp
uint8_t max_speed = 250;

int left_motor_speed = 200;
int right_motor_speed = 200;

uint8_t turn_speed = 150;

int kp = 70;
int kd = 10;
```

### Parameter Explanation

| Parameter | Description |
|------|------|
| max_speed | Maximum PWM speed |
| left_motor_speed | Base speed of left motor |
| right_motor_speed | Base speed of right motor |
| turn_speed | Speed used during turns |
| kp | Proportional gain |
| kd | Derivative gain |

---

# ⏱ Delay Parameters

```cpp
#define turn_delay 10
#define u_turn_delay 50
#define stop_timer 30
```

These parameters control **turn timing and stop detection**.

---

# 🚀 Features

✔ PD based smooth line following  
✔ 7 sensor high precision detection  
✔ Automatic left/right turn detection  
✔ U-turn recovery if line lost  
✔ Stop detection at intersections  
✔ High speed control using ESP32  

---

# 📚 Concepts Used

This project demonstrates:

- PID Control System
- Sensor data processing
- Embedded motor control
- Autonomous robot navigation
- ESP32 programming

---

# 🎯 Applications

- Line follower robot competitions
- Autonomous warehouse robots
- Industrial AGV navigation
- Robotics learning projects
