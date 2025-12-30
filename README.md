# 🤖 ESP32 PID Line Follower Robot with NeoPixel Indicator

This project is a **PID-based line following robot** using **ESP32**, **5 IR line sensors**, and **NeoPixel LEDs** for visual feedback.  
The robot adjusts motor speeds dynamically based on line position for precise tracking.

---

## 🧠 Working Principle
1. 5 IR sensors detect the line (digital HIGH/LOW)
2. Robot calculates **line position** using weighted average
3. PID controller adjusts motor speeds:
   - **Proportional (P)** → reacts to current error
   - **Integral (I)** → compensates for accumulated error
   - **Derivative (D)** → predicts error change
4. NeoPixel LEDs indicate the **current line position**
5. Timeout logic stops robot if line is lost for >1.5 seconds

---

## 🧰 Components Used
- ESP32 Development Board
- 5 x IR Line Sensors
- 2 x DC Motors + L293D / L298N Motor Driver
- 5 x NeoPixel LEDs (WS2812)
- Robot chassis & wheels
- Battery / Power supply
- Jumper wires

---

## 🔌 Pin Configuration (As per Code)

### 🔹 Motor Pins
| Motor | Forward | Backward | PWM |
|-------|---------|----------|-----|
| LEFT  | ena_b (27) | ena_f (14) | speed_b (12) |
| RIGHT | enb_b (19) | enb_f (23) | speed_a (13) |

### 🔹 Line Sensors
| Sensor | ESP32 Pin | Weight |
|--------|-----------|--------|
| S1     | 34        | 0      |
| S2     | 35        | 1      |
| S3     | 32        | 2      |
| S4     | 33        | 3      |
| S5     | 25        | 4      |

### 🔹 NeoPixel LEDs
| LEDs | Pin |
|------|-----|
| 5 x WS2812 | 5 |

---

## ⚙️ Adjustable Parameters

```cpp
float basespeed = 200; // Base motor speed (0-255)
float kp = 100;        // Proportional gain
float ki = 0;          // Integral gain
float kd = 0;          // Derivative gain
unsigned long timeout_duration = 1500; // Stop if line lost

