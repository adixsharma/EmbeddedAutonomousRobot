# Autonomous Robtot – Embedded Line-Following Robot with Obstacle Avoidance

**Authors:**  Aditya Sharma, Zachary Howe, Yu-Ming Chen, and James Nguyen

**Platform:** Raspberry Pi 4 · Embedded C  
**Project Type:** Group Term Project

---

## 📦 Project Overview

**EmbeddedAutonomousRobot** is an autonomous robot programmed in low-level C to follow a line using PID control and intelligently avoid obstacles using 2 ultrasonic sensors. Built on the Raspberry Pi 4, the robot uses a multithreaded design to handle real-time sensor input and motor control with minimal latency.

---

## 🔧 Hardware Components

- Raspberry Pi 4
- (5) IR reflectance sensors (line detection)
- (2) ultrasonic sensors (HC-SR04 for front and side detection)
- Waveshare Motor Driver HAT (PCA9685 for PWM motor control)
- (2) 12v Motors 130 rpm
- Drive chassis

---

## ⚙️ Software Architecture

The codebase is implemented in low-level C with three concurrent threads:

### 🧵 Thread Overview

1. **Main Control Thread**
   - Polls IR sensors
   - Computes PID error
   - Sets motor speeds via PWM for steering

2. **Front Sonar Thread**
   - Measures distance using HC-SR04
   - Triggers avoidance logic if an obstacle is < 30 cm

3. **Side Sonar Thread**
   - Activated during avoidance
   - Follows wall/object edge until the line is reacquired

All threads share data using `volatile` globals and coordinate access through `pthread_mutex_t` locks to prevent race conditions.


---

## 🧠 PID Line-Following Logic

The robot computes a steering correction based on the relative position of the line under its IR sensors. A weighted average of sensor readings is used to derive a deviation error, which is then passed through a PID controller:

```c
error = weighted_sum(sensor_values) - center_reference;
correction = Kp * error + Ki * integral + Kd * derivative;
```

---

## 🚧 Obstacle Avoidance Logic

- If front sonar detects an obstacle within 30 cm:
  1. The robot halts
  2. Pivots left to disengage from the line until the side sonar sensor detects the object
  3. Utilize side sonar to follow the object’s edge
  4. Once the line is detected again, returns to PID Line following mode

This makes the robot capable of navigating dynamic environments with blockages.

---

## 🧰 Skills Demonstrated

- Embedded C with POSIX threads
- Real-time control with sensor fusion
- PID tuning and feedback systems
- Custom GPIO and I2C driver integration
- Team-based software development and hardware assembly

---


