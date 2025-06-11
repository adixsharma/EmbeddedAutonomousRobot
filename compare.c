/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Aditya Sharma, Zachary Howe
 * Student ID:: 917586584, 923229694
 * Github-Name:: adixsharma, zhowe1
 * Project:: Car + Color Sensor Integration
 *
 * File:: main.c
 *
 * Description::
 *   Three-threaded PID line follower (5 sensors) + ultrasonic
 *   obstacle detection (HC-SR04). Integrated RGB sensor (TCS34725)
 *   reads color in control thread via i2c address switching.
 **************************************************************/

 #include "gpio_library/chips/pca9685.h"
 #include "gpio_library/core/i2c_access.h"
 #include "gpio_library/core/pins.h"
 #include "gpio_library/TCS34725/tcs_controller.h"
 
 #include <math.h>
 #include <pthread.h>
 #include <signal.h>
 #include <stdio.h>
 #include <stdlib.h>
 #include <time.h>
 #include <unistd.h>
 
 #define FRONT_COUNT 5
 static const int FRONT_PINS[FRONT_COUNT] = {26, 19, 13, 6, 5};
 
 #define PWMA 0
 #define AIN1 1
 #define AIN2 2
 #define BIN1 3
 #define BIN2 4
 #define PWMB 5
 
 #define KP 7
 #define KI 0.02
 #define KD 0.5
 
 #define BASE_SPEED 100.0
 #define SENSOR_DELAY 5000
 #define CONTROL_DELAY 10000
 #define SONAR_DELAY 50000
 
 #define TRIG_PIN 23
 #define ECHO_PIN 24
 #define SOUND_SPEED_CM_NS 0.000034
 #define NANO_TO_SECOND 1000000000LL
 #define OBSTACLE_DIST_CM 25.0
 #define MIN_OBSTACLE_DIST_CM 2.0
 
 // Color sensor
 #define COLOR_SENSE_SPEED 4
 
 static volatile int front_vals[FRONT_COUNT];
 static pthread_mutex_t sensor_lock;
 
 static volatile double last_distance_cm = 1e6;
 static pthread_mutex_t sonar_lock;
 
 static volatile int running = 1;
 static volatile unsigned int *gpio_map = NULL;
 const double STEER_GAIN = 1.0;
 
 i2c_data i2c_color;
 i2c_data* i2c;
 
 void handle_sigint(int sig) {
   (void)sig;
   running = 0;
 }
 
 void stop_motors(void) {
   i2c_set_target_address(0x40);
   pca9685_set_level(AIN1, 0);
   pca9685_set_level(AIN2, 0);
   pca9685_set_pwm_dutycycle(PWMA, 0);
   pca9685_set_level(BIN1, 0);
   pca9685_set_level(BIN2, 0);
   pca9685_set_pwm_dutycycle(PWMB, 0);
 }
 
 void *sensor_thread(void *arg) {
   (void)arg;
   while (running) {
     pthread_mutex_lock(&sensor_lock);
     for (int i = 0; i < FRONT_COUNT; i++)
       front_vals[i] = read_pin(gpio_map, FRONT_PINS[i]) ? 1 : 0;
     pthread_mutex_unlock(&sensor_lock);
     usleep(SENSOR_DELAY);
   }
   return NULL;
 }
 
 long long getNanoTime(void) {
   struct timespec ts;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   return (long long)ts.tv_sec * NANO_TO_SECOND + ts.tv_nsec;
 }
 
 void sendTrigPulse(void) {
   write_pin(gpio_map, TRIG_PIN, 0);
   usleep(5);
   write_pin(gpio_map, TRIG_PIN, 1);
   usleep(10);
   write_pin(gpio_map, TRIG_PIN, 0);
 }
 
 double measureDistance(void) {
   long long start, end, dur;
   sendTrigPulse();
   while (!read_pin(gpio_map, ECHO_PIN)) if (!running) return -1;
   start = getNanoTime();
   while (read_pin(gpio_map, ECHO_PIN)) if (!running) return -1;
   end = getNanoTime();
   dur = end - start;
   return (dur / 2.0) * SOUND_SPEED_CM_NS;
 }
 
 void *sonar_thread(void *arg) {
   (void)arg;
   while (running) {
     double d = measureDistance();
     pthread_mutex_lock(&sonar_lock);
     last_distance_cm = d;
     pthread_mutex_unlock(&sonar_lock);
     usleep(SONAR_DELAY);
   }
   return NULL;
 }
 
 typedef struct {
   double Kp, Ki, Kd;
   double integral, last_error;
 } PID;
 
 void PID_Init(PID *p) {
   p->Kp = KP;
   p->Ki = KI;
   p->Kd = KD;
   p->integral = 0;
   p->last_error = 0;
 }
 
 double PID_Update(PID *p, double err, double dt) {
   p->integral += err * dt;
   double deriv = (err - p->last_error) / dt;
   double out = p->Kp * err + p->Ki * p->integral + p->Kd * deriv;
   p->last_error = err;
   return out;
 }
 
 double compute_error(void) {
   static const int W[FRONT_COUNT] = {-8, -2, 0, +2, +8};
   int sum = 0, cnt = 0;
   pthread_mutex_lock(&sensor_lock);
   for (int i = 0; i < FRONT_COUNT; i++) {
     sum += front_vals[i] * W[i];
     cnt += front_vals[i];
   }
   pthread_mutex_unlock(&sensor_lock);
   return cnt ? (double)sum / cnt : 0.0;
 }
 
 int dominant_color(uint16_t r, uint16_t g, uint16_t b) {
   if (r < 40 && g < 40 && b < 40) return 0;
   if (r >= 3 * g && r >= 3 * b) return 1;
   if (b >= 3 * r && b >= 3 * g) return 2;
   if (g >= 3 * r && g >= 3 * b) return 3;
   return 0;
 }
 
 void *control_thread(void *arg) {
   (void)arg;
   PID pid;
   PID_Init(&pid);
   struct timespec prev, curr;
   clock_gettime(CLOCK_MONOTONIC, &prev);
 
   while (running) {
     pthread_mutex_lock(&sonar_lock);
     double dist = last_distance_cm;
     pthread_mutex_unlock(&sonar_lock);
 
     if (dist >= MIN_OBSTACLE_DIST_CM && dist < OBSTACLE_DIST_CM) {
       printf("Obstacle at %.1f cm! Stopping\n", dist);
       stop_motors();
       sleep(3);
       pid.integral = 0;
       pid.last_error = 0;
     }
 
     clock_gettime(CLOCK_MONOTONIC, &curr);
     double dt = (curr.tv_sec - prev.tv_sec) + (curr.tv_nsec - prev.tv_nsec) / 1e9;
     prev = curr;
 
     int s[FRONT_COUNT];
     pthread_mutex_lock(&sensor_lock);
     for (int i = 0; i < FRONT_COUNT; i++) s[i] = front_vals[i];
     pthread_mutex_unlock(&sensor_lock);
 
     if ((s[1] && s[2] && s[3] && !s[0] && !s[4]) ||
         (s[2] && !s[1] && !s[0] && !s[3] && !s[4])) {
       pid.integral = 0;
       pid.last_error = 0;
     }
 
     double err = compute_error();
     if ((!s[0] && !s[1] && !s[2] && !s[3] && !s[4]) ||
         (s[0] && s[1] && s[2] && s[3] && s[4])) {
       err = pid.last_error + (pid.integral * 0.001);
     }
 
     double steer = PID_Update(&pid, err, dt) * STEER_GAIN;
     double L = BASE_SPEED - steer;
     double R = BASE_SPEED + steer;
     if (L < 0) L = 0; else if (L > 100) L = 100;
     if (R < 0) R = 0; else if (R > 100) R = 100;
 
     i2c_set_target_address(0x40);
     pca9685_set_level(AIN1, 1); pca9685_set_level(AIN2, 0);
     pca9685_set_pwm_dutycycle(PWMA, (int)(L + .5));
     pca9685_set_level(BIN1, 1); pca9685_set_level(BIN2, 0);
     pca9685_set_pwm_dutycycle(PWMB, (int)(R + .5));
 
     rgb_color c = sense_color(i2c);
     int color_id = (c.is_valid ? dominant_color(c.red, c.green, c.blue) : 0);
 
     // For testing purposes, replace with button logic
     const char *label = "Unknown";
     if (color_id == 1) label = "Red";
     else if (color_id == 2) label = "Blue";
     else if (color_id == 3) label = "Green";
 
     printf("SENS:"); for (int i = 0; i < FRONT_COUNT; i++) printf("%d", s[i]);
     printf(" ERR=%.2f S=%.2f L=%.0f R=%.0f DIST=%.1f COLOR=%s\n", err, steer, L, R, dist, label);
 
     usleep(CONTROL_DELAY);
   }
   return NULL;
 }
 
 int main(void) {
   signal(SIGINT, handle_sigint);
   pthread_mutex_init(&sensor_lock, NULL);
   pthread_mutex_init(&sonar_lock, NULL);
 
   gpio_map = pca9685_init();
   if (!gpio_map) {
     fprintf(stderr, "PCA9685 init failed\n");
     return 1;
   }
 
   for (int i = 0; i < FRONT_COUNT; i++) set_input(gpio_map, FRONT_PINS[i]);
   set_output(gpio_map, TRIG_PIN);
   set_input(gpio_map, ECHO_PIN);
   // Turn on lead
   set_output(gpio_map, 14);
   write_pin(gpio_map, 14, 1);
   
   pca9685_set_pwm_freq(100);
 
   i2c_color = duplicate_path();
   i2c_color.address = 0x29;
   i2c = &i2c_color;
 
   if (rgb_init(i2c) < 0) printf("Color sensor init failed\n");
   set_timing(i2c, COLOR_SENSE_SPEED);
   set_gain(i2c, GAIN_16X);
 
   pthread_t t1, t2, t3;
   pthread_create(&t1, NULL, sensor_thread, NULL);
   pthread_create(&t2, NULL, sonar_thread, NULL);
   pthread_create(&t3, NULL, control_thread, NULL);
 
   pthread_join(t1, NULL);
   pthread_join(t2, NULL);
   pthread_join(t3, NULL);
 
   stop_motors();
   motor_exit();
   hardware_i2c_clean(i2c);
   printf("Exited cleanly.\n");
   return 0;
 }
 