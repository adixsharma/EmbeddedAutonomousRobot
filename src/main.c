/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: main.c
 *
 * Description::
 *   Three‑threaded PID line follower (5 sensors) + ultrasonic
 *   obstacle detection (HC‑SR04).  Sonar runs in its own thread
 *   so it can wait for echo without slowing the PID loop.
 **************************************************************/
#include "gpio_library/core/i2c_access.h"
#include "gpio_library/chips/pca9685.h"
#include "gpio_library/core/pins.h"
#include "gpio_library/TCS34725/tcs_controller.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <math.h>

/* ───── parameters you’ll most likely tune ───── */
#define CLEAR_MS 1100 /* ms straight after wall‑lock          */
#define CLEAR_L 55.0  /* left duty in that segment            */
#define CLEAR_R 65.0  /* right duty                           */

/* ───── tape sensors (BCM) ───── */
#define N_TAPE 5
static const int TAPE_PIN[N_TAPE] = {26, 19, 13, 6, 5};

/* ───── PCA9685 channels ───── */
#define PWMA 0 /* L‑PWM  */
#define AIN1 1 /* L fwd  */
#define AIN2 2 /* L rev  */
#define BIN1 3 /* R fwd  */
#define BIN2 4 /* R rev  */
#define PWMB 5 /* R‑PWM  */
#define HAT_ADDR 0x40

/* ───── ultrasonic pins ───── */
#define FRONT_TRIG 23
#define FRONT_ECHO 24
#define SIDE_TRIG 20
#define SIDE_ECHO 21

/* ───── constants ───── */
#define CM_PER_NSEC 0.000034
#define GUARD_NS 60000000LL /* 60 ms echo watchdog */

#define KP_LINE 20.0
#define KI_LINE 0.2
#define KD_LINE 1.5
#define BASE_PWM 80.0
#define STEER_CLAMP 40.0

#define KP_WALL 6.0

#define DETECT_CM 30.0
#define MIN_CM 2.0
#define WALL_LOCK_MAX_CM 60.0
#define LOST_WALL_CM 10.0

#define PIVOT_OUT 35.0
#define PIVOT_IN 20.0
#define GENTLE_OUT (PIVOT_OUT / 2)
#define GENTLE_IN (PIVOT_IN / 2)

/* ───── thread cadence ───── */
#define US_TAPE 5000
#define US_SONAR 50000
#define US_CTRL 10000

// Color
#define COLOR_SENSE_SPEED 4

/* ───── globals ───── */
static volatile unsigned int *gpio = NULL;
static volatile int tape_raw[N_TAPE] = {0};
static volatile double dF_cm = 1e9, dS_cm = 1e9;
static pthread_mutex_t m_tape = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t m_sonar = PTHREAD_MUTEX_INITIALIZER;
static volatile int running = 1;

i2c_data i2c_color;
i2c_data *i2c;

/* ───── utility ───── */
static inline long long nowns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* ───── sonar helper ───── */
static double sonar_cm(int trig, int echo)
{
    write_pin(gpio, trig, 0);
    usleep(5);
    write_pin(gpio, trig, 1);
    usleep(10);
    write_pin(gpio, trig, 0);

    long long start = nowns();
    while (!read_pin(gpio, echo) && running)
    {
        if (nowns() - start > GUARD_NS)
            return NAN;
    }
    long long t0 = nowns();
    while (read_pin(gpio, echo) && running)
    {
        if (nowns() - t0 > GUARD_NS)
            return NAN;
    }
    long long t1 = nowns();

    return ((t1 - t0) * CM_PER_NSEC) / 2.0;
}

/* ───── simple PID ───── */
typedef struct
{
    double P, I, D, i, prev;
} PID;
static void PID_init(PID *p, double P, double I, double D)
{
    p->P = P;
    p->I = I;
    p->D = D;
    p->i = 0;
    p->prev = 0;
}
static double PID_step(PID *p, double err, double dt)
{
    p->i += err * dt;
    double der = (err - p->prev) / dt;
    p->prev = err;
    return p->P * err + p->I * p->i + p->D * der;
}

/* ───── tape centroid error ───── */
static double tape_error(void)
{
    static const int W[N_TAPE] = {-8, -2, 0, 2, 8};
    int sum = 0, cnt = 0;
    pthread_mutex_lock(&m_tape);
    for (int i = 0; i < N_TAPE; i++)
    {
        sum += tape_raw[i] * W[i];
        cnt += tape_raw[i];
    }
    pthread_mutex_unlock(&m_tape);
    return cnt ? (double)sum / cnt : 0.0;
}

/* ───── motor helpers ───── */
static inline void Lf(void)
{
    pca9685_set_level(AIN1, 1);
    pca9685_set_level(AIN2, 0);
}
static inline void Lr(void)
{
    pca9685_set_level(AIN1, 0);
    pca9685_set_level(AIN2, 1);
}
static inline void Rf(void)
{
    pca9685_set_level(BIN1, 1);
    pca9685_set_level(BIN2, 0);
}
static inline void Rr(void)
{
    pca9685_set_level(BIN1, 0);
    pca9685_set_level(BIN2, 1);
}
static void stop(void)
{
    i2c_set_target_address(HAT_ADDR);
    Lf();
    Rf();
    pca9685_set_pwm_dutycycle(PWMA, 0);
    pca9685_set_pwm_dutycycle(PWMB, 0);
}

/* ───── sensor threads ───── */
static void *thTape(void *arg)
{
    (void)arg;
    while (running)
    {
        pthread_mutex_lock(&m_tape);
        for (int i = 0; i < N_TAPE; i++)
            tape_raw[i] = read_pin(gpio, TAPE_PIN[i]) ? 1 : 0;
        pthread_mutex_unlock(&m_tape);
        usleep(US_TAPE);
    }
    return NULL;
}
static void *thSonar(void *arg)
{
    (void)arg;
    while (running)
    {
        double f = sonar_cm(FRONT_TRIG, FRONT_ECHO);
        double s = sonar_cm(SIDE_TRIG, SIDE_ECHO);
        pthread_mutex_lock(&m_sonar);
        if (!isnan(f))
            dF_cm = f;
        if (!isnan(s))
            dS_cm = s;
        pthread_mutex_unlock(&m_sonar);
        usleep(US_SONAR);
    }
    return NULL;
}

int dominant_color(uint16_t r, uint16_t g, uint16_t b)
{
    printf("Color: %i, %i, %i\n", r, g, b);
    if (r < 40 && g < 40 && b < 40)
        return 0;
    if (r >= 3 * g && r >= 3 * b)
        return 1;
    if (b >= 3 * r && b >= 3 * g)
        return 2;
    if (g >= 3 * r && g >= 3 * b)
        return 3;
    return 0;
}

/* ───── state machine ───── */
typedef enum
{
    LINE_MODE,
    OBJECT_MODE
} MODE;
typedef enum
{
    PIVOT_RIGHT,
    DRIVE_ALONG,
    DRIVE_PID,
    WALL_ARC,
    PIVOT_RIGHT_ALIGN
} STEP;

static void *thCtrl(void *arg)
{
    (void)arg;

    PID pid_line;
    PID_init(&pid_line, KP_LINE, KI_LINE, KD_LINE);

    MODE mode = LINE_MODE;
    STEP step = PIVOT_RIGHT;

    double wall_ref = 20.0, last_good_ds = 1e9;
    long long clear_start_ns = 0, align_start_ns = 0;

    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);

    while (running)
    {
        /* --- copy shared --- */
        int t[N_TAPE];
        double dF, dS;
        pthread_mutex_lock(&m_tape);
        for (int i = 0; i < N_TAPE; i++)
            t[i] = tape_raw[i];
        pthread_mutex_unlock(&m_tape);

        pthread_mutex_lock(&m_sonar);
        dF = dF_cm;
        dS = dS_cm;
        pthread_mutex_unlock(&m_sonar);

        /* --- dt --- */
        struct timespec tn;
        clock_gettime(CLOCK_MONOTONIC, &tn);
        double dt = (tn.tv_sec - tp.tv_sec) + (tn.tv_nsec - tp.tv_nsec) / 1e9;
        tp = tn;
        if (dt <= 0)
            dt = 1e-3;

        rgb_color c = sense_color(i2c);

        int color_id = 0;
        color_id = dominant_color(c.red, c.green, c.blue);

        // For testing purposes, replace with button logic
        const char *label = "Unknown";
        if (color_id == 1)
            label = "Red";
        else if (color_id == 2)
            label = "Blue";
        else if (color_id == 3)
            label = "Green";

        printf("COLOR=%s\n", label);

        /* ============  LINE MODE  ============ */
        if (mode == LINE_MODE)
        {
            if (dF > MIN_CM && dF < DETECT_CM)
            {
                stop();
                mode = OBJECT_MODE;
                step = PIVOT_RIGHT;
                printf(">> Obstacle %.1f cm — OBJECT_MODE\n", dF);
                continue;
            }

            double err = tape_error();
            if ((!t[0] && !t[1] && !t[2] && !t[3] && !t[4]) || (t[0] && t[1] && t[2] && t[3] && t[4]))
                err = pid_line.prev;

            double steer = PID_step(&pid_line, err, dt);
            if (steer > STEER_CLAMP)
                steer = STEER_CLAMP;
            if (steer < -STEER_CLAMP)
                steer = -STEER_CLAMP;

            double L = BASE_PWM - steer;
            double R = BASE_PWM + steer;
            if (L < 20)
            {
                L = 20;
            }
            if (L > 100)
            {
                L = 100;
            }
            if (R < 20)
            {
                R = 20;
            }
            if (R > 100)
            {
                R = 100;
            }

            i2c_set_target_address(HAT_ADDR);
            Lf();
            Rf();
            pca9685_set_pwm_dutycycle(PWMA, (int)L);
            pca9685_set_pwm_dutycycle(PWMB, (int)R);
            continue;
        }

        /* ============  OBJECT MODE  ============ */
        switch (step)
        {
            /* A) pivot right until wall detected */
        case PIVOT_RIGHT:
            i2c_set_target_address(HAT_ADDR);
            Lr();
            Rf();
            pca9685_set_pwm_dutycycle(PWMA, PIVOT_IN);
            pca9685_set_pwm_dutycycle(PWMB, PIVOT_OUT);

            if (dS > MIN_CM && dS < WALL_LOCK_MAX_CM)
            {
                wall_ref = last_good_ds = dS;
                clear_start_ns = nowns();
                step = DRIVE_ALONG;
                printf("   Wall locked %.1f cm — DRIVE_ALONG\n", wall_ref);
            }
            break;

            /* B) straight segment to clear nose */
        case DRIVE_ALONG:
            int tapecnt = 0;
            for (int i = 0; i < N_TAPE; i++)
                tapecnt += t[i];
            if (tapecnt >= 2)
            {
                step = PIVOT_RIGHT_ALIGN;
                align_start_ns = nowns();
                printf("   tape (%d sensors) — PIVOT_RIGHT_ALIGN\n", tapecnt);
            }

            i2c_set_target_address(HAT_ADDR);
            Lf();
            Rf();
            pca9685_set_pwm_dutycycle(PWMA, (int)CLEAR_L);
            pca9685_set_pwm_dutycycle(PWMB, (int)CLEAR_R);

            if (nowns() - clear_start_ns > CLEAR_MS * 1000000LL)
            {
                step = DRIVE_PID;
                printf("   straight done — DRIVE_PID\n");
            }

            break;

            /* C) forward with proportional gap control */
        case DRIVE_PID:
        {
            int echo_ok = (!isnan(dS) && dS < 400);
            if (echo_ok)
                last_good_ds = dS;

            double gap = echo_ok ? dS : last_good_ds;
            if (gap > wall_ref + LOST_WALL_CM)
                gap = wall_ref + LOST_WALL_CM;

            double err = wall_ref - gap;
            if (err > 20)
                err = 20;
            if (err < -20)
                err = -20;

            double steer = KP_WALL * err;
            if (steer > STEER_CLAMP)
                steer = STEER_CLAMP;
            if (steer < -STEER_CLAMP)
                steer = -STEER_CLAMP;

            const double BASE_FWD = 50.0;
            double L = BASE_FWD;
            double R = BASE_FWD + steer;
            if (R < 30)
            {
                R = 30;
            }
            if (R > 60)
            {
                R = 60;
            }

            i2c_set_target_address(HAT_ADDR);
            Lf();
            Rf();
            pca9685_set_pwm_dutycycle(PWMA, (int)L);
            pca9685_set_pwm_dutycycle(PWMB, (int)R);

            int tapecnt = 0;
            for (int i = 0; i < N_TAPE; i++)
                tapecnt += t[i];
            if (tapecnt >= 2)
            {
                step = PIVOT_RIGHT_ALIGN;
                align_start_ns = nowns();
                printf("   tape (%d sensors) — PIVOT_RIGHT_ALIGN\n", tapecnt);
            }
            else
            {
                i2c_set_target_address(HAT_ADDR);
                Lf();
                Rf();
                pca9685_set_pwm_dutycycle(PWMA, PIVOT_IN + 15);
                pca9685_set_pwm_dutycycle(PWMB, PIVOT_OUT);

                if (dS > MIN_CM && dS < WALL_LOCK_MAX_CM)
                {
                    wall_ref = last_good_ds = dS;
                    clear_start_ns = nowns();
                    step = WALL_ARC;
                    printf("   ESCAPE: Wall locked %.1f cm — DRIVE_ALONG\n", wall_ref);
                }
            }
            break;
        }

            /* D) wide CCW wall arc (legacy) */
        case WALL_ARC:
        {
            int echo_ok = (!isnan(dS) && dS < 400);
            if (echo_ok)
                last_good_ds = dS;

            double gap = echo_ok ? dS : last_good_ds;
            if (gap > wall_ref + LOST_WALL_CM)
                gap = wall_ref + LOST_WALL_CM;

            double err = wall_ref - gap;
            double steer = KP_WALL * err;
            if (steer > STEER_CLAMP)
                steer = STEER_CLAMP;
            if (steer < -STEER_CLAMP)
                steer = -STEER_CLAMP;

            const double BASE = 50.0;
            double L = BASE - steer;
            double R = BASE + steer;
            if (L < 40)
            {
                L = 40;
            }
            if (L > 80)
            {
                L = 80;
            }
            if (R < 40)
            {
                R = 40;
            }
            if (R > 80)
            {
                R = 80;
            }

            i2c_set_target_address(HAT_ADDR);
            Lf();
            Rf();
            pca9685_set_pwm_dutycycle(PWMA, (int)L * 1.3);
            pca9685_set_pwm_dutycycle(PWMB, (int)R / 1.5);

            int tapecnt = 0;
            for (int i = 0; i < N_TAPE; i++)
                tapecnt += t[i];
            if (tapecnt >= 2)
            {
                step = PIVOT_RIGHT_ALIGN;
                align_start_ns = nowns();
                printf("   tape (%d sensors) — PIVOT_RIGHT_ALIGN\n", tapecnt);
            }

            if (gap > wall_ref + LOST_WALL_CM)
            {
                step = DRIVE_ALONG;
                printf("Going to DRIVE_ALONG from DRIVE_PID | WALL LOST");
            }
            break;
        }

            /* E) gentle pivot right to re‑centre on tape */
        case PIVOT_RIGHT_ALIGN:
        {
            i2c_set_target_address(HAT_ADDR);
            Lr();
            Rf();
            pca9685_set_pwm_dutycycle(PWMA, BASE_PWM - 20);
            pca9685_set_pwm_dutycycle(PWMB, BASE_PWM - 20);

            usleep(300000);
            int centre = t[1] || t[2] || t[3];
            double elapsed = (nowns() - align_start_ns) / 1e9;
            if (centre || elapsed > 2.5)
            {
                stop();
                mode = LINE_MODE;
                PID_init(&pid_line, KP_LINE, KI_LINE, KD_LINE);
                printf(">> Back on tape — LINE_MODE\n");
            }
            break;
        }
        } /* step switch */
    } /* while */

    return NULL;
}

/* ───── ctrl‑c ───── */
static void on_sigint(int s)
{
    (void)s;
    running = 0;
}

/* ───── main ───── */
int main(void)
{
    signal(SIGINT, on_sigint);

    gpio = pca9685_init();
    if (!gpio)
    {
        perror("pca9685_init");
        return 1;
    }
    pca9685_set_pwm_freq(100);

    // Turn on lead
    set_output(gpio, 14);
    write_pin(gpio, 14, 1);

    for (int i = 0; i < N_TAPE; i++)
        set_input(gpio, TAPE_PIN[i]);
    set_output(gpio, FRONT_TRIG);
    set_input(gpio, FRONT_ECHO);
    set_output(gpio, SIDE_TRIG);
    set_input(gpio, SIDE_ECHO);

    i2c_color = duplicate_path();
    i2c_color.address = 0x29;
    i2c = &i2c_color;

    if (rgb_init(i2c) < 0)
        printf("Color sensor init failed\n");
    set_timing(i2c, COLOR_SENSE_SPEED);
    set_gain(i2c, GAIN_16X);

    pthread_t t1, t2, t3;
    pthread_create(&t1, NULL, thTape, NULL);
    pthread_create(&t2, NULL, thSonar, NULL);
    pthread_create(&t3, NULL, thCtrl, NULL);

    pthread_join(t1, NULL);
    pthread_join(t2, NULL);
    pthread_join(t3, NULL);

    // Turn on lead
    write_pin(gpio, 14, 0);

    stop();
    motor_exit();
    puts("Exited cleanly.");
    return 0;
}
