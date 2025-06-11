/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: WASD_Control.c
 *
 * Description::
 * Code for WASD control of the car. Used for testing purposes.
 **************************************************************/
#include "gpio_library/core/i2c_access.h"
#include "gpio_library/chips/pca9685.h"
#include "gpio_library/core/pins.h"
#include <stdio.h>
#include <fcntl.h>
#include <signal.h>
#include <setjmp.h>
#include <unistd.h>
#include <termios.h>

#define AIN1 1
#define AIN2 2
#define BIN1 3
#define BIN2 4

#define PWMA 0
#define PWMB 5

jmp_buf close_buf;
struct termios orig_termios;

void reset_terminal_mode()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    tcgetattr(STDIN_FILENO, &orig_termios);
    new_termios = orig_termios;

    new_termios.c_lflag &= ~(ICANON | ECHO); // disable canonical mode and echo
    new_termios.c_cc[VMIN] = 0;
    new_termios.c_cc[VTIME] = 1;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

int kbhit()
{
    char ch;
    return read(STDIN_FILENO, &ch, 1) == 1 ? ch : 0;
}

void stop_motors()
{

    i2c_set_target_address(0x52);
    pca9685_set_pwm_dutycycle(PWMA, 0);
    pca9685_set_pwm_dutycycle(PWMB, 0);
    pca9685_set_level(AIN1, 0);
    pca9685_set_level(AIN2, 0);
    pca9685_set_level(BIN1, 0);
    pca9685_set_level(BIN2, 0);

    i2c_set_target_address(0x40);

    pca9685_set_pwm_dutycycle(PWMA, 0);
    pca9685_set_pwm_dutycycle(PWMB, 0);
    pca9685_set_level(AIN1, 0);
    pca9685_set_level(AIN2, 0);
    pca9685_set_level(BIN1, 0);
    pca9685_set_level(BIN2, 0);
}

void signal_handler(int sig)
{
    longjmp(close_buf, sig);
}

int main()
{
    volatile unsigned int *gpio = pca9685_init();

    if (!gpio || set_input(gpio, 24) != 0)
    {
        printf("Initialization failed!\n");
        return -1;
    }

    signal(SIGINT, signal_handler);
    set_conio_terminal_mode();

    if (setjmp(close_buf) == 0)
    {
        pca9685_set_pmw_freq(100);
        printf("WASD motor control started. Press 'q' to quit.\n");

        while (1)
        {
            char c = kbhit();
            if (c == 0)
                continue;

            switch (c)
            {
            case 'w': // forward
                i2c_set_target_address(0x40);
                pca9685_set_level(AIN1, 0);
                pca9685_set_level(AIN2, 1);
                pca9685_set_level(BIN1, 0);
                pca9685_set_level(BIN2, 1);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x52);

                pca9685_set_level(AIN1, 0);
                pca9685_set_level(AIN2, 1);
                pca9685_set_level(BIN1, 0);
                pca9685_set_level(BIN2, 1);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x40);
                break;

            case 's': // backward
                i2c_set_target_address(0x40);
                pca9685_set_level(AIN1, 1);
                pca9685_set_level(AIN2, 0);
                pca9685_set_level(BIN1, 1);
                pca9685_set_level(BIN2, 0);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x52);

                pca9685_set_level(AIN1, 1);
                pca9685_set_level(AIN2, 0);
                pca9685_set_level(BIN1, 1);
                pca9685_set_level(BIN2, 0);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x40);
                break;

            case 'a': // turn left
                i2c_set_target_address(0x40);
                pca9685_set_level(AIN1, 1);
                pca9685_set_level(AIN2, 0);
                pca9685_set_level(BIN1, 0);
                pca9685_set_level(BIN2, 1);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x52);

                pca9685_set_level(AIN1, 1);
                pca9685_set_level(AIN2, 0);
                pca9685_set_level(BIN1, 0);
                pca9685_set_level(BIN2, 1);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x40);
                break;

            case 'd': // turn right
                i2c_set_target_address(0x40);
                pca9685_set_level(AIN1, 0);
                pca9685_set_level(AIN2, 1);
                pca9685_set_level(BIN1, 1);
                pca9685_set_level(BIN2, 0);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x52);

                pca9685_set_level(AIN1, 0);
                pca9685_set_level(AIN2, 1);
                pca9685_set_level(BIN1, 1);
                pca9685_set_level(BIN2, 0);
                pca9685_set_pwm_dutycycle(PWMA, 100);
                pca9685_set_pwm_dutycycle(PWMB, 100);

                i2c_set_target_address(0x40);
                break;

            case ' ': // stop
                stop_motors();
                break;

            case 'q': // quit
                goto quit;
            }
        }
    }

quit:
    reset_terminal_mode();
    stop_motors();
    motor_exit();
    printf("\nMotors stopped. Exiting...\n");
    return 0;
}
