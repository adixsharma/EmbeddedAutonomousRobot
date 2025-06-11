LIBRARY =./src/gpio_library

CC = gcc

CFLAGS = -Wall -Wextra -Werror -Iinclude -g

EXECUTABLES = car

ASSIGNMENT1B_SRC = \
	./src/main.c \
	$(LIBRARY)/core/pins.c \
	$(LIBRARY)/core/timer.c \
	$(LIBRARY)/core/i2c_access.c \
	$(LIBRARY)/chips/pca9685.c \
	$(LIBRARY)/motor_hat/motor_config.c \
	$(LIBRARY)/TCS34725/tcs_controller.c \
	$(LIBRARY)/TCS34725/color_converter.c

all: $(EXECUTABLES)

$(EXECUTABLES): $(ASSIGNMENT1B_SRC)
	$(CC) $(CFLAGS) -o $@ $^ -lm

run: $(EXECUTABLES);
	./$(EXECUTABLES)

clean:
	rm -f $(EXECUTABLES)