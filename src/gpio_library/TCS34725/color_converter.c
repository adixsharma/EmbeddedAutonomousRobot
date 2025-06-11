/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: color_converter.c
 *
 * Description::
 * Handling color conversion for the TCS34725 color sensor.
 **************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "gpio_library/TCS34725/color_converter.h"

// Define if using a common anode LED (1: yes, 0: no)
// When using a common anode, the gamma table values are inverted.
#define COMMON_ANODE 0

// Static gamma table and initialization flag
static uint8_t gamma_table[256];
static int gamma_initialized = 0;

/**
 * Initializes the gamma lookup table.
 *
 * For each possible value [0, 255], this calculates:
 *   x = (i / 255.0)^2.2 * 255.0
 * If using common anode LEDs, it inverts the value (255 - x).
 */
static void init_gamma_table(void)
{
    for (int i = 0; i < 256; i++)
    {
        float x = i / 255.0f;
        x = powf(x, 2.2f);
        x *= 255.0f;
        if (COMMON_ANODE)
        {
            gamma_table[i] = 255 - (uint8_t)(x + 0.5f);
        }
        else
        {
            gamma_table[i] = (uint8_t)(x + 0.5f);
        }
    }
    gamma_initialized = 1;
}

/**
 * Convert raw RGB sensor values to a human-readable color.
 *
 * This function first normalizes each channel by dividing by the clear value
 * and scaling to 255.
 * It then applies gamma correction using a precomputed lookup table.
 * Finally, it forms a hex string and approximates the color name (with confidence)
 * by comparing the corrected values to a set of predetermined reference colors.
 */
color_result convert_color(uint16_t raw_r, uint16_t raw_g, uint16_t raw_b,
                           uint16_t clear, int shadow_match)
{
    color_result result;

    // Initialize gamma table once
    if (!gamma_initialized)
    {
        init_gamma_table();
    }

    // Normalize raw values using clear channel.
    // The sensor's clear value represents the overall light intensity.
    float r_norm = (clear > 0) ? ((float)raw_r / clear) * 255.0f : 0;
    float g_norm = (clear > 0) ? ((float)raw_g / clear) * 255.0f : 0;
    float b_norm = (clear > 0) ? ((float)raw_b / clear) * 255.0f : 0;

    // Clamp normalized values to range 0-255.
    if (r_norm > 255)
        r_norm = 255;
    if (g_norm > 255)
        g_norm = 255;
    if (b_norm > 255)
        b_norm = 255;

    // Apply gamma correction using the lookup table.
    uint8_t corrected_r = gamma_table[(uint8_t)r_norm];
    uint8_t corrected_g = gamma_table[(uint8_t)g_norm];
    uint8_t corrected_b = gamma_table[(uint8_t)b_norm];

    printf("RGB values:\n");
    printf("Red: %i\n", corrected_r);
    printf("Green: %i\n", corrected_g);
    printf("Blue: %i\n", corrected_b);

    // Format the gamma-corrected values into a hex string (e.g., "#RRGGBB").
    sprintf(result.hex, "#%02X%02X%02X", corrected_r, corrected_g, corrected_b);

    // Define a set of reference colors to approximate the sensed color.
    typedef struct
    {
        const char *name;
        uint8_t r, g, b;
    } color_ref;

    color_ref colors[] = {
        {"Black", 0, 0, 0},
        {"White", 255, 255, 255},

        // Primary Colors
        {"Red", 255, 0, 0},
        {"Green", 0, 255, 0},
        {"Blue", 0, 0, 255},

        // Secondary Colors
        {"Cyan", 0, 255, 255},
        {"Magenta", 255, 0, 255},
        {"Yellow", 255, 255, 0},

        // Tertiary Colors
        {"Orange", 255, 165, 0},       // Between Red and Yellow
        {"Chartreuse", 127, 255, 0},   // Between Yellow and Green
        {"Spring Green", 0, 255, 127}, // Between Green and Cyan
        {"Azure", 0, 127, 255},        // Between Cyan and Blue
        {"Violet", 127, 0, 255},       // Between Blue and Magenta
        {"Rose", 255, 0, 127}          // Between Magenta and Red
    };

    int num_colors = sizeof(colors) / sizeof(colors[0]);

    // Compare the gamma-corrected RGB values to each reference color using
    // Euclidean distance.
    float min_distance = 1e9;
    int closest_index = 0;
    for (int i = 0; i < num_colors; i++)
    {
        float dr = (float)corrected_r - colors[i].r;
        float dg = (float)corrected_g - colors[i].g;
        float db = (float)corrected_b - colors[i].b;
        float distance = sqrtf(dr * dr + dg * dg + db * db);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_index = i;
        }
    }

    // Copy the best-matched color name.
    strncpy(result.color_name, colors[closest_index].name, sizeof(result.color_name));
    result.color_name[sizeof(result.color_name) - 1] = '\0';

    // Compute a confidence value from the normalized Euclidean distance.
    // The maximum possible distance in RGB space is between (0,0,0) and (255,255,255).
    float max_distance = sqrtf(255.0f * 255.0f * 3); // ≈441.67
    float confidence = 1.0f - (min_distance / max_distance);
    if (confidence < 0.0f)
    {
        confidence = 0.0f;
    }

    // If the sensor's shadow registers do not match, set confidence to 0.
    if (!shadow_match)
    {
        confidence = 0.0f;
    }
    result.confidence = confidence;

    return result;
}