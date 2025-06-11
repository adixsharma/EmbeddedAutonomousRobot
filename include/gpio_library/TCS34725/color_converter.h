/**************************************************************
 * Class:: CSC-615-01 Spring 2025
 * Name:: Zachary Howe, Yu-Ming Chen, Aditya Sharma, James Nguyen
 * Student ID:: 923229694, 923313947, 917586584, 922182661
 * Github-Name:: Zhowe1
 * Project:: Car
 *
 * File:: color_converter.h
 *
 * Description::
 * The header file for translating the TCS34725 color sensor data.
 * Contains the define variables for the color_converter.c.
 **************************************************************/
#ifndef COLOR_CONVERTER_H
#define COLOR_CONVERTER_H

#include <stdint.h>

/**
 * @brief Structure holding the conversion results.
 *
 * @note - hex: A string representing the RGB color in hexadecimal format
 *         (e.g., "#RRGGBB").
 * @note - color_name: An approximated name for the color.
 * @note - confidence: A confidence value (between 0.0 and 1.0) indicating how
 *         close the sensor reading is to the reference color.
 */
typedef struct
{
    char hex[8];         // "#RRGGBB" plus the null terminator
    char color_name[20]; // Approximated color name (e.g., "Red", "Blue", etc.)
    float confidence;    // Confidence in the approximation
                         // (1.0 = perfect match; 0.0 = no match)
} color_result;

/**
 * Converts raw sensor values to a human-readable color.
 *
 * The function performs the following steps:
 *   1. Normalizes raw red, green, and blue values using the clear channel.
 *   2. Applies gamma correction via a lookup table (the table is built once).
 *   3. Formats the gamma-corrected values into a hexadecimal string.
 *   4. Approximates a color name by comparing to known reference colors using
 *      Euclidean distance.
 *   5. Computes a confidence value (scaled from 0 to 1). If the shadow registers
 *      do not match, the confidence is set to 0.
 *
 * @param raw_r         Raw red channel value from the sensor.
 * @param raw_g         Raw green channel value from the sensor.
 * @param raw_b         Raw blue channel value from the sensor.
 * @param clear         Clear channel value used to normalize the RGB channels.
 * @param shadow_match  Flag indicating if the shadow registers match
 *                      (nonzero = match; 0 = mismatch).
 *
 * @return A color_result structure containing the hex color string, approximate
 *         color name, and confidence.
 */
color_result convert_color(uint16_t raw_r, uint16_t raw_g, uint16_t raw_b,
                           uint16_t clear, int shadow_match);

#endif // COLOR_CONVERTER_H