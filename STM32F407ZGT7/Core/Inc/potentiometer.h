#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H

#include "main.h"

/*
 * Linear Potentiometer Angle Sensor
 *
 * 10kΩ WH148 linear pot mounted at pendulum pivot
 * Connected to ADC3, 10-bit resolution (0-1023)
 *
 * Wiring:
 *   Pin 1 (left)  → GND
 *   Pin 2 (wiper) → ADC input pin
 *   Pin 3 (right) → 3.3V
 *
 * The pot's rotation range is ~270° (±135° from center)
 * For balance, we only use a small range around center (±40-50°)
 *
 * Advantages over IMU:
 *   - Zero latency (~1µs ADC read vs ~400µs I2C + filter)
 *   - Absolute angle (no drift, no integration)
 *   - No vibration noise (mechanical, not inertial)
 *   - No complementary filter needed
 */

/* ADC handle — declared in main, extern here */
extern ADC_HandleTypeDef hadc3;

/* ── Configuration ── */
#define POT_ADC_MAX      1023       /* 10-bit resolution */
#define POT_CENTER_RAW   512        /* ADC value at vertical (calibrate this!) */
#define POT_RANGE_DEG    270.0f     /* WH148 total rotation range in degrees */
#define POT_RANGE_RAD    (POT_RANGE_DEG * 3.14159265f / 180.0f)  /* ~4.712 rad */

/* Conversion: raw ADC → radians from vertical
 * Each ADC count = POT_RANGE_RAD / POT_ADC_MAX ≈ 0.00461 rad ≈ 0.264°
 */
#define POT_RAD_PER_COUNT (POT_RANGE_RAD / (float)POT_ADC_MAX)

/* ── Read angle from potentiometer ── */
static inline float Pot_Read_Angle(void)
{
    /* Start conversion and wait (blocking, ~1-2µs at 84MHz APB2) */
    HAL_ADC_Start(&hadc3);
    HAL_ADC_PollForConversion(&hadc3, 1);  /* 1ms timeout — conversion takes <5µs */
    uint16_t raw = (uint16_t)HAL_ADC_GetValue(&hadc3);

    /* Convert to radians: 0 at center, positive one way, negative the other */
    float angle = ((float)raw - (float)POT_CENTER_RAW) * POT_RAD_PER_COUNT;

    return angle;
}

/* ── Calibrate center position ──
 * Call once at startup with pendulum held vertical.
 * Returns the raw ADC value to use as POT_CENTER_RAW.
 * Average N samples for accuracy.
 */
static inline uint16_t Pot_Calibrate_Center(uint16_t samples)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < samples; i++) {
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 1);
        sum += HAL_ADC_GetValue(&hadc3);
        HAL_Delay(2);
    }
    return (uint16_t)(sum / samples);
}

#endif /* POTENTIOMETER_H */
