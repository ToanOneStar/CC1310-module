/**
 ******************************************************************************
 * @file    mpu6050_angles.c
 * @brief   MPU6050 angle calculation module implementation
 * @author  Clean Code Implementation
 * @date    2025-11-28
 ******************************************************************************
 */

#include "mpu6050_angles.h"
#include <stdio.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/
#define ANGLE_STRING_MAX_LENGTH 64U

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Calculate 3-axis angles from MPU6050 sensor data
 */
void MPU6050_CalculateAngles(const MPU6050_t* mpu_data, MPU6050_Angles_t* angles) {
    /* Validate input parameters */
    if ((mpu_data == NULL) || (angles == NULL)) {
        return;
    }

    /*
   * Roll (X-axis rotation): Use Kalman-filtered angle
   * This provides stable roll angle with reduced noise
   */
    angles->roll = (float)mpu_data->KalmanAngleX;

    /*
   * Pitch (Y-axis rotation): Use Kalman-filtered angle
   * This provides stable pitch angle with reduced noise
   */
    angles->pitch = (float)mpu_data->KalmanAngleY;

    /*
   * Yaw (Z-axis rotation): Use gyroscope Z-axis data
   * Note: Yaw requires integration and may drift over time
   * For accurate yaw, a magnetometer (like HMC5883L) is recommended
   */
    angles->yaw = (float)mpu_data->Gz;
}

/**
 * @brief Format angle value to string for UART transmission
 */
uint16_t MPU6050_FormatAngleString(const char* angle_name, float angle_value, char* buffer, uint16_t buffer_size) {
    uint16_t length = 0U;
    int16_t integer_part;
    uint16_t decimal_part;
    float abs_value;

    /* Validate input parameters */
    if ((angle_name == NULL) || (buffer == NULL) || (buffer_size == 0U)) {
        return 0U;
    }

    /* Get absolute value for decimal calculation */
    abs_value = angle_value;
    if (abs_value < 0.0f) {
        abs_value = -abs_value;
    }

    /* Convert float to integer and decimal parts */
    integer_part = (int16_t)angle_value;
    decimal_part = (uint16_t)((abs_value - (float)((int16_t)abs_value)) * 100.0f);

    /* Format the string with angle name and value */
    length = (uint16_t)snprintf(buffer, buffer_size, "%s: %d.%02u degrees\r\n", angle_name, integer_part, decimal_part);

    /* Ensure null termination */
    if (length >= buffer_size) {
        buffer[buffer_size - 1U] = '\0';
        length = buffer_size - 1U;
    }

    return length;
}
