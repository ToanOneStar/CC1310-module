/**
 ******************************************************************************
 * @file    mpu6050_angles.h
 * @brief   MPU6050 angle calculation module header
 * @author  Clean Code Implementation
 * @date    2025-11-28
 ******************************************************************************
 */

#ifndef INC_MPU6050_ANGLES_H_
#define INC_MPU6050_ANGLES_H_

#include <stddef.h>
#include <stdint.h>
#include "mpu6050.h"

/**
 * @brief Structure to hold calculated angles in degrees
 */
typedef struct {
    float roll;  /**< Roll angle (rotation around X-axis) in degrees */
    float pitch; /**< Pitch angle (rotation around Y-axis) in degrees */
    float yaw;   /**< Yaw angle (rotation around Z-axis) in degrees */
} MPU6050_Angles_t;

/**
 * @brief Calculate 3-axis angles from MPU6050 sensor data
 *
 * This function computes roll, pitch, and yaw angles using Kalman-filtered
 * accelerometer and gyroscope data from the MPU6050 sensor.
 *
 * @param[in]  mpu_data  Pointer to MPU6050 data structure
 * @param[out] angles    Pointer to angles structure to store results
 *
 * @note Roll and Pitch use Kalman filter for accuracy
 * @note Yaw calculation requires integration and may drift over time
 */
void MPU6050_CalculateAngles(const MPU6050_t* mpu_data, MPU6050_Angles_t* angles);

/**
 * @brief Format angle value to string for UART transmission
 *
 * @param[in]  angle_name  Name of the angle (e.g., "Roll", "Pitch", "Yaw")
 * @param[in]  angle_value Angle value in degrees
 * @param[out] buffer      Buffer to store formatted string
 * @param[in]  buffer_size Size of the buffer
 *
 * @return Number of characters written to buffer
 */
uint16_t MPU6050_FormatAngleString(const char* angle_name, float angle_value, char* buffer, uint16_t buffer_size);

#endif /* INC_MPU6050_ANGLES_H_ */
