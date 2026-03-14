/*
 * mpu6050.h
 *
 * Ported for CC1310 / TI-RTOS.
 * Uses IIC wrapper for I2C communication.
 *
 * Hardware:
 *   DIO6 = SCL
 *   DIO8 = SDA
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "../../Drivers/inc/IIC.h"

// MPU6050 structure
typedef struct {
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
} MPU6050_t;

// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

uint8_t Kalman_MPU6050_Init(void);

void Kalman_MPU6050_Read_Accel(MPU6050_t* DataStruct);

void Kalman_MPU6050_Read_Gyro(MPU6050_t* DataStruct);

void Kalman_MPU6050_Read_Temp(MPU6050_t* DataStruct);

void Kalman_MPU6050_Read_All(MPU6050_t* DataStruct);

double Kalman_getAngle(Kalman_t* Kalman, double newAngle, double newRate, double dt);

#endif /* INC_MPU6050_H_ */
