/*
 * mpu6050.c
 *
 * Ported for CC1310 / TI-RTOS.
 * Uses IIC wrapper for I2C communication.
 *
 * Kalman filter implementation for angle calculation.
 */

#include "mpu6050.h"
#include <math.h>
#include <stddef.h>
#include "../../Drivers/inc/IIC.h"

#define RAD_TO_DEG        57.295779513082320876798154814105

#define WHO_AM_I_REG      0x75
#define PWR_MGMT_1_REG    0x6B
#define SMPLRT_DIV_REG    0x19
#define ACCEL_CONFIG_REG  0x1C
#define ACCEL_XOUT_H_REG  0x3B
#define TEMP_OUT_H_REG    0x41
#define GYRO_CONFIG_REG   0x1B
#define GYRO_XOUT_H_REG   0x43

/* MPU6050 I2C address (7-bit, AD0=GND) */
#define MPU6050_ADDR      0x68

/* Accelerometer Z axis corrector for ±2g range */
#define ACCEL_Z_CORRECTOR 14418.0

/* Timer for Kalman filter dt calculation */
/* timer was: static uint32_t timer = 0; - removed to avoid unused warning */

/* Kalman filter states */
static Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

static Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

/*---------------------------------------------------------------------------
 * Kalman_MPU6050_Init
 *---------------------------------------------------------------------------*/
uint8_t Kalman_MPU6050_Init(void) {
    uint8_t check;
    uint8_t Data;

    /* Check device ID WHO_AM_I */
    check = I2C_Read_Register(MPU6050_ADDR, WHO_AM_I_REG);

    if (check != 0x68) {
        /* Device not found */
        return 1;
    }

    /* Power management register - wake up sensor */
    Data = 0;
    I2C_Write_Register(MPU6050_ADDR, PWR_MGMT_1_REG, Data);

    /* Set data rate: 1kHz / (1 + 7) = 125 Hz */
    Data = 0x07;
    I2C_Write_Register(MPU6050_ADDR, SMPLRT_DIV_REG, Data);

    /* Set accelerometer configuration: ±2g */
    Data = 0x00;
    I2C_Write_Register(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);

    /* Set gyroscope configuration: ±250 °/s */
    Data = 0x00;
    I2C_Write_Register(MPU6050_ADDR, GYRO_CONFIG_REG, Data);

    /* Reset Kalman filter state - reset the angle estimates */
    KalmanX.angle = 0.0;
    KalmanX.bias = 0.0;
    KalmanY.angle = 0.0;
    KalmanY.bias = 0.0;

    return 0;
}

/*---------------------------------------------------------------------------
 * Kalman_MPU6050_Read_Accel
 *---------------------------------------------------------------------------*/
void Kalman_MPU6050_Read_Accel(MPU6050_t* DataStruct) {
    uint8_t Rec_Data[6];

    if (DataStruct == NULL) {
        return;
    }

    /* Read 6 bytes starting from ACCEL_XOUT_H register */
    if (!I2C_Read_Burst(MPU6050_ADDR, ACCEL_XOUT_H_REG, Rec_Data, 6)) {
        return;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /* Convert RAW values to acceleration in 'g' */
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_Z_CORRECTOR;
}

/*---------------------------------------------------------------------------
 * Kalman_MPU6050_Read_Gyro
 *---------------------------------------------------------------------------*/
void Kalman_MPU6050_Read_Gyro(MPU6050_t* DataStruct) {
    uint8_t Rec_Data[6];

    if (DataStruct == NULL) {
        return;
    }

    /* Read 6 bytes starting from GYRO_XOUT_H register */
    if (!I2C_Read_Burst(MPU6050_ADDR, GYRO_XOUT_H_REG, Rec_Data, 6)) {
        return;
    }

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /* Convert RAW values to dps (°/s) */
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

/*---------------------------------------------------------------------------
 * Kalman_MPU6050_Read_Temp
 *---------------------------------------------------------------------------*/
void Kalman_MPU6050_Read_Temp(MPU6050_t* DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    if (DataStruct == NULL) {
        return;
    }

    /* Read 2 bytes starting from TEMP_OUT_H register */
    if (!I2C_Read_Burst(MPU6050_ADDR, TEMP_OUT_H_REG, Rec_Data, 2)) {
        return;
    }

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / 340.0f + 36.53f);
}

/*---------------------------------------------------------------------------
 * Kalman_MPU6050_Read_All
 *---------------------------------------------------------------------------*/
void Kalman_MPU6050_Read_All(MPU6050_t* DataStruct) {
    uint8_t Rec_Data[14];
    int16_t temp;

    if (DataStruct == NULL) {
        return;
    }

    /* Read 14 bytes starting from ACCEL_XOUT_H register */
    if (!I2C_Read_Burst(MPU6050_ADDR, ACCEL_XOUT_H_REG, Rec_Data, 14)) {
        return;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    /* Convert to physical units */
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / ACCEL_Z_CORRECTOR;
    DataStruct->Temperature = (float)(temp / 340.0f + 36.53f);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    /* Calculate Kalman angles - using simple time increment */
    /* Note: For CC1310, you need to provide actual dt from your loop */
    double dt = 0.01; /* Default 10ms */
    double roll;
    double roll_sqrt = sqrt((double)DataStruct->Accel_X_RAW * (double)DataStruct->Accel_X_RAW
                            + (double)DataStruct->Accel_Z_RAW * (double)DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0) {
        roll = atan((double)DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    } else {
        roll = 0.0;
    }
    double pitch = atan2(-(double)DataStruct->Accel_X_RAW, (double)DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90)) {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    } else {
        DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    }

    if (fabs(DataStruct->KalmanAngleY) > 90) {
        DataStruct->Gx = -DataStruct->Gx;
    }
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
}

/*---------------------------------------------------------------------------
 * Kalman_getAngle
 *---------------------------------------------------------------------------*/
double Kalman_getAngle(Kalman_t* Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}
