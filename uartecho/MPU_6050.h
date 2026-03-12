/*
 * MPU_6050.h
 *
 * Driver MPU6050 cho CC1310 / TI-RTOS.
 * Su dung IIC.h lam tang I2C ha tang.
 *
 * Tinh nang:
 *   - Khoi tao MPU6050 (wake-up, cau hinh accel ±2g, gyro ±250°/s)
 *   - Doc raw 14 byte (accel + temp + gyro) voi 1 burst read
 *   - Tinh goc Roll, Pitch, Yaw dung Complementary Filter
 *
 * Hardware:
 *   DIO6 = SCL
 *   DIO8 = SDA
 *
 * MPU6050 dia chi 7-bit: 0x68 (AD0=GND)
 */

#ifndef MPU_6050_H_
#define MPU_6050_H_

#include <stdint.h>

/*---------------------------------------------------------------------------
 * Ket qua goc
 *---------------------------------------------------------------------------*/
typedef struct {
  float roll;  /* Goc quay quanh truc X, don vi do */
  float pitch; /* Goc quay quanh truc Y, don vi do */
  float yaw;   /* Goc quay quanh truc Z (tich phan gyro), don vi do */
} MPU6050_Angle_t;

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief Khoi tao MPU6050.
 *        Ham nay can duoc goi sau khi IIC_Init() thanh cong.
 * @return 0 neu thanh cong, 1 neu loi (WHO_AM_I sai).
 */
uint8_t MPU6050_Init(void);

/**
 * @brief Doc toan bo du lieu cam bien va tinh goc.
 * @param[out] angles  Con tro cau truc luu ket qua goc.
 * @param[in]  dt      Buoc thoi gian (giay) ke tu lan doc truoc.
 *                     Vi du: 0.01f cho chu ky 10 ms.
 */
void MPU6050_GetAngle(MPU6050_Angle_t *angles, float dt);

/* Debug: WHO_AM_I va raw data cuoi cung */
uint8_t MPU6050_GetWhoAmI(void);
void MPU6050_GetLastRaw(int16_t *ax, int16_t *ay, int16_t *az,
                        int16_t *gx, int16_t *gy, int16_t *gz);

/* Scan I2C bus with WHO_AM_I register read (0x75). Returns number of ACKs. */
int MPU6050_Scan(uint8_t *out_addrs, int max);

/* Debug: read 1 register, returns 0xFF if fail */
uint8_t MPU6050_ReadReg(uint8_t reg);

/*---------------------------------------------------------------------------
 * Debug API
 *---------------------------------------------------------------------------*/

/**
 * @brief Lay so loi I2C dem duoc.
 * @return So lan I2C_Read_Burst that bai.
 */
int MPU6050_GetI2cErrorCount(void);

#endif /* MPU_6050_H_ */
