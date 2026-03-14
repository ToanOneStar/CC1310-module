/*
 * MPU_6050.c
 *
 * Driver MPU6050 cho CC1310 / TI-RTOS.
 *
 * Thuat toan:
 *   - Complementary Filter cho Roll va Pitch:
 *       angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
 *       alpha = 0.98 (tin gyro 98%, tin accel 2%)
 *   - Yaw: tich phan thuan tuy gyro Z (khoong co magnetometer => se drift)
 *
 * Thanh ghi MPU6050 quan trong:
 *   0x75 WHO_AM_I        -> 0x68
 *   0x6B PWR_MGMT_1      -> 0x00 (wake up)
 *   0x19 SMPLRT_DIV      -> 0x07 (1 kHz / 8 = 125 Hz data rate)
 *   0x1C ACCEL_CONFIG    -> 0x00 (±2g, 16384 LSB/g)
 *   0x1B GYRO_CONFIG     -> 0x00 (±250°/s, 131 LSB/°/s)
 *   0x3B ACCEL_XOUT_H    -> dau burst read 14 byte
 */

#include "../inc/MPU_6050.h"

#include <stdbool.h>
#include <ti/drivers/I2C.h>
#include "../../Config/Board.h"

#include <math.h>
#include <string.h>
#include <unistd.h>
#include <xdc/runtime/System.h>

/*---------------------------------------------------------------------------
 * Private constants
 *---------------------------------------------------------------------------*/
/* Dia chi 7-bit: 0x68 (AD0=GND) hoac 0x69 (AD0=VCC) */
#define MPU6050_ADDR_0    0x68u
#define MPU6050_ADDR_1    0x69u

/* Thanh ghi */
#define REG_WHO_AM_I      0x75u
#define REG_PWR_MGMT_1    0x6Bu
#define REG_SMPLRT_DIV    0x19u
#define REG_ACCEL_CONFIG  0x1Cu
#define REG_GYRO_CONFIG   0x1Bu
#define REG_ACCEL_XOUT_H  0x3Bu /* Bat dau burst 14 byte */

/* He so scale */
#define ACCEL_SCALE_2G    16384.0f /* LSB/g  khi FS_SEL=0 */
#define GYRO_SCALE_250DPS 131.0f   /* LSB/°/s khi FS_SEL=0 */

/* Complementary filter: alpha gần 1 => tin gyro nhieu hon */
#define COMP_ALPHA        0.99f

/* Output Low-Pass Filter: gia tri 0.0-1.0, nho hon = muot hon */
#define OUTPUT_LPF_ALPHA  0.05f

/* Chuyen doi radian sang do */
#define RAD2DEG           57.29577951f

/*---------------------------------------------------------------------------
 * Private state
 *---------------------------------------------------------------------------*/
static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;

/* Low-pass filtered angles (smoothed output) */
static float s_roll_filtered = 0.0f;
static float s_pitch_filtered = 0.0f;
static float s_yaw_filtered = 0.0f;

static int s_first_sample = 1;    /* Flag khoi tao lan dau tien */
static int s_i2c_error_count = 0; /* Dem so loi I2C */

/* Calibration data */
static MPU6050_Calibration_t s_cal = {
    .ax_offset = 0, .ay_offset = 0, .az_offset = 0, .gx_offset = 0, .gy_offset = 0, .gz_offset = 0};
static bool s_calibrated = false;

static I2C_Handle s_i2c = NULL;
static uint8_t s_addr = MPU6050_ADDR_0;
static uint8_t s_whoami = 0xFFu;
static int16_t s_ax_raw = 0;
static int16_t s_ay_raw = 0;
static int16_t s_az_raw = 0;
static int16_t s_gx_raw = 0;
static int16_t s_gy_raw = 0;
static int16_t s_gz_raw = 0;

static bool mpu_i2c_open(void) {
    if (s_i2c != NULL) {
        return true;
    }

    I2C_init();
    I2C_Params params;
    I2C_Params_init(&params);
    params.bitRate = I2C_100kHz;

    s_i2c = I2C_open(Board_I2C0, &params);
    return (s_i2c != NULL);
}

static bool mpu_i2c_write_reg(uint8_t reg, uint8_t data) {
    uint8_t txBuf[2] = {reg, data};
    I2C_Transaction txn;
    memset(&txn, 0, sizeof(txn));
    txn.slaveAddress = s_addr;
    txn.writeBuf = txBuf;
    txn.writeCount = 2u;
    txn.readBuf = NULL;
    txn.readCount = 0u;
    return I2C_transfer(s_i2c, &txn);
}

static bool mpu_i2c_read_reg(uint8_t reg, uint8_t* out) {
    if (out == NULL) {
        return false;
    }
    I2C_Transaction txn;
    memset(&txn, 0, sizeof(txn));
    txn.slaveAddress = s_addr;
    txn.writeBuf = &reg;
    txn.writeCount = 1u;
    txn.readBuf = out;
    txn.readCount = 1u;
    return I2C_transfer(s_i2c, &txn);
}

static bool mpu_i2c_read_burst(uint8_t reg, uint8_t* buf, uint8_t len) {
    if ((buf == NULL) || (len == 0u)) {
        return false;
    }
    I2C_Transaction txn;
    memset(&txn, 0, sizeof(txn));
    txn.slaveAddress = s_addr;
    txn.writeBuf = &reg;
    txn.writeCount = 1u;
    txn.readBuf = buf;
    txn.readCount = len;
    return I2C_transfer(s_i2c, &txn);
}

int MPU6050_Scan(uint8_t* out_addrs, int max) {
    int count = 0;
    uint8_t who = 0xFFu;
    uint8_t reg = REG_WHO_AM_I;

    if (!mpu_i2c_open()) {
        return -1;
    }

    uint8_t addr;
    for (addr = 0x03u; addr <= 0x77u; addr++) {
        I2C_Transaction txn;
        memset(&txn, 0, sizeof(txn));
        txn.slaveAddress = addr;
        txn.writeBuf = &reg;
        txn.writeCount = 1u;
        txn.readBuf = &who;
        txn.readCount = 1u;
        if (I2C_transfer(s_i2c, &txn)) {
            if ((out_addrs != NULL) && (count < max)) {
                out_addrs[count] = addr;
            }
            count++;
        }
    }

    return count;
}

uint8_t MPU6050_ReadReg(uint8_t reg) {
    uint8_t val = 0xFFu;
    if (!mpu_i2c_open()) {
        return 0xFFu;
    }
    (void)mpu_i2c_read_reg(reg, &val);
    return val;
}

static bool mpu_is_valid_id(uint8_t who) {
    return (who == 0x68u) || (who == 0x70u) || (who == 0x71u);
}

static bool mpu_detect_addr(uint8_t* who) {
    uint8_t w = 0xFFu;
    s_addr = MPU6050_ADDR_0;
    (void)mpu_i2c_read_reg(REG_WHO_AM_I, &w);
    if (mpu_is_valid_id(w)) {
        if (who) {
            *who = w;
        }
        return true;
    }

    s_addr = MPU6050_ADDR_1;
    w = 0xFFu;
    (void)mpu_i2c_read_reg(REG_WHO_AM_I, &w);
    if (mpu_is_valid_id(w)) {
        if (who) {
            *who = w;
        }
        return true;
    }

    if (who) {
        *who = w;
    }
    return false;
}

/*---------------------------------------------------------------------------
 * MPU6050_Init
 *---------------------------------------------------------------------------*/
uint8_t MPU6050_Init(void) {
    uint8_t who;
    bool ok;

    if (!mpu_i2c_open()) {
        return 3u; /* I2C open fail */
    }

    /* Delay truoc khi doc (cho MPU6050 khoi dong xong) */
    usleep(50000); /* 50ms delay */

    /* Kiem tra WHO_AM_I */
    who = 0xFFu;
    (void)mpu_detect_addr(&who);
    if (who != 0x68u) {
        /* Thu lai sau khi reset */
        (void)mpu_i2c_write_reg(REG_PWR_MGMT_1, 0x80u); /* Reset */
        usleep(10000);                                  /* 10ms */
        who = 0xFFu;
        (void)mpu_detect_addr(&who);
    }
    if (!mpu_is_valid_id(who)) {
        return 1u;
    }
    s_whoami = who;

    /* Wake up: xoa PWR_MGMT_1 bit SLEEP */
    ok = mpu_i2c_write_reg(REG_PWR_MGMT_1, 0x00u);
    usleep(10000); /* 10ms delay sau khi wake up */

    /* Data rate = gyro output rate / (1 + SMPLRT_DIV)
   * 0x07 => 8000/(1+7) = 1000 Hz (khi DLPF tat) */
    ok = mpu_i2c_write_reg(REG_SMPLRT_DIV, 0x07u) && ok;
    usleep(5000); /* 5ms delay */

    /* Accel full-scale: ±2g */
    ok = mpu_i2c_write_reg(REG_ACCEL_CONFIG, 0x00u) && ok;
    usleep(5000); /* 5ms delay */

    /* Gyro full-scale: ±250 °/s */
    ok = mpu_i2c_write_reg(REG_GYRO_CONFIG, 0x00u) && ok;
    usleep(5000); /* 5ms delay */

    if (!ok) {
        return 2u;
    }

    /* Auto-calibrate at level position (takes ~1 second with 500 samples) */
    /* IMPORTANT: Place MPU6050 at level position before calling MPU6050_Init() */
    if (!MPU6050_AutoCalibrate(500u)) {
        return 4u; /* Calibration failed */
    }

    /* Reset trang thai filter */
    s_roll = 0.0f;
    s_pitch = 0.0f;
    s_yaw = 0.0f;
    s_roll_filtered = 0.0f;
    s_pitch_filtered = 0.0f;
    s_yaw_filtered = 0.0f;
    s_first_sample = 1;

    return 0u;
}

/*---------------------------------------------------------------------------
 * MPU6050_GetAngle
 *---------------------------------------------------------------------------*/
void MPU6050_GetAngle(MPU6050_Angle_t* angles, float dt) {
    uint8_t raw[14];
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    float ax, ay, az;
    float gx, gy, gz;
    float roll_acc, pitch_acc;

    if (angles == NULL) {
        return;
    }

    /* Doc 14 byte: ACCEL(6) + TEMP(2) + GYRO(6) */
    if (!mpu_i2c_read_burst(REG_ACCEL_XOUT_H, raw, 14u)) {
        /* Giu gia tri cu neu loi */
        s_i2c_error_count++;
        angles->roll = s_roll;
        angles->pitch = s_pitch;
        angles->yaw = s_yaw;
        return;
    }

    /* Giai ma raw -> int16 (big-endian) */
    ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
    ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
    az_raw = (int16_t)((raw[4] << 8) | raw[5]);
    /* raw[6..7] = TEMP - bo qua */
    gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
    gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
    gz_raw = (int16_t)((raw[12] << 8) | raw[13]);

    /* Apply calibration offset */
    if (s_calibrated) {
        ax_raw -= s_cal.ax_offset;
        ay_raw -= s_cal.ay_offset;
        az_raw -= s_cal.az_offset;
        gx_raw -= s_cal.gx_offset;
        gy_raw -= s_cal.gy_offset;
        gz_raw -= s_cal.gz_offset;
    }

    s_ax_raw = ax_raw;
    s_ay_raw = ay_raw;
    s_az_raw = az_raw;
    s_gx_raw = gx_raw;
    s_gy_raw = gy_raw;
    s_gz_raw = gz_raw;

    /* Chuyen sang don vi vat ly */
    ax = (float)ax_raw / ACCEL_SCALE_2G; /* g */
    ay = (float)ay_raw / ACCEL_SCALE_2G;
    az = (float)az_raw / ACCEL_SCALE_2G;
    gx = (float)gx_raw / GYRO_SCALE_250DPS; /* °/s */
    gy = (float)gy_raw / GYRO_SCALE_250DPS;
    gz = (float)gz_raw / GYRO_SCALE_250DPS;

    /* Tinh goc tu accelerometer
   * roll_acc : -180 .. +180 do
   * pitch_acc: -90  .. +90  do */
    roll_acc = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD2DEG;
    pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;

    if (s_first_sample) {
        /* Lan dau: khoi tao bang gia tri accelerometer */
        s_roll = roll_acc;
        s_pitch = pitch_acc;
        s_yaw = 0.0f;
        s_first_sample = 0;
    } else {
        /* Complementary filter */
        s_roll = COMP_ALPHA * (s_roll + gx * dt) + (1.0f - COMP_ALPHA) * roll_acc;
        s_pitch = COMP_ALPHA * (s_pitch + gy * dt) + (1.0f - COMP_ALPHA) * pitch_acc;
        /* Yaw: tich phan thuan tuy (khong co accel correction) */
        s_yaw += gz * dt;

        /* Giu yaw trong [-180, 180) */
        if (s_yaw > 180.0f) {
            s_yaw -= 360.0f;
        }
        if (s_yaw < -180.0f) {
            s_yaw += 360.0f;
        }
    }

    /* Apply Low-Pass Filter for smoother output */
    if (s_first_sample) {
        s_roll_filtered = s_roll;
        s_pitch_filtered = s_pitch;
        s_yaw_filtered = s_yaw;
    } else {
        s_roll_filtered = s_roll_filtered + OUTPUT_LPF_ALPHA * (s_roll - s_roll_filtered);
        s_pitch_filtered = s_pitch_filtered + OUTPUT_LPF_ALPHA * (s_pitch - s_pitch_filtered);
        s_yaw_filtered = s_yaw_filtered + OUTPUT_LPF_ALPHA * (s_yaw - s_yaw_filtered);
    }

    angles->roll = s_roll_filtered;
    angles->pitch = s_pitch_filtered;
    angles->yaw = s_yaw_filtered;
}

/*---------------------------------------------------------------------------
 * MPU6050_GetI2cErrorCount
 *---------------------------------------------------------------------------*/
int MPU6050_GetI2cErrorCount(void) {
    return s_i2c_error_count;
}

uint8_t MPU6050_GetWhoAmI(void) {
    return s_whoami;
}

void MPU6050_GetLastRaw(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    if (ax) {
        *ax = s_ax_raw;
    }
    if (ay) {
        *ay = s_ay_raw;
    }
    if (az) {
        *az = s_az_raw;
    }
    if (gx) {
        *gx = s_gx_raw;
    }
    if (gy) {
        *gy = s_gy_raw;
    }
    if (gz) {
        *gz = s_gz_raw;
    }
}

/*---------------------------------------------------------------------------
 * MPU6050_AutoCalibrate
 *---------------------------------------------------------------------------*/
bool MPU6050_AutoCalibrate(uint16_t samples) {
    uint8_t raw[14];
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint16_t i;

    if (samples == 0) {
        return false;
    }

    /* Collect samples */
    for (i = 0; i < samples; i++) {
        if (!mpu_i2c_read_burst(REG_ACCEL_XOUT_H, raw, 14u)) {
            s_i2c_error_count++;
            usleep(2000); /* 2ms delay */
            continue;
        }

        int16_t gx_raw = (int16_t)((raw[8] << 8) | raw[9]);
        int16_t gy_raw = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gz_raw = (int16_t)((raw[12] << 8) | raw[13]);
        int16_t ax_raw = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t ay_raw = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t az_raw = (int16_t)((raw[4] << 8) | raw[5]);

        sum_gx += gx_raw;
        sum_gy += gy_raw;
        sum_gz += gz_raw;
        sum_ax += ax_raw;
        sum_ay += ay_raw;
        sum_az += az_raw;

        usleep(2000); /* 2ms between samples */
    }

    /* Calculate offsets */
    /* Gyro: ideal = 0 when stationary */
    s_cal.gx_offset = (int16_t)(sum_gx / samples);
    s_cal.gy_offset = (int16_t)(sum_gy / samples);
    s_cal.gz_offset = (int16_t)(sum_gz / samples);

    /* Accel: ideal values when horizontal (level) */
    s_cal.ax_offset = (int16_t)(sum_ax / samples);
    s_cal.ay_offset = (int16_t)(sum_ay / samples);
    s_cal.az_offset = (int16_t)((sum_az / samples) - (int16_t)ACCEL_SCALE_2G);

    s_calibrated = true;

    return true;
}

/*---------------------------------------------------------------------------
 * MPU6050_GetCalibration
 *---------------------------------------------------------------------------*/
void MPU6050_GetCalibration(MPU6050_Calibration_t* cal) {
    if (cal != NULL) {
        *cal = s_cal;
    }
}

/*---------------------------------------------------------------------------
 * MPU6050_SetCalibration
 *---------------------------------------------------------------------------*/
void MPU6050_SetCalibration(MPU6050_Calibration_t* cal) {
    if (cal != NULL) {
        s_cal = *cal;
        s_calibrated = true;
    }
}

/*---------------------------------------------------------------------------
 * MPU6050_ResetCalibration
 *---------------------------------------------------------------------------*/
void MPU6050_ResetCalibration(void) {
    s_cal.ax_offset = 0;
    s_cal.ay_offset = 0;
    s_cal.az_offset = 0;
    s_cal.gx_offset = 0;
    s_cal.gy_offset = 0;
    s_cal.gz_offset = 0;
    s_calibrated = false;
}

/*---------------------------------------------------------------------------
 * MPU6050_IsCalibrated
 *---------------------------------------------------------------------------*/
bool MPU6050_IsCalibrated(void) {
    return s_calibrated;
}
