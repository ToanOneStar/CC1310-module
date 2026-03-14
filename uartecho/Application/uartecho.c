/*
 * ======== uartecho.c ========
 * CC1310: Test MPU6050 (No UART version)
 *
 * Hardware: CC1310 module + MPU6050
 * I2C: DIO6=SCL, DIO8=SDA
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <ti/devices/cc13x0/driverlib/i2c.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/prcm.h>
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>

#include "../Config/Board.h"
#include "../Drivers/inc/MPU_6050.h"

#define LOOP_PERIOD_US 100000u /* 100ms */

static void i2c0_drvlib_init(void) {
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON) {}
    PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0);
    PRCMLoadSet();
    while (!PRCMLoadGet()) {}

    /* Configure pins for I2C using driverlib helper */
    IOCPinTypeI2c(IOC_BASE, IOID_3, IOID_2); /* SDA, SCL */

    I2CMasterInitExpClk(I2C0_BASE, SysCtrlClockGet(), false);
    I2CMasterEnable(I2C0_BASE);
}

static uint8_t i2c0_drvlib_read_reg(uint8_t addr7, uint8_t reg) {
    uint32_t err;

    I2CMasterSlaveAddrSet(I2C0_BASE, addr7, false);
    I2CMasterDataPut(I2C0_BASE, reg);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C0_BASE)) {}
    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE) {
        return 0xFFu;
    }

    I2CMasterSlaveAddrSet(I2C0_BASE, addr7, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C0_BASE)) {}
    err = I2CMasterErr(I2C0_BASE);
    if (err != I2C_MASTER_ERR_NONE) {
        return 0xFFu;
    }

    return (uint8_t)I2CMasterDataGet(I2C0_BASE);
}

/* Blink functions with different patterns */
void* mainThread(void* arg0) {
    static volatile MPU6050_Angle_t g_angles;
    static volatile int g_i2c_err = 0;
    static volatile uint8_t g_whoami = 0u;
    static volatile int16_t g_ax = 0, g_ay = 0, g_az = 0;
    static volatile int16_t g_gx = 0, g_gy = 0, g_gz = 0;
    static volatile int g_init_status = -1;
    static volatile int g_scan_count = 0;
    static volatile uint8_t g_scan0 = 0, g_scan1 = 0, g_scan2 = 0, g_scan3 = 0;
    static volatile uint8_t g_who_live = 0xFFu;
    static volatile uint8_t g_pwr1 = 0xFFu;
    static volatile uint8_t g_who_dl_68 = 0xFFu;
    static volatile uint8_t g_who_dl_69 = 0xFFu;
    static volatile uint32_t g_loop = 0;
    const float dt = (float)LOOP_PERIOD_US / 1000000.0f;

    /* Driverlib direct I2C test (before TI I2C driver) */
    i2c0_drvlib_init();
    g_who_dl_68 = i2c0_drvlib_read_reg(0x68u, 0x75u);
    g_who_dl_69 = i2c0_drvlib_read_reg(0x69u, 0x75u);

    /* Stage 1: MPU6050 init (I2C open inside driver) */
    g_init_status = 123;
    uint8_t initResult = MPU6050_Init();
    g_init_status = (int)initResult;
    {
        uint8_t addrs[4] = {0, 0, 0, 0};
        g_scan_count = MPU6050_Scan(addrs, 4);
        g_scan0 = addrs[0];
        g_scan1 = addrs[1];
        g_scan2 = addrs[2];
        g_scan3 = addrs[3];
    }

    /* Main loop */
    while (1) {
        g_i2c_err = MPU6050_GetI2cErrorCount();
        MPU6050_GetAngle((MPU6050_Angle_t*)&g_angles, dt);
        g_whoami = MPU6050_GetWhoAmI();
        MPU6050_GetLastRaw((int16_t*)&g_ax,
                           (int16_t*)&g_ay,
                           (int16_t*)&g_az,
                           (int16_t*)&g_gx,
                           (int16_t*)&g_gy,
                           (int16_t*)&g_gz);
        if ((g_loop % 10u) == 0u) {
            g_who_live = MPU6050_ReadReg(0x75u); /* WHO_AM_I */
            g_pwr1 = MPU6050_ReadReg(0x6Bu);     /* PWR_MGMT_1 */
        }
        g_loop++;
        usleep(LOOP_PERIOD_US);
    }
}
