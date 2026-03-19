/*
 * ======== uartecho.c ========
 * CC1310: Test MPU6050 + RF Transmission
 *
 * Hardware: CC1310 module + MPU6050
 * I2C: IO2=SCL, IO4=SDA
 * RF: Transmit angles via RF API (not EasyLink)
 */

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <ti/devices/cc13x0/driverlib/gpio.h>
#include <ti/devices/cc13x0/driverlib/i2c.h>
#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/prcm.h>
#include <ti/devices/cc13x0/driverlib/sys_ctrl.h>

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/* RF status definitions */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

#include <smartrf_settings/smartrf_settings.h>
#include "../Config/Board.h"
#include "../Drivers/inc/MPU_6050.h"

#define LOOP_PERIOD_US    100000u /* 100ms */
#define RF_PAYLOAD_LENGTH 16      /* RF packet size: roll(4) + pitch(4) + yaw(4) + seq(2) + status(2) */
#define PACKET_INTERVAL   100000  /* TX every 100ms */

/*---------------------------------------------------------------------------
 * RF Variables
 *---------------------------------------------------------------------------*/
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint8_t packet[RF_PAYLOAD_LENGTH];
static uint16_t rf_seqNumber = 0;
static volatile int g_rf_tx_count = 0;
static volatile int g_rf_tx_err = 0;

/*---------------------------------------------------------------------------
 * Global Variables
 *---------------------------------------------------------------------------*/
static volatile int g_init_status = -1;
static volatile int g_scan_count = 0;

/*---------------------------------------------------------------------------
 * Convert float to bytes (IEEE 754)
 *---------------------------------------------------------------------------*/
static void float_to_bytes(float val, uint8_t* buf) {
    union {
        float f;
        uint8_t b[4];
    } u;

    u.f = val;
    buf[0] = u.b[0];
    buf[1] = u.b[1];
    buf[2] = u.b[2];
    buf[3] = u.b[3];
}

/*---------------------------------------------------------------------------
 * Initialize RF
 *---------------------------------------------------------------------------*/
static int rf_init(void) {
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Configure TX packet */
    RF_cmdPropTx.pktLen = RF_PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif

    if (rfHandle == NULL) {
        g_rf_tx_err = -1;
        return -1;
    }

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    return 0;
}

/*---------------------------------------------------------------------------
 * Transmit angles via RF
 *---------------------------------------------------------------------------*/
static int rf_send_angles(float roll, float pitch, float yaw) {
    /* Build packet: roll(4) + pitch(4) + yaw(4) + seq(2) + status(2) = 16 bytes */
    float_to_bytes(roll, &packet[0]);
    float_to_bytes(pitch, &packet[4]);
    float_to_bytes(yaw, &packet[8]);

    /* Sequence number */
    packet[12] = (uint8_t)(rf_seqNumber >> 8);
    packet[13] = (uint8_t)(rf_seqNumber++);

    /* Status */
    packet[14] = (uint8_t)g_init_status;
    packet[15] = (uint8_t)g_scan_count;

    /* Send packet */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx,
                                               RF_PriorityNormal, NULL, 0);

    if (terminationReason == RF_EventLastCmdDone) {
        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        if (cmdStatus == PROP_DONE_OK) {
            g_rf_tx_count++;
            return 0;
        }
    }

    g_rf_tx_err++;
    return -1;
}

/*---------------------------------------------------------------------------
 * Main Thread
 *---------------------------------------------------------------------------*/
void* mainThread(void* arg0) {
    static volatile MPU6050_Angle_t g_angles;
    static volatile int g_i2c_err = 0;
    static volatile uint8_t g_whoami = 0u;
    static volatile int16_t g_ax = 0, g_ay = 0, g_az = 0;
    static volatile int16_t g_gx = 0, g_gy = 0, g_gz = 0;
    static volatile uint8_t g_scan0 = 0, g_scan1 = 0, g_scan2 = 0, g_scan3 = 0;
    static volatile uint8_t g_who_live = 0xFFu;
    static volatile uint8_t g_pwr1 = 0xFFu;
    static volatile uint32_t g_loop = 0;
    const float dt = (float)LOOP_PERIOD_US / 1000000.0f;

    /* Enable I2C power domain FIRST - required for TI I2C driver */
    PRCMPowerDomainOn(PRCM_DOMAIN_SERIAL);
    while (PRCMPowerDomainStatus(PRCM_DOMAIN_SERIAL) != PRCM_DOMAIN_POWER_ON) {}
    PRCMPeripheralRunEnable(PRCM_PERIPH_I2C0);
    PRCMLoadSet();
    while (!PRCMLoadGet()) {}

    /* Initialize MPU6050 using TI I2C driver */
    g_init_status = 123;
    uint8_t initResult = MPU6050_Init();
    g_init_status = (int)initResult;

    /* Scan I2C bus */
    {
        uint8_t addrs[4] = {0, 0, 0, 0};
        g_scan_count = MPU6050_Scan(addrs, 4);
        g_scan0 = addrs[0];
        g_scan1 = addrs[1];
        g_scan2 = addrs[2];
        g_scan3 = addrs[3];
    }

    /* Initialize RF */
    rf_init();

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

        /* Transmit angles via RF */
        rf_send_angles(g_angles.roll, g_angles.pitch, g_angles.yaw);

        if ((g_loop % 10u) == 0u) {
            g_who_live = MPU6050_ReadReg(0x75u);
            g_pwr1 = MPU6050_ReadReg(0x6Bu);
        }
        g_loop++;
        usleep(LOOP_PERIOD_US);
    }
}
