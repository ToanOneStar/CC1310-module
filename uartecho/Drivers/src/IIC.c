/*
 * IIC.c
 *
 * Implementation cua wrapper I2C cho CC1310 / TI-RTOS.
 * I2C bus duoc chia se cho toan bo thread => chi open 1 lan.
 */

#include "../inc/IIC.h"

#include <stddef.h>
#include <string.h>
#include <unistd.h>

/* Fallback I2C speed definitions if not defined in TI headers */
#ifndef I2C_100kHz
#define I2C_100kHz 100000
#endif

#ifndef I2C_400kHz
#define I2C_400kHz 400000
#endif

/* Handle toan cuc - chi open 1 lan trong IIC_Init() */
static I2C_Handle g_i2cHandle = NULL;
static I2C_Params g_i2cParams;
static bool g_i2cInitialized = false;
static uint32_t g_i2cSpeed = I2C_400kHz;

/*---------------------------------------------------------------------------
 * IIC_Init
 *---------------------------------------------------------------------------*/
bool IIC_Init(uint_least8_t index) {
    if (g_i2cHandle != NULL) {
        /* Da khoi tao roi - khong can open them */
        return true;
    }

    I2C_init();

    I2C_Params_init(&g_i2cParams);
    g_i2cParams.bitRate = g_i2cSpeed;
    g_i2cParams.transferMode = I2C_MODE_BLOCKING;
    g_i2cParams.transferCallbackFxn = NULL;

    g_i2cHandle = I2C_open(index, &g_i2cParams);

    g_i2cInitialized = (g_i2cHandle != NULL);
    return g_i2cInitialized;
}

/*---------------------------------------------------------------------------
 * IIC_InitSpeed
 *---------------------------------------------------------------------------*/
bool IIC_InitSpeed(uint_least8_t index, uint32_t speed) {
    g_i2cSpeed = speed;
    return IIC_Init(index);
}

/*---------------------------------------------------------------------------
 * I2C_IsInitialized
 *---------------------------------------------------------------------------*/
bool I2C_IsInitialized(void) {
    return g_i2cInitialized;
}

/*---------------------------------------------------------------------------
 * I2C_Read_Register
 *---------------------------------------------------------------------------*/
uint8_t I2C_Read_Register(uint8_t slaveAddr, uint8_t reg) {
    uint8_t result = 0xFFu;
    if (g_i2cHandle == NULL) {
        return result;
    }

    I2C_Transaction txn;
    memset(&txn, 0, sizeof(txn));
    txn.slaveAddress = slaveAddr;
    txn.writeBuf = &reg;
    txn.writeCount = 1u;
    txn.readBuf = &result;
    txn.readCount = 1u;

    bool status = I2C_transfer(g_i2cHandle, &txn);
    /* Add small delay after transfer */
    usleep(10);
    return status ? result : 0xFFu;
}

/*---------------------------------------------------------------------------
 * I2C_Write_Register
 *---------------------------------------------------------------------------*/
bool I2C_Write_Register(uint8_t slaveAddr, uint8_t reg, uint8_t data) {
    if (g_i2cHandle == NULL) {
        return false;
    }

    uint8_t txBuf[2] = {reg, data};

    I2C_Transaction txn;
    memset(&txn, 0, sizeof(txn));
    txn.slaveAddress = slaveAddr;
    txn.writeBuf = txBuf;
    txn.writeCount = 2u;
    txn.readBuf = NULL;
    txn.readCount = 0u;

    return I2C_transfer(g_i2cHandle, &txn);
}

/*---------------------------------------------------------------------------
 * I2C_Read_Burst
 *---------------------------------------------------------------------------*/
bool I2C_Read_Burst(uint8_t slaveAddr, uint8_t reg, uint8_t* buf, uint8_t len) {
    if ((g_i2cHandle == NULL) || (buf == NULL) || (len == 0u)) {
        return false;
    }

    I2C_Transaction txn;
    memset(&txn, 0, sizeof(txn));
    txn.slaveAddress = slaveAddr;
    txn.writeBuf = &reg;
    txn.writeCount = 1u;
    txn.readBuf = buf;
    txn.readCount = len;

    return I2C_transfer(g_i2cHandle, &txn);
}
