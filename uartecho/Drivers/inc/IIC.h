/*
 * IIC.h
 *
 * Wrapper I2C don gian cho CC1310 / TI-RTOS.
 *
 * Hardware mapping:
 *   DIO6 = SCL
 *   DIO8 = SDA
 *
 * Su dung:
 *   IIC_Init(Board_I2C0);
 *   I2C_Read_Register(slaveAddr7bit, reg);
 *   I2C_Write_Register(slaveAddr7bit, reg, data);
 *   I2C_Read_Burst(slaveAddr7bit, reg, *buf, len);
 */

#ifndef IIC_H_
#define IIC_H_

#include <stdbool.h>
#include <stdint.h>
#include <ti/drivers/I2C.h>

/*---------------------------------------------------------------------------
 * Public API
 *---------------------------------------------------------------------------*/

/**
 * @brief  Khoi tao bo dieu khien I2C.
 * @param  index  Chi so I2C (vi du: Board_I2C0).
 * @return true neu thanh cong, false neu that bai.
 */
bool IIC_Init(uint_least8_t index);

/**
 * @brief  Kiem tra xem I2C da duoc khoi tao chua.
 * @return true neu da khoi tao, false neu chua.
 */
bool I2C_IsInitialized(void);

/**
 * @brief  Khoi tao I2C voi toc do tuy chon.
 * @param  index  Chi so I2C (vi du: Board_I2C0).
 * @param  speed  Toc do I2C (I2C_100kHz hoac I2C_400kHz).
 * @return true neu thanh cong, false neu that bai.
 */
bool IIC_InitSpeed(uint_least8_t index, uint32_t speed);

/**
 * @brief  Doc 1 byte tu thanh ghi cua thiet bi.
 * @param  slaveAddr  Dia chi 7-bit cua thiet bi I2C.
 * @param  reg        Dia chi thanh ghi.
 * @return Gia tri byte doc duoc (0xFF neu loi).
 */
uint8_t I2C_Read_Register(uint8_t slaveAddr, uint8_t reg);

/**
 * @brief  Ghi 1 byte vao thanh ghi cua thiet bi.
 * @param  slaveAddr  Dia chi 7-bit cua thiet bi I2C.
 * @param  reg        Dia chi thanh ghi.
 * @param  data       Byte can ghi.
 * @return true neu thanh cong.
 */
bool I2C_Write_Register(uint8_t slaveAddr, uint8_t reg, uint8_t data);

/**
 * @brief  Doc nhieu byte lien tiep tu thanh ghi.
 * @param  slaveAddr  Dia chi 7-bit cua thiet bi I2C.
 * @param  reg        Dia chi thanh ghi bat dau.
 * @param  buf        Con tro vung dem nhan du lieu.
 * @param  len        So byte can doc.
 * @return true neu thanh cong.
 */
bool I2C_Read_Burst(uint8_t slaveAddr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif /* IIC_H_ */
