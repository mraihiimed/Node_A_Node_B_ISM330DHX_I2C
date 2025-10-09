/*
 * ism330dhcx.h
 *
 *  Created on: Oct 9, 2025
 *      Author: Geek
 */

#ifndef INC_ISM330DHCX_H_
#define INC_ISM330DHCX_H_

void ISM330DHCX_ReadRaw(MotionRaw_T* motion);
uint8_t ISM330_I2C_Read(uint8_t reg);
void ISM330_I2C_Write(uint8_t reg, uint8_t value);
void ISM330_Init(void);

#endif /* INC_ISM330DHCX_H_ */
