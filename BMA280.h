/*
 * BMA280.h
 *
 *  Created on: May 20, 2018
 *      Author: Kevin Kuwata
 *
 */

#include "msp.h"

#ifndef BMA280_H_
#define BMA280_H_

#define BMA_DEAULT_ADDR                  (0x18) //SDO tied to ground
#define ALT_ADDR_ACCL                       (0X19) //SDO tied to VDDIO

/*
 * @name: initBMADriver(void);
 * @brief: configures gpio pins, i2c bus UCB3 , sets up 2 interrupt pin:
 *          10.1 for Accl  Interrupt 1
 *          1.1 for Accl Interrupt 2
 *          //PRIMARY MODE FOR I2C 10.2 and 10.3
 * @inputs: none
 * @return: none
 * */
void initBMADriver(void);


/*  @name: void writeRegister(uint8_t address, uint8_t reg, uint8_t value);
 *  @brief: writes to the desired device address a register, using repeated start
 * */
void writeRegister(uint8_t address, uint8_t reg, uint8_t value);

/* @name: readSingle(address, reg, result)
 * @brief: read a single byte from the designated register, known issue with the eUSCI module that
 *          single reads are not handled properly.
 *          TODO: fix the misread, requires double read to get
 *          accurate value
*  @input: device address, desired register, address where to store result.
 * */
void readSingle(uint8_t address, uint8_t reg, uint8_t* result);


/* @name: void readMultiple(uint8_t address, uint8_t reg, uint8_t numBytes,uint8_t* result)
 * @brief: read multiple number of bytes from the given i2c address. does not use repeated start
 * @input: slave address, register to read, number of bytes to read, and then where to put the result
 */
void readMultiple(uint8_t address, uint8_t reg, uint8_t numBytes,uint8_t* result);

/* @name: void beginTransmission(uint8_t address)
 * @brief: sets the start bit, does not determine if Write or Read. That is left for user
 *          and is done so in the readRegister or writeRegister functions
 * */
void beginTransmission(uint8_t address);


/* @name: void stopTransmission(void)
 * @brief: sets the stop bit for eUSCIB3
 * */
void stopTransmission(void);




#endif /* BMA280_H_ */
