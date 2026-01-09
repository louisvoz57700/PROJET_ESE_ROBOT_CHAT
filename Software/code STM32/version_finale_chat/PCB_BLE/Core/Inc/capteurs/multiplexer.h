/*
 * multiplexer.h
 *
 *  Created on: Oct 29, 2025
 *      Author: louisvoz
 */

#ifndef INC_MULTIPLEXER_H_
#define INC_MULTIPLEXER_H_

#include "i2c.h"

#define MUX_I2C_HANDLER hi2c1

#define I2C_MUX_ADDR  (0x70 << 1)  // Adresse 7 bits décalée pour HAL

#endif /* INC_MULTIPLEXER_H_ */
