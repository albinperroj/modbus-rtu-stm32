/*
 * rtuHandlerHelp.h
 *
 *  Created on: Jun 22, 2021
 *      Author: Albin Perroj
 */

#ifndef RTUHANDLERHELP_H_
#define RTUHANDLERHELP_H_

/*----------------------------------------------------------
 * My includes
 */
#include "stm32f1xx_hal.h"
#include "inttypes.h"
#include "stdbool.h"
#include "math.h"


/*--------------------------------------------------------
 * Defines
 */

#define XBuffer_MAX 128

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))


#endif /* RTUHANDLERHELP_H_ */
