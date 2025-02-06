/*!
 * @file biss_c_master_hal.h
 * @author Kirill Rostovskiy (kmrost@lenzencoders.com)
 * @brief BiSS C Master Hardware abstraction layer driver
 * @version 0.1
 * @copyright Lenz Encoders (c) 2024
 */

#ifndef __BISS_C_MASTER_HAL_H
#define __BISS_C_MASTER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
/**
 * @brief BiSS C Master hardware abstruction layer initialization function
 * 
 */
void BiSS_C_Master_HAL_Init(void);

/**
 * @struct AngleDataRenishaw_t
 * 
 */
typedef struct{
    uint16_t angle_data:16; /**< Value of Angle */
} AngleDataRenishaw_t;
	
/**
 * @brief Get the Angle Renishaw object
 * 
 * @return AngleDataRenishaw_t 
 */
static inline AngleDataRenishaw_t getAngleRenishaw(void){
	extern volatile AngleDataRenishaw_t AngleDataRenishaw;
	return(AngleDataRenishaw);
}
#ifdef __cplusplus
}
#endif

#endif /* __BISS_C_MASTER_HAL_H */
