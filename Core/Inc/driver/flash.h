/*
 * flash.h
 *
 *  Created on: Jul 11, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_FLASH_H_
#define INC_DRIVER_FLASH_H_

#include "stm32wbxx_hal.h"

#define FLASH_COLOR_TABLE_ADDR_LEFT   ((uint32_t)0x0807F800)               // Page 255
#define FLASH_COLOR_TABLE_ADDR_RIGHT  ((uint32_t)0x0807F000)               // Page 254 (하나 위)
#define FLASH_COLOR_ENTRY_SIZE        (sizeof(reference_entry_t))


void flash_write_color_reference(uint8_t sensor_side, uint8_t color_index, reference_entry_t entry);
reference_entry_t flash_read_color_reference(uint8_t sensor_side, uint8_t color_index);
void flash_erase_color_table(uint8_t sensor_side);



#endif /* INC_DRIVER_FLASH_H_ */
