/*
 * rgb.h
 *
 *  Created on: Jun 26, 2025
 *      Author: RCY
 */

#ifndef INC_DRIVER_RGB_H_
#define INC_DRIVER_RGB_H_



#include "stm32wbxx_hal.h"

typedef enum
{/*
	COLOR_BLACK 		= 0,
	COLOR_RED			= 1,
	COLOR_ORANGE		= 2,
	COLOR_YELLOW		= 3,
	COLOR_GREEN			= 4,
	COLOR_LIGHT_GREEN	= 5,
	COLOR_CYAN			= 6,
	COLOR_LIGHT_BLUE	= 7,
	COLOR_BLUE			= 8,
	COLOR_INDIGO		= 9,
	COLOR_VIOLET		= 10,
	COLOR_PINK			= 11,
	COLOR_WHITE			= 12,
	COLOR_COUNT			= 13*/

	COLOR_RED = 0,
	COLOR_ORANGE,
	COLOR_YELLOW,
	COLOR_GREEN,
	COLOR_BLUE,
	COLOR_PURPLE,
	COLOR_LIGHT_GREEN,
	COLOR_SKY_BLUE,
	COLOR_PINK,
	COLOR_BLACK,
	COLOR_WHITE,
	COLOR_GRAY,
	COLOR_COUNT
} color_t;

typedef struct
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}rgb_led_t;

void rgb_init(void);
void rgb_set_color(color_t color);


#endif /* INC_DRIVER_RGB_H_ */
