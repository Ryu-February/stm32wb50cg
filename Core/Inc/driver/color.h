/*
 * color.h
 *
 *  Created on: Jun 26, 2025
 *      Author: RCY
 */


#ifndef INC_DRIVER_COLOR_H_
#define INC_DRIVER_COLOR_H_

#include "stm32wbxx_hal.h"
#include "rgb.h"


#define MODE_CALIBRATION	1

#define BH1745_ADDR_LEFT        	0x38 // 7bit << 1
#define BH1745_ADDR_RIGHT      		0x39 // 7bit << 1

// Register Addresses
#define BH1745_REG_MODE_CTRL1   	0x41
#define BH1745_REG_MODE_CTRL2   	0x42
#define BH1745_REG_MODE_CTRL3   	0x44
#define BH1745_REG_RED_DATA_LSB 	0x50
#define BH1745_REG_GREEN_DATA_LSB 	0x52
#define BH1745_REG_BLUE_DATA_LSB 	0x54
#define BH1745_REG_CLEAR_DATA_LSB 	0x56

#define BH1745_I2C_ADDR         (0x38 << 1) // 0x70
#define BH1745_REG_MANUFACTURER_ID   0x92

typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} bh1745_color_data_t;


typedef struct {
    float r_ratio;
    float g_ratio;
    float b_ratio;
} rgb_ratio_t;

typedef struct {
    rgb_ratio_t ratio;
    color_t color;
    uint16_t offset;
} reference_entry_t;

void bh1745_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data);
void bh1745_init(uint8_t dev_addr);

uint16_t bh1745_read_u16(uint8_t dev_addr, uint8_t lsb_reg);
bh1745_color_data_t bh1745_read_rgbc(uint8_t dev_addr);
bh1745_color_data_t bh1745_read_rgbc(uint8_t dev_addr);

void save_color_reference(uint8_t sensor_side, color_t color, uint16_t r, uint16_t g, uint16_t b);
rgb_ratio_t get_rgb_ratio(uint16_t r, uint16_t g, uint16_t b);
color_t classify_color(uint8_t left_right, uint16_t r, uint16_t g, uint16_t b, uint16_t c);
const char* color_to_string(color_t color);
void load_color_reference_table(void);
void debug_print_color_reference_table(void);
uint32_t calculate_brightness(uint16_t r, uint16_t g, uint16_t b);
void calculate_color_brightness_offset(void);


#endif /* INC_DRIVER_COLOR_H_ */
