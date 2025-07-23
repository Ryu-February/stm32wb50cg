/*
 * color.c
 *
 *  Created on: Jun 26, 2025
 *      Author: RCY
 */



#include <stdlib.h>
#include "i2c.h"
#include "uart.h"
#include <math.h>
#include "color.h"
#include "rgb.h"
#include "flash.h"
#include "step.h"


//const bh1745_color_data_t reference_colors[COLOR_COUNT] = {
//    [COLOR_RED]        = {116, 278, 188, 43},
//    [COLOR_ORANGE]     = {124, 300, 187, 45},
//    [COLOR_YELLOW]     = {129, 371, 193, 49},
//    [COLOR_GREEN]      = {89, 308, 200, 43},
//    [COLOR_BLUE]       = {94, 294, 243, 43},
//    [COLOR_PURPLE]     = {93, 276, 217, 42},
//    [COLOR_LIGHT_GREEN]= {101, 329, 191, 45},
//    [COLOR_SKY_BLUE]   = {92, 331, 282, 45},
//    [COLOR_PINK]       = {122, 289, 224, 45},
//    [COLOR_BLACK]      = {86, 267, 182, 40},
//    [COLOR_WHITE]      = {128, 396, 293, 47},
//    [COLOR_GRAY]       = {110, 336, 244, 43}
//};

reference_entry_t color_reference_tbl_left[COLOR_COUNT];
reference_entry_t color_reference_tbl_right[COLOR_COUNT];

extern uint8_t offset_side;
extern uint16_t offset_black;
extern uint16_t offset_white;
extern uint16_t offset_average;



void bh1745_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    i2c_write(dev_addr, reg, data);
}

void bh1745_init(uint8_t dev_addr)
{
    // 1. SW Reset
    bh1745_write_reg(dev_addr, 0x40, 0x80);  // SYSTEM_CONTROL: Software Reset
    HAL_Delay(10);

    // 2. Measurement Time 설정 (MODE_CONTROL1)
    bh1745_write_reg(dev_addr, 0x41, 0x00);  // 160ms

    // 3. Gain 설정 + RGBC Enable (MODE_CONTROL2)
    bh1745_write_reg(dev_addr, 0x42, 0x12);  // GAIN = 1x, Bit4(RGBC_EN) = 1

    // 4. RGB 측정 트리거 (MODE_CONTROL3)
    bh1745_write_reg(dev_addr, 0x44, 0x02);  // RGB measurement start
}

uint16_t bh1745_read_u16(uint8_t dev_addr, uint8_t lsb_reg)
{
    uint8_t lsb = i2c_read(dev_addr, lsb_reg);
    uint8_t msb = i2c_read(dev_addr, lsb_reg + 1);
    return (msb << 8) | lsb;
}

bh1745_color_data_t bh1745_read_rgbc(uint8_t dev_addr)
{
    bh1745_color_data_t color;

    color.red   = bh1745_read_u16(dev_addr, 0x50);
    color.green = bh1745_read_u16(dev_addr, 0x52);
    color.blue  = bh1745_read_u16(dev_addr, 0x54);
    color.clear = bh1745_read_u16(dev_addr, 0x56);

    return color;
}


void save_color_reference(uint8_t sensor_side, color_t color, uint16_t r, uint16_t g, uint16_t b)
{
    rgb_ratio_t ratio	= get_rgb_ratio(r, g, b);
    uint64_t 	offset	= calculate_brightness(r, g, b);

    reference_entry_t entry = { .ratio = ratio, .color = color, .offset = offset };

    if (sensor_side == BH1745_ADDR_LEFT)
    {
        color_reference_tbl_left[color] = entry;
    }
    else
    {
        color_reference_tbl_right[color] = entry;
    }

    // Flash에 저장!
    flash_write_color_reference(sensor_side, color, entry);
}

rgb_ratio_t get_rgb_ratio(uint16_t r, uint16_t g, uint16_t b)
{
    float total = (float)r + g + b;
    rgb_ratio_t result = {0};

    if (total > 0.0f)
    {
        result.r_ratio = r / total;
        result.g_ratio = g / total;
        result.b_ratio = b / total;
    }

//    uart_printf("[pr]: %1f [pg]: %1f [pb]: %1f\r\n", result.r_ratio, result.g_ratio, result.b_ratio);

    return result;
}

color_t classify_color(uint8_t left_right, uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
	const float w_r = 1.2f;  // R 가중치
	const float w_g = 1.0f;  // G 가중치
	const float w_b = 1.0f;  // B 가중치

	rgb_ratio_t input = get_rgb_ratio(r, g, b);

	float min_dist = 1e9;
	color_t best_match = COLOR_GRAY;

    const reference_entry_t* table;
    int table_size;

    if (left_right == BH1745_ADDR_LEFT)  // LEFT
    {
        table = color_reference_tbl_left;
        table_size = sizeof(color_reference_tbl_left) / sizeof(reference_entry_t);
    }
    else // RIGHT
    {
        table = color_reference_tbl_right;
        table_size = sizeof(color_reference_tbl_right) / sizeof(reference_entry_t);
    }

    for (int i = 0; i < table_size; i++)
    {
        float dr = input.r_ratio - table[i].ratio.r_ratio;
        float dg = input.g_ratio - table[i].ratio.g_ratio;
        float db = input.b_ratio - table[i].ratio.b_ratio;

        float dist = w_r * dr * dr + w_g * dg * dg + w_b * db * db;

        if (dist < min_dist)
        {
            min_dist = dist;
            best_match = table[i].color;
        }
    }

	return best_match;
}

const char* color_to_string(color_t color)
{
    static const char* color_names[] =
    {
        "RED",
        "ORANGE",
        "YELLOW",
        "GREEN",
        "BLUE",
        "PURPLE",
        "LIGHT_GREEN",
        "SKY_BLUE",
        "PINK",
        "BLACK",
        "WHITE",
        "GRAY"
    };

    if (color < 0 || color >= COLOR_COUNT)
        return "UNKNOWN";

    return color_names[color];
}

void load_color_reference_table(void)
{
    for (int i = 0; i < COLOR_COUNT; i++)
    {
        color_reference_tbl_left[i] = flash_read_color_reference(BH1745_ADDR_LEFT, i);
        color_reference_tbl_right[i] = flash_read_color_reference(BH1745_ADDR_RIGHT, i);
    }
}

void debug_print_color_reference_table(void)
{
    uart_printf("=== LEFT COLOR REFERENCE TABLE ===\r\n");
    for (int i = 0; i < COLOR_COUNT; i++)
    {
        rgb_ratio_t r = color_reference_tbl_left[i].ratio;
        color_t c = color_reference_tbl_left[i].color;

        uart_printf("[%d | %s] [R]: %1.3f, [G]: %1.3f, [B]: %1.3f\r\n",
                    i, color_to_string(c), r.r_ratio, r.g_ratio, r.b_ratio);
    }

    uart_printf("=== RIGHT COLOR REFERENCE TABLE ===\r\n");
    for (int i = 0; i < COLOR_COUNT; i++)
    {
        rgb_ratio_t r = color_reference_tbl_right[i].ratio;
        color_t c = color_reference_tbl_right[i].color;

        uart_printf("[%d | %s] [R]: %1.3f, [G]: %1.3f, [B]: %1.3f\r\n",
                    i, color_to_string(c), r.r_ratio, r.g_ratio, r.b_ratio);
    }
    uart_printf("=== BRIGHTNESS OFFSET TABLE ===\r\n");
    uart_printf("offset_black: %d | offset_white: %d\r\n", offset_black, offset_white);
	uart_printf("offset_aver: %d\r\n", offset_average);
}

uint32_t calculate_brightness(uint16_t r, uint16_t g, uint16_t b)
{
//    return 0.2126f * r + 0.7152f * g + 0.0722f * b;

    return (218 * r + 732 * g + 74 * b) >> 10;
}

void calculate_color_brightness_offset(void)
{
    offset_black = abs(color_reference_tbl_left[COLOR_BLACK].offset - color_reference_tbl_right[COLOR_BLACK].offset);
    offset_white = abs(color_reference_tbl_left[COLOR_WHITE].offset - color_reference_tbl_right[COLOR_WHITE].offset);

    offset_side = (color_reference_tbl_left[COLOR_BLACK].offset > color_reference_tbl_right[COLOR_BLACK].offset)
    					? LEFT : RIGHT;
    offset_average = (offset_black + offset_white) / 2;
}
