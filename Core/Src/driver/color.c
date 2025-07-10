/*
 * color.c
 *
 *  Created on: Jun 26, 2025
 *      Author: RCY
 */

#include "i2c.h"
#include "uart.h"
#include <math.h>
#include "color.h"
#include "rgb.h"

#define MAX(a,b) (((a) > (b)) ? (a) : (b))

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

const reference_entry_t color_reference_table[] = {
    {{0.1607f, 0.4281f, 0.4111f}, COLOR_RED},
    {{0.1689f, 0.4400f, 0.3911f}, COLOR_ORANGE},
    {{0.1608f, 0.4937f, 0.3456f}, COLOR_YELLOW},
	{{0.1239f, 0.4784f, 0.3977f}, COLOR_LIGHT_GREEN},
    {{0.1213f, 0.4656f, 0.4132f}, COLOR_GREEN},
    {{0.1065f, 0.4233f, 0.4702f}, COLOR_SKY_BLUE},
    {{0.1141f, 0.4225f, 0.4634f}, COLOR_BLUE},
    {{0.1259f, 0.4187f, 0.4553f}, COLOR_PURPLE},
    {{0.1589f, 0.4121f, 0.4290f}, COLOR_PINK},
	{{0.1203f, 0.4491f, 0.4305f}, COLOR_BLACK},
	{{0.1241f, 0.4510f, 0.4248f}, COLOR_WHITE},
	{{0.1242f, 0.4497f, 0.4261f}, COLOR_GRAY},
};



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

    uart_printf("[pr]: %1f [pg]: %1f [pb]: %1f\r\n", result.r_ratio, result.g_ratio, result.b_ratio);

    return result;
}

color_t classify_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
	const float w_r = 1.2f;  // R 가중치
	const float w_g = 1.0f;  // G 가중치
	const float w_b = 1.0f;  // B 가중치

	rgb_ratio_t input = get_rgb_ratio(r, g, b);

	float min_dist = 1e9;
	color_t best_match = COLOR_GRAY;

	for (int i = 0; i < sizeof(color_reference_table) / sizeof(reference_entry_t); i++)
	{
		float dr = input.r_ratio - color_reference_table[i].ratio.r_ratio;
		float dg = input.g_ratio - color_reference_table[i].ratio.g_ratio;
		float db = input.b_ratio - color_reference_table[i].ratio.b_ratio;

		float dist = w_r * dr * dr + w_g * dg * dg + w_b * db * db;

		if (dist < min_dist)
		{
			min_dist = dist;
			best_match = color_reference_table[i].color;
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
