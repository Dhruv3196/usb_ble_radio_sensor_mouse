#ifndef SENSOR_FUNC_H
#define SENSOR_FUNC_H

#include <stdint.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// #include <zephyr.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>

/* STEP 1.2 - Include the header files for SPI, GPIO and devicetree */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

// struct motion_burst {
// 	uint8_t Motion;
// 	uint8_t Observation;
// 	uint8_t Del_X_L;
// 	int8_t Del_X_H;
// 	uint8_t Del_Y_L;
// 	int8_t Del_Y_H;
// 	uint8_t squal;
// 	uint8_t RawData_Sum;
// 	uint8_t Max_RawData;
// 	uint8_t Min_RawData;
// 	uint8_t Shutter_up;
// 	uint8_t Shutter_low;
// };

// struct motion_burst {
// 	uint8_t Shutter_low;
// 	uint8_t Shutter_up;
// 	uint8_t Min_RawData;
// 	uint8_t Max_RawData;
// 	uint8_t RawData_Sum;
// 	uint8_t squal;
// 	int8_t Del_Y_H;
// 	uint8_t Del_Y_L;
// 	int8_t Del_X_H;
// 	uint8_t Del_X_L;
// 	uint8_t Observation;
// 	uint8_t Motion;	
// };

//timing definitions 
#define CLK_SPEED       2000000  //Hz
#define Tsrad			2 //us
#define Tsrr			2
#define Tsrw			2
#define Tsww			5
#define Tswr			5

//register definitions
#define MOTION_BURST_REG		0x16
#define POWER_UP_RESET_REG		0x3A
#define ACCESS_REG				0x7F
#define SET_RESOLUTION_REG		0x47
#define RESOLUTION_X_LOW_REG	0x48
#define RESOLUTION_X_HIGH_REG	0x49
#define RESOLUTION_Y_LOW_REG	0x4A
#define RESOLUTION_Y_HIGH_REG	0x4B
#define MOTION_CTRL_REG			0x5C
#define PERFORMANCE_REG			0x40



//function prototypes
long get_time(void);
int paw_read_regs(uint8_t reg, uint8_t *pdata, uint8_t size);
int paw_write_reg(uint16_t add, uint8_t data);
int update_values_motion_burst(void);
void CPI_set(uint16_t resolution_x, uint16_t resolution_y);
void power_up_init_reg(void);

#endif