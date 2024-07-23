#include "sensor_func.h"

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// #include <zephyr.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>
#include <stdint.h>

/* STEP 1.2 - Include the header files for SPI, GPIO and devicetree */
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>

extern struct motion_burst MotionBurstData;
extern struct spi_dt_spec spispec;
extern const struct gpio_dt_spec ncs_pin;
extern int64_t Time_read;
extern int64_t Time_write;
extern bool flag_write;
bool volatile burst = false;

long get_time(void)        //this function to calculate the current time in us
{
	uint32_t cycles = k_cycle_get_32();
	uint32_t freq = sys_clock_hw_cycles_per_sec();
	uint64_t uptime_us = ((uint64_t)cycles * 1000000U) / freq;
	return uptime_us;
}

// int paw_read_regs(uint8_t reg, uint8_t *pdata, uint8_t size) //reg= writing register, 
int paw_read_regs(uint8_t reg, uint8_t dataRecv[], uint8_t size)
{                                                                   //pdata= pointer to where data is stored, 
	int err=0;														//size= size of incoming data
	uint8_t tx_buffer[1];
	tx_buffer[0] = (0<<7) | reg;
	// printk("value in tx buffer is %x\n",tx_buffer[0]);
	// struct spi_buf tx_spi_buf = {.buf = (void *)&tx_buffer, .len = 1};
	struct spi_buf tx_spi_buf[] = {
		{.buf = tx_buffer, .len = sizeof(tx_buffer)},
	};
	struct spi_buf_set tx_spi_buf_set = {.buffers = tx_spi_buf, .count = 1};
	
	struct spi_buf rx_spi_bufs[] = {
		{.buf = dataRecv, .len = size},
	};
	struct spi_buf_set rx_spi_buf_set = {.buffers = rx_spi_bufs, .count = 1};
	//check for timing in these loops
	int64_t t=get_time();
	if(flag_write){
		while(t-Time_write<Tswr){
			t=get_time();
		}
	}
	else{
		while(t-Time_read<Tsrr){
			t=get_time();
			// LOG_INF("%lli\n",t-Time_read);
		}
	}
	gpio_pin_set_dt(&ncs_pin,0);
	do{
		err = spi_write_dt(&spispec,&tx_spi_buf_set);
		if (err < 0) {break;}

		err = spi_read_dt(&spispec,&rx_spi_buf_set);

	} while (false);
	gpio_pin_set_dt(&ncs_pin,1);
	flag_write=false;
	burst = false;
	// k_usleep(Tsrr);
	return err;
}


int paw_write_reg(uint16_t regaddr, uint8_t data)
{
	int err=0;
	// k_cycle_get_64();
	// k_usleep(10);
	uint8_t tx_buffer[] = {((1<<7) | regaddr) , data};
	// printk("value in tx buffer is %x\n",tx_buffer[0]);
	struct spi_buf tx_spi_buf[] = {
		{.buf = tx_buffer, .len = sizeof(tx_buffer)},
	};
	struct spi_buf_set tx_spi_buf_set = {.buffers = tx_spi_buf, .count = 1};

	int64_t t=get_time();   //checks for time 
	if(flag_write){
		while(t-Time_write<Tsww-(16*1E6/CLK_SPEED)){   	//It has been optimized for PAW3395
			t=get_time();								//refer Datasheet 
		}
	}
	else{
		while(t-Time_read<Tsrw-(8*1E6/CLK_SPEED)){
			t=get_time();
		}
	}
	gpio_pin_set_dt(&ncs_pin,0);
	err = spi_write_dt(&spispec,&tx_spi_buf_set);
	gpio_pin_set_dt(&ncs_pin,1);
	flag_write=true;
	if (err < 0) {
		printk("spi_write_dt() for writing data failed failed, err %d \n", err);
		return err;
	}
	burst=false;
	// printk(".");
	return 80085;
}

int update_values_motion_burst(void){
	uint8_t value[12];
	int err=0;
	!burst ? paw_write_reg(MOTION_BURST_REG,0x00) : NULL;
	err=paw_read_regs(MOTION_BURST_REG, value, 12);

	
	// pMotionBurstData = &value;
	burst=true;

	// paw_write_reg(0x02,0x00);

	return err;
}
// int update_values_motion_burst(void){
// 	uint8_t value[1];
// 	int err;
// 	err=paw_read_regs(0x02, value, 1);
// 	// pMotionBurstData->Motion = value[0];
// 	err=paw_read_regs(0x03, value, 1);
// 	// pMotionBurstData->Del_X_L = value[0];
// 	err=paw_read_regs(0x04, value, 1);
// 	// pMotionBurstData->Del_X_H = value[0];
// 	err=paw_read_regs(0x05, value, 1);
// 	// pMotionBurstData->Del_Y_L = value[0];
// 	err=paw_read_regs(0x06, value, 1);
// 	// pMotionBurstData->Del_Y_H = value[0];
// 	paw_write_reg(0x02,0x0);

// 	return err;
// }
void CPI_set(uint16_t resolution_x, uint16_t resolution_y)
{
	uint8_t x[2];
	uint8_t y[2];
	
	uint16_t data_x = (resolution_x/50) - 1;
	uint16_t data_y = (resolution_y/50) - 1;
	x[0] = data_x;
	x[1] = data_x >> 8;
	y[0] = data_y;
	y[1] = data_y >> 8;

	printk("\nCPI_X: %X %X || CPI_Y: %X %X\n",x[1],x[0],y[1],y[0]);

	paw_write_reg(RESOLUTION_X_LOW_REG,x[0]);
	paw_write_reg(RESOLUTION_X_HIGH_REG,x[1]);
	paw_write_reg(RESOLUTION_Y_LOW_REG,y[0]);
	paw_write_reg(RESOLUTION_Y_HIGH_REG,y[1]);
	paw_write_reg(SET_RESOLUTION_REG,0x01);
}

void power_up_init_reg(void){   //function needs optimisation, store these values in a list in another file.
	void alternative_seq(void){
		printk("\nalt seq initiated\n");
		paw_write_reg(0x7F, 0x14);
		paw_write_reg(0x6C, 0x00);
		paw_write_reg(0x7F, 0x00);
	}
	paw_write_reg(0x7F, 0x07);
	paw_write_reg(0x40, 0x41);
	paw_write_reg(0x7F, 0x00);
	paw_write_reg(0x40, 0x80);
	paw_write_reg(0x7F, 0x0E);
	paw_write_reg(0x55, 0x0D);
	paw_write_reg(0x56, 0x1B);
	paw_write_reg(0x57, 0xE8);
	paw_write_reg(0x58, 0xD5);
	paw_write_reg(0x7F, 0x14);
	paw_write_reg(0x42, 0xBC);
	paw_write_reg(0x43, 0x74);
	paw_write_reg(0x4B, 0x20);
	paw_write_reg(0x4D, 0x00);
	paw_write_reg(0x53, 0x0E);
	paw_write_reg(0x7F, 0x05);
	paw_write_reg(0x44, 0x04);
	paw_write_reg(0x4D, 0x06);
	paw_write_reg(0x51, 0x40);
	paw_write_reg(0x53, 0x40);
	paw_write_reg(0x55, 0xCA);
	paw_write_reg(0x5A, 0xE8);
	paw_write_reg(0x5B, 0xEA);
	paw_write_reg(0x61, 0x31);
	paw_write_reg(0x62, 0x64);
	paw_write_reg(0x6D, 0xB8);
	paw_write_reg(0x6E, 0x0F);	//column-1
	paw_write_reg(0x70, 0x02);
	paw_write_reg(0x4A, 0x2A);
	paw_write_reg(0x60, 0x26);
	paw_write_reg(0x7F, 0x06);
	paw_write_reg(0x6D, 0x70);
	paw_write_reg(0x6E, 0x60);
	paw_write_reg(0x6F, 0x04);
	paw_write_reg(0x53, 0x02);
	paw_write_reg(0x55, 0x11);
	paw_write_reg(0x7A, 0x01);
	paw_write_reg(0x7D, 0x51);
	paw_write_reg(0x7F, 0x07);
	paw_write_reg(0x41, 0x10);
	paw_write_reg(0x42, 0x32);
	paw_write_reg(0x43, 0x00);
	paw_write_reg(0x7F, 0x08);
	paw_write_reg(0x71, 0x4F);
	paw_write_reg(0x7F, 0x09);
	paw_write_reg(0x62, 0x1F);
	paw_write_reg(0x63, 0x1F);
	paw_write_reg(0x65, 0x03);
	paw_write_reg(0x66, 0x03);
	paw_write_reg(0x67, 0x1F);
	paw_write_reg(0x68, 0x1F);
	paw_write_reg(0x69, 0x03);
	paw_write_reg(0x6A, 0x03);	
	paw_write_reg(0x6C, 0x1F);	//column-2
	paw_write_reg(0x6D, 0x1F);
	paw_write_reg(0x51, 0x04);
	paw_write_reg(0x53, 0x20);
	paw_write_reg(0x54, 0x20);
	paw_write_reg(0x71, 0x0C);
	paw_write_reg(0x72, 0x07);
	paw_write_reg(0x73, 0x07);
	paw_write_reg(0x7F, 0x0A);
	paw_write_reg(0x4A, 0x14);
	paw_write_reg(0x4C, 0x14);
	paw_write_reg(0x55, 0x19);
	paw_write_reg(0x7F, 0x14);
	paw_write_reg(0x4B, 0x30);
	paw_write_reg(0x4C, 0x03);
	paw_write_reg(0x61, 0x0B);
	paw_write_reg(0x62, 0x0A);
	paw_write_reg(0x63, 0x02);
	paw_write_reg(0x7F, 0x15);
	paw_write_reg(0x4C, 0x02);
	paw_write_reg(0x56, 0x02);
	paw_write_reg(0x41, 0x91);
	paw_write_reg(0x4D, 0x0A);
	paw_write_reg(0x7F, 0x0C);
	paw_write_reg(0x4A, 0x10);
	paw_write_reg(0x4B, 0x0C);
	paw_write_reg(0x4C, 0x40);
	paw_write_reg(0x41, 0x25);
	paw_write_reg(0x55, 0x18);
	paw_write_reg(0x56, 0x14);
	paw_write_reg(0x49, 0x0A);
	paw_write_reg(0x42, 0x00);
	paw_write_reg(0x43, 0x2D);
	paw_write_reg(0x44, 0x0C);
	paw_write_reg(0x54, 0x1A);
	paw_write_reg(0x5A, 0x0D);
	paw_write_reg(0x5F, 0x1E);
	paw_write_reg(0x5B, 0x05);
	paw_write_reg(0x5E, 0x0F);
	paw_write_reg(0x7F, 0x0D);
	paw_write_reg(0x48, 0xDD);
	paw_write_reg(0x4F, 0x03);
	paw_write_reg(0x52, 0x49);	//column-3
	paw_write_reg(0x51, 0x00);
	paw_write_reg(0x54, 0x5B);
	paw_write_reg(0x53, 0x00);
	paw_write_reg(0x56, 0x64);
	paw_write_reg(0x55, 0x00);
	paw_write_reg(0x58, 0xA5);
	paw_write_reg(0x57, 0x02);
	paw_write_reg(0x5A, 0x29);
	paw_write_reg(0x5B, 0x47);
	paw_write_reg(0x5C, 0x81);
	paw_write_reg(0x5D, 0x40);
	paw_write_reg(0x71, 0xDC);
	paw_write_reg(0x70, 0x07);
	paw_write_reg(0x73, 0x00);
	paw_write_reg(0x72, 0x08);
	paw_write_reg(0x75, 0xDC);
	paw_write_reg(0x74, 0x07);
	paw_write_reg(0x77, 0x00);
	paw_write_reg(0x76, 0x08);
	paw_write_reg(0x7F, 0x10);
	paw_write_reg(0x4C, 0xD0);
	paw_write_reg(0x7F, 0x00);
	paw_write_reg(0x4F, 0x63);
	paw_write_reg(0x4E, 0x00);
	paw_write_reg(0x52, 0x63);
	paw_write_reg(0x51, 0x00);
	paw_write_reg(0x54, 0x54);
	paw_write_reg(0x5A, 0x10);
	paw_write_reg(0x77, 0x4F);
	paw_write_reg(0x47, 0x01);
	paw_write_reg(0x5B, 0x40);
	paw_write_reg(0x64, 0x60);
	paw_write_reg(0x65, 0x06);
	paw_write_reg(0x66, 0x13);
	paw_write_reg(0x67, 0x0F);
	paw_write_reg(0x78, 0x01);
	paw_write_reg(0x79, 0x9C);
	paw_write_reg(0x40, 0x00);
	paw_write_reg(0x55, 0x02);
	paw_write_reg(0x23, 0x70);
	paw_write_reg(0x22, 0x01) ;
	k_usleep(1000);				//column-4
	uint8_t value[1];	
	bool flag=true;
	for(uint8_t i = 0; i<=60 ; i++){	//column-5
		int err;
			
		err=paw_read_regs(0x6C, value, 1);
		if(value[0] == 0x80){
			flag=false;
			break;
		}
		k_usleep(870);
	}
	flag ? alternative_seq() : NULL;   //alternative sequence define 
	paw_write_reg(0x22, 0x00);
	paw_write_reg(0x55, 0x00);
	paw_write_reg(0x7F, 0x07);
	paw_write_reg(0x40, 0x40);
	paw_write_reg(0x7F, 0x00);
	paw_write_reg(0x68, 0x01);	//column-6
	printk("Power-up sequence completed\n");   
}