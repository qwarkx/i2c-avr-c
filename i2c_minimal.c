#include <util/twi.h>
#include <avr/power.h>
#include <avr/io.h>

#include "i2c.h"
#include "../USART_serial/usart_2.h"

#define	inb(addr) (addr)
#define	inw(addr) (addr)
#define	outb(addr, data) addr = (data)
#define	outw(addr, data) addr = (data)

 

//#define F_SCL 100000UL // SCL frequency
//#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)

void i2c_init(uint16_t bitrate, uint8_t prescale)
{
	/* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
	//power_twi_enable();
	
	//TWSR = 0;
	TWSR = prescale;
	i2c_set_bitrate(bitrate);
	
	//TWCR = (1<<TWEN);
}

void i2c_set_bitrate(uint16_t bitrateKHz)
{
	unsigned char bitrate_div;
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR))
	TWSR &=~(1<<TWPS0);
	TWSR &=~(1<<TWPS1);

	//calculate bitrate division
	bitrate_div = ((F_CPU/1000L)/bitrateKHz);
	if(bitrate_div >= 16)
	bitrate_div = (bitrate_div-16)/2;
	outb(TWBR, bitrate_div);
	//printf("\ni2c_bitrate	--------------- %u\n",TWBR);
}

void i2c_start(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}

void i2c_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	while(TWCR & (1<<TWSTO));
}

void i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while (!(TWCR & (1<<TWINT)));
}

uint8_t i2c_read_ack(void)
{
	uint8_t data;
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while (!(TWCR & (1<<TWINT)));
	data=TWDR;
	return data;
}

uint8_t i2c_read_nack(void)
{
	uint8_t data;
	TWCR = _BV(TWINT) | _BV(TWEN);
	while (!(TWCR & _BV(TWINT)));
	data=TWDR;
	i2c_stop();	
	return data;
}

uint8_t i2c_status(void)
{
	uint8_t twsr;
	twsr=(TWSR & TW_STATUS_MASK) & 0xF8;
	
	return twsr;
}
/*
uint8_t i2c_readd(uint8_t ack)
{
	switch(ack){
		case 1:
			return i2c_read_ack(); 
		break;
		
		case 0:
			return i2c_read_nack(); 
		break;
	}
}*/

void i2c_write_reg(uint8_t add, uint8_t reg, uint8_t val) 
{
	i2c_start();
	i2c_write(add);			// I2C write direction
	i2c_write(reg);        // register selection
	i2c_write(val);        // value to write in register
	i2c_stop();
}

uint8_t i2c_read_reg8(uint8_t add, uint8_t reg)
{
	uint8_t val;
	
	//i2c_read_reg_to_buf(add, reg, &val, 1);
	 
	i2c_start();
	i2c_write(add|0x00);		
	i2c_write(reg);      
	i2c_start();
	i2c_write(add|0x01);
	val=i2c_read_nack();
	return val;
}
uint16_t i2c_read_reg16(uint8_t add, uint8_t reg)
{
	uint16_t val;
	
	//i2c_read_reg_to_buf(add, reg, &val, 2);
	
	i2c_start();
	i2c_write(add|0x00);
	i2c_write(reg);
	i2c_start();
	i2c_write(add|0x01);
	val=(i2c_read_ack()<<8)|i2c_read_nack();
	return val;
}

uint8_t i2c_bit_check(uint8_t add, uint8_t reg, uint8_t bit)
{
	uint8_t byte;

	i2c_start();
	i2c_write(add|0x00);
	i2c_write(reg);
	i2c_start();
	i2c_write(add|0x01);
	byte=i2c_read(0);
	return bit_read(byte,bit);
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size) {
	i2c_start();
	i2c_write(add|0x00); // I2C write direction
	i2c_write(reg);        // register selection
	i2c_start();
	i2c_write(add|0x01);  // I2C read direction
	uint8_t *b = buf;
	while (--size) *b++ = i2c_read(1); // acknowledge all but the final byte
	*b = i2c_read(0);
}









void i2c_get_adreses(uint8_t i2c_scann, uint8_t i2c_add_shifting){
	
	uint8_t device_buffer[127];
	
	if (i2c_scann)	{
		uint8_t nDevices;
		
		//if (i2c_add_shifting){address=address<<1;}
		
		for( uint8_t address = 1; address < 127; address++ )
		{
			uint8_t accesable=0x00;
			uint8_t status=0x00;
			i2c_start();
			i2c_write(i2c_add_shifting? (address<<1):address);
			accesable=i2c_status();
			printf("add: 0x%X  | status : 0x%X\n", address, accesable);
			_delay_ms(0);
			//status=i2c_status();
			i2c_stop();
			
			//printf("\nAdress data: 0x%x",address);
			//printf("    accesable or not: 0x%x",accesable);
			//printf("    status data: 0x%x\n",status);
			_delay_ms(0);
			
			if (accesable == TW_MT_SLA_ACK)
			{
				printf("I2C device found at address: 0x");
				if (address<16)
				printf("0");
				printf("%X",address);
				printf("  !\n");
				device_buffer[nDevices]=address;
				nDevices++;
			}
			else if (accesable==0x04)
			{
				printf("\nUnknow error at address: 0x");
				if (address<16)
				printf("0");
				printf("%X\n",address);
			}
		}
		if (nDevices == 0)
		printf("\nNo I2C devices found\n");
		else
		printf("\n");
		for (uint8_t i=0;i<nDevices;i++){
			printf("Devices found at: 0x%X\n",device_buffer[i]);
			// 			if (device_buffer[i]<16)
			// 			printf("0");
			// 			printf("%X\n",device_buffer[i]);
		}
		_delay_ms(68);
	}
}
