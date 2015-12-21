#ifndef I2C_H_
#define I2C_H_

#define I2C_SDA					PC4
#define I2C_SDA_PORT			PORTC
#define I2C_SDA_DDR				DDRC

#define I2C_SCL					PC5
#define I2C_SCL_PORT			PORTC
#define I2C_SCL_DDR				DDRC


#include <util/delay.h>
#include "../USART_serial/usart_2.h"


//#include <util/twi.h>

// Master
#define TW_START					0x08
#define TW_REP_START				0x10
// Master Transmitter
#define TW_MT_SLA_ACK				0x18
#define TW_MT_SLA_NACK				0x20
#define TW_MT_DATA_ACK				0x28
#define TW_MT_DATA_NACK				0x30
#define TW_MT_ARB_LOST				0x38
// Master Receiver
#define TW_MR_ARB_LOST				0x38
#define TW_MR_SLA_ACK				0x40
#define TW_MR_SLA_NACK				0x48
#define TW_MR_DATA_ACK				0x50
#define TW_MR_DATA_NACK				0x58
// Slave Transmitter
#define TW_ST_SLA_ACK				0xA8
#define TW_ST_ARB_LOST_SLA_ACK		0xB0
#define TW_ST_DATA_ACK				0xB8
#define TW_ST_DATA_NACK				0xC0
#define TW_ST_LAST_DATA				0xC8
// Slave Receiver
#define TW_SR_SLA_ACK				0x60
#define TW_SR_ARB_LOST_SLA_ACK		0x68
#define TW_SR_GCALL_ACK				0x70
#define TW_SR_ARB_LOST_GCALL_ACK	0x78
#define TW_SR_DATA_ACK				0x80
#define TW_SR_DATA_NACK				0x88
#define TW_SR_GCALL_DATA_ACK		0x90
#define TW_SR_GCALL_DATA_NACK		0x98
#define TW_SR_STOP					0xA0
// Misc
#define TW_NO_INFO					0xF8
#define TW_BUS_ERROR				0x00

// defines and constants
#define TWCR_CMD_MASK				0x0F
#define TWSR_STATUS_MASK			0xF8

// return values
#define I2C_OK						0x00
#define I2C_ERROR_NODEV				0x01

#define I2C_READ					0x01
#define I2C_WRITE					0x00



#define i2c_read(ack)  (uint8_t)((ack) ? i2c_read_ack() : i2c_read_nack())

void i2c_init(uint16_t bitrate, uint8_t prescale);
void i2c_set_bitrate(uint16_t bitrateKHz);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
uint8_t i2c_status(void);

uint8_t i2c_bit_check(uint8_t add, uint8_t reg, uint8_t bit);

void i2c_write_reg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_read_reg8(uint8_t add, uint8_t reg);
uint16_t i2c_read_reg16(uint8_t add, uint8_t reg);

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);

void i2c_get_adreses(uint8_t i2c_scann, uint8_t i2c_add_shifting);
#endif /* I2C_H_ */
