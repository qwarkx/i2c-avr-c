# i2c-avr-c
i2c communication with minimal functions in C

Good for starters and experts. This library can handle everithing.

Initialising the I2C :
bitrate       -bitrate of the communication;
prescale      -prescaler of the clk;
bitrate, prescale  i2c_init(400,0);    i2c_init(100,1);

void i2c_init(uint16_t bitrate, uint8_t prescale);


Setting and calculating the I2C bitrate:
bitrateKHz   -bitrate of the communication;
bitrateKHz  i2c_set_bitrate(400);    i2c_set_bitrate(100);

void i2c_set_bitrate(uint16_t bitrateKHz);


Starting the communication:
void i2c_start(void);


Stopping the communication:
void i2c_stop(void);


Writing to the bus:
void i2c_write(uint8_t data);


Reading a acknowledgement:
Afte reading the ack packet the I2C is not going to be stopd;

uint8_t i2c_read_ack(void);


Reading a not acknowledgement:
Afte reading the nack packet the I2C is stopped with the  void i2c_stop(void)  function;

uint8_t i2c_read_nack(void);


Reading I2C status register:
Returns the status regiszter data;

uint8_t i2c_status(void);


Automaticali writing 1 byte of data to register:
add   -address of the senzor;
reg   -register of the senzor that you want to read;
val   -value that is being writen;

void i2c_write_reg(uint8_t add, uint8_t reg, uint8_t val);


Automaticaly reading 1 bytes from the specified register:
add   -address of the senzor;
reg   -register of the senzor that you want to read;

uint8_t i2c_read_reg8(uint8_t add, uint8_t reg);


Automaticaly reading 2 bytes from the specified register:
add   -address of the senzor;
reg   -register of the senzor that you want to read;

uint16_t i2c_read_reg16(uint8_t add, uint8_t reg);


Cheking the bit in the readed register:
add   -address of the senzor;
reg   -register of the senzor that you want to read;
bit   -the bit that you want to check;

uint8_t i2c_bit_check(uint8_t add, uint8_t reg, uint8_t bit);


Reading register to buffer:
add   -address of the senzor;
reg   -register of the senzor that you want to read;
buf   -free space for data;
size  -size of the readed data;

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size);


Simple finction to write outh I2C addresses:
i2c_scann         - true if you want to scan for addresses;
i2c_add_shifting  -true if you want to shift the I2C addres;
The address shifting is a necesery must to communikate 
with the senzors or what ever you put on the I2C bus.

void i2c_get_adreses(uint8_t i2c_scann, uint8_t i2c_add_shifting);
