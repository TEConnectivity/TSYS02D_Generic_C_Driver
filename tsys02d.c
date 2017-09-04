/**
 * \file tsys02d.c
 *
 * \brief TSYS02D Temperature sensor driver source file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 * For details on programming, refer to TSYS02D datasheet :
 * http://www.meas-spec.com/downloads/Digital_Sensor_TSYS02D.pdf
 *
 */

#include "tsys02d.h"

 /**
  * The header "i2c.h" has to be implemented for your own platform to 
  * conform the following protocol :
  *
  * enum i2c_transfer_direction {
  * 	I2C_TRANSFER_WRITE = 0,
  * 	I2C_TRANSFER_READ  = 1,
  * };
  * 
  * enum status_code {
  * 	STATUS_OK           = 0x00,
  * 	STATUS_ERR_OVERFLOW	= 0x01,
  *		STATUS_ERR_TIMEOUT  = 0x02,
  * };
  * 
  * struct i2c_master_packet {
  * 	// Address to slave device
  * 	uint16_t address;
  * 	// Length of data array
  * 	uint16_t data_length;
  * 	// Data array containing all data to be transferred
  * 	uint8_t *data;
  * };
  * 
  * void i2c_master_init(void);
  * enum status_code i2c_master_read_packet_wait(struct i2c_master_packet *const packet);
  * enum status_code i2c_master_write_packet_wait(struct i2c_master_packet *const packet);
  * enum status_code i2c_master_write_packet_wait_no_stop(struct i2c_master_packet *const packet);
  */
#include "i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// TSYS02D device address
#define TSYS02D_ADDR										0x40 //0b1000000

// TSYS02D device commands
#define TSYS02D_RESET_COMMAND								0xFE
#define TSYS02D_READ_TEMPERATURE_W_HOLD_COMMAND				0xE3
#define TSYS02D_READ_TEMPERATURE_WO_HOLD_COMMAND			0xF3
#define TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND			0xFA0F
#define TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND				0xFCC9
#define TSYS02D_WRITE_USER_REG_COMMAND						0xE6
#define TSYS02D_READ_USER_REG_COMMAND						0xE7

// Coefficients for temperature computation
#define COEFF_MUL											(175.72)
#define COEFF_ADD											(-46.85)

// Conversion timings - Spec values
#define TSYS02D_CONVERSION_TIME_T_14b						50000
#define TSYS02D_CONVERSION_TIME_T_13b						25000
#define TSYS02D_CONVERSION_TIME_T_12b						13000
#define TSYS02D_CONVERSION_TIME_T_11b						7000

#define TSYS02D_RESET_TIME									15     // ms value

// TSYS02D User Register masks and bit position
#define TSYS02D_USER_REG_RESOLUTION_MASK					0x81
#define TSYS02D_USER_REG_END_OF_BATTERY_MASK				0x40
#define TSYS02D_USER_REG_DISABLE_OTP_RELOAD_MASK			0x2
#define TSYS02D_USER_REG_RESERVED_MASK                      (~(		TSYS02D_USER_REG_RESOLUTION_MASK			\
																|	TSYS02D_USER_REG_END_OF_BATTERY_MASK		\
																|	TSYS02D_USER_REG_DISABLE_OTP_RELOAD_MASK ))
// HTU User Register values
// Resolution
#define TSYS02D_USER_REG_RESOLUTION_T_14b					0x00
#define TSYS02D_USER_REG_RESOLUTION_T_13b					0x80
#define TSYS02D_USER_REG_RESOLUTION_T_12b					0x01
#define TSYS02D_USER_REG_RESOLUTION_T_11b					0x81
// End of battery status
#define TSYS02D_USER_REG_END_OF_BATTERY_VDD_ABOVE_2_25V		0x00
#define TSYS02D_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V		0x40

static struct i2c_master_module dev_inst_tsys02d;
uint32_t tsys02d_conversion_time = TSYS02D_CONVERSION_TIME_T_14b;
enum tsys02d_i2c_master_mode i2c_master_mode;

// Static functions
static enum tsys02d_status tsys02d_write_command(uint8_t);
static enum tsys02d_status tsys02d_write_command_no_stop(uint8_t);
static enum tsys02d_status tsys02d_read_user_register(uint8_t *);
static enum tsys02d_status tsys02d_write_user_register(uint8_t );
static enum tsys02d_status tsys02d_conversion_and_read_adc(uint16_t *);
static enum tsys02d_status tsys02d_crc_check( uint16_t, uint8_t);

/**
 * \brief Configures the SERCOM I2C master to be used with the TSYS02D device.
 */
void tsys02d_init(void)
{
	i2c_master_mode = TSYS02D_i2c_no_hold;
	
	/* Initialize and enable device with config. */
	i2c_master_init();
}

/**
 * \brief Check whether TSYS02D device is connected
 *
 * \return bool : status of TSYS02D
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool tsys02d_is_connected(void)
{
	enum status_code i2c_status;
	
	struct i2c_master_packet transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 0,
		.data        = NULL,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status != STATUS_OK)
		return false;
	
	return true;
}
	
/**
 * \brief Reset the TSYS02D device
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status  tsys02d_reset(void)
{
	enum tsys02d_status status;

	status =  tsys02d_write_command(TSYS02D_RESET_COMMAND);
	if( status != tsys02d_status_ok )
		return status;
		
	tsys02d_conversion_time = TSYS02D_CONVERSION_TIME_T_14b;
	delay_ms(TSYS02D_RESET_TIME);

	return tsys02d_status_ok;
}

/**
 * \brief Set I2C master mode.
 *
 * \param[in] tsys02d_i2c_master_mode : I2C mode
 *
 */
void tsys02d_set_i2c_master_mode(enum tsys02d_i2c_master_mode mode)
{
	i2c_master_mode = mode;
	return;
}

/**
 * \brief Writes the TSYS02D 8-bits command with the value passed
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_write_command( uint8_t cmd)
{
	enum status_code i2c_status;
	uint8_t data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 1,
		.data        = data,
	};
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;
	
	return tsys02d_status_ok;
}

/**
 * \brief Writes the TSYS02D 8-bits command with the value passed
 *        Do not send the STOP bit in the I2C transfer
 *
 * \param[in] uint8_t : Command value to be written.
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_write_command_no_stop( uint8_t cmd)
{
	enum status_code i2c_status;
	uint8_t data[1];
		
	data[0] = cmd;
		
	struct i2c_master_packet transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 1,
		.data        = data,
	};
	
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait_no_stop(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;
	
	return tsys02d_status_ok;
}

/**
 * \brief Check CRC
 *
 * \param[in] uint16_t : variable on which to check CRC
 * \param[in] uint8_t : CRC value
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : CRC check is OK
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d_crc_check( uint16_t value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeroes as specified in spec
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc )
		return 	tsys02d_status_ok;
	else
		return tsys02d_status_crc_error;
}

/**
 * \brief Reads the TSYS02D user register.
 *
 * \param[out] uint8_t* : Storage of user register value
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_read_user_register(uint8_t *value)
{
	enum tsys02d_status status;
	enum status_code i2c_status;
	uint8_t buffer[1];
	buffer[0] = 0;

	/* Read data */
	struct i2c_master_packet read_transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 1,
		.data        = buffer,
	};
	
	// Send the Read Register Command
	status = tsys02d_write_command(TSYS02D_READ_USER_REG_COMMAND);
	if( status != tsys02d_status_ok )
		return status;
	
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;

	*value = buffer[0];
	
	return tsys02d_status_ok;
}

/**
 * \brief Writes the tsys02d user register with value
 *        Will read and keep the unreserved bits of the register
 *
 * \param[in] uint8_t : Register value to be set.
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_write_user_register(uint8_t value)
{
	enum tsys02d_status status;
	enum status_code i2c_status;
	uint8_t reg;
	uint8_t data[2];
	
	status = tsys02d_read_user_register(&reg);
	if( status != tsys02d_status_ok )
		return status;
	
	// Clear bits of reg that are not reserved
	reg &= TSYS02D_USER_REG_RESERVED_MASK;
	// Set bits from value that are not reserved
	reg |= (value & ~TSYS02D_USER_REG_RESERVED_MASK);
	
	data[0] = TSYS02D_WRITE_USER_REG_COMMAND;
	data[1] = reg;

	struct i2c_master_packet transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 2,
		.data        = data,
	};
	
	/* Do the transfer */
	i2c_status = i2c_master_write_packet_wait(&transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;
		
	return tsys02d_status_ok;
}

/**
 * \brief Reads the temperature ADC value
 *
 * \param[out] uint16_t* : Temperature ADC value.
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d_conversion_and_read_adc(uint16_t *adc)
{
	enum tsys02d_status status = tsys02d_status_ok;
	enum status_code i2c_status;
	uint16_t _adc;
	uint8_t buffer[3];
	uint8_t crc;
	
	buffer[0] = 0;
	buffer[1] = 0;
	buffer[2] = 0;

	/* Read data */
    struct i2c_master_packet read_transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 3,
		.data        = buffer,
	};
	
	if( i2c_master_mode == tsys02d_i2c_hold) {
		status = tsys02d_write_command_no_stop(TSYS02D_READ_TEMPERATURE_W_HOLD_COMMAND);
	}
	else {
		status = tsys02d_write_command(TSYS02D_READ_TEMPERATURE_WO_HOLD_COMMAND);
		// 100ms delay - 50ms does not work ...
		delay_ms(tsys02d_conversion_time/1000);
	}
	if( status != tsys02d_status_ok)
		return status;
		
    i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;

	_adc = (buffer[0] << 8) | buffer[1];
	crc = buffer[2];
	
	// compute CRC
	status = tsys02d_crc_check(_adc,crc);
	if( status != tsys02d_status_ok)
		return status;
	
	*adc = _adc;

	return status;
}

/**
 * \brief Reads the tsys02d serial number.
 *
 * \param[out] uint64_t* : Serial number
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d_read_serial_number(uint64_t * serial_number)
{
	enum tsys02d_status status;
	enum status_code i2c_status;
	uint8_t cmd_data[2];
	uint8_t rcv_data[14];
	uint8_t i;
		
	struct i2c_master_packet transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 2,
		.data        = cmd_data,
	};
	struct i2c_master_packet read_transfer = {
		.address     = TSYS02D_ADDR,
		.data_length = 8,
		.data        = rcv_data,
	};

	// Read the first 8 bytes
	cmd_data[0] = (TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND>>8)&0xFF;
	cmd_data[1] = TSYS02D_READ_SERIAL_FIRST_8BYTES_COMMAND&0xFF;
		
	/* Do the transfer */
	i2c_master_write_packet_wait_no_stop(&transfer);
    i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;

	// Read the last 6 bytes
	cmd_data[0] = (TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND>>8)&0xFF;
	cmd_data[1] = TSYS02D_READ_SERIAL_LAST_6BYTES_COMMAND&0xFF;
	
	read_transfer.data = &rcv_data[8];
	read_transfer.data_length = 6;

	/* Do the transfer */
	i2c_master_write_packet_wait_no_stop(&transfer);
	i2c_status = i2c_master_read_packet_wait(&read_transfer);
	if( i2c_status == STATUS_ERR_OVERFLOW )
		return tsys02d_status_no_i2c_acknowledge;
	if( i2c_status != STATUS_OK)
		return tsys02d_status_i2c_transfer_error;
	
	for( i=0 ; i<8 ; i+=2 ) {
		status = tsys02d_crc_check(rcv_data[i],rcv_data[i+1]);
		if( status != tsys02d_status_ok )
			return status;
	}
	for( i=8 ; i<14 ; i+=3 ) {
		status = tsys02d_crc_check( ((rcv_data[i]<<8)|(rcv_data[i+1])),rcv_data[i+2] );
		if( status != tsys02d_status_ok )
			return status;
	}
	
	*serial_number =	((uint64_t)rcv_data[0]<<56) | ((uint64_t)rcv_data[2]<<48) | ((uint64_t)rcv_data[4]<<40) | ((uint64_t)rcv_data[6]<<32)
					|	((uint64_t)rcv_data[8]<<24) | ((uint64_t)rcv_data[9]<<16) | ((uint64_t)rcv_data[11]<<8) | ((uint64_t)rcv_data[12]<<0);
	
	return tsys02d_status_ok;
	
}

/**
 * \brief Set temperature ADC resolution.
 *
 * \param[in] tsys02d_resolution : Resolution requested
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d_set_resolution(enum tsys02d_resolution res)
{
	enum tsys02d_status status;
	uint8_t reg_value, tmp=0;
	uint32_t conversion_time = TSYS02D_CONVERSION_TIME_T_14b;
	
	if( res == tsys02d_resolution_t_14b) {
		tmp = TSYS02D_USER_REG_RESOLUTION_T_14b;
		conversion_time = TSYS02D_CONVERSION_TIME_T_14b;
	}
	else if( res == tsys02d_resolution_t_13b) {
		tmp = TSYS02D_USER_REG_RESOLUTION_T_13b;
		conversion_time = TSYS02D_CONVERSION_TIME_T_13b;
	}
	else if( res == tsys02d_resolution_t_12b) {
		tmp = TSYS02D_USER_REG_RESOLUTION_T_12b;
		conversion_time = TSYS02D_CONVERSION_TIME_T_12b;
	}
	else if( res == tsys02d_resolution_t_11b) {
		tmp = TSYS02D_USER_REG_RESOLUTION_T_11b;
		conversion_time = TSYS02D_CONVERSION_TIME_T_11b;
	}
		
	status = tsys02d_read_user_register(&reg_value);
	if( status != tsys02d_status_ok )
		return status;
	
	// Clear the resolution bits
	reg_value &= ~TSYS02D_USER_REG_RESOLUTION_MASK;
	reg_value |= tmp & TSYS02D_USER_REG_RESOLUTION_MASK;
	
	tsys02d_conversion_time = conversion_time;

	status = tsys02d_write_user_register(reg_value);

	// Re-initialize I2C to account for conversion time updates
	tsys02d_init();
	
	return status;
}

/**
 * \brief Reads the temperature ADC value and compute the degree Celsius one.
 *
 * \param[out] float* : Celsius Degree temperature value
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 *       - tsys02d_status_crc_error : CRC check error
 */
enum tsys02d_status tsys02d_read_temperature( float *temperature)
{
	enum tsys02d_status	status;
	uint16_t adc;
	
	status = tsys02d_conversion_and_read_adc( &adc);
	if( status != tsys02d_status_ok)
		return status;
	
	// Perform conversion function
	*temperature = adc * COEFF_MUL / (1<<16) + COEFF_ADD;
	
	return status;
}

/**
 * \brief Provide battery status
 *
 * \param[out] tsys02d_battery_status* : Battery status
 *                      - tsys02d_battery_ok,
 *                      - tsys02d_battery_low
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_get_battery_status(enum tsys02d_battery_status *bat)
{
	enum tsys02d_status	status;
	uint8_t reg_value;

	status = tsys02d_read_user_register(&reg_value);
	if( status != tsys02d_status_ok)
		return status;

	if( reg_value & TSYS02D_USER_REG_END_OF_BATTERY_VDD_BELOW_2_25V )
		*bat = tsys02d_battery_low;
	else
		*bat = tsys02d_battery_ok;
		
	return status;
}

#ifdef __cplusplus
}
#endif
