/**
 * \file tsys02d.h
 *
 * \brief TSYS02D Temperature sensor driver header file
 *
 * Copyright (c) 2016 Measurement Specialties. All rights reserved.
 *
 */

#ifndef TSYS02D_H_INCLUDED
#define TSYS02D_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Enums

enum tsys02d_i2c_master_mode {
	tsys02d_i2c_hold,
	tsys02d_i2c_no_hold
};

enum tsys02d_status {
	tsys02d_status_ok,
	tsys02d_status_no_i2c_acknowledge,
	tsys02d_status_i2c_transfer_error,
	tsys02d_status_crc_error
};

enum tsys02d_resolution {
	tsys02d_resolution_t_14b,
	tsys02d_resolution_t_13b,
	tsys02d_resolution_t_12b,
	tsys02d_resolution_t_11b
};

enum tsys02d_battery_status {
	tsys02d_battery_ok,
	tsys02d_battery_low
};

enum tsys02d_heater_status {
	tsys02d_heater_off,
	tsys02d_heater_on
};

// Functions

/**
 * \brief Configures the SERCOM I2C master to be used with the TSYS02D device.
 */
void tsys02d_init(void);

/**
 * \brief Check whether TSYS02D device is connected
 *
 * \return bool : status of TSYS02D
 *       - true : Device is present
 *       - false : Device is not acknowledging I2C address
  */
bool tsys02d_is_connected(void);

/**
 * \brief Reset the TSYS02D device
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_reset(void);

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
enum tsys02d_status tsys02d_read_serial_number(uint64_t *);

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
enum tsys02d_status tsys02d_set_resolution(enum tsys02d_resolution);

/**
 * \brief Set I2C master mode.
 *
 * \param[in] tsys02d_i2c_master_mode : I2C mode
 *
 */
void tsys02d_set_i2c_master_mode(enum tsys02d_i2c_master_mode);

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
enum tsys02d_status tsys02d_read_temperature(float *);

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
enum tsys02d_status tsys02d_get_battery_status(enum tsys02d_battery_status*);

/**
 * \brief Enable heater
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_enable_heater(void);

/**
 * \brief Disable heater
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_disable_heater(void);

/**
 * \brief Get heater status
 *
 * \param[in] tsys02d_heater_status* : Return heater status (above or below 2.5V)
 *	                    - tsys02d_heater_off,
 *                      - tsys02d_heater_on
 *
 * \return tsys02d_status : status of TSYS02D
 *       - tsys02d_status_ok : I2C transfer completed successfully
 *       - tsys02d_status_i2c_transfer_error : Problem with i2c transfer
 *       - tsys02d_status_no_i2c_acknowledge : I2C did not acknowledge
 */
enum tsys02d_status tsys02d_get_heater_status(enum tsys02d_heater_status*);

#endif /* TSYS02D_H_INCLUDED */