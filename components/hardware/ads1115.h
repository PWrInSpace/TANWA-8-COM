///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 27.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of ADS1115 driver. It is used to read data from ADS1115 analog 
/// to digital converter.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_ADS1115_H_
#define PWRINSPACE_ADS1115_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADS1115_MAX_VALUE 0x7fff //!< Maximum ADC value

/**
 * Gain amplifier
 */
typedef enum
{
    ADS1115_GAIN_6V144 = 0, //!< +-6.144V
    ADS1115_GAIN_4V096,     //!< +-4.096V
    ADS1115_GAIN_2V048,     //!< +-2.048V (default)
    ADS1115_GAIN_1V024,     //!< +-1.024V
    ADS1115_GAIN_0V512,     //!< +-0.512V
    ADS1115_GAIN_0V256,     //!< +-0.256V
    ADS1115_GAIN_0V256_2,   //!< +-0.256V (same as ADS1115_GAIN_0V256)
    ADS1115_GAIN_0V256_3,   //!< +-0.256V (same as ADS1115_GAIN_0V256)
} ads1115_gain_t;

/**
 * Gain amplifier values
 */
extern const float ads1115_gain_values[];

/**
 * Input multiplexer configuration (ADS1115 only)
 */
typedef enum
{
    ADS1115_MUX_0_1 = 0, //!< positive = AIN0, negative = AIN1 (default)
    ADS1115_MUX_0_3,     //!< positive = AIN0, negative = AIN3
    ADS1115_MUX_1_3,     //!< positive = AIN1, negative = AIN3
    ADS1115_MUX_2_3,     //!< positive = AIN2, negative = AIN3
    ADS1115_MUX_0_GND,   //!< positive = AIN0, negative = GND
    ADS1115_MUX_1_GND,   //!< positive = AIN1, negative = GND
    ADS1115_MUX_2_GND,   //!< positive = AIN2, negative = GND
    ADS1115_MUX_3_GND,   //!< positive = AIN3, negative = GND
} ads1115_mux_t;

/**
 * Data rate
 */
typedef enum
{
    ADS1115_DATA_RATE_8 = 0, //!< 8 samples per second
    ADS1115_DATA_RATE_16,    //!< 16 samples per second
    ADS1115_DATA_RATE_32,    //!< 32 samples per second
    ADS1115_DATA_RATE_64,    //!< 64 samples per second
    ADS1115_DATA_RATE_128,   //!< 128 samples per second (default)
    ADS1115_DATA_RATE_250,   //!< 250 samples per second
    ADS1115_DATA_RATE_475,   //!< 475 samples per second
    ADS1115_DATA_RATE_860    //!< 860 samples per second
} ads1115_data_rate_t;

/**
 * Operational mode
 */
typedef enum
{
    ADS1115_MODE_CONTINUOUS = 0, //!< Continuous conversion mode
    ADS1115_MODE_SINGLE_SHOT    //!< Power-down single-shot mode (default)
} ads1115_mode_t;

/**
 * Comparator mode (ADS1114 and ADS1115 only)
 */
typedef enum
{
    ADS1115_COMP_MODE_NORMAL = 0, //!< Traditional comparator with hysteresis (default)
    ADS1115_COMP_MODE_WINDOW      //!< Window comparator
} ads1115_comp_mode_t;

/**
 * Comparator polarity (ADS1114 and ADS1115 only)
 */
typedef enum
{
    ADS1115_COMP_POLARITY_LOW = 0, //!< Active low (default)
    ADS1115_COMP_POLARITY_HIGH     //!< Active high
} ads1115_comp_polarity_t;

/**
 * Comparator latch (ADS1114 and ADS1115 only)
 */
typedef enum
{
    ADS1115_COMP_LATCH_DISABLED = 0, //!< Non-latching comparator (default)
    ADS1115_COMP_LATCH_ENABLED       //!< Latching comparator
} ads1115_comp_latch_t;

/**
 * Comparator queue
 */
typedef enum
{
    ADS1115_COMP_QUEUE_1 = 0,   //!< Assert ALERT/RDY pin after one conversion
    ADS1115_COMP_QUEUE_2,       //!< Assert ALERT/RDY pin after two conversions
    ADS1115_COMP_QUEUE_4,       //!< Assert ALERT/RDY pin after four conversions
    ADS1115_COMP_QUEUE_DISABLED //!< Disable comparator (default)
} ads1115_comp_queue_t;

typedef enum {
    ADS1115_OK = 0,
    ADS1115_FAIL,
    ADS1115_INIT_ERR,
    ADS1115_READ_ERR,
    ADS1115_WRITE_ERR,
    ADS1115_NULL_ARG,
} ads1115_status_t;

// I2C access functions
typedef bool (*ads1115_I2C_write)(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);
typedef bool (*ads1115_I2C_read)(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);

typedef struct {
    ads1115_I2C_write _i2c_write;
    ads1115_I2C_read _i2c_read;
    uint8_t i2c_address;
} ads1115_struct_t;

/**
 * @brief Get device operational status
 *
 * @param ads1115 device instance pointer
 * @param[out] busy true when device performing conversion
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_is_busy(ads1115_struct_t *ads1115, bool *busy);

/**
 * @brief Begin a single conversion
 *
 * @note Only in single-shot mode.
 *
 * @param ads1115 device instance pointer
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_start_conversion(ads1115_struct_t *ads1115);

/**
 * @brief Read last conversion result
 *
 * @param ads1115 device instance pointer
 * @param[out] value last conversion result
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_value(ads1115_struct_t *ads1115, int16_t *value);

/**
 * @brief Read the programmable gain amplifier configuration
 *
 * @note Use ads1115_gain_values() for real voltage.
 *
 * @param ads1115 device instance pointer
 * @param[out] gain gain value
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_gain(ads1115_struct_t *ads1115, ads1115_gain_t *gain);

/**
 * @brief Configure the programmable gain amplifier
 *
 * @param ads1115 device instance pointer
 * @param[in] gain gain value
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @return `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_gain(ads1115_struct_t *ads1115, ads1115_gain_t gain);

/**
 * @brief Read the input multiplexer configuration
 *
 * @param ads1115 device instance pointer
 * @param[out] mux input multiplexer configuration
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_input_mux(ads1115_struct_t *ads1115, ads1115_mux_t *mux);

/**
 * @brief Configure the input multiplexer configuration
 *
 * @param ads1115 device instance pointer
 * @param[in] mux input multiplexer configuration
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_input_mux(ads1115_struct_t *ads1115, ads1115_mux_t mux);

/**
 * @brief Read the device operating mode
 *
 * @param ads1115 device instance pointer
 * @param[out] mode device operating mode
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_mode(ads1115_struct_t *ads1115, ads1115_mode_t *mode);

/**
 * @brief Set the device operating mode
 *
 * @param ads1115 device instance pointer
 * @param[in] mode device operating mode
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_mode(ads1115_struct_t *ads1115, ads1115_mode_t mode);

/**
 * @brief Read the data rate
 *
 * @param ads1115 device instance pointer
 * @param[out] rate data rate
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_data_rate(ads1115_struct_t *ads1115, ads1115_data_rate_t *rate);

/**
 * @brief Configure the data rate
 *
 * @param ads1115 device instance pointer
 * @param[in] rate data rate
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_data_rate(ads1115_struct_t *ads1115, ads1115_data_rate_t rate);

/**
 * @brief Get comparator mode
 *
 * @param ads1115 device instance pointer
 * @param[out] mode comparator mode
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_comp_mode(ads1115_struct_t *ads1115, ads1115_comp_mode_t *mode);

/**
 * @brief Set comparator mode
 *
 * @param ads1115 device instance pointer
 * @param[in] mode comparator mode
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_comp_mode(ads1115_struct_t *ads1115, ads1115_comp_mode_t mode);

/**
 * @brief Get polarity of the comparator output pin ALERT/RDY
 *
 * @param ads1115 device instance pointer
 * @param[out] polarity comparator output pin polarity
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_comp_polarity(ads1115_struct_t *ads1115, ads1115_comp_polarity_t *polarity);

/**
 * @brief Set polarity of the comparator output pin ALERT/RDY
 *
 * @param ads1115 device instance pointer
 * @param[in] polarity comparator output pin polarity
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_comp_polarity(ads1115_struct_t *ads1115, ads1115_comp_polarity_t polarity);

/**
 * @brief Get comparator output latch mode
 *
 * @param ads1115 device instance pointer
 * @param[out] latch comparator output latch mode
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_comp_latch(ads1115_struct_t *ads1115, ads1115_comp_latch_t *latch);

/**
 * @brief Set comparator output latch mode
 *
 * @param ads1115 device instance pointer
 * @param[in] latch comparator output latch mode
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_comp_latch(ads1115_struct_t *ads1115, ads1115_comp_latch_t latch);

/**
 * @brief Get comparator queue size
 *
 * @note Get number of the comparator conversions before pin ALERT/RDY assertion.
 *
 * @param ads1115 device instance pointer
 * @param[out] queue number of the comparator conversions
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_comp_queue(ads1115_struct_t *ads1115, ads1115_comp_queue_t *queue);

/**
 * @brief Set comparator queue size
 *
 * @note Set number of the comparator conversions before pin ALERT/RDY assertion or disable comparator.
 *
 * @param ads1115 device instance pointer
 * @param[in] queue number of the comparator conversions
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_comp_queue(ads1115_struct_t *ads1115, ads1115_comp_queue_t queue);

/**
 * @brief Get the lower threshold value used by comparator
 *
 * @param ads1115 device instance pointer
 * @param[out] th lower threshold value
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_comp_low_thresh(ads1115_struct_t *ads1115, int16_t *th);

/**
 * @brief Set the lower threshold value used by comparator
 *
 * @param ads1115 device instance pointer
 * @param[in] th lower threshold value
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_comp_low_thresh(ads1115_struct_t *ads1115, int16_t th);

/**
 * @brief Get the upper threshold value used by comparator
 *
 * @param ads1115 device instance pointer
 * @param[out] th upper threshold value
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_READ_ERR` on i2c read error
 */
ads1115_status_t ads1115_get_comp_high_thresh(ads1115_struct_t *ads1115, int16_t *th);

/**
 * @brief Set the upper threshold value used by comparator
 *
 * @param ads1115 device instance pointer
 * @param[in] th Upper threshold value
 * @return `ads1115_status_t`
 * @retval `ADS1115_OK` on success
 * @retval `ADS1115_FAIL` on error
 * @retval `ADS1115_WRITE_ERR` on i2c write error
 */
ads1115_status_t ads1115_set_comp_high_thresh(ads1115_struct_t *ads1115, int16_t th);

#ifdef __cplusplus
}
#endif

#endif /* PWRINSPACE_ADS1115_H_ */