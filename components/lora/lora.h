// Copyright 2023 PWr in Space, Krzysztof Gliwiński
#pragma once

#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK 0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK 0x40

#define PA_OUTPUT_RFO_PIN 0
#define PA_OUTPUT_PA_BOOST_PIN 1

#define TIMEOUT_RESET 100

/*
 * Register definitions
 */
#define REG_FIFO 0x00
#define REG_OP_MODE 0x01
#define REG_FRF_MSB 0x06
#define REG_FRF_MID 0x07
#define REG_FRF_LSB 0x08
#define REG_PA_CONFIG 0x09
#define REG_LNA 0x0c
#define REG_FIFO_ADDR_PTR 0x0d
#define REG_FIFO_TX_BASE_ADDR 0x0e
#define REG_FIFO_RX_BASE_ADDR 0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS 0x12
#define REG_RX_NB_BYTES 0x13
#define REG_PKT_SNR_VALUE 0x19
#define REG_PKT_RSSI_VALUE 0x1a
#define REG_MODEM_CONFIG_1 0x1d
#define REG_MODEM_CONFIG_2 0x1e
#define REG_PREAMBLE_MSB 0x20
#define REG_PREAMBLE_LSB 0x21
#define REG_PAYLOAD_LENGTH 0x22
#define REG_MODEM_CONFIG_3 0x26
#define REG_RSSI_WIDEBAND 0x2c
#define REG_DETECTION_OPTIMIZE 0x31
#define REG_DETECTION_THRESHOLD 0x37
#define REG_SYNC_WORD 0x39
#define REG_DIO_MAPPING_1 0x40
#define REG_VERSION 0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE 0x80
#define MODE_SLEEP 0x00
#define MODE_STDBY 0x01
#define MODE_TX 0x03
#define MODE_RX_CONTINUOUS 0x05
#define MODE_RX_SINGLE 0x06

/*
 * PA configuration
 */
#define PA_BOOST 0x80

/*!
  \brief Lora functions return values enum
*/
typedef enum {
  LORA_OK = 0,
  LORA_INIT_ERR,
  LORA_WRITE_ERR,
  LORA_TRANSMIT_ERR,
  LORA_RECEIVE_ERR,
  LORA_CONFIG_ERR
} lora_err_t;

typedef enum {
  LORA_GPIO_MODE_DISABLE = 0,
  LORA_GPIO_MODE_INPUT,
  LORA_GPIO_MODE_OUTPUT
} lora_gpio_mode_t;


typedef enum {
  LORA_IRQ_D0_RXDONE = 0x00,
  LORA_IRQ_D0_TXDONE = 0x01,
  LORA_IRQ_D0_CADDONE = 0x10,
} lora_dio0_mapping_t;

/*!
 * \brief Enum for LoRa bandwith in Hz
 */
typedef enum {
  LORA_BW_7_8_kHz = 0,
  LORA_BW_10_4_kHz,
  LORA_BW_15_6_kHz,
  LORA_BW_20_8_kHz,
  LORA_BW_31_25_kHz,
  LORA_BW_41_7_kHz,
  LORA_BW_62_5_kHz,
  LORA_BW_125_kHz,
  LORA_BW_250_kHz,
  LORA_BW_500_kHz,
} lora_bandwith_t;

/*!
 * \brief Enum for LoRa spreading factor in chips / symbol
 *        Used in the lora_set_spreading_factor method
 */
typedef enum {
  LORA_SF_64_CoS = 6,
  LORA_SF_128_CoS,
  LORA_SF_256_CoS,
  LORA_SF_512_CoS,
  LORA_SF_1024_CoS,
  LORA_SF_2048_CoS,
  LORA_SF_4096_CoS
} lora_spreading_factor_t;

/*!
 * \brief Enum for LoRa TX Power
 */
// TODO(Glibus): check if it is ok
typedef enum {
  LORA_TX_POWER_14_dBm = 2,
  LORA_TX_POWER_20_dBm = 17
} lora_tx_power_t;

typedef bool (*lora_SPI_transmit)(uint8_t _in[2], uint8_t _val[2]);
typedef void (*lora_delay)(uint32_t _ms);
typedef bool (*lora_GPIO_set_level)(uint8_t _gpio_num, uint8_t _level);
typedef void (*lora_log)(const char *info);

typedef struct {
  lora_SPI_transmit _spi_transmit;
  lora_delay _delay;
  lora_GPIO_set_level _gpio_set_level;
  lora_log log;
  uint8_t rst_gpio_num;
  uint8_t cs_gpio_num;
  uint8_t d0_gpio_num;
  int16_t implicit_header;
  int32_t frequency;
} lora_struct_t;

/*!
 * \brief Perform hardware initialization.
 */
lora_err_t lora_init(lora_struct_t *lora);

/*!
 * \brief Create default config and set parameters
 */
lora_err_t lora_default_config(lora_struct_t *lora);

/*!
 * \brief Write a value to a register.
 * \param reg Register index.
 * \param val Value to write.
 * \return lora_err_t value
 */
lora_err_t lora_write_reg(lora_struct_t *lora, int16_t reg, int16_t val);

/*!
 * \brief Read the current value of a register.
 * \param reg Register index.
 * \returns Value of the register.
 */
uint8_t lora_read_reg(lora_struct_t *lora, int16_t reg);

/*!
 * \brief Perform physical reset on the Lora chip
 * \throw Assert if _gpio_set_level fails
 */
void lora_reset(lora_struct_t *lora);

/*!
 * \brief Configure explicit header mode.
 * Packet size will be included in the frame.
 */
lora_err_t lora_explicit_header_mode(lora_struct_t *lora);

/*!
 * \brief Configure implicit header mode.
 * All packets will have a predefined size.
 * \param size Size of the packets.
 */
lora_err_t lora_implicit_header_mode(lora_struct_t *lora, int16_t size);

/*!
 * \brief Sets the radio transceiver in idle mode.
 * \note Must be used to change registers and access the FIFO.
 */
lora_err_t lora_idle(lora_struct_t *lora);

/*!
 * \brief Sets the radio transceiver in sleep mode.
 * \note Low power consumption and FIFO is lost.
 */
lora_err_t lora_sleep(lora_struct_t *lora);

/*!
 * \brief Sets the radio transceiver in receive mode.
 * \note Incoming packets will be received.
 */
lora_err_t lora_set_receive_mode(lora_struct_t *lora);

/*!
 * \brief Configure power level for transmission
 * \param level 2 or 17, from least to most power
 */
lora_err_t lora_set_tx_power(lora_struct_t *lora, lora_tx_power_t level);

/*!
 * \brief Set carrier frequency.
 * \param frequency Frequency in Hz
 */
lora_err_t lora_set_frequency(lora_struct_t *lora, int32_t frequency);

/*!
 * \brief Get the frequency set on LoRa
 * \returns LoRa frequency in Hz
 */
int32_t lora_get_frequency(lora_struct_t *lora);

/*!
 * \brief Set spreading factor.
 * \param sf 6-12, Spreading factor to use.
 * \returns LORA_OK if operation successful, LORA_CONFIG_ERR otherwise
 */
lora_err_t lora_set_spreading_factor(lora_struct_t *lora,
                                     lora_spreading_factor_t sf);

/*!
 * \brief Set bandwidth (bit rate)
 * \param sbw Bandwidth in Hz (up to 500000)
 * \note When using low frequency (below 169 MHz), only sf up to
 *       125 kHz is supported
 */
lora_err_t lora_set_bandwidth(lora_struct_t *lora, lora_bandwith_t sbw);

/*!
 * \brief Set coding rate
 * \param denominator 5-8, Denominator for the coding rate 4/x
 */
lora_err_t lora_set_coding_rate(lora_struct_t *lora, int16_t denominator);

/*!
 * \brief Set the size of preamble.
 * \param length Preamble length in symbols.
 */
lora_err_t lora_set_preamble_length(lora_struct_t *lora, int32_t length);

/*!
 * \brief Change radio sync word.
 * \param sw New sync word to use.
 */
lora_err_t lora_set_sync_word(lora_struct_t *lora, int16_t sw);

/*!
 * \brief Enable appending/verifying packet CRC.
 */
lora_err_t lora_enable_crc(lora_struct_t *lora);

/*!
 * \brief Disable appending/verifying packet CRC.
 */
lora_err_t lora_disable_crc(lora_struct_t *lora);

/*!
 * \brief Fills the REG_FIFO buffer with desired values, and the
 * REG_PAYLOAD_LENGTH with buffer size
 * \param buf - array of 8bit values
 * \param size - sizeof(buf)
 * \returns LORA_OK if writing to buffers is ok,
 * LORA_WRITE_ERR otherwise
 */
lora_err_t lora_fill_fifo_buf_to_send(lora_struct_t *lora, uint8_t *buf,
                                      int16_t size);

/*!
 * \brief Writes the REG_OP_MODE to mode TX
 * \returns LORA_OK if all goes good, LORA_WRITE_ERR otherwise
 */
lora_err_t lora_start_transmission(lora_struct_t *lora);

/*!
 * \brief Checks whether the transmission has finished
 * \returns True if finished, false otherwise
 */
bool lora_check_tx_done(lora_struct_t *lora);

/*!
 * \brief Writes the REG_IRQ_FLAGS buffer with IRQ_TX_DONE_MASK
 * \returns LORA_OK :D - LORA_WRITE_ERR :C
 */
lora_err_t lora_write_irq_flags(lora_struct_t *lora);

/*!
 * \brief Send a packet. DOES NOT go into receive mode automatically afterwards.
 * \param buf Data to be sent
 * \param size Size of data.
 */
lora_err_t lora_send_packet(lora_struct_t *lora, uint8_t *buf, int16_t size);

/*!
 * \brief Read a received packet.
 * \param buf Buffer for the data.
 * \param size Available size in buffer (bytes).
 * \returns Number of bytes received (zero if no packet available).
 */
int16_t lora_receive_packet(lora_struct_t *lora, uint8_t *buf, int16_t size);

/*!
 * \returns non-zero if there is data to read (packet received).
 */
lora_err_t lora_received(lora_struct_t *lora);

/*!
 * \returns last packet's RSSI.
 */
int16_t lora_packet_rssi(lora_struct_t *lora);

/*!
 * \returns last packet's SNR (signal to noise ratio).
 */
float lora_packet_snr(lora_struct_t *lora);

lora_err_t lora_map_d0_interrupt(lora_struct_t *lora, lora_dio0_mapping_t mode);

/*!
 * \brief Shutdown hardware.
 */
void lora_close(lora_struct_t *lora);

/// \brief Not supported
int16_t lora_initialized(lora_struct_t *lora);

/// \brief Dump registers :D
void lora_dump_registers(lora_struct_t *lora);