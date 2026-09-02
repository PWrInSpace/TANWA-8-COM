// Copyright 2023 PWr in Space, Krzysztof Gliwiński
#include "lora.h"

#include "esp_timer.h"

#define TAG "LORA"

#define LORA_TX_TIMEOUT_US 200000LL

/* Fallback externs to MCU helpers in case the lora struct doesn't have pointers set */
extern bool _lora_spi_transmit(uint8_t _in[2], uint8_t _val[2]);
extern bool _lora_gpio_set_level(uint8_t gpio, uint8_t level);
extern void _lora_delay_ms(uint32_t ms);
extern void _lora_log(const char *info);

lora_err_t lora_init(lora_struct_t *lora) {
  lora_err_t ret = LORA_OK;

  /*
   * Perform hardware reset.
   */
  lora_reset(lora);

  /*
   * Check version.
   */
  uint8_t version;
  uint8_t i = 0;
  while (i++ < TIMEOUT_RESET) {
    version = lora_read_reg(lora, REG_VERSION);
    if (version == 0x12) break;
    lora->_delay(2);
  }
  assert(i <= TIMEOUT_RESET + 1);  // at the end of the loop above, the max
                                   // value i can reach is TIMEOUT_RESET + 1

  ret |= lora_default_config(lora);

  return ret;
}

lora_err_t lora_default_config(lora_struct_t *lora) {
  lora_err_t ret = LORA_OK;
  lora_sleep(lora);
  ret |= lora_write_reg(lora, REG_FIFO_RX_BASE_ADDR, 0);
  ret |= lora_write_reg(lora, REG_FIFO_TX_BASE_ADDR, 0);
  ret |= lora_write_reg(lora, REG_LNA, lora_read_reg(lora, REG_LNA) | 0x03);
  ret |= lora_write_reg(lora, REG_MODEM_CONFIG_3, 0x04);
  lora_set_tx_power(lora, 17);

  lora_idle(lora);
  return ret;
}

  lora_err_t lora_write_reg(lora_struct_t *lora, int16_t reg, int16_t val) {
  uint8_t out[2] = {0x80 | reg, val};
  uint8_t in[2];

    if (lora == NULL) {
      ESP_LOGE(TAG, "lora struct is NULL!");
      return LORA_WRITE_ERR;
    }

    ESP_LOGD(TAG, "lora_write_reg called: lora=%p, _spi_transmit=%p, reg=0x%02x, val=0x%02x",
             (void *)lora, (void *)lora->_spi_transmit, (int)reg, (int)val);

    lora_SPI_transmit spi = lora->_spi_transmit ? lora->_spi_transmit : _lora_spi_transmit;
    if (spi == NULL) {
      if (lora->log) lora->log("ERROR: SPI transmit function is not set (no fallback)!");
      ESP_LOGE(TAG, "SPI transmit function is not set (no fallback)!");
      return LORA_WRITE_ERR;
    }

    return spi(in, out) == 1 ? LORA_OK : LORA_WRITE_ERR;
}

uint8_t lora_read_reg(lora_struct_t *lora, int16_t reg) {
  uint8_t out[2] = {reg, 0xff};
  uint8_t in[2];

  if (lora == NULL) {
    ESP_LOGE(TAG, "lora struct is NULL!");
    return 0x00;
  }

  ESP_LOGD(TAG, "lora_read_reg called: lora=%p, _spi_transmit=%p, reg=0x%02x",
           (void *)lora, (void *)lora->_spi_transmit, (int)reg);

  lora_SPI_transmit spi = lora->_spi_transmit ? lora->_spi_transmit : _lora_spi_transmit;
  if (spi == NULL) {
    if (lora->log) lora->log("ERROR: SPI transmit function is not set (no fallback)!");
    ESP_LOGE(TAG, "SPI transmit function is not set (no fallback)!");
    return 0x00;
  }

  spi(in, out);
  return in[1];
}

void lora_reset(lora_struct_t *lora) {
  if (lora == NULL) {
    ESP_LOGE(TAG, "lora struct is NULL! cannot reset");
    return;
  }

  lora_GPIO_set_level gpio = lora->_gpio_set_level ? lora->_gpio_set_level : _lora_gpio_set_level;
  lora_delay delay = lora->_delay ? lora->_delay : _lora_delay_ms;
  lora_log logger = lora->log ? lora->log : _lora_log;

  if (gpio == NULL) {
    if (logger) logger("ERROR: gpio_set_level function is not set (no fallback)!");
    ESP_LOGE(TAG, "gpio_set_level function is not set (no fallback)!");
    return;
  }

  gpio(lora->rst_gpio_num, 0);
  if (delay) delay(1);
  gpio(lora->rst_gpio_num, 1);
  if (delay) delay(10);
}

lora_err_t lora_explicit_header_mode(lora_struct_t *lora) {
  lora_err_t ret = LORA_OK;
  lora->implicit_header = 0;
  ret |= lora_write_reg(lora, REG_MODEM_CONFIG_1,
                        lora_read_reg(lora, REG_MODEM_CONFIG_1) & 0xfe);
  return ret;
}

lora_err_t lora_implicit_header_mode(lora_struct_t *lora, int16_t size) {
  lora_err_t ret = LORA_OK;
  lora->implicit_header = 1;
  ret |= lora_write_reg(lora, REG_MODEM_CONFIG_1,
                        lora_read_reg(lora, REG_MODEM_CONFIG_1) | 0x01);
  ret |= lora_write_reg(lora, REG_PAYLOAD_LENGTH, size);
  return ret;
}

lora_err_t lora_idle(lora_struct_t *lora) {
  lora_err_t ret = LORA_OK;
  ret |= lora_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
  return ret;
}

lora_err_t lora_sleep(lora_struct_t *lora) {
  lora_err_t ret = LORA_OK;
  ret |= lora_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
  return ret;
}

lora_err_t lora_set_receive_mode(lora_struct_t *lora) {
  lora_err_t ret = LORA_OK;
  ret |= lora_write_reg(lora, REG_OP_MODE,
                        MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
  return ret;
}

lora_err_t lora_set_tx_power(lora_struct_t *lora, lora_tx_power_t level) {
  lora_err_t ret = LORA_OK;
  // RF9x module uses PA_BOOST pin
  int16_t new_level = (int16_t)level;
  ret |= lora_write_reg(lora, REG_PA_CONFIG, PA_BOOST | (new_level - 2));
  return ret;
}

lora_err_t lora_set_frequency(lora_struct_t *lora, int32_t frequency) {
  lora_err_t ret = LORA_OK;
  lora->frequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

  ret |= lora_write_reg(lora, REG_FRF_MSB, (uint8_t)(frf >> 16));
  ret |= lora_write_reg(lora, REG_FRF_MID, (uint8_t)(frf >> 8));
  ret |= lora_write_reg(lora, REG_FRF_LSB, (uint8_t)(frf >> 0));
  return ret;
}

int32_t lora_get_frequency(lora_struct_t *lora) { return lora->frequency; }

lora_err_t lora_set_spreading_factor(lora_struct_t *lora,
                                     lora_spreading_factor_t sf) {
  lora_err_t ret = LORA_OK;
  if (sf == LORA_SF_64_CoS) {
    ret |= lora_write_reg(lora, REG_DETECTION_THRESHOLD, 0x0c);
    ret |= lora_write_reg(lora, REG_DETECTION_OPTIMIZE, 0xc5);
  } else if (sf == LORA_SF_4096_CoS) {
    ret |= lora_write_reg(lora, REG_DETECTION_OPTIMIZE, 0xc3);
    ret |= lora_write_reg(lora, REG_DETECTION_THRESHOLD, 0x0a);
  }

  ret |= lora_write_reg(
      lora, REG_MODEM_CONFIG_2,
      (lora_read_reg(lora, REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
  return ret;
}

lora_err_t lora_set_bandwidth(lora_struct_t *lora, lora_bandwith_t sbw) {
  if (sbw < 0 || sbw > 9) {
    lora->log("ERROR: Setting bandwith unsuccessful: sbw out of range");
    return LORA_CONFIG_ERR;
  }

  if (sbw >= 8) {
    int32_t freq = lora_get_frequency(lora);
    if (freq <= 169E6) {
      lora->log("INFO: In the set frequency set bandwith is not supported!");
      return LORA_CONFIG_ERR;
    }
  }

  int16_t bw;
  lora_err_t ret = LORA_OK;
  bw = (int16_t)sbw;

  ret |= lora_write_reg(
      lora, REG_MODEM_CONFIG_1,
      (lora_read_reg(lora, REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  return ret;
}

lora_err_t lora_set_coding_rate(lora_struct_t *lora, int16_t denominator) {
  lora_err_t ret = LORA_OK;
  if (denominator < 5)
    denominator = 5;
  else if (denominator > 8)
    denominator = 8;

  int16_t cr = denominator - 4;
  ret |= lora_write_reg(
      lora, REG_MODEM_CONFIG_1,
      (lora_read_reg(lora, REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
  return ret;
}

lora_err_t lora_set_preamble_length(lora_struct_t *lora, int32_t length) {
  lora_err_t ret = LORA_OK;
  ret |= lora_write_reg(lora, REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  ret |= lora_write_reg(lora, REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
  return ret;
}

lora_err_t lora_set_sync_word(lora_struct_t *lora, int16_t sw) {
  return lora_write_reg(lora, REG_SYNC_WORD, sw);
}

lora_err_t lora_enable_crc(lora_struct_t *lora) {
  return lora_write_reg(lora, REG_MODEM_CONFIG_2,
                        lora_read_reg(lora, REG_MODEM_CONFIG_2) | 0x04);
}

lora_err_t lora_disable_crc(lora_struct_t *lora) {
  return lora_write_reg(lora, REG_MODEM_CONFIG_2,
                        lora_read_reg(lora, REG_MODEM_CONFIG_2) & 0xfb);
}

lora_err_t lora_fill_fifo_buf_to_send(lora_struct_t *lora, uint8_t *buf,
                                      int16_t size) {
  lora_err_t ret = LORA_OK;
  /*
   * Transfer data to radio.
   */
  ret |= lora_idle(lora);
  ret |= lora_write_reg(lora, REG_FIFO_ADDR_PTR, 0);

  for (int16_t i = 0; i < size; i++) {
    ret |= lora_write_reg(lora, REG_FIFO, *buf++);
  }
  ret |= lora_write_reg(lora, REG_PAYLOAD_LENGTH, size);
  return ret;
}

lora_err_t lora_start_transmission(lora_struct_t *lora) {
  /*
   * Start transmission and wait for conclusion.
   */
  return lora_write_reg(lora, REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

bool lora_check_tx_done(lora_struct_t *lora) {
  return (lora_read_reg(lora, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) != 0x00;
}

lora_err_t lora_write_irq_flags(lora_struct_t *lora) {
  return lora_write_reg(lora, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

lora_err_t lora_send_packet(lora_struct_t *lora, uint8_t *buf, int16_t size) {

  lora_err_t ret = LORA_OK;
  ret |= lora_fill_fifo_buf_to_send(lora, buf, size);
  if (ret != LORA_OK) {
    return LORA_TRANSMIT_ERR;
  }

  ret |= lora_write_reg(lora, REG_IRQ_FLAGS, 0xFF);
  ret |= lora_map_d0_interrupt(lora, LORA_IRQ_D0_TXDONE);

  int64_t tx_start_us = esp_timer_get_time();
  int64_t deadline_us = tx_start_us + LORA_TX_TIMEOUT_US;
  ret |= lora_start_transmission(lora);
  if (ret != LORA_OK) {
    lora_map_d0_interrupt(lora, LORA_IRQ_D0_RXDONE);
    return LORA_TRANSMIT_ERR;
  }

  while (!lora_check_tx_done(lora)) {
    if (esp_timer_get_time() >= deadline_us) {
      uint8_t op_mode = lora_read_reg(lora, REG_OP_MODE);
      uint8_t irq_flags = lora_read_reg(lora, REG_IRQ_FLAGS);
      ESP_LOGE(TAG, "TX_DONE timeout — op_mode=0x%02x irq=0x%02x", op_mode, irq_flags);
      lora->last_tx_duration_us = 0;
      lora_idle(lora);
      lora_write_reg(lora, REG_IRQ_FLAGS, 0xFF);
      lora_map_d0_interrupt(lora, LORA_IRQ_D0_RXDONE);
      return LORA_TRANSMIT_ERR;
    }
    if (lora->_delay != NULL) {
      lora->_delay(1);
    }
  }

  lora->last_tx_duration_us = (uint32_t)(esp_timer_get_time() - tx_start_us);

  ret |= lora_write_irq_flags(lora);
  ret |= lora_map_d0_interrupt(lora, LORA_IRQ_D0_RXDONE);
  return ret == LORA_OK ? LORA_OK : LORA_TRANSMIT_ERR;
}

int16_t lora_receive_packet(lora_struct_t *lora, uint8_t *buf, int16_t size) {
  int16_t len = 0;

  /*
   * Check interrupts.
   */
  int16_t irq = lora_read_reg(lora, REG_IRQ_FLAGS);
  if ((irq & IRQ_RX_DONE_MASK) == 0) return 0;
  lora_write_reg(lora, REG_IRQ_FLAGS, irq);
  if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

  /*
   * Find packet size.
   */
  if (lora->implicit_header) {
    len = lora_read_reg(lora, REG_PAYLOAD_LENGTH);
  } else {
    len = lora_read_reg(lora, REG_RX_NB_BYTES);
  }

  /*
   * Transfer data from radio.
   */
  lora_idle(lora);
  lora_write_reg(lora, REG_FIFO_ADDR_PTR,
                 lora_read_reg(lora, REG_FIFO_RX_CURRENT_ADDR));
  if (len > size) {
    len = size;
  }
  for (int16_t i = 0; i < len; i++) {
    buf[i] = lora_read_reg(lora, REG_FIFO);
  }

  lora_write_reg(lora, REG_FIFO_ADDR_PTR, 0x00);

  return len;
}

lora_err_t lora_received(lora_struct_t *lora) {
  if (lora == NULL) {
    ESP_LOGE(TAG, "lora_received called with NULL lora pointer");
    return LORA_RECEIVE_ERR;
  }

  if (lora_read_reg(lora, REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) {
    return LORA_OK;
  }

  if (lora->log) lora->log("ERROR: No packet received");
  // ESP_LOGE(TAG, "No packet received");
  return LORA_RECEIVE_ERR;
}

int16_t lora_packet_rssi(lora_struct_t *lora) {
  return (lora_read_reg(lora, REG_PKT_RSSI_VALUE) -
          (lora->frequency < 868E6 ? 164 : 157));
}

float lora_packet_snr(lora_struct_t *lora) {
  return ((int8_t)lora_read_reg(lora, REG_PKT_SNR_VALUE)) * 0.25;
}

lora_err_t lora_map_d0_interrupt(lora_struct_t *lora, lora_dio0_mapping_t mode) {
  lora_err_t ret = lora_write_reg(lora, REG_DIO_MAPPING_1,(mode << 6));
  if(ret != LORA_OK) {
    lora->log("ERROR: Failed to map DIO0 interrupt");
    ESP_LOGE(TAG, "ERROR: Failed to map DIO0 interrupt");
    return ret;
  }
  lora->_delay(2);  // wait for the register to be updated
  ESP_LOGD(TAG, "Mapped DIO0 to mode %d", mode);
  return LORA_OK;
}

void lora_close(lora_struct_t *lora) {
  lora_sleep(lora);
  //   close(__spi);  FIXME: end hardware features after lora_close
  //   close(__cs);
  //   close(__rst);
  //   __spi = -1;
  //   __cs = -1;
  //   __rst = -1;
}

void lora_dump_registers(lora_struct_t *lora) {
  int16_t i;
  printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++) {
    printf("%02X ", lora_read_reg(lora, i));
    if ((i & 0x0f) == 0x0f) printf("\n");
  }
  printf("\n");
}