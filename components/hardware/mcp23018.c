///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 14.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//

#include "mcp23018.h"
#include "esp_log.h"

#define TAG "MCP23018"

static mcp23018_status_t write_reg_8b(mcp23018_struct_t *mcp23018, const uint8_t reg, uint8_t val) {
    bool ret = true;
    ret = mcp23018->_i2c_write(mcp23018->i2c_address, reg, &val, 1);
    if (ret != true) {
        ESP_LOGE(TAG, "MCP23018 write reg 8b failed");
        return MCP23018_WRITE_ERR;
    }
    return MCP23018_OK;
}

static mcp23018_status_t write_reg_16b(mcp23018_struct_t *mcp23018, const uint8_t reg, uint16_t val) {
    bool ret = true;
    uint8_t data[2] = {val >> 8, val & 0xFF};
    ret = mcp23018->_i2c_write(mcp23018->i2c_address, reg, data, 2);
    if (ret != true) {
        ESP_LOGE(TAG, "MCP23018 write reg 16b failed");
        return MCP23018_WRITE_ERR;
    }
    return MCP23018_OK;
}

static mcp23018_status_t read_reg_8b(mcp23018_struct_t *mcp23018, const uint8_t reg, uint8_t *val) {
    bool ret = true;
    ret = mcp23018->_i2c_read(mcp23018->i2c_address, reg, val, 1);
    if (ret != true) {
        ESP_LOGE(TAG, "MCP23018 read reg 8b failed");
        return MCP23018_READ_ERR;
    }
    return MCP23018_OK;
}

static mcp23018_status_t read_reg_16b(mcp23018_struct_t *mcp23018, const uint8_t reg, uint16_t *val) {
    bool ret = true;
    uint8_t data[2];
    ret = mcp23018->_i2c_read(mcp23018->i2c_address, reg, data, 2);
    *val = (data[0] << 8) | data[1];
    if (ret != true) {
        ESP_LOGE(TAG, "MCP23018 read reg 16b failed");
        return MCP23018_READ_ERR;
    }
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_init(mcp23018_struct_t *mcp23018, uint8_t mode) {
    mcp23018_status_t ret = MCP23018_OK;
    uint8_t reg_val = 0;

    reg_val = (IOCON_INTCC | IOCON_INTPOL | IOCON_ODR | IOCON_MIRROR); // allowed modes
    ret = write_reg_8b(mcp23018, MCP23018_REG_IOCON, reg_val);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 IOCON register initialization failed");
        return ret;
    }
    mcp23018->iocon = reg_val;
    // Default all pins to input
    ret = mcp23018_set_port_mode(mcp23018, MCP23018_PORT_A, MCP23018_ALL_INPUT);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 PORT A initialization failed");
        return ret;
    }
    ret = mcp23018_set_port_mode(mcp23018, MCP23018_PORT_B, MCP23018_ALL_INPUT);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 PORT B initialization failed");
        return ret;
    }
    // Default all pins are not inverted
    ret = mcp23018_set_port_polarity(mcp23018, MCP23018_PORT_A, MCP23018_ALL_NORMAL);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 PORT A polarity initialization failed");
        return ret;
    }
    ret = mcp23018_set_port_polarity(mcp23018, MCP23018_PORT_B, MCP23018_ALL_NORMAL);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 PORT B polarity initialization failed");
        return ret;
    }
    // Default all pins to low
    ret = mcp23018_digital_write_port(mcp23018, MCP23018_PORT_A, MCP23018_ALL_LOW);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 PORT A digital write initialization failed");
        return ret;
    }
    ret = mcp23018_digital_write_port(mcp23018, MCP23018_PORT_B, MCP23018_ALL_LOW);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 PORT B digital write initialization failed");
        return ret;
    }
    // Clear interrupts
    uint16_t ports = 0;
    ret = read_reg_16b(mcp23018, (uint8_t)MCP23018_REG_GPIO0, &ports);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 clear interrupts failed");
        return ret;
    }
    mcp23018->ports[0] = ports >> 8;
    mcp23018->ports[1] = ports & 0xFF;

    return MCP23018_OK;
}

mcp23018_status_t mcp23018_digital_write(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_value_ctrl_t value) {
    mcp23018_status_t ret = MCP23018_OK;
    uint16_t reg_val = 0;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Check if pin is correct
    if (pin > 7) {
        ESP_LOGE(TAG, "MCP23018 pin out of range");
        return MCP23018_FAIL;
    }
    // Check if value is correct
    if (value > 1) {
        ESP_LOGE(TAG, "MCP23018 pin value out of range");
        return MCP23018_FAIL;
    }
    // Set pin value
    if (value == MCP23018_HIGH) {
        reg_val = mcp23018->ports[port] |= (1 << pin);
    } else {
        reg_val = mcp23018->ports[port] &= ~(1 << pin);
    }
    // Write to port    
    ret = write_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_GPIO0 + port), mcp23018->ports[port]);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 digital write failed");
        return ret;
    }
    // Save port value
    mcp23018->ports[port] = reg_val;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_digital_write_port(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t value) {
    mcp23018_status_t ret = MCP23018_OK;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Write to port
    ret = write_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_GPIO0 + port), value);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 digital write failed");
        return ret;
    }
    // Save port value
    mcp23018->ports[port] = value;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_digital_read(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_value_ctrl_t *value) {
    mcp23018_status_t ret = MCP23018_OK;
    uint8_t reg_val = 0;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Check if pin is correct
    if (pin > 7) {
        ESP_LOGE(TAG, "MCP23018 pin out of range");
        return MCP23018_FAIL;
    }
    // Read from port
    ret = read_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_GPIO0 + port), &reg_val);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 digital read failed");
        return ret;
    }
    // Set pin value
    if (reg_val & (1 << pin)) {
        *value = MCP23018_HIGH;
    } else {
        *value = MCP23018_LOW;
    }
    // Save pin value
    mcp23018->ports[port] = reg_val;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_digital_read_port(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *value) {
    mcp23018_status_t ret = MCP23018_OK;
    uint8_t reg_val = 0;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Read from port
    ret = read_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_GPIO0 + port), &reg_val);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 digital read failed");
        return ret;
    }
    // Set port value
    *value = reg_val;
    // Save port value
    mcp23018->ports[port] = reg_val;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_get_pin_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_mode_ctrl_t *mode) {
    if (port == MCP23018_PORT_A) {
        *mode = (mcp23018_mode_ctrl_t)((mcp23018->dirRegisters[0] >> pin) & 0x01);
    } else {
        *mode = (mcp23018_mode_ctrl_t)((mcp23018->dirRegisters[1] >> pin) & 0x01);
    }
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_set_pin_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_mode_ctrl_t mode) {
    mcp23018_status_t ret = MCP23018_OK;
    uint8_t reg_val = 0;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Check if pin is correct
    if (pin > 7) {
        ESP_LOGE(TAG, "MCP23018 pin out of range");
        return MCP23018_FAIL;
    }
    // Check if mode is correct
    if (mode > 1) {
        ESP_LOGE(TAG, "MCP23018 pin mode out of range");
        return MCP23018_FAIL;
    }
    // Set register value
    reg_val = ~(mcp23018->dirRegisters[port]);
    if (mode == MCP23018_INPUT) {
        reg_val |= (1 << pin);
    } else {
        reg_val &= ~(1 << pin);
    }
    // Write port
    ret = mcp23018_set_port_mode(mcp23018, port, reg_val);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 pin mode set failed");
        return ret;
    }
    // Save port value
    mcp23018->dirRegisters[port] = reg_val;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_get_port_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *mode) {
    if (port == MCP23018_PORT_A) {
        *mode = (mcp23018_mode_ctrl_t)(mcp23018->dirRegisters[0]);
    } else {
        *mode = (mcp23018_mode_ctrl_t)(mcp23018->dirRegisters[1]);
    }
    return MCP23018_OK;
}
mcp23018_status_t mcp23018_set_port_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t mode) {
    mcp23018_status_t ret = MCP23018_OK;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Write port
    ret = write_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_IODIR0 + port), mode);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 port mode set failed");
        return ret;
    }
    // Save port value
    mcp23018->dirRegisters[port] = mode;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_get_pin_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_polarity_ctrl_t *polarity) {
    if (port == MCP23018_PORT_A) {
        *polarity = (mcp23018_polarity_ctrl_t)((mcp23018->polRegisters[0] >> pin) & 0x01);
    } else {
        *polarity = (mcp23018_polarity_ctrl_t)((mcp23018->polRegisters[1] >> pin) & 0x01);
    }
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_set_pin_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_polarity_ctrl_t polarity) {
    mcp23018_status_t ret = MCP23018_OK;
    uint8_t reg_val = 0;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Check if pin is correct
    if (pin > 7) {
        ESP_LOGE(TAG, "MCP23018 pin out of range");
        return MCP23018_FAIL;
    }
    // Check if polarity is correct
    if (polarity > 1) {
        ESP_LOGE(TAG, "MCP23018 pin polarity out of range");
        return MCP23018_FAIL;
    }
    // Set register value
    reg_val = ~(mcp23018->polRegisters[port]);
    if (polarity == MCP23018_NORMAL) {
        reg_val |= (1 << pin);
    } else {
        reg_val &= ~(1 << pin);
    }
    // Write port
    ret = mcp23018_set_port_polarity(mcp23018, port, reg_val);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 pin polarity set failed");
        return ret;
    }
    // Save port value
    mcp23018->polRegisters[port] = reg_val;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_get_port_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *polarity) {
    if (port == MCP23018_PORT_A) {
        *polarity = (mcp23018_polarity_ctrl_t)(mcp23018->polRegisters[0]);
    } else {
        *polarity = (mcp23018_polarity_ctrl_t)(mcp23018->polRegisters[1]);
    }
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_set_port_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t polarity) {
    mcp23018_status_t ret = MCP23018_OK;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Write port
    ret = write_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_IPOL0 + port), polarity);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 port polarity set failed");
        return ret;
    }
    // Save port value
    mcp23018->polRegisters[port] = polarity;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_get_pin_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_pullup_ctrl_t *pullup) {
    if (port == MCP23018_PORT_A) {
        *pullup = (mcp23018_pullup_ctrl_t)((mcp23018->pullupRegisters[0] >> pin) & 0x01);
    } else {
        *pullup = (mcp23018_pullup_ctrl_t)((mcp23018->pullupRegisters[1] >> pin) & 0x01);
    }
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_set_pin_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_pullup_ctrl_t pullup) {
    mcp23018_status_t ret = MCP23018_OK;
    uint8_t reg_val = 0;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Check if pin is correct
    if (pin > 7) {
        ESP_LOGE(TAG, "MCP23018 pin out of range");
        return MCP23018_FAIL;
    }
    // Check if pullup is correct
    if (pullup > 1) {
        ESP_LOGE(TAG, "MCP23018 pin pullup out of range");
        return MCP23018_FAIL;
    }
    // Set register value
    reg_val = ~(mcp23018->pullupRegisters[port]);
    if (pullup == MCP23018_PU_ENABLE) {
        reg_val |= (1 << pin);
    } else {
        reg_val &= ~(1 << pin);
    }
    // Write port
    ret = mcp23018_set_port_pullup(mcp23018, port, reg_val);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 pin pullup set failed");
        return ret;
    }
    // Save port value
    mcp23018->pullupRegisters[port] = reg_val;
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_get_port_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *pullup) {
    if (port == MCP23018_PORT_A) {
        *pullup = (mcp23018_pullup_ctrl_t)(mcp23018->pullupRegisters[0]);
    } else {
        *pullup = (mcp23018_pullup_ctrl_t)(mcp23018->pullupRegisters[1]);
    }
    return MCP23018_OK;
}

mcp23018_status_t mcp23018_set_port_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t pullup) {
    mcp23018_status_t ret = MCP23018_OK;
    // Check if port is correct
    if (port > 1) {
        ESP_LOGE(TAG, "MCP23018 port out of range");
        return MCP23018_FAIL;
    }
    // Write port
    ret = write_reg_8b(mcp23018, (uint8_t)(MCP23018_REG_GPPU0 + port), pullup);
    if (ret != MCP23018_OK) {
        ESP_LOGE(TAG, "MCP23018 port pullup set failed");
        return ret;
    }
    // Save port value
    mcp23018->pullupRegisters[port] = pullup;
    return MCP23018_OK;
}