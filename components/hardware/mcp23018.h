///===-----------------------------------------------------------------------------------------===//
///
/// Copyright (c) PWr in Space. All rights reserved.
/// Created: 14.01.2024 by Michał Kos
///
///===-----------------------------------------------------------------------------------------===//
///
/// \file
/// This file contains declaration of MCP23018 class which is used to control MCP23018 gpio expander.
/// The interrupt functionality is not implemented.
///===-----------------------------------------------------------------------------------------===//

#ifndef PWRINSPACE_MCP23018_H_
#define PWRINSPACE_MCP23018_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MCP23018_CTRL_ID     0x20

// Register addresses for IOCON.BANK = 0 which is
// the only addressing mode supported by this library.
// For the sake of library consistency the manufacturer's
// use of the portA, portB convention is abandoned and
// replaced by the use of ports[0], ports[1]
#define MCP23018_REG_IODIR0    0x00
#define MCP23018_REG_IPOL0     0x02
#define MCP23018_REG_GPINTEN0  0x04
#define MCP23018_REG_DEFVAL0   0x06
#define MCP23018_REG_INTCON0   0x08
#define MCP23018_REG_IOCON     0x0A
#define MCP23018_REG_GPPU0     0x0C
#define MCP23018_REG_INTF0     0x0E
#define MCP23018_REG_INTCAP0   0x10
#define MCP23018_REG_GPIO0     0x12
#define MCP23018_REG_OLAT0     0x14

// Bit masks for IOCON register control bits
#define MCP23018_IOCON_INTCC   0x01   // 0=reading GPIO, 1=reading INTCAP clears interrupt
#define MCP23018_IOCON_INTPOL  0x02   // 0=ActiveLow, 1=ActiveHigh interrupt polarity
#define MCP23018_IOCON_ODR     0x04   // 0=ActiveDriver, 1=OpenDrain Interrupt (overrides INTPOL)

// Bit masks for IOCON register control bits
typedef enum {
    IOCON_INTCC = 0x01,     // 0=reading GPIO, 1=reading INTCAP clears interrupt
    IOCON_INTPOL = 0x02,    // 0=ActiveLow, 1=ActiveHigh interrupt polarity
    IOCON_ODR = 0x04,       // 0=ActiveDriver, 1=OpenDrain Interrupt (overrides INTPOL)
    // Bits 3 & 4 (0x08 and 0x10) are unimplemented
    IOCON_SEQOP = 0x20,     // 0=Enable, 1=Disable sequential & incrementing register addressing
    IOCON_MIRROR = 0x40,    // 0=Disable, 1=Enable internal wire-OR-ing of the two interrupts
    IOCON_BANK = 0x80,      // 0=Disable, 1=Enable separation of registers into two banks 
} mcp23018_iocon_ctrl_bits_t;

// Values for bit and port polarity
typedef enum {
    // Bit polarity
    MCP23018_NORMAL = 0,
    MCP23018_INVERTED = 1,
    // Port polarity
    MCP23018_ALL_NORMAL = 0x00,
    MCP23018_ALL_INVERTED = 0xFF,
} mcp23018_polarity_ctrl_t;

// Values for pull up resistor bit and port
typedef enum {
    // Bit pull up
    MCP23018_PU_ENABLE = 1,
    MCP23018_PU_DISABLE = 0,
    // Port pull up
    MCP23018_ALL_PU_ENABLE = 0xFF,
    MCP23018_ALL_PU_DISABLE = 0x00,
} mcp23018_pullup_ctrl_t;

// Values for bit and port interrupt enables
typedef enum {
    // Bit interrupt enable
    MCP23018_INTR_ENABLE = 1,
    MCP23018_INTR_DISABLE = 0,
    // Port interrupt enable
    MCP23018_ALL_INTR_ENABLE = 0xFF,
    MCP23018_ALL_INTR_DISABLE = 0x00,
} mcp23018_interrupt_ctrl_t;

// Values for bit and port mode 
typedef enum {
    // Bit direction
    MCP23018_INPUT = 0,
    MCP23018_OUTPUT = 1,
    // Port direction
    MCP23018_ALL_INPUT = 0x00,
    MCP23018_ALL_OUTPUT = 0xFF,
} mcp23018_mode_ctrl_t;

// Values for bit and port values
typedef enum {
    // Bit value
    MCP23018_LOW = 0,
    MCP23018_HIGH = 1,
    // Port value
    MCP23018_ALL_LOW = 0x00,
    MCP23018_ALL_HIGH = 0xFF,
} mcp23018_value_ctrl_t;

// Port number
typedef enum {
    MCP23018_PORT_A = 0,
    MCP23018_PORT_B = 1,
} mcp23018_port_t;

// Pin number
typedef enum {
    MCP23018_PIN_0 = 0,
    MCP23018_PIN_1 = 1,
    MCP23018_PIN_2 = 2,
    MCP23018_PIN_3 = 3,
    MCP23018_PIN_4 = 4,
    MCP23018_PIN_5 = 5,
    MCP23018_PIN_6 = 6,
    MCP23018_PIN_7 = 7,
} mcp23018_pin_t;

typedef enum {
    MCP23018_OK = 0,
    MCP23018_FAIL,
    MCP23018_INIT_ERR,
    MCP23018_READ_ERR,
    MCP23018_WRITE_ERR,
} mcp23018_status_t;

// I2C access functions
typedef bool (*mcp23018_I2C_write)(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);
typedef bool (*mcp23018_I2C_read)(uint8_t address, uint8_t reg, uint8_t *data, uint8_t len);

typedef struct {
    mcp23018_I2C_write _i2c_write;
    mcp23018_I2C_read _i2c_read;
    uint8_t i2c_address;
    uint8_t iocon;
    // Gpio control registers
    uint8_t dirRegisters[2];
	uint8_t polRegisters[2];
	uint8_t pullupRegisters[2];
	uint8_t ports[2];
} mcp23018_struct_t;

mcp23018_status_t mcp23018_init(mcp23018_struct_t *mcp23018, uint8_t mode);

mcp23018_status_t mcp23018_digital_write(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_value_ctrl_t value);
mcp23018_status_t mcp23018_digital_write_port(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t value);

mcp23018_status_t mcp23018_digital_read(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_value_ctrl_t *value);
mcp23018_status_t mcp23018_digital_read_port(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *value);

mcp23018_status_t mcp23018_get_pin_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_mode_ctrl_t *mode);
mcp23018_status_t mcp23018_set_pin_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_mode_ctrl_t mode);

mcp23018_status_t mcp23018_get_port_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *mode);
mcp23018_status_t mcp23018_set_port_mode(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t mode);

mcp23018_status_t mcp23018_get_pin_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_polarity_ctrl_t *polarity);
mcp23018_status_t mcp23018_set_pin_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_polarity_ctrl_t polarity);

mcp23018_status_t mcp23018_get_port_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *polarity);
mcp23018_status_t mcp23018_set_port_polarity(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t polarity);

mcp23018_status_t mcp23018_get_pin_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_pullup_ctrl_t *pullup);
mcp23018_status_t mcp23018_set_pin_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, mcp23018_pin_t pin, mcp23018_pullup_ctrl_t pullup);

mcp23018_status_t mcp23018_get_port_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t *pullup);
mcp23018_status_t mcp23018_set_port_pullup(mcp23018_struct_t *mcp23018, mcp23018_port_t port, uint8_t pullup);

#ifdef __cplusplus
}
#endif

#endif /* PWRINSPACE_MCP23018_H_ */