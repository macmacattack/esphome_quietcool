#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

namespace esphome {
namespace quiet_cool {

// OEM Speed Commands
const uint8_t CMD_OFF    = 0x00; // Speed bits = 00
const uint8_t CMD_LOW    = 0x10; // Speed bits = 01
const uint8_t CMD_MEDIUM = 0x20; // Speed bits = 10
const uint8_t CMD_HIGH   = 0x30; // Speed bits = 11

// OEM Duration Mask (Lower 4 bits)
const uint8_t DUR_HOLD   = 0x0F; // Continuous ON

class QuietCoolTransmitter : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  gpio_num_t gdo0_pin = GPIO_NUM_2; // D1
  gpio_num_t cs_pin   = GPIO_NUM_1; // D0

  // Your existing 7-byte paired remote ID
  uint8_t remote_id[7] = {0x2D, 0xD4, 0x06, 0xCB, 0x00, 0xF7, 0xF2};

  void setup() override {
    gpio_reset_pin(this->cs_pin);
    gpio_set_direction(this->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->cs_pin, 1); 

    gpio_reset_pin(this->gdo0_pin);
    gpio_set_direction(this->gdo0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->gdo0_pin, 0);

    this->spi_setup();
    this->init_cc1101();
  }

  void write_reg(uint8_t addr, uint8_t value) {
    this->enable(); 
    gpio_set_level(this->cs_pin, 0); 
    this->write_byte(addr);
    this->write_byte(value);
    gpio_set_level(this->cs_pin, 1); 
    this->disable(); 
  }

  void write_strobe(uint8_t strobe) {
    this->enable();
    gpio_set_level(this->cs_pin, 0); 
    this->write_byte(strobe);
    gpio_set_level(this->cs_pin, 1);
    this->disable();
  }

  void init_cc1101() {
    this->write_strobe(0x30); // SRES Reset
    delay(10);

    // --- OEM FIRMWARE CC1101 REGISTER PROFILE ---
    this->write_reg(0x00, 0x29); // IOCFG2
    this->write_reg(0x02, 0x2D); // IOCFG0: Async Serial Input mode (GDO0)
    this->write_reg(0x03, 0x07); // FIFOTHR
    this->write_reg(0x04, 0xD3); // SYNC1 (0xD3)
    this->write_reg(0x05, 0x91); // SYNC0 (0x91)
    this->write_reg(0x07, 0x04); // PKTCTRL1
    this->write_reg(0x08, 0x32); // PKTCTRL0: Asynchronous Serial Mode
    this->write_reg(0x0B, 0x06); // FSCTRL1
    
    // Frequency Configuration: Exact 433.92 MHz
    this->write_reg(0x0D, 0x10); // FREQ2
    this->write_reg(0x0E, 0xB1); // FREQ1
    this->write_reg(0x0F, 0x3B); // FREQ0 (433.92 MHz)

    // Data Rate: 2400 Baud
    this->write_reg(0x10, 0xF6); // MDMCFG4
    this->write_reg(0x11, 0x83); // MDMCFG3
    this->write_reg(0x12, 0x00); // MDMCFG2: 2-FSK, No Sync
    this->write_reg(0x13, 0x00); // MDMCFG1
    this->write_reg(0x14, 0xF8); // MDMCFG0

    // Deviation: +/- 10 kHz
    this->write_reg(0x15, 0x25); // DEVIATN

    this->write_reg(0x18, 0x18); // MCSM0
    this->write_reg(0x19, 0x16); // FOCCFG
    this->write_reg(0x1A, 0x6C); // BSCFG
    this->write_reg(0x21, 0x56); // FREND1
    this->write_reg(0x22, 0x10); // FREND0
    this->write_reg(0x23, 0xE9); // FSCAL3
    this->write_reg(0x24, 0x2A); // FSCAL2
    this->write_reg(0x25, 0x00); // FSCAL1
    this->write_reg(0x26, 0x1F); // FSCAL0
    this->write_reg(0x3E, 0xC0); // PATABLE (+10 dBm max power)

    this->write_strobe(0x36); // SIDLE
    ESP_LOGI("quiet_cool", "CC1101 initialized with 2-FSK 433.92MHz 2400 Baud profile.");
  }

  void send_raw_bits(const uint8_t *data, size_t len) {
    portDISABLE_INTERRUPTS();

    // 2400 Baud = 416.67 us per bit
    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(this->gdo0_pin, (b >> bit) & 1);
        ets_delay_us(417);
      }
    }

    gpio_set_level(this->gdo0_pin, 0);
    portENABLE_INTERRUPTS();
  }

  void transmit_command(uint8_t speed_cmd, uint8_t duration_cmd) {
    uint8_t payload_cmd = speed_cmd | duration_cmd;

    // Packet Layout: 4-byte Preamble + 2-byte Hardware Sync + 7-byte ID + Duplicate Command Byte
    uint8_t packet[15] = {
      0xAA, 0xAA, 0xAA, 0xAA,                                 // Preamble
      0xD3, 0x91,                                             // Hardware Sync
      remote_id[0], remote_id[1], remote_id[2], remote_id[3], 
      remote_id[4], remote_id[5], remote_id[6],               // 7-Byte Remote ID
      payload_cmd, payload_cmd                                // Command Byte (x2)
    };

    ESP_LOGD("quiet_cool", "Transmitting 15-Byte Paired Frame (CMD: 0x%02X)", payload_cmd);

    for (int i = 0; i < 3; i++) {
      this->write_strobe(0x36); // SIDLE
      gpio_set_level(this->gdo0_pin, 0);
      this->write_strobe(0x35); // Enter TX mode
      
      ets_delay_us(1000); // Allow PLL to lock
      
      this->send_raw_bits(packet, 15);

      this->write_strobe(0x36); // Return to SIDLE
      delay(28); 
    }
  }

  void turn_off() { this->transmit_command(CMD_OFF, DUR_HOLD); }
  void turn_on_high() { this->transmit_command(CMD_HIGH, DUR_HOLD); }
  void turn_on_medium() { this->transmit_command(CMD_MEDIUM, DUR_HOLD); }
  void turn_on_low() { this->transmit_command(CMD_LOW, DUR_HOLD); }
};

}  
}
