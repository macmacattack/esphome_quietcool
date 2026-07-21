#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h" // Native ESP-IDF GPIO Drivers

namespace esphome {
namespace quiet_cool {

// Speed Command Bytes
const uint8_t SPEED_HIGH   = 0xB0;
const uint8_t SPEED_MEDIUM = 0xA0;
const uint8_t SPEED_LOW    = 0x90;

// Duration Command Bytes
const uint8_t DUR_OFF = 0x00;
const uint8_t DUR_1H  = 0x01;
const uint8_t DUR_2H  = 0x02;
const uint8_t DUR_4H  = 0x04;
const uint8_t DUR_8H  = 0x08;
const uint8_t DUR_12H = 0x0C;
const uint8_t DUR_ON  = 0x0F;

class QuietCoolTransmitter : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  // Use native ESP-IDF GPIO types
  gpio_num_t gdo0_pin = GPIO_NUM_2;
  gpio_num_t gdo2_pin = GPIO_NUM_4;

  // Your baked-in Remote ID
  uint8_t remote_id[7] = {0x2D, 0xD4, 0x06, 0xCB, 0x00, 0xF7, 0xF2};

  void setup() override {
    // Initialize pins natively using ESP-IDF
    gpio_reset_pin(this->gdo0_pin);
    gpio_set_direction(this->gdo0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->gdo0_pin, 0);

    gpio_reset_pin(this->gdo2_pin);
    gpio_set_direction(this->gdo2_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->gdo2_pin, 0);

    this->spi_setup();
    this->init_cc1101();
  }

  void write_reg(uint8_t addr, uint8_t value) {
    this->enable();
    this->write_byte(addr);
    this->write_byte(value);
    this->disable();
  }

  void write_strobe(uint8_t strobe) {
    this->enable();
    this->write_byte(strobe);
    this->disable();
  }

  uint8_t read_reg(uint8_t addr) {
    this->enable();
    this->write_byte(addr | 0x80);
    uint8_t val = this->read_byte();
    this->disable();
    return val;
  }

  void init_cc1101() {
    this->write_strobe(0x30); // SRES Reset
    delay(10);

    uint8_t ver = this->read_reg(0xF1);
    ESP_LOGI("quiet_cool", "CC1101 Chip Version: 0x%02X", ver);

    // CC1101 Configuration Registers (433.897 MHz, 2-FSK, Direct Async Mode)
    this->write_reg(0x00, 0x29); // IOCFG2
    this->write_reg(0x02, 0x06); // IOCFG0
    this->write_reg(0x0B, 0x06); // FSCTRL1
    this->write_reg(0x0C, 0x00); // FSCTRL0
    this->write_reg(0x0D, 0x10); // FREQ2 (433.897 MHz)
    this->write_reg(0x0E, 0xB0); // FREQ1
    this->write_reg(0x0F, 0x71); // FREQ0
    this->write_reg(0x10, 0xF6); // MDMCFG4 (2.4 kBaud)
    this->write_reg(0x11, 0x83); // MDMCFG3
    this->write_reg(0x12, 0x00); // MDMCFG2 (2-FSK, direct mode)
    this->write_reg(0x13, 0x00); // MDMCFG1
    this->write_reg(0x14, 0xF8); // MDMCFG0
    this->write_reg(0x15, 0x15); // DEVIATN (10 kHz)
    this->write_reg(0x18, 0x18); // MCSM0
    this->write_reg(0x20, 0xFB); // WORCTRL
    this->write_reg(0x22, 0x10); // FREND0
    this->write_reg(0x23, 0xE9); // FSCAL3
    this->write_reg(0x24, 0x2A); // FSCAL2
    this->write_reg(0x25, 0x00); // FSCAL1
    this->write_reg(0x26, 0x1F); // FSCAL0
    this->write_reg(0x3E, 0xC0); // PATABLE (+10 dBm Power)

    this->write_strobe(0x36); // SIDLE
    ESP_LOGI("quiet_cool", "Native CC1101 setup complete.");
  }

  void send_byte_array(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(this->gdo0_pin, (b >> bit) & 1);
        delayMicroseconds(415); // ESP-IDF safe timing
      }
    }
  }

  void transmit_command(uint8_t speed, uint8_t duration) {
    uint8_t cmd_byte = speed | duration;
    
    // Dynamically build the 20-byte RF payload
    uint8_t packet[20] = {
      0x15, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, // 9 bytes of Preamble/Sync
      remote_id[0], remote_id[1], remote_id[2], remote_id[3], 
      remote_id[4], remote_id[5], remote_id[6],             // 7 bytes of User Remote ID
      cmd_byte, cmd_byte,                                   // 2 bytes of Command
      0x00, 0x00                                            // 2 bytes of Padding
    };

    ESP_LOGD("quiet_cool", "Transmitting Dynamic Payload. CMD: 0x%02X", cmd_byte);

    for (int i = 0; i < 3; i++) {
      gpio_set_level(this->gdo0_pin, 0);
      this->write_strobe(0x35); // STX (Enter TX mode)
      
      this->send_byte_array(packet, 20);

      this->write_strobe(0x36); // SIDLE
      delay(18); // Yields to FreeRTOS tasks & Watchdog Timer
    }
  }

  // Home Assistant Helpers
  void turn_off() { this->transmit_command(SPEED_HIGH, DUR_OFF); }
  void turn_on_high() { this->transmit_command(SPEED_HIGH, DUR_ON); }
  void turn_on_medium() { this->transmit_command(SPEED_MEDIUM, DUR_ON); }
  void turn_on_low() { this->transmit_command(SPEED_LOW, DUR_ON); }
};

}  // namespace quiet_cool
}  // namespace esphome
