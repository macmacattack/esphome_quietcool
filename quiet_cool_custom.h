#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h"

namespace esphome {
namespace quiet_cool {

class QuietCoolTransmitter : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  gpio_num_t cs_pin   = GPIO_NUM_1; // D0

  void setup() override {
    gpio_reset_pin(this->cs_pin);
    gpio_set_direction(this->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->cs_pin, 1); 

    this->spi_setup();
    delay(100);

    // Read CC1101 Version Register (0x31 | 0xC0 = 0xF1)
    this->enable();
    gpio_set_level(this->cs_pin, 0);
    this->write_byte(0xF1);
    uint8_t version = this->read_byte();
    gpio_set_level(this->cs_pin, 1);
    this->disable();

    // Read CC1101 Part Number Register (0x30 | 0xC0 = 0xF0)
    this->enable();
    gpio_set_level(this->cs_pin, 0);
    this->write_byte(0xF0);
    uint8_t partnum = this->read_byte();
    gpio_set_level(this->cs_pin, 1);
    this->disable();

    ESP_LOGI("cc1101_check", "========================================");
    ESP_LOGI("cc1101_check", "CC1101 PARTNUM READ: 0x%02X (Expected: 0x00)", partnum);
    ESP_LOGI("cc1101_check", "CC1101 VERSION READ: 0x%02X (Expected: 0x14 or 0x04)", version);
    ESP_LOGI("cc1101_check", "========================================");

    if (version == 0x00 || version == 0xFF) {
      ESP_LOGE("cc1101_check", "CRITICAL ERROR: SPI Communication Failed! Check CS/CLK/MISO/MOSI wiring.");
    } else {
      ESP_LOGI("cc1101_check", "SUCCESS: CC1101 SPI Bus is Working!");
    }
  }
};

}
}
