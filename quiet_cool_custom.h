#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

namespace esphome {
namespace quiet_cool {

const uint8_t SPEED_HIGH   = 0xB0;
const uint8_t SPEED_MEDIUM = 0xA0;
const uint8_t SPEED_LOW    = 0x90;

const uint8_t DUR_OFF = 0x00;
const uint8_t DUR_ON  = 0x0F;

class QuietCoolTransmitter : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  gpio_num_t gdo0_pin = GPIO_NUM_2; // D1 / GPIO2
  gpio_num_t cs_pin   = GPIO_NUM_1; // D0 / GPIO1

  // Exact paired remote ID from your dev unit
  uint8_t remote_id[7] = {0x2D, 0xD4, 0x06, 0xCB, 0x00, 0xF7, 0xF2};

  void setup() override {
    gpio_reset_pin(this->cs_pin);
    gpio_set_direction(this->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->cs_pin, 1); 

    gpio_reset_pin(this->gdo0_pin);
    gpio_set_direction(this->gdo0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->gdo0_pin, 0);

    this->spi_setup();
    delay(20);
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

    // CC1101 Register Setup matching ccrome RF config
    this->write_reg(0x00, 0x29); 
    this->write_reg(0x01, 0x2E); 
    this->write_reg(0x02, 0x2D); // Direct Async Output on GDO0
    this->write_reg(0x03, 0x07); 
    this->write_reg(0x04, 0xD3); 
    this->write_reg(0x05, 0x91); 
    this->write_reg(0x06, 0xFF); 
    this->write_reg(0x07, 0x04); 
    this->write_reg(0x08, 0x32); // Asynchronous Direct Mode
    this->write_reg(0x09, 0x00); 
    this->write_reg(0x0A, 0x00); 
    this->write_reg(0x0B, 0x06); 
    this->write_reg(0x0C, 0x00); 

    // 433.897 MHz
    this->write_reg(0x0D, 0x10); 
    this->write_reg(0x0E, 0xB0); 
    this->write_reg(0x0F, 0x71); 

    // 2500 Baud Rate (400us per bit)
    this->write_reg(0x10, 0xF6); 
    this->write_reg(0x11, 0x83); 

    this->write_reg(0x12, 0x00); 
    this->write_reg(0x13, 0x00); 
    this->write_reg(0x14, 0xF8); 
    this->write_reg(0x15, 0x25); 
    this->write_reg(0x16, 0x07); 
    this->write_reg(0x17, 0x30); 
    this->write_reg(0x18, 0x18); 
    this->write_reg(0x19, 0x16); 
    this->write_reg(0x1A, 0x6C); 
    this->write_reg(0x1B, 0x43); 
    this->write_reg(0x1C, 0x40); 
    this->write_reg(0x1D, 0x91); 
    this->write_reg(0x21, 0x56); 
    this->write_reg(0x22, 0x10); 
    this->write_reg(0x23, 0xE9); 
    this->write_reg(0x24, 0x2A); 
    this->write_reg(0x25, 0x00); 
    this->write_reg(0x26, 0x1F); 
    this->write_reg(0x3E, 0xC0); // Max power (+10 dBm)

    this->write_strobe(0x36); // SIDLE
    ESP_LOGI("quiet_cool", "CC1101 ESP32-S3 Watchdog-Safe Driver Ready.");
  }

  // Precision transmission with ROM delay and task yielding
  void send_bits_from_bytes(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(this->gdo0_pin, (b >> bit) & 1);
        ets_delay_us(400); // 400us URH Baud Delay
      }
    }
    gpio_set_level(this->gdo0_pin, 0);
  }

  void transmit_command(uint8_t speed, uint8_t duration) {
    uint8_t cmd_byte = speed | duration;

    uint8_t packet[20] = {
      0x15, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 
      remote_id[0], remote_id[1], remote_id[2], remote_id[3], 
      remote_id[4], remote_id[5], remote_id[6],             
      cmd_byte, cmd_byte,                                   
      0x00, 0x00                                            
    };

    ESP_LOGD("quiet_cool", "Transmitting CMD 0x%02X safely without watchdog panic...", cmd_byte);

    for (int i = 0; i < 6; i++) {
      gpio_set_level(this->gdo0_pin, 0);
      this->write_strobe(0x35); // STX
      
      ets_delay_us(1000); 
      
      this->send_bits_from_bytes(packet, 20);

      this->write_strobe(0x36); // SIDLE
      
      // CRITICAL FIX FOR ESP32-S3: Feed Watchdog and yield CPU
      App.feed_wdt();
      vTaskDelay(pdMS_TO_TICKS(25)); 
    }
  }

  void turn_off() { this->transmit_command(SPEED_HIGH, DUR_OFF); }
  void turn_on_high() { this->transmit_command(SPEED_HIGH, DUR_ON); }
  void turn_on_medium() { this->transmit_command(SPEED_MEDIUM, DUR_ON); }
  void turn_on_low() { this->transmit_command(SPEED_LOW, DUR_ON); }
};

}  
}
