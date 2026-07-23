#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h"

namespace esphome {
namespace quiet_cool {

// Speed Command Bytes
const uint8_t SPEED_HIGH   = 0xB0;
const uint8_t SPEED_MEDIUM = 0xA0;
const uint8_t SPEED_LOW    = 0x90;

// Duration Command Bytes
const uint8_t DUR_OFF = 0x00;
const uint8_t DUR_ON  = 0x0F;

class QuietCoolTransmitter : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  gpio_num_t gdo0_pin = GPIO_NUM_2;
  gpio_num_t cs_pin   = GPIO_NUM_1;

  // YOUR actual paired remote ID!
  uint8_t remote_id[7] = {0x2D, 0xD4, 0x06, 0xCB, 0x00, 0xF7, 0xF2};

  void setup() override {
    gpio_reset_pin(this->cs_pin);
    gpio_set_direction(this->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->cs_pin, 1); 

    // GDO0 is now an INPUT so we can watch the CC1101 hardware packet status!
    gpio_reset_pin(this->gdo0_pin);
    gpio_set_direction(this->gdo0_pin, GPIO_MODE_INPUT);

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

    // --- HARDWARE PACKET ENGINE (FIFO MODE) SETTINGS ---
    // These perfectly recreate the Elechouse default state.
    this->write_reg(0x00, 0x29); 
    this->write_reg(0x01, 0x2E); 
    this->write_reg(0x02, 0x06); // GDO0 asserts when Sync Word is sent, de-asserts at end of packet
    this->write_reg(0x03, 0x07); 
    this->write_reg(0x04, 0xD3); // Hardware Sync Word High
    this->write_reg(0x05, 0x91); // Hardware Sync Word Low
    this->write_reg(0x06, 0xFF); 
    this->write_reg(0x07, 0x04); 
    this->write_reg(0x08, 0x05); // PKTCTRL0: FIFO Packet Engine with CRC Checksum Enabled!
    this->write_reg(0x09, 0x00); 
    this->write_reg(0x0A, 0x00); 
    this->write_reg(0x0B, 0x06); 
    this->write_reg(0x0C, 0x00); 
    this->write_reg(0x0D, 0x10); 
    this->write_reg(0x0E, 0xB0); 
    this->write_reg(0x0F, 0x71); // 433.897 MHz
    this->write_reg(0x10, 0xF6); // 2.4k Baud
    this->write_reg(0x11, 0x83); 
    this->write_reg(0x12, 0x00); // 2-FSK Modulation
    this->write_reg(0x13, 0x22); 
    this->write_reg(0x14, 0xF8); 
    this->write_reg(0x15, 0x25); // 10kHz deviation
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
    this->write_reg(0x3E, 0xC0); // Max power

    this->write_strobe(0x36); // SIDLE
    ESP_LOGI("quiet_cool", "Hardware FIFO Packet Engine Initialized Successfully!");
  }

  void transmit_command(uint8_t speed, uint8_t duration) {
    uint8_t cmd_byte = speed | duration;
    
    // The exact 20-byte payload array
    uint8_t packet[20] = {
      0x15, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 
      remote_id[0], remote_id[1], remote_id[2], remote_id[3], 
      remote_id[4], remote_id[5], remote_id[6],             
      cmd_byte, cmd_byte,                                   
      0x00, 0x00                                            
    };

    ESP_LOGD("quiet_cool", "Loading payload into CC1101 TX Memory. CMD: 0x%02X", cmd_byte);

    for (int i = 0; i < 3; i++) {
      this->write_strobe(0x36); // Ensure radio is idle
      this->write_strobe(0x3B); // Flush any old data in TX FIFO
      
      this->enable();
      gpio_set_level(this->cs_pin, 0);
      
      // 0x7F triggers a Burst Write into the CC1101's Memory Buffer
      this->write_byte(0x7F); 
      
      // The Hardware Packet engine demands the length of the array first
      this->write_byte(20); 
      
      // Load the actual array into memory
      for (int j = 0; j < 20; j++) {
        this->write_byte(packet[j]);
      }
      gpio_set_level(this->cs_pin, 1);
      this->disable();
      
      // Pull the trigger (Enter TX mode)
      this->write_strobe(0x35); 
      
      // Wait for the hardware to automatically push the bits out of the antenna.
      // The CC1101 pulls GDO0 High when it starts, and drops it Low when the CRC is done.
      uint32_t timeout = millis();
      while(gpio_get_level(this->gdo0_pin) == 0 && (millis() - timeout < 50)) { delay(1); }
      while(gpio_get_level(this->gdo0_pin) == 1 && (millis() - timeout < 100)) { delay(1); }

      delay(18); // Pause between repeats
    }
    ESP_LOGD("quiet_cool", "Hardware packet transmission complete!");
  }

  void turn_off() { this->transmit_command(SPEED_HIGH, DUR_OFF); }
  void turn_on_high() { this->transmit_command(SPEED_HIGH, DUR_ON); }
  void turn_on_medium() { this->transmit_command(SPEED_MEDIUM, DUR_ON); }
  void turn_on_low() { this->transmit_command(SPEED_LOW, DUR_ON); }
};

}  
}
