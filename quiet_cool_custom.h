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

  // Matches the exact remote ID paired on your dev unit
  uint8_t remote_id[7] = {0x2D, 0xD4, 0x06, 0xCB, 0x00, 0xF7, 0xF2};

  void setup() override {
    // 1. Force CS Pin HIGH to prevent SPI bus locks
    gpio_reset_pin(this->cs_pin);
    gpio_set_direction(this->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->cs_pin, 1); 

    // 2. Setup GDO0 Pin for bitstream transmission
    gpio_reset_pin(this->gdo0_pin);
    gpio_set_direction(this->gdo0_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->gdo0_pin, 0);

    this->spi_setup();
    delay(20);

    // 3. Run full hardware SPI Diagnostics on boot
    this->run_cc1101_diagnostics();

    // 4. Initialize CC1101 registers
    this->init_cc1101();
  }

  uint8_t read_reg(uint8_t addr) {
    this->enable();
    gpio_set_level(this->cs_pin, 0);
    this->write_byte(addr | 0x80); // Read bit set
    uint8_t val = this->read_byte();
    gpio_set_level(this->cs_pin, 1);
    this->disable();
    return val;
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

  void run_cc1101_diagnostics() {
    ESP_LOGI("cc1101_diag", "--- Starting CC1101 SPI Diagnostics ---");

    // Test 1: Read PARTNUM and VERSION registers
    uint8_t partnum = this->read_reg(0x30); 
    uint8_t version = this->read_reg(0x31);
    
    ESP_LOGI("cc1101_diag", "PARTNUM: 0x%02X (Expected: 0x00)", partnum);
    ESP_LOGI("cc1101_diag", "VERSION: 0x%02X (Expected: 0x04 or 0x14)", version);

    // Test 2: Read/Write loopback test on register IOCFG2 (0x00)
    this->write_reg(0x00, 0x2A);
    uint8_t readback = this->read_reg(0x00);
    ESP_LOGI("cc1101_diag", "Register Write/Read Test (IOCFG2): Set 0x2A, Read back 0x%02X -> %s", 
             readback, (readback == 0x2A) ? "PASS" : "FAIL");

    // Test 3: Check MARCSTATE radio state machine register
    this->write_strobe(0x36); // SIDLE
    delay(2);
    uint8_t marcstate = this->read_reg(0x35) & 0x1F;
    ESP_LOGI("cc1101_diag", "MARCSTATE: 0x%02X (Expected IDLE: 0x01 or 0x08)", marcstate);

    ESP_LOGI("cc1101_diag", "--- End CC1101 SPI Diagnostics ---");
  }

  void init_cc1101() {
    this->write_strobe(0x30); // SRES Reset
    delay(10);

    // CC1101 registers configured for 433.897 MHz direct mode
    this->write_reg(0x00, 0x29); 
    this->write_reg(0x01, 0x2E); 
    this->write_reg(0x02, 0x2D); 
    this->write_reg(0x03, 0x07); 
    this->write_reg(0x04, 0xD3); 
    this->write_reg(0x05, 0x91); 
    this->write_reg(0x06, 0xFF); 
    this->write_reg(0x07, 0x04); 
    this->write_reg(0x08, 0x32); 
    this->write_reg(0x09, 0x00); 
    this->write_reg(0x0A, 0x00); 
    this->write_reg(0x0B, 0x06); 
    this->write_reg(0x0C, 0x00); 
    this->write_reg(0x0D, 0x10); 
    this->write_reg(0x0E, 0xB0); 
    this->write_reg(0x0F, 0x71); // 433.897 MHz
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
    this->write_reg(0x3E, 0xC0); // PATABLE (+10dBm max power)

    this->write_strobe(0x36); // SIDLE
    ESP_LOGI("quiet_cool", "CC1101 ESP-IDF driver initialized successfully!");
  }

  void send_bits_from_bytes(const uint8_t *data, size_t len) {
    portDISABLE_INTERRUPTS();

    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(this->gdo0_pin, (b >> bit) & 1);
        ets_delay_us(417); // ~2400 Baud rate
      }
    }

    gpio_set_level(this->gdo0_pin, 0);
    portENABLE_INTERRUPTS();
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

    ESP_LOGD("quiet_cool", "Transmitting CMD 0x%02X under ESP-IDF...", cmd_byte);

    for (int i = 0; i < 3; i++) {
      gpio_set_level(this->gdo0_pin, 0);
      this->write_strobe(0x35); // Enter TX state
      
      ets_delay_us(1000); 
      
      this->send_bits_from_bytes(packet, 20);

      this->write_strobe(0x36); // Return to SIDLE
      delay(28); 
    }
  }

  void turn_off() { this->transmit_command(SPEED_HIGH, DUR_OFF); }
  void turn_on_high() { this->transmit_command(SPEED_HIGH, DUR_ON); }
  void turn_on_medium() { this->transmit_command(SPEED_MEDIUM, DUR_ON); }
  void turn_on_low() { this->transmit_command(SPEED_LOW, DUR_ON); }
};

}  
}
