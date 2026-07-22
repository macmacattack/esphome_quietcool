#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "esphome/core/hal.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

namespace esphome {
namespace quiet_cool {

// The original, raw bit-strings.
static const char *const SPEED_SETTINGS[] = {
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100100110011001100110000", // PRE (0)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011000110110001000", // H1 (1)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011001010110010000", // H2 (2)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011010010110100000", // H4 (3)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011100010111000000", // H8 (4)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011110010111100000", // H12 (5)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011111110111111000", // HON (6)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101011000010110000000", // HOFF (7)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010000110100001000", // M1 (8)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010001010100010000", // M2 (9)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010010010100100000", // M4 (10)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010100010101000000", // M8 (11)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010110010101100000", // M12 (12)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010111110101111000", // MON (13)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101010000010100000000", // MOFF (14)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001000110010001000", // L1 (15)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001001010010010000", // L2 (16)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001010010010100000", // L4 (17)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001100010011000000", // L8 (18)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001110010011100000", // L12 (19)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001111110011111000", // LON (20)
    "0000101011010101010101010101010101010101010101010101010101010101010101010001011011101010000000110110010110000000011110111111100101001000010010000000", // LOFF (21)
};

class QuietCoolTransmitter : public Component, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_2MHZ> {
 public:
  gpio_num_t gdo0_pin = GPIO_NUM_2;
  gpio_num_t gdo2_pin = GPIO_NUM_4;
  gpio_num_t cs_pin   = GPIO_NUM_1;

  void setup() override {
    gpio_reset_pin(this->cs_pin);
    gpio_set_direction(this->cs_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(this->cs_pin, 1); 

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

  uint8_t read_reg(uint8_t addr) {
    this->enable();
    gpio_set_level(this->cs_pin, 0);
    this->write_byte(addr | 0x80);
    uint8_t val = this->read_byte();
    gpio_set_level(this->cs_pin, 1);
    this->disable();
    return val;
  }

  void init_cc1101() {
    this->write_strobe(0x30); // SRES Reset
    delay(10);

    uint8_t ver = this->read_reg(0xF1);
    ESP_LOGI("quiet_cool", "CC1101 Found! Chip Version: 0x%02X", ver);

    // The Ultimate ELECHOUSE Configuration Clone
    this->write_reg(0x00, 0x29); // IOCFG2
    this->write_reg(0x01, 0x2E); // IOCFG1
    this->write_reg(0x02, 0x2D); // IOCFG0: CRITICAL - Sets GDO0 as Input for Async TX!
    this->write_reg(0x03, 0x07); // FIFOTHR
    this->write_reg(0x04, 0xD3); // SYNC1
    this->write_reg(0x05, 0x91); // SYNC0
    this->write_reg(0x06, 0xFF); // PKTLEN
    this->write_reg(0x07, 0x04); // PKTCTRL1
    this->write_reg(0x08, 0x32); // PKTCTRL0: CRITICAL - Async serial mode
    this->write_reg(0x09, 0x00); // ADDR
    this->write_reg(0x0A, 0x00); // CHANNR
    this->write_reg(0x0B, 0x06); // FSCTRL1
    this->write_reg(0x0C, 0x00); // FSCTRL0
    this->write_reg(0x0D, 0x10); // FREQ2
    this->write_reg(0x0E, 0xB0); // FREQ1
    this->write_reg(0x0F, 0x71); // FREQ0 (433.897 MHz)
    this->write_reg(0x10, 0xF6); // MDMCFG4 (2.398 kBaud)
    this->write_reg(0x11, 0x83); // MDMCFG3
    this->write_reg(0x12, 0x00); // MDMCFG2 (2-FSK, No Sync)
    this->write_reg(0x13, 0x00); // MDMCFG1 (No Preamble)
    this->write_reg(0x14, 0xF8); // MDMCFG0
    this->write_reg(0x15, 0x25); // DEVIATN: CRITICAL - Math fixed to exact Arduino 10kHz formula
    this->write_reg(0x16, 0x07); // MCSM2
    this->write_reg(0x17, 0x30); // MCSM1
    this->write_reg(0x18, 0x18); // MCSM0
    this->write_reg(0x19, 0x16); // FOCCFG
    this->write_reg(0x1A, 0x6C); // BSCFG
    this->write_reg(0x1B, 0x43); // AGCCTRL2
    this->write_reg(0x1C, 0x40); // AGCCTRL1
    this->write_reg(0x1D, 0x91); // AGCCTRL0
    this->write_reg(0x21, 0x56); // FREND1
    this->write_reg(0x22, 0x10); // FREND0
    this->write_reg(0x23, 0xE9); // FSCAL3
    this->write_reg(0x24, 0x2A); // FSCAL2
    this->write_reg(0x25, 0x00); // FSCAL1
    this->write_reg(0x26, 0x1F); // FSCAL0
    this->write_reg(0x3E, 0xC0); // PATABLE: Bumping to MAX Power (+10dBm) to ensure it reaches!

    this->write_strobe(0x36); // SIDLE
  }

  void send_raw_string(const char *cmd) {
    size_t len = strlen(cmd);
    
    portDISABLE_INTERRUPTS();
    
    // 1. Send the crucial "00" preamble to tune the receiver AGC
    gpio_set_level(this->gdo0_pin, 0);
    ets_delay_us(417); // Delay bumped to 417 to match Arduino overhead
    gpio_set_level(this->gdo0_pin, 0);
    ets_delay_us(417);

    // 2. Iterate over the literal text string exactly like the old code did
    for (size_t i = 0; i < len; i++) {
      gpio_set_level(this->gdo0_pin, (cmd[i] == '1') ? 1 : 0);
      ets_delay_us(417); 
    }
    
    portENABLE_INTERRUPTS();
  }

  void transmit_command(int index) {
    const char *cmd = SPEED_SETTINGS[index];

    ESP_LOGD("quiet_cool", "Transmitting string payload #%d", index);

    for (int i = 0; i < 3; i++) {
      gpio_set_level(this->gdo0_pin, 0);
      this->write_strobe(0x35); // Enter TX mode
      
      ets_delay_us(1000); // Give CC1101 time to calibrate PLL and enter TX
      
      this->send_raw_string(cmd);

      this->write_strobe(0x36); // Back to SIDLE
      
      delay(28); // 10ms + 18ms original delays
    }
  }

  void turn_off() { this->transmit_command(7); }         // HOFF
  void turn_on_high() { this->transmit_command(6); }     // HON
  void turn_on_medium() { this->transmit_command(13); }  // MON
  void turn_on_low() { this->transmit_command(20); }     // LON
};

}  
}
