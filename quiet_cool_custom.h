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
    this->init_cc1101_base();
  }

  uint8_t read_reg(uint8_t addr) {
    this->enable();
    gpio_set_level(this->cs_pin, 0);
    uint8_t header = (addr >= 0x30) ? (addr | 0xC0) : (addr | 0x80);
    this->write_byte(header);
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

  void write_fifo(const uint8_t *data, size_t len) {
    this->enable();
    gpio_set_level(this->cs_pin, 0);
    this->write_byte(0x7F);
    for (size_t i = 0; i < len; i++) {
      this->write_byte(data[i]);
    }
    gpio_set_level(this->cs_pin, 1);
    this->disable();
  }

  void set_frequency(uint32_t freq_hz) {
    uint32_t freq_reg = (uint32_t)(((uint64_t)freq_hz << 16) / 26000000);
    this->write_reg(0x0D, (freq_reg >> 16) & 0xFF);
    this->write_reg(0x0E, (freq_reg >> 8) & 0xFF);
    this->write_reg(0x0F, freq_reg & 0xFF);
  }

  void init_cc1101_base() {
    this->write_strobe(0x30); // Reset
    delay(10);

    this->write_reg(0x00, 0x29); 
    this->write_reg(0x01, 0x2E); 
    this->write_reg(0x02, 0x2D); // GDO0 Serial Out
    this->write_reg(0x03, 0x07); 
    this->write_reg(0x04, 0xD3); 
    this->write_reg(0x05, 0x91); 
    this->write_reg(0x06, 0x14); 
    this->write_reg(0x07, 0x04); 
    this->write_reg(0x08, 0x32); // Async direct mode default
    this->write_reg(0x09, 0x00); 
    this->write_reg(0x0A, 0x00); 
    this->write_reg(0x0B, 0x06); 
    this->write_reg(0x0C, 0x00); 

    this->set_frequency(433897000); // 433.897 MHz default

    this->write_reg(0x10, 0xF6); // 2500 baud
    this->write_reg(0x11, 0x83); 
    this->write_reg(0x12, 0x00); // 2-FSK
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
    ESP_LOGI("quiet_cool", "CC1101 Radio Ready (2500 Baud / 400us URH Timing).");
  }

  void send_raw_bits(const uint8_t *data, size_t len, uint32_t bit_delay_us) {
    portDISABLE_INTERRUPTS();
    for (size_t i = 0; i < len; i++) {
      uint8_t b = data[i];
      for (int bit = 7; bit >= 0; bit--) {
        gpio_set_level(this->gdo0_pin, (b >> bit) & 1);
        ets_delay_us(bit_delay_us);
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

    ESP_LOGD("quiet_cool", "Transmitting CMD 0x%02X at 2500 baud (400us)...", cmd_byte);

    for (int i = 0; i < 6; i++) {
      gpio_set_level(this->gdo0_pin, 0);
      this->write_strobe(0x35); // STX
      
      ets_delay_us(1000); 
      
      this->send_raw_bits(packet, 20, 400); // 400us matching URH recording

      this->write_strobe(0x36); // SIDLE
      delay(25); 
    }
  }

  // --- NON-BLOCKING WATCHDOG-SAFE RF SWEEP ---
  void run_full_rf_sweep() {
    ESP_LOGI("rf_sweep", "==================================================");
    ESP_LOGI("rf_sweep", ">>> STARTING QUIETCOOL FULL RF COMBINATION SWEEP <<<");
    ESP_LOGI("rf_sweep", "==================================================");

    uint8_t cmd_high = 0xBF; // High Speed
    
    uint8_t packet_std[20] = {
      0x15, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 
      remote_id[0], remote_id[1], remote_id[2], remote_id[3], 
      remote_id[4], remote_id[5], remote_id[6],             
      cmd_high, cmd_high, 0x00, 0x00                                            
    };

    uint32_t freqs[] = {433897000, 433920000, 433880000};
    const char* freq_names[] = {"433.897MHz", "433.920MHz", "433.880MHz"};

    int test_num = 1;

    for (int f = 0; f < 3; f++) {
      this->set_frequency(freqs[f]);

      uint32_t bit_delays[] = {400, 395, 405}; // 2500 Baud URH timing +/- 1%
      for (int d = 0; d < 3; d++) {
        ESP_LOGI("rf_sweep", "[TEST #%d] Async GPIO Bit-Bang | Freq: %s | Delay: %dus | Bursts: 8", test_num++, freq_names[f], bit_delays[d]);
        
        this->write_reg(0x02, 0x2D);
        this->write_reg(0x08, 0x32);
        
        for (int b = 0; b < 8; b++) {
          this->write_strobe(0x35);
          ets_delay_us(1000);
          this->send_raw_bits(packet_std, 20, bit_delays[d]);
          this->write_strobe(0x36);
          delay(20);
        }
        
        App.feed_wdt();
        vTaskDelay(pdMS_TO_TICKS(2000));
      }

      ESP_LOGI("rf_sweep", "[TEST #%d] HW FIFO Mode | Freq: %s | PKTCTRL0: 0x00 | Bursts: 10", test_num++, freq_names[f]);
      this->write_reg(0x02, 0x06);
      this->write_reg(0x06, 0x14);
      this->write_reg(0x08, 0x00);

      for (int b = 0; b < 10; b++) {
        this->write_strobe(0x36);
        this->write_strobe(0x3B);
        this->write_fifo(packet_std, 20);
        this->write_strobe(0x35);
        delay(25);
      }
      this->write_strobe(0x36);
      
      App.feed_wdt();
      vTaskDelay(pdMS_TO_TICKS(2000));
    }

    ESP_LOGI("rf_sweep", "==================================================");
    ESP_LOGI("rf_sweep", ">>> QUIETCOOL RF SWEEP COMPLETE (%d Tests Run) <<<", test_num - 1);
    ESP_LOGI("rf_sweep", "==================================================");
  }

  void turn_off() { this->transmit_command(SPEED_HIGH, DUR_OFF); }
  void turn_on_high() { this->transmit_command(SPEED_HIGH, DUR_ON); }
  void turn_on_medium() { this->transmit_command(SPEED_MEDIUM, DUR_ON); }
  void turn_on_low() { this->transmit_command(SPEED_LOW, DUR_ON); }
};

}  
}
