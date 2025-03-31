#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#include <esp_adc/adc_cali.h>             // Migration from 4.4 to 5.0
#include "esp_adc/adc_cali_scheme.h"      // Migration from 4.4 to 5.0
#include "esp_adc/adc_oneshot.h"          // Migration from 4.4 to 5.0
#endif

namespace esphome {
namespace adc_oneshot {

#ifdef USE_ESP32
// clang-format off
#if (ESP_IDF_VERSION_MAJOR == 4 && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 7)) || \
    (ESP_IDF_VERSION_MAJOR == 5 && \
     ((ESP_IDF_VERSION_MINOR == 0 && ESP_IDF_VERSION_PATCH >= 5) || \
      (ESP_IDF_VERSION_MINOR == 1 && ESP_IDF_VERSION_PATCH >= 3) || \
      (ESP_IDF_VERSION_MINOR >= 2)) \
    )
// clang-format on
static const adc_atten_t ADC_ATTEN_DB_12_COMPAT = ADC_ATTEN_DB_12;
#else
static const adc_atten_t ADC_ATTEN_DB_12_COMPAT = ADC_ATTEN_DB_11;
#endif
#endif  // USE_ESP32

class ADCOneshotSensor : public sensor::Sensor, public PollingComponent, public voltage_sampler::VoltageSampler {
 public:
#ifdef USE_ESP32
  adc_channel_t channel1_;
  adc_channel_t channel2_;

  void set_channel1(adc_channel_t channel) {
    this->channel1_ = channel;
  }

  void set_channel2(adc_channel_t channel) {
    this->channel2_ = channel;
    this->channel1_ = ADC_CHANNEL_9;
 }
 

#endif

  /// Update ADC values
  void update() override;
  /// Setup ADC
  void setup() override;
  void dump_config() override;
  /// `HARDWARE_LATE` setup priority
  float get_setup_priority() const override;
  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }
  void set_sample_count(uint8_t sample_count);
  float sample() override;

#ifdef USE_ESP8266
  std::string unique_id() override;
#endif

#ifdef USE_RP2040
  void set_is_temperature() { this->is_temperature_ = true; }
#endif

 protected:
  InternalGPIOPin *pin_;
  bool output_raw_{false};
  uint8_t sample_count_{1};

#ifdef USE_RP2040
  bool is_temperature_{false};
#endif

};

}  // namespace adc_oneshot
}  // namespace esphome
