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


enum class SamplingMode : uint8_t { 
  AVG = 0, 
  MIN = 1, 
  MAX = 2 
};

enum class UnitMode : uint8_t {
  ADC_CONV_SINGLE_UNIT_1 = 0,
  ADC_CONV_SINGLE_UNIT_2 = 1,
  ADC_CONV_BOTH_UNIT = 2,
  ADC_CONV_ALTER_UNIT = 3
};

enum class BitWidthMode : uint8_t {
  ADC_BITWIDTH_DEFAULT = 0,
  ADC_BITWIDTH_9 = 1,
  ADC_BITWIDTH_10 = 2,
  ADC_BITWIDTH_11 = 3,
  ADC_BITWIDTH_12 = 4,
  ADC_BITWIDTH_13 = 5
};

enum class WorkingMode : uint8_t { 
  ADC_ULP_MODE_DISABLE = 0, 
  ADC_ULP_MODE_FSM = 1,
  ADC_ULP_MODE_RISCV = 2
};


const LogString *sampling_mode_to_str(SamplingMode mode);

class Aggregator {
  public:
   void add_sample(uint32_t value);
   uint32_t aggregate();
   Aggregator(SamplingMode mode);
 
  protected:
   SamplingMode mode_{SamplingMode::AVG};
   uint32_t aggr_{0};
   uint32_t samples_{0};
 };

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
 
 void set_attenuation(adc_atten_t attenuation) { this->attenuation_ = attenuation; }

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
  void set_sampling_mode(SamplingMode sampling_mode);


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
  SamplingMode sampling_mode_{SamplingMode::AVG};
  WorkingMode working_mode_{WorkingMode::ADC_ULP_MODE_DISABLE};
  BitWidthMode bitwidth_mode_{BitWidthMode::ADC_BITWIDTH_DEFAULT};
  UnitMode unit_mode_{UnitMode::ADC_CONV_SINGLE_UNIT_1};
#ifdef USE_ESP32
  adc_atten_t attenuation_{ADC_ATTEN_DB_12};

#endif

#ifdef USE_RP2040
  bool is_temperature_{false};
#endif

};

}  // namespace adc_oneshot
}  // namespace esphome
