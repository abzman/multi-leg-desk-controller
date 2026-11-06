// =====================================================================
//  QuadEncoder.h  —  4x quadrature decoding, two interchangeable backends
//
//    PCNT backend : ESP32 hardware pulse counter. Needs Arduino-ESP32
//                   core 3.x (ESP-IDF 5). Counts all 4 edges, has a
//                   hardware glitch filter, and accumulates to 32 bits.
//    ISR backend  : CHANGE interrupts on both pins with a state-table
//                   decoder. Works on any ESP32 or core version. Good to a
//                   few tens of kHz of edges per encoder.
// =====================================================================
#pragma once
#include <Arduino.h>
#include "Config.h"

#if ENCODER_USE_PCNT && defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define QE_PCNT 1
  #include "driver/pulse_cnt.h"
  #include "driver/gpio.h"
#else
  #define QE_PCNT 0
  #include "soc/soc.h"
  #include "soc/gpio_reg.h"
  #include "soc/soc_caps.h"
  // Everything the ISR touches must sit in IRAM/DRAM. NVS (flash) writes
  // disable the flash cache, and an ISR reading from flash would crash.
  // index = (prev<<2)|curr : +1/-1 for valid steps, 0 for none or invalid
  static DRAM_ATTR const int8_t QE_TABLE[16] = { 0,-1, 1, 0,  1, 0, 0,-1,  -1, 0, 0, 1,  0, 1,-1, 0 };
  static inline uint8_t IRAM_ATTR qeFastRead(uint8_t pin) {
  #if SOC_GPIO_PIN_COUNT > 32
    if (pin >= 32) return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
  #endif
    return (REG_READ(GPIO_IN_REG) >> pin) & 1;
  }
#endif

class QuadEncoder {
public:
  bool begin(uint8_t pinA, uint8_t pinB, bool invert) {
    _a = pinA; _b = pinB; _invert = invert;
#if QE_PCNT
    return beginPCNT();
#else
    return beginISR();
#endif
  }

  // Raw count with the direction fix and zero offset applied.
  int32_t read() {
    int32_t raw;
#if QE_PCNT
    int c = 0;
    pcnt_unit_get_count(_unit, &c);
    raw = c;
#else
    portENTER_CRITICAL(&_mux);
    raw = _count;
    portEXIT_CRITICAL(&_mux);
#endif
    if (_invert) raw = -raw;
    return raw - _offset;
  }

  // Make the current physical position read as 'value'.
  void set(int32_t value) {
    _offset = 0;
    _offset = read() - value;
  }

  // Change direction at runtime. The reading flips sign, so callers
  // re-zero afterwards.
  void setInvert(bool invert) { _invert = invert; }
  bool inverted() const { return _invert; }

  const char* backend() const { return QE_PCNT ? "PCNT" : "ISR"; }

private:
  uint8_t _a = 0, _b = 0;
  bool    _invert = false;
  int32_t _offset = 0;

#if QE_PCNT
  pcnt_unit_handle_t    _unit = nullptr;
  pcnt_channel_handle_t _chA  = nullptr, _chB = nullptr;

  bool beginPCNT() {
    pcnt_unit_config_t uc = {};
    uc.low_limit  = -30000;
    uc.high_limit =  30000;
    uc.flags.accum_count = 1;          // extend to 32 bits in software on wrap
    if (pcnt_new_unit(&uc, &_unit) != ESP_OK) return false;

    pcnt_glitch_filter_config_t fc = {};
    fc.max_glitch_ns = ENCODER_GLITCH_NS;
    pcnt_unit_set_glitch_filter(_unit, &fc);

    // Channel A counts on A edges, with B as the direction level.
    pcnt_chan_config_t ca = {};
    ca.edge_gpio_num  = _a;
    ca.level_gpio_num = _b;
    if (pcnt_new_channel(_unit, &ca, &_chA) != ESP_OK) return false;

    // Channel B counts on B edges, with A as the direction level.
    pcnt_chan_config_t cb = {};
    cb.edge_gpio_num  = _b;
    cb.level_gpio_num = _a;
    if (pcnt_new_channel(_unit, &cb, &_chB) != ESP_OK) return false;

    pcnt_channel_set_edge_action (_chA, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(_chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP,    PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action (_chB, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(_chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP,    PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    // Watch points at the limits are required for accum_count to work.
    pcnt_unit_add_watch_point(_unit, uc.high_limit);
    pcnt_unit_add_watch_point(_unit, uc.low_limit);

    // Weak internal pull-ups help on pins that have them (not 34–39).
    // External pull-ups are still recommended.
    if (_a < 34) gpio_pullup_en((gpio_num_t)_a);
    if (_b < 34) gpio_pullup_en((gpio_num_t)_b);

    pcnt_unit_enable(_unit);
    pcnt_unit_clear_count(_unit);
    pcnt_unit_start(_unit);
    return true;
  }
#else
  volatile int32_t _count = 0;
  volatile uint8_t _state = 0;
  portMUX_TYPE     _mux   = portMUX_INITIALIZER_UNLOCKED;

  static void IRAM_ATTR isr(void* arg) {
    QuadEncoder* e = static_cast<QuadEncoder*>(arg);
    uint8_t s = (qeFastRead(e->_a) << 1) | qeFastRead(e->_b);
    portENTER_CRITICAL_ISR(&e->_mux);
    e->_count += QE_TABLE[(e->_state << 2) | s];
    e->_state = s;
    portEXIT_CRITICAL_ISR(&e->_mux);
  }

  bool beginISR() {
    pinMode(_a, _a < 34 ? INPUT_PULLUP : INPUT);
    pinMode(_b, _b < 34 ? INPUT_PULLUP : INPUT);
    _state = (digitalRead(_a) << 1) | digitalRead(_b);
    attachInterruptArg(digitalPinToInterrupt(_a), isr, this, CHANGE);
    attachInterruptArg(digitalPinToInterrupt(_b), isr, this, CHANGE);
    return true;
  }
#endif
};
