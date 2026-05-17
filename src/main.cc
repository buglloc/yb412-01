#include <Arduino.h>

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

#ifndef TAG_CUSTOMER_CODE
#error "TAG_CUSTOMER_CODE must be defined in platformio.ini"
#endif

#ifndef TAG_SERIAL_NUMBER
#error "TAG_SERIAL_NUMBER must be defined in platformio.ini"
#endif

/*
Hardware, ATtiny412:
  - PA2 / physical pin 5 -> FIELD_IN, digital input without internal pull-up
  - PA3 / physical pin 7 -> MOD_OUT, active-high
  - PA7 / physical pin 3 -> STATUS_LED, active-high PWM output
*/

namespace {

namespace tag_cfg {
// EM4100/EM-Marine compatible 40-bit payload.
constexpr uint8_t kCustomerCode = static_cast<uint8_t>(TAG_CUSTOMER_CODE);
constexpr uint32_t kSerialNumber = static_cast<uint32_t>(TAG_SERIAL_NUMBER);
static_assert(TAG_CUSTOMER_CODE <= UINT8_MAX);
static_assert(TAG_SERIAL_NUMBER <= UINT32_MAX);
} // namespace tag_cfg

namespace pin_cfg {
constexpr uint8_t kModOutMask = PIN3_bm;
constexpr uint8_t kFieldInMask = PIN2_bm;
constexpr uint8_t kStatusLedMask = PIN7_bm;
} // namespace pin_cfg

namespace field_cfg {
constexpr uint8_t kSamples = 32;
constexpr uint8_t kMinHighSamples = 24;
constexpr uint8_t kSampleMicros = 50;
constexpr uint8_t kFramesPerCheck = 32;
constexpr uint8_t kMaxConsecutiveMisses = 2;
static_assert(kMinHighSamples <= kSamples);
static_assert(kMaxConsecutiveMisses > 0);
} // namespace field_cfg

namespace modulation_cfg {
constexpr uint8_t kFrameBits = 64;
constexpr uint8_t kFrameBytes = kFrameBits / 8;
constexpr uint16_t kFrameHalfBits = kFrameBits * 2;
constexpr uint16_t kHalfBitMicros = 256; // RF/64 at 125 kHz
constexpr uint16_t kHalfBitTimerTicks = (F_CPU / 1000000UL) * kHalfBitMicros;
static_assert((kFrameBits % 8) == 0);
static_assert(kFrameHalfBits <= UINT8_MAX);
static_assert(kHalfBitTimerTicks <= UINT16_MAX);
} // namespace modulation_cfg

namespace led_cfg {
constexpr uint8_t kPwmTop = 254;
constexpr uint8_t kPwmClockSelect = TCA_SPLIT_CLKSEL_DIV16_gc;
constexpr uint8_t kTcaPortmuxMask = PORTMUX_TCA00_bm; // TCA0 WO0 on PA7 for ATtiny412.
constexpr uint8_t kTcaOutputEnableMask = TCA_SPLIT_LCMP0EN_bm;
constexpr uint8_t kStartDuty = 4;
constexpr uint8_t kMinDuty = 3;
constexpr uint8_t kMaxDuty = 28;
constexpr uint8_t kDutyStep = 2;
constexpr uint8_t kFramesPerStep = 1;
constexpr uint8_t kFadeOutDutyStep = 2;
constexpr uint16_t kFadeOutStepDelayMicros = 6000;
static_assert(kMinDuty <= kStartDuty);
static_assert(kStartDuty <= kMaxDuty);
static_assert(kMaxDuty <= kPwmTop);
static_assert(kDutyStep > 0);
static_assert(kFramesPerStep > 0);
static_assert(kFadeOutDutyStep > 0);
} // namespace led_cfg

struct Em4100Frame
{
  uint8_t bytes[modulation_cfg::kFrameBytes];

  void clear()
  {
    for (uint8_t &byte : bytes) {
      byte = 0;
    }
  }

  bool getBit(uint8_t index) const
  {
    const uint8_t mask = static_cast<uint8_t>(0x80 >> (index & 0x07));
    return (bytes[index >> 3] & mask) != 0;
  }

  void setBit(uint8_t index, bool value)
  {
    const uint8_t mask = static_cast<uint8_t>(0x80 >> (index & 0x07));
    uint8_t &byte = bytes[index >> 3];
    if (value) {
      byte |= mask;
    } else {
      byte &= ~mask;
    }
  }
};

struct RuntimeState
{
  bool transmitting;
  uint8_t fieldMisses;
};

Em4100Frame g_frame = {};
RuntimeState g_runtime = {};

namespace status_led {

struct LedState
{
  uint8_t duty;
  uint8_t frameTicks;
  int8_t dutyDirection;
  bool active;
};

volatile LedState g_state = {};

void routeTcaToLedPin()
{
#if defined(PORTMUX_TCAROUTEA)
  PORTMUX.TCAROUTEA = (PORTMUX.TCAROUTEA & ~led_cfg::kTcaPortmuxMask) | led_cfg::kTcaPortmuxMask;
#elif defined(PORTMUX_CTRLC)
  PORTMUX.CTRLC = (PORTMUX.CTRLC & ~led_cfg::kTcaPortmuxMask) | led_cfg::kTcaPortmuxMask;
#endif
}

void setDuty(uint8_t duty)
{
  TCA0.SPLIT.LCMP0 = duty;
  if (duty == 0) {
    TCA0.SPLIT.CTRLB &= ~led_cfg::kTcaOutputEnableMask;
    VPORTA.OUT &= ~pin_cfg::kStatusLedMask;
  } else {
    TCA0.SPLIT.CTRLB |= led_cfg::kTcaOutputEnableMask;
  }
}

void configure()
{
  VPORTA.OUT &= ~pin_cfg::kStatusLedMask;
  VPORTA.DIR |= pin_cfg::kStatusLedMask;
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;

  routeTcaToLedPin();
  TCA0.SPLIT.CTRLA = 0;
  TCA0.SPLIT.CTRLB = 0;
  TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
  TCA0.SPLIT.INTCTRL = 0;
  TCA0.SPLIT.INTFLAGS = TCA_SPLIT_LUNF_bm | TCA_SPLIT_HUNF_bm;
  TCA0.SPLIT.LPER = led_cfg::kPwmTop;
  TCA0.SPLIT.HPER = led_cfg::kPwmTop;
  setDuty(0);
}

void start()
{
  g_state.duty = led_cfg::kStartDuty;
  g_state.frameTicks = 0;
  g_state.dutyDirection = 1;
  g_state.active = true;

  TCA0.SPLIT.CTRLA = led_cfg::kPwmClockSelect | TCA_SPLIT_ENABLE_bm;
  setDuty(led_cfg::kStartDuty);
}

void stop()
{
  setDuty(0);
  TCA0.SPLIT.CTRLA = 0;
  g_state.duty = 0;
  g_state.frameTicks = 0;
  g_state.dutyDirection = 1;
  g_state.active = false;
}

void fadeOutAndStop()
{
  if (!g_state.active) {
    stop();
    return;
  }

  g_state.active = false;
  uint8_t duty = g_state.duty;

  while (duty > 0) {
    duty = duty > led_cfg::kFadeOutDutyStep ? duty - led_cfg::kFadeOutDutyStep : 0;
    setDuty(duty);

    if (duty > 0) {
      delayMicroseconds(led_cfg::kFadeOutStepDelayMicros);
    }
  }

  TCA0.SPLIT.CTRLA = 0;
  g_state.duty = 0;
  g_state.frameTicks = 0;
  g_state.dutyDirection = 1;
}

void handleFrameTick()
{
  if (!g_state.active) {
    return;
  }

  ++g_state.frameTicks;
  if (g_state.frameTicks < led_cfg::kFramesPerStep) {
    return;
  }
  g_state.frameTicks = 0;

  int16_t nextDuty =
      static_cast<int16_t>(g_state.duty) + static_cast<int16_t>(g_state.dutyDirection) * led_cfg::kDutyStep;
  if (nextDuty >= led_cfg::kMaxDuty) {
    nextDuty = led_cfg::kMaxDuty;
    g_state.dutyDirection = -1;
  } else if (nextDuty <= led_cfg::kMinDuty) {
    nextDuty = led_cfg::kMinDuty;
    g_state.dutyDirection = 1;
  }

  g_state.duty = static_cast<uint8_t>(nextDuty);
  setDuty(g_state.duty);
}

} // namespace status_led

namespace modulator {

struct ModulatorState
{
  const Em4100Frame *activeFrame;
  uint8_t nextBit;
  uint8_t framesRemaining;
  bool nextSecondHalf;
  bool finishing;
  bool running;
};

volatile ModulatorState g_state = {};

inline void setActive(bool active)
{
  if (active) {
    VPORTA.OUT |= pin_cfg::kModOutMask;
  } else {
    VPORTA.OUT &= ~pin_cfg::kModOutMask;
  }
}

void configureOutput()
{
  VPORTA.DIR |= pin_cfg::kModOutMask;
  setActive(false);
}

void configureTimer()
{
  TCB0.CTRLA = 0;
  TCB0.CTRLB = TCB_CNTMODE_INT_gc;
  TCB0.EVCTRL = 0;
  TCB0.CCMP = modulation_cfg::kHalfBitTimerTicks;
  TCB0.CNT = 0;
  TCB0.INTCTRL = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
}

void stopTimer()
{
  TCB0.CTRLA = 0;
  TCB0.INTCTRL = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
  setActive(false);
  g_state.running = false;
}

void setManchesterHalf(bool bit, bool secondHalf)
{
  // Validated against the reader: 1 is low-to-high, 0 is high-to-low.
  setActive(secondHalf ? bit : !bit);
}

void advanceManchesterHalf()
{
  if (g_state.finishing) {
    stopTimer();
    return;
  }

  const Em4100Frame *frame = g_state.activeFrame;
  const bool bit = frame->getBit(g_state.nextBit);
  setManchesterHalf(bit, g_state.nextSecondHalf);

  if (g_state.nextSecondHalf) {
    g_state.nextSecondHalf = false;
    ++g_state.nextBit;

    if (g_state.nextBit >= modulation_cfg::kFrameBits) {
      g_state.nextBit = 0;
      status_led::handleFrameTick();
      --g_state.framesRemaining;
      g_state.finishing = g_state.framesRemaining == 0;
    }
  } else {
    g_state.nextSecondHalf = true;
  }
}

void idleUntilBurstDone()
{
  set_sleep_mode(SLEEP_MODE_IDLE);

  while (true) {
    cli();
    if (!g_state.running) {
      sleep_disable();
      sei();
      return;
    }

    sleep_enable();
    sei();
    sleep_cpu();
    sleep_disable();
  }
}

void sendFrames(const Em4100Frame &frame, uint8_t frameCount)
{
  if (frameCount == 0) {
    return;
  }

  cli();
  g_state.activeFrame = &frame;
  g_state.nextBit = 0;
  g_state.framesRemaining = frameCount;
  g_state.nextSecondHalf = true;
  g_state.finishing = false;
  g_state.running = true;

  setManchesterHalf(frame.getBit(0), false);
  TCB0.CNT = 0;
  TCB0.INTFLAGS = TCB_CAPT_bm;
  TCB0.INTCTRL = TCB_CAPTEI_bm;
  TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | TCB_ENABLE_bm;
  sei();

  idleUntilBurstDone();
}

void handleTimerInterrupt()
{
  TCB0.INTFLAGS = TCB_CAPT_bm;
  if (g_state.running) {
    advanceManchesterHalf();
  }
}

} // namespace modulator

namespace field {

inline bool readInput()
{
  return (VPORTA.IN & pin_cfg::kFieldInMask) != 0;
}

void configureInput()
{
  VPORTA.DIR &= ~pin_cfg::kFieldInMask;
  PORTA.PIN2CTRL = PORT_ISC_INTDISABLE_gc;
  VPORTA.INTFLAGS = pin_cfg::kFieldInMask;
}

bool isPresent()
{
  uint8_t highSamples = 0;

  for (uint8_t i = 0; i < field_cfg::kSamples; ++i) {
    if (readInput()) {
      ++highSamples;
      if (highSamples >= field_cfg::kMinHighSamples) {
        return true;
      }
    }
    delayMicroseconds(field_cfg::kSampleMicros);
  }

  return false;
}

void enableWakeInterrupt()
{
  // PA2 is fully asynchronous, so rising edge can wake from power-down.
  VPORTA.INTFLAGS = pin_cfg::kFieldInMask;
  PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | PORT_ISC_RISING_gc;
}

void disableWakeInterrupt()
{
  PORTA.PIN2CTRL &= ~PORT_ISC_gm;
  VPORTA.INTFLAGS = pin_cfg::kFieldInMask;
}

} // namespace field

namespace em4100 {

constexpr uint8_t kHeaderBits = 9;
constexpr uint8_t kPayloadNibbles = 10;
constexpr uint8_t kNibbleBits = 4;
constexpr uint8_t kStopBits = 1;
static_assert(kHeaderBits + kPayloadNibbles * (kNibbleBits + 1) + kNibbleBits + kStopBits ==
              modulation_cfg::kFrameBits);

void appendBit(Em4100Frame &frame, uint8_t &index, bool bit)
{
  frame.setBit(index++, bit);
}

void buildFrame(Em4100Frame &frame)
{
  const uint64_t payload = (static_cast<uint64_t>(tag_cfg::kCustomerCode) << 32) | tag_cfg::kSerialNumber;

  frame.clear();
  uint8_t bitIndex = 0;
  uint8_t columnParity[kNibbleBits] = {};

  for (uint8_t i = 0; i < kHeaderBits; ++i) {
    appendBit(frame, bitIndex, true);
  }

  for (uint8_t nibbleIndex = 0; nibbleIndex < kPayloadNibbles; ++nibbleIndex) {
    const uint8_t shift = (kPayloadNibbles - 1 - nibbleIndex) * kNibbleBits;
    const uint8_t nibble = (payload >> shift) & 0x0F;
    uint8_t rowParity = 0;

    for (uint8_t column = 0; column < kNibbleBits; ++column) {
      const bool bit = ((nibble >> (kNibbleBits - 1 - column)) & 0x01) != 0;
      appendBit(frame, bitIndex, bit);
      rowParity ^= bit;
      columnParity[column] ^= bit;
    }

    appendBit(frame, bitIndex, rowParity != 0);
  }

  for (uint8_t column = 0; column < kNibbleBits; ++column) {
    appendBit(frame, bitIndex, columnParity[column] != 0);
  }

  appendBit(frame, bitIndex, false);
}

} // namespace em4100

namespace board_power {

void configureUnusedPins()
{
  PORTA.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;

#ifdef ADC_ENABLE_bm
  ADC0.CTRLA &= ~ADC_ENABLE_bm;
#endif
}

} // namespace board_power

namespace sleep_control {

void untilFieldHigh()
{
  modulator::setActive(false);

  cli();
  if (field::readInput()) {
    sei();
    return;
  }

  field::enableWakeInterrupt();
  if (field::readInput()) {
    field::disableWakeInterrupt();
    sei();
    return;
  }

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
#if defined(BODS) && defined(BODSE)
  sleep_bod_disable();
#endif
  sei();
  sleep_cpu();

  cli();
  sleep_disable();
  field::disableWakeInterrupt();
  sei();
}

} // namespace sleep_control

namespace app {

void startTransmitting()
{
  g_runtime.transmitting = true;
  g_runtime.fieldMisses = 0;
  status_led::start();
}

void stopTransmittingAndSleep()
{
  g_runtime.transmitting = false;
  g_runtime.fieldMisses = 0;
  status_led::fadeOutAndStop();
  sleep_control::untilFieldHigh();
}

void setup()
{
  field::configureInput();
  modulator::configureOutput();
  modulator::configureTimer();
  board_power::configureUnusedPins();
  status_led::configure();
  em4100::buildFrame(g_frame);
  sei();
}

void loop()
{
  if (!g_runtime.transmitting) {
    if (!field::isPresent()) {
      sleep_control::untilFieldHigh();
      return;
    }

    startTransmitting();
  }

  modulator::sendFrames(g_frame, field_cfg::kFramesPerCheck);

  if (field::isPresent()) {
    g_runtime.fieldMisses = 0;
    return;
  }

  ++g_runtime.fieldMisses;
  if (g_runtime.fieldMisses < field_cfg::kMaxConsecutiveMisses) {
    return;
  }

  stopTransmittingAndSleep();
}

} // namespace app

} // namespace

ISR(PORTA_PORT_vect)
{
  const uint8_t flags = VPORTA.INTFLAGS;
  VPORTA.INTFLAGS = flags;
  sleep_disable();
}

ISR(TCB0_INT_vect)
{
  modulator::handleTimerInterrupt();
}

void setup()
{
  app::setup();
}

void loop()
{
  app::loop();
}
