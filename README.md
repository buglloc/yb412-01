# YB412-01

Minimal ATtiny412 firmware that emulates an EM4100/EM-Marine compatible 125 kHz RFID tag payload.

## Hardware

- MCU: ATtiny412
- `PA2` / physical pin 5: `FIELD_IN`, digital input, no internal pull-up
- `PA3` / physical pin 7: `MOD_OUT`, active-high driver output for a NPN base
- `PA7` / physical pin 3: `STATUS_LED`, active-high PWM output

## Tag ID

The EM4100 payload is 40 bits:

- customer code: 8 bits
- serial number: 32 bits

Set the payload in `platformio.ini`:

```ini
build_flags =
  -DTAG_CUSTOMER_CODE=0x12
  -DTAG_SERIAL_NUMBER=0x3456789A
```

Use hexadecimal values without quotes. The firmware requires both flags and validates that the customer code fits in 8 bits and the serial number fits in 32 bits.

## Build

```sh
pio run
```

## Upload

```sh
pio run -t upload
```
