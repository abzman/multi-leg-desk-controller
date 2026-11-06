# Hardware test sketches

One small sketch per part of the desk, so you can bring the hardware up
piece by piece before running the full firmware. Each one uses the same
`Config.h` as the firmware, so it tests your actual pins, addresses and
settings. Open a test's `.ino` in the Arduino IDE, flash it, and open the
Serial Monitor at **115200 baud** with line ending set to **Newline**.

Flashing a test doesn't erase the firmware's saved settings (position,
presets, calibration). Flash the firmware again afterwards and it carries on.

## Suggested bring-up order

| # | Sketch | Needs | What it proves |
|---|--------|-------|----------------|
| 1 | `test_display` | display on GPIO 23/27 | TM1651 answers, every segment works, brightness, all firmware messages |
| 2 | `test_buttons` | UP/DOWN on GPIO 18/19 | presses, hold times, contact bounce, the both-buttons combo |
| 3 | `test_preset_ladder` | R-2R ladder on GPIO 33 | each preset button's voltage, noise and margin; chords rejected |
| 4 | `test_motor_driver` | BTS7960 board on I2C 21/22, 24 V | bus, address, PWM, each port both ways, brake/coast, raw channels |
| 5 | `test_encoders` | encoders (+ external pull-ups) | counts, speed, raw A/B levels; finds dead lines and missing pull-ups |
| 6 | `test_leg` | all of the above | each leg moves and counts; measure `COUNTS_PER_MM`; duty sweep for `MIN_DUTY`/`KFF` |
| 7 | `test_settings` | nothing | shows or resets what the firmware saved in flash |
| 8 | `test_wifi_mqtt` | `secrets.h` copied in | Wi-Fi, broker login, publish/subscribe round trip |

Then flash the firmware and run `calib` (directions), `home`, and
`btncal` (preset buttons).

## Safety

`test_motor_driver` and `test_leg` drive motors **open loop**, with no
synchronisation:

* Every run is time-limited, and **any key in the Serial Monitor stops it**.
* Driving **one leg on its own tilts the desk**. Keep duties low (0.2–0.3)
  and runs short, or test with the desktop off / motors on the bench.
* `test_leg`'s multi-leg moves stop if the legs drift more than
  `SYNC_FAULT_MM` apart.

## Keeping the tests in sync

The Arduino IDE only compiles files inside a sketch's own folder, so each
test has its own copy of the headers it needs (`Config.h`, `MotorDriver.h`,
`QuadEncoder.h`, `TM1651Display.h`). After changing any of them in the main
sketch, run:

    python3 update_headers.py

It also copies `secrets.h` into `test_wifi_mqtt` if the main sketch has one.

## Command summary

| Sketch | Commands |
|--------|----------|
| test_display | `show <txt>` `b <0-7>` `on` `off` `seg <digit> <hex>` `walk` `count` `msgs` `all` |
| test_buttons | (none, just press buttons) |
| test_preset_ladder | `live` `levels` `noise` `table` |
| test_motor_driver | `scan` `run <port> <duty> [ms]` `all <duty> [ms]` `brake <p>` `coast <p>` `regs` `ch <n> <val>` `power on/off` `map` |
| test_encoders | `z` `raw` `fast` `p` |
| test_leg | `pulse <leg> <duty> [ms]` `all <duty> [ms]` `scale <mm>` `sweep` `show` |
| test_settings | `dump` `clear position/presets/directions/buttons` `clear all yes` |
| test_wifi_mqtt | `pub <text>` `status` |
