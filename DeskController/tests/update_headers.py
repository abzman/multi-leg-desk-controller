#!/usr/bin/env python3
"""Copy the shared headers from the main sketch into each test sketch.

The Arduino IDE only compiles files inside a sketch's own folder, so each
test carries its own copy of the headers it needs. Run this after you
change Config.h (pins, addresses, COUNTS_PER_MM...) or any driver header,
so the tests keep matching the firmware:

    python3 update_headers.py
"""
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
MAIN = HERE.parent                      # the DeskController sketch folder

NEEDS = {
    "test_motor_driver":  ["Config.h", "MotorDriver.h"],
    "test_encoders":      ["Config.h", "QuadEncoder.h"],
    "test_leg":           ["Config.h", "MotorDriver.h", "QuadEncoder.h"],
    "test_buttons":       ["Config.h"],
    "test_preset_ladder": ["Config.h"],
    "test_display":       ["Config.h", "TM1651Display.h"],
    "test_settings":      ["Config.h"],
    "test_wifi_mqtt":     ["Config.h"],
}

for test, headers in NEEDS.items():
    for h in headers:
        shutil.copy2(MAIN / h, HERE / test / h)
    print(f"{test:20s} <- {', '.join(headers)}")

secrets = MAIN / "secrets.h"
if secrets.exists():
    shutil.copy2(secrets, HERE / "test_wifi_mqtt" / "secrets.h")
    print("test_wifi_mqtt       <- secrets.h")
else:
    print("(no secrets.h in the main sketch yet; test_wifi_mqtt needs one)")
