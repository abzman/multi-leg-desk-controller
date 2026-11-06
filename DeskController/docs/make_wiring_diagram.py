#!/usr/bin/env python3
"""Generates docs/wiring_diagram.svg (render to PNG with: rsvg-convert -z 1.5 -b white ...).
Pin numbers match Config.h. Edit here and re-run if you change the wiring."""
from pathlib import Path

W, H = 1500, 1310
out = []
def add(s): out.append(s)

C = dict(v24="#d62728", v5="#ff7f0e", v33="#c2185b", gnd="#222222", i2c="#1f77b4",
         enc="#7b3fa0", btn="#2e7d32", disp="#00838f", lad="#8d6e63", mot="#8b0000", relay="#555555")

def text(x, y, s, size=13, anchor="start", weight="normal", color="#222", italic=False):
    st = ' font-style="italic"' if italic else ''
    add(f'<text x="{x}" y="{y}" font-size="{size}" text-anchor="{anchor}" font-weight="{weight}" fill="{color}"{st}>{s}</text>')

def box(x, y, w, h, title, sub=None, fill="#f7f7f7", stroke="#444"):
    add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="{fill}" stroke="{stroke}" stroke-width="1.5"/>')
    text(x + w / 2, y + 22, title, 15, "middle", "bold")
    if sub: text(x + w / 2, y + 40, sub, 12, "middle", color="#555")

def wire(pts, color, width=2.2, dash=None):
    d = "M" + " L".join(f"{x},{y}" for x, y in pts)
    da = f' stroke-dasharray="{dash}"' if dash else ''
    add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"{da}/>')

def dot(x, y, color): add(f'<circle cx="{x}" cy="{y}" r="3.5" fill="{color}"/>')
def pin(x, y, color): add(f'<circle cx="{x}" cy="{y}" r="4" fill="#fff" stroke="{color}" stroke-width="2"/>')

def tag(x, y, label, color, anchor="middle"):
    """net label pill centred on (x, y)"""
    w = 10 + 7.6 * len(label)
    x0 = x - w / 2 if anchor == "middle" else (x if anchor == "start" else x - w)
    add(f'<rect x="{x0}" y="{y-10}" width="{w}" height="20" rx="10" fill="#fff" stroke="{color}" stroke-width="1.8"/>')
    text(x0 + w / 2, y + 4.5, label, 12, "middle", "bold", color)

def res_v(x, y1, y2, color):  # vertical resistor between y1 and y2 (body in the middle)
    m = (y1 + y2) / 2
    wire([(x, y1), (x, m - 9)], color, 1.8); wire([(x, m + 9), (x, y2)], color, 1.8)
    add(f'<rect x="{x-5}" y="{m-9}" width="10" height="18" rx="2" fill="#fff" stroke="{color}" stroke-width="1.8"/>')

def switch_h(x1, x2, y, color):  # pushbutton, horizontal
    m = (x1 + x2) / 2
    wire([(x1, y), (m - 14, y)], color); wire([(m + 14, y), (x2, y)], color)
    wire([(m - 14, y), (m + 12, y - 10)], color, 2)
    wire([(m, y - 18), (m, y - 6)], color, 1.6); wire([(m - 6, y - 18), (m + 6, y - 18)], color, 1.6)
    dot(m - 14, y, color); dot(m + 14, y, color)

add(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}" '
    f'font-family="DejaVu Sans, Arial, sans-serif">')
add(f'<rect width="{W}" height="{H}" fill="#fff"/>')
text(40, 30, "4-leg standing desk controller: wiring", 20, weight="bold")
text(W - 40, 30, "pins match Config.h  ·  ESP32-WROVER-E", 13, "end", color="#666")

# ---------------------------------------------------------------- power
box(40, 60, 190, 100, "24 V PSU", "sized for 4 motors")
text(225, 88, "+", 16, "end", "bold", C["v24"]); text(225, 138, "−", 18, "end", "bold", C["gnd"])
box(290, 60, 190, 100, "Buck 24 V → 5 V", "≥1 A, for the ESP32")
box(560, 60, 210, 100, "Kill relay / MOSFET", "optional, active HIGH")
# fuse + unswitched 24 V: PSU+ -> fuse -> buck VIN and relay COM
wire([(230, 85), (260, 85)], C["v24"])
add(f'<rect x="242" y="78" width="16" height="14" rx="3" fill="#fff" stroke="{C["v24"]}" stroke-width="1.8"/>')
text(250, 72, "fuse", 11, "middle", color=C["v24"])
wire([(258, 85), (290, 85)], C["v24"])
dot(272, 85, C["v24"]); wire([(272, 85), (272, 48), (530, 48), (530, 135), (560, 135)], C["v24"])
text(400, 44, "+24 V (always on)", 11, "middle", color=C["v24"])
wire([(230, 135), (290, 135)], C["gnd"])
text(290 + 6, 89, "VIN", 11, color="#555"); text(296, 139, "GND", 11, color="#555")
text(566, 139, "COM", 11, color="#555")
# switched 24 V to the motor board
wire([(770, 135), (1120, 135), (1120, 230)], C["v24"], 3)
text(945, 128, "+24 V switched → motor board", 12, "middle", color=C["v24"], weight="bold")
text(764, 139, "NO", 11, "end", color="#555")
# buck outputs -> ESP32 5V / GND
wire([(330, 160), (330, 200), (520, 200), (520, 230)], C["v5"])
wire([(400, 160), (400, 188), (560, 188), (560, 230)], C["gnd"])
text(336, 176, "5V", 11, color=C["v5"]); text(406, 176, "GND", 11, color=C["gnd"])
# relay coil side
tag(590, 182, "5V", C["v5"]); wire([(590, 160), (590, 172)], C["v5"], 1.8)
tag(650, 182, "GND", C["gnd"]); wire([(650, 160), (650, 172)], C["gnd"], 1.8)
wire([(720, 160), (720, 230)], C["relay"])
text(726, 176, "IN", 11, color="#555")

# ---------------------------------------------------------------- ESP32
EX0, EX1, EY0, EY1 = 470, 760, 230, 1250
add(f'<rect x="{EX0}" y="{EY0}" width="{EX1-EX0}" height="{EY1-EY0}" rx="10" fill="#eef3fb" stroke="#1f4e79" stroke-width="2"/>')
text((EX0 + EX1) / 2, 422, "ESP32-WROVER-E", 17, "middle", "bold", "#1f4e79")
text((EX0 + EX1) / 2, 440, "dev board", 12, "middle", color="#555")
text((EX0 + EX1) / 2, 462, "never use:", 11, "middle", color="#a00")
text((EX0 + EX1) / 2, 477, "0 2 5 12 15 (strapping)", 11, "middle", color="#a00")
text((EX0 + EX1) / 2, 492, "6–11 (flash) · 16 17 (PSRAM)", 11, "middle", color="#a00")
text((EX0 + EX1) / 2, 760, "spare: GPIO 14", 11, "middle", color="#555")
# top pins
for x, lab, col in [(520, "5V", C["v5"]), (560, "GND", C["gnd"]), (720, "GPIO 4", C["relay"])]:
    pin(x, EY0, col); text(x, EY0 + 20, lab, 11, "middle", color="#333")
text(752, EY0 + 34, "(PIN_MOTOR_POWER)", 10, "end", color="#777")

def epin_left(y, label, color): pin(EX0, y, color); text(EX0 + 10, y + 4, label, 12, color="#333")
def epin_right(y, label, color): pin(EX1, y, color); text(EX1 - 10, y + 4, label, 12, "end", color="#333")

# ---------------------------------------------------------------- display
box(40, 240, 260, 140, "LED display", "Gotek LEDC68 (TM1651)", "#eaf7f8", C["disp"])
for y, lab in [(300, "CLK"), (340, "DIO")]:
    pin(300, y, C["disp"]); text(292, y + 4, lab, 12, "end")
tag(80, 300, "3V3", C["v33"]); tag(80, 340, "GND", C["gnd"])
text(60, 368, "power from 3.3 V, NOT 5 V", 11, color="#a00")
epin_left(300, "GPIO 23  CLK", C["disp"]); epin_left(340, "GPIO 27  DIO", C["disp"])
wire([(300, 300), (EX0, 300)], C["disp"]); wire([(300, 340), (EX0, 340)], C["disp"])
dot(400, 340, C["disp"]); res_v(400, 340, 372, C["disp"]); wire([(400, 372), (400, 380)], C["v33"], 1.8)
tag(400, 392, "3V3", C["v33"]); text(412, 360, "10k", 11, color="#555"); text(330, 412, "(optional, long wires)", 10, color="#777")

# ---------------------------------------------------------------- UP / DOWN
box(40, 440, 260, 170, "UP / DOWN buttons", "momentary, to GND", "#eef7ee", C["btn"])
for y, lab, gp in [(520, "UP", 18), (570, "DOWN", 19)]:
    tag(78, y, "GND", C["gnd"]); switch_h(98, 260, y, C["btn"])
    text(179, y + 20, lab, 12, "middle", "bold", C["btn"])
    wire([(260, y), (EX0, y)], C["btn"]); epin_left(y, f"GPIO {gp}  {lab}", C["btn"])
text(320, 600, "internal pull-ups", 10, color="#777")

# ---------------------------------------------------------------- preset ladder
box(40, 650, 260, 200, "Preset buttons P1–P4", "R-2R ladder", "#f6f1ee", C["lad"])
text(170, 732, "R = 10k, 2R = 20k, 1k pull-downs", 11, "middle", color="#555")
text(170, 750, "buttons switch to 3.3 V", 11, "middle", color="#555")
text(170, 768, "100 nF from OUT to GND", 11, "middle", color="#555")
text(170, 792, "details: preset_ladder.png", 11, "middle", color=C["lad"], weight="bold")
tag(90, 825, "3V3", C["v33"]); tag(170, 825, "GND", C["gnd"])
pin(300, 710, C["lad"]); text(292, 714, "OUT", 12, "end")
wire([(300, 710), (EX0, 710)], C["lad"]); epin_left(710, "GPIO 33  ladder", C["lad"])
text(320, 732, "ADC1 only", 10, color="#777")

# ---------------------------------------------------------------- motor board
BX0, BX1, BY0, BY1 = 1000, 1440, 230, 560
box(BX0, BY0, BX1 - BX0, BY1 - BY0, "i2c BTS7960 motor board", "PCA9685 @ 0x40 · ISO1540 isolated I2C · 8× BTS7960B", "#fdf0f0", C["mot"])
pin(1120, BY0, C["v24"]); text(1126, BY0 - 8, "V+", 11, color="#333")
pin(1200, BY0, C["gnd"]); text(1206, BY0 - 8, "V−", 11, color="#333")
wire([(1200, BY0), (1200, 175)], C["gnd"]); tag(1200, 165, "PSU −", C["gnd"])
text(1020, 286, "qwiic", 11, color="#777")
qw = [(300, "GND", C["gnd"], "GND"), (330, "3V3", C["v33"], "3V3"), (360, "SDA", C["i2c"], "GPIO 21  SDA"), (390, "SCL", C["i2c"], "GPIO 22  SCL")]
for y, lab, col, elab in qw:
    pin(BX0, y, col); text(BX0 + 10, y + 4, f"{lab}", 12, color="#333")
    wire([(EX1, y), (BX0, y)], col); epin_right(y, elab, col)
text(1080, 420, "qwiic pin order: 1 GND · 2 3V3 · 3 SDA · 4 SCL", 11, color="#555")
text(1080, 438, "ESP32 3.3 V powers the ISO1540's ESP32 side", 11, color="#555")
# optional I2C pull-ups
dot(845, 360, C["i2c"]); res_v(845, 360, 330, C["i2c"]); dot(845, 330, C["v33"])
dot(885, 390, C["i2c"]); res_v(885, 390, 424, C["i2c"]); tag(885, 436, "3V3", C["v33"])
text(905, 414, "2.2–3.3k pull-ups", 10, color="#555"); text(905, 428, "(recommended)", 10, color="#777")
# ports
ports = [(470, "P1", "J5 1-2", 0), (495, "P2", "J5 3-4", 1), (520, "P3", "J3 1-2", 2), (545, "P4", "J3 3-4", 3)]
text(1080, 462, "motor ports (J2/J4 screw terminals in parallel):", 11, color="#555")
for y, p, j, leg in ports:
    wire([(BX1 - 120, y), (BX1, y)], C["mot"], 3); pin(BX1, y, C["mot"])
    text(BX1 - 126, y + 4, f"{p}  {j}  → leg {leg}", 12, "end", "bold", C["mot"])

# ---------------------------------------------------------------- legs
legs = [("Leg 0", "front-left", 34, 35, "P1 (J5 1-2)"), ("Leg 1", "front-right", 36, 39, "P2 (J5 3-4)"),
        ("Leg 2", "rear-left", 32, 13, "P3 (J3 1-2)"), ("Leg 3", "rear-right", 25, 26, "P4 (J3 3-4)")]
LX0, LX1 = 1020, 1440
for i, (name, where, ga, gb, port) in enumerate(legs):
    y0 = 610 + i * 165
    add(f'<rect x="{LX0}" y="{y0}" width="{LX1-LX0}" height="135" rx="8" fill="#f5f0fa" stroke="{C["enc"]}" stroke-width="1.5"/>')
    text(LX0 + 16, y0 + 24, f"{name}", 15, weight="bold"); text(LX0 + 78, y0 + 24, f"({where})", 12, color="#555")
    ya, yb = y0 + 55, y0 + 95
    pin(LX0, ya, C["enc"]); pin(LX0, yb, C["enc"])
    text(LX0 + 12, ya + 4, "encoder A", 12); text(LX0 + 12, yb + 4, "encoder B", 12)
    tag(LX0 + 175, ya, "VCC → 5V*", C["v5"]); tag(LX0 + 175, yb, "GND", C["gnd"])
    # motor
    wire([(LX1 - 60, ya), (LX1 - 20, ya)], C["mot"], 3); wire([(LX1 - 60, yb), (LX1 - 20, yb)], C["mot"], 3)
    add(f'<circle cx="{LX1-90}" cy="{(ya+yb)/2}" r="22" fill="#fff" stroke="{C["mot"]}" stroke-width="2"/>')
    text(LX1 - 90, (ya + yb) / 2 + 5, "M", 15, "middle", "bold", C["mot"])
    wire([(LX1 - 72, ya + 6), (LX1 - 60, ya)], C["mot"], 2); wire([(LX1 - 72, yb - 6), (LX1 - 60, yb)], C["mot"], 2)
    text(LX1 - 14, y0 + 124, f"24 V motor ← board {port}", 11, "end", "bold", C["mot"])
    # encoder lines from ESP32 with 3.3k pull-ups (A: up, B: down)
    epin_right(ya, f"GPIO {ga}  L{i}A", C["enc"]); epin_right(yb, f"GPIO {gb}  L{i}B", C["enc"])
    wire([(EX1, ya), (LX0, ya)], C["enc"]); wire([(EX1, yb), (LX0, yb)], C["enc"])
    dot(870, ya, C["enc"]); res_v(870, ya, ya - 34, C["enc"]); tag(870, ya - 44, "3V3", C["v33"])
    dot(930, yb, C["enc"]); res_v(930, yb, yb + 34, C["enc"]); tag(930, yb + 44, "3V3", C["v33"])
    text(884, ya - 18, "3.3k", 10, color="#555"); text(944, yb + 22, "3.3k", 10, color="#555")

# ---------------------------------------------------------------- notes / legend
NY = 880
add(f'<rect x="40" y="{NY}" width="400" height="412" rx="8" fill="#fffdf3" stroke="#bbb" stroke-width="1.2"/>')
text(56, NY + 26, "Notes", 15, weight="bold")
notes = [
    "Kill relay: set PIN_MOTOR_POWER 4 in Config.h.",
    "  It must be ON when GPIO 4 is HIGH. Many relay",
    "  boards are active-LOW: use an active-HIGH board",
    "  or a MOSFET module. No relay? Wire +24 V straight",
    "  to the board's V+ and leave PIN_MOTOR_POWER unset.",
    "The ESP32 runs from its own buck, so cutting motor",
    "  power never resets the controller.",
    "*Encoders: most desk-leg hall sensors want 5 V.",
    "  Pull-ups go to 3.3 V, NOT 5 V (open-collector",
    "  outputs). 5 V push-pull outputs need a level",
    "  shifter. GPIO 34-39 have no internal pull-ups.",
    "Leg directions don't matter: run 'calib'.",
    "Fuse the 24 V line, sized to your PSU.",
]
for k, n in enumerate(notes): text(56, NY + 52 + k * 18, n, 12, color="#333")
ly = NY + 52 + len(notes) * 18 + 14
text(56, ly, "Wire colours", 13, weight="bold")
leg = [("24 V", C["v24"]), ("5 V", C["v5"]), ("3.3 V", C["v33"]), ("GND", C["gnd"]), ("I2C", C["i2c"]),
       ("encoders", C["enc"]), ("motor", C["mot"]), ("buttons", C["btn"]), ("display", C["disp"]), ("ladder", C["lad"])]
for k, (lab, col) in enumerate(leg):
    x = 56 + (k % 4) * 95; y = ly + 22 + (k // 4) * 22
    wire([(x, y - 4), (x + 20, y - 4)], col, 3); text(x + 25, y, lab, 11)
text(56, ly + 96, "pill labels = connect to that supply/ground", 11, color="#666", italic=True)

add('</svg>')
Path(__file__).with_name("wiring_diagram.svg").write_text("\n".join(out))
print("wrote wiring_diagram.svg")
