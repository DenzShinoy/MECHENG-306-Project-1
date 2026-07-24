# ME306 Project 1 — X-Y Plotter Pin Map

Hardware: Arduino Mega 2560 + DFRobot L298P Motor Shield (PWM jumper mode)
Motors: Pololu #2288, 172:1 (171.79:1) gearmotor, 48 CPR encoder
Supply: SMPS set to 6 V, 1.25 A

---

## 1. Motor shield — fixed by stacking header

The shield sits directly on the Mega. These pins are connected internally;
no wires to run.

| Mega pin | Function      | Notes   |
|----------|---------------|---------|
| D4       | M1 direction  |         |
| D5       | M1 PWM (E1)   | Timer3  |
| D6       | M2 PWM (E2)   | Timer4  |
| D7       | M2 direction  |         |

Set the four control-mode jumpers to **PWM**. PLL mode swaps D4/D5 and D6/D7.

---

## 2. Encoders — junction PCB to Mega

| PCB pin  | Mega pin | Type                       |
|----------|----------|----------------------------|
| `LEncA`  | D2       | INT4 — attachInterrupt     |
| `LEncB`  | D30      | plain digital, read in ISR |
| `REncA`  | D3       | INT5 — attachInterrupt     |
| `REncB`  | D32      | plain digital, read in ISR |
| `VCC`    | 5V       |                            |
| `GND`    | GND      |                            |

D2/D3 chosen because they are the only Mega interrupt pins with no
secondary function (D18/D19 = Serial1, D20/D21 = I2C).

---

## 3. Limit switches — junction PCB to Mega

| PCB pin         | Mega pin | Config                    |
|-----------------|----------|---------------------------|
| `TOP`           | D22      | INPUT_PULLUP, active LOW  |
| `BOTTOM`        | D24      | INPUT_PULLUP, active LOW  |
| `LEFT`          | D26      | INPUT_PULLUP, active LOW  |
| `RIGHT`         | D28      | INPUT_PULLUP, active LOW  |
| switch common   | GND      |                           |

No external resistors. Software debouncing required (project brief 2.1.4).

---

## 4. Power and motor leads — screw terminals, not Mega pins

| From                  | To                              |
|-----------------------|---------------------------------|
| `LM+` / `LM-` (PCB)   | Shield M1+ / M1-                |
| `RM+` / `RM-` (PCB)   | Shield M2+ / M2-                |
| SMPS 6 V barrel jack  | Shield PWRIN (+/-)              |
| Shield power jumpers  | Both on **PWRIN**, not VIN      |

Ground is common through the seated shield. Do not add a second GND wire.

Supply is 1.25 A; stall current is 2.2 A per motor. Cap PWM during
bring-up (~150/255) and never leave both motors stalled.

---

## 5. Reserved — do not use

| Pin(s)    | Reason                     |
|-----------|----------------------------|
| D0, D1    | Serial (USB, G-code in)    |
| D18, D19  | Serial1 — kept free        |
| D20, D21  | I2C                        |
| D50-D53   | SPI                        |

## 6. Free

D8-D17, D23, D25, D27, D29, D31, D33-D49, A0-A15

Note: the D22-D53 block extends past the shield and stays exposed.
Anchor wires there; check them first if a switch reads erratically.

---

## 7. Header block

```cpp
// --- Motor shield (PWM jumper mode) ---
#define M1_DIR   4
#define M1_PWM   5
#define M2_PWM   6
#define M2_DIR   7

// --- Encoders ---
#define ENC_L_A  2    // INT4
#define ENC_L_B  30
#define ENC_R_A  3    // INT5
#define ENC_R_B  32

// --- Limit switches (active LOW) ---
#define SW_TOP   22
#define SW_BOT   24
#define SW_LEFT  26
#define SW_RIGHT 28
```

---

## 8. Encoder resolution

- 48 counts per rev of motor shaft (4x decode, both edges both channels)
- Gear ratio 171.79:1
- 4x decode: 48 x 171.79 = **8246 counts/rev** at output shaft
- 2x decode (A channel only, CHANGE): **4123 counts/rev**
- 1x decode (A channel only, RISING): **2062 counts/rev**

Current wiring supports 2x. More than adequate for the mechanics.

---

## 9. Kinematics (CoreXY)

```
dA = dX + dY          dX = (dA + dB) / 2
dB = dX - dY          dY = (dA - dB) / 2
```

Direction table (`+` = the direction produced by `digitalWrite(Mx, HIGH)`):

| Pen motion | M1 | M2 |
|------------|----|----|
| Right      | +  | +  |
| Left       | -  | -  |
| Up         | -  | +  |
| Down       | +  | -  |
| Diag R-U   | 0  | +  |
| Diag R-D   | +  | 0  |
| Diag L-U   | -  | 0  |
| Diag L-D   | 0  | -  |

If an axis runs mirrored, swap that motor's two wires at the shield
terminal rather than patching signs in software.

---

## 10. Bring-up order

1. Power off. Set four mode jumpers to PWM, two power jumpers to PWRIN.
2. Meter the SMPS output — confirm 6 V before it touches the shield.
3. Wire encoders and switches only. Power over USB alone.
   Turn each axis by hand: counts change. Press each switch: pin reads 0.
4. Power down. Wire motor leads. Apply SMPS.
5. Run bring-up sketch, check directions against section 9, swap leads
   as needed.
