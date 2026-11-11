# Hardware: 10-inch quad, Pixhawk 2.4.8, companion computer

This is a build guide, not a shopping list with SKUs. Part numbers go stale and
2.4.8 boards are mostly clones, so it says what to buy and what to check.

## Know your board first

"Pixhawk 2.4.8" is the community name for a clone of the original 3DR Pixhawk 1
(FMUv2 family, STM32F427). Things that matter for this project:

| Fact | Consequence |
|---|---|
| Many of these chips only expose **1 MB** of the 2 MB flash | Use ArduPilot's **`Pixhawk1-1M`** build, which has features trimmed to fit. **It refuses to run on a board with a true 2 MB chip** (`2M flash - use Pixhawk1 firmware`); in that case use the full `Pixhawk1` build. Try 1M first: it tells you if it's the wrong one. See [firmware.md](firmware.md). |
| It is an old board | **Current PX4 does not officially support FMUv2.** ArduPilot is the practical firmware. |
| No native DDS / micro-ROS on a 1 MB build | The ROS 2 link is **MAVROS over MAVLink** on a companion computer, not a direct DDS bridge. |
| Main outputs go through the IO co-processor | Use **standard PWM ESCs** (optionally OneShot125). The 1M build explicitly disables DShot on the IO outputs. |
| The 1M build only compiles in **u-blox** (and NMEA) GPS drivers | Buy a u-blox-based GPS (the common Neo-M8N class is fine). Other GPS types won't be recognised. |
| The 1M build drops `GUIDED_NOGPS`, `FOLLOW`, `FLOWHOLD`, `ZIGZAG`, OSD, MSP and gimbal mounts | **Outdoor flight with GPS** is the supported path for the ROS 2 demo. |
| Clone quality varies | Budget for a real power module, an **external GPS+compass**, and bench-testing the board before trusting it. Treat the prototype as disposable. |

If you outgrow it, a newer Pixhawk-class board (6C, Cube Orange, ...) gets you
native DDS on ArduPilot and PX4 support. The ROS 2 side here keeps working
through MAVROS on the new board unchanged.

## Airframe and propulsion (10" class)

Sizing rules of thumb. Check them against the thrust table for your actual
motor + prop + voltage before buying.

* **Frame:** about 450-500 mm motor-to-motor diagonal. Smaller than that and
  10" props overlap.
* **Props:** 10x4.5 or 10x5, two CW and two CCW.
* **Motors:** ~800-1000 KV on 4S, or ~600-750 KV on 6S, "2212 to 2814" class.
* **ESCs:** 30-40 A, standard PWM, with a BEC you can leave unconnected.
* **Battery:** 4S or 6S LiPo, 4000-6000 mAh, XT60.
* **Weight budget (very rough):** frame 400-600 g, motors+props 600-800 g,
  ESCs ~100 g, avionics ~200 g, battery 450-700 g, companion ~80 g.
  Expect **1.8-2.5 kg** take-off weight.
* **Thrust target:** at least **2:1 thrust-to-weight**, so roughly 1 kg of
  thrust per motor at the pack voltage you will fly. Less and the drone is
  sluggish and has no margin for a failsafe landing.

## Parts list

| Item | Notes |
|---|---|
| Flight controller | Pixhawk 2.4.8 + SD card (for logs) + safety switch + buzzer |
| Power module | Rated for **your pack voltage** (many cheap modules stop at 4S) and ≥ your max current. Outputs ~5.3 V for the FC. |
| GPS + compass | Neo-M8N class on a mast, away from power wires. The 2.4.8's own compass is useless inside the frame. |
| RC receiver + transmitter | PPM-sum or S.BUS to the RC IN port. You will want at least 6 channels (modes + a kill/land switch). |
| Telemetry radio (optional) | On TELEM1, for a ground station in the field. SiK 915/433 MHz per your region. |
| Companion computer | Raspberry Pi 4 or 5 (or similar), running Ubuntu 24.04. |
| Companion power | **Separate 5 V ≥ 3 A UBEC** from the battery. Never power it from the Pixhawk. |
| USB-serial adapter | 3.3 V FTDI-style, for the first bring-up and as a fallback if the Pi UART gives trouble. |
| Vibration damping | Foam or soft-mount for the FC. Vibration is the #1 cause of bad EKF behaviour. |
| Sundries | Battery straps, prop guards for early tests, a LiPo-safe bag, a tether or a net. |

## Wiring

```
                      ┌──────────────── 4S/6S LiPo ───────────────┐
                      │                                           │
               Power module ──(5.3V + V/I sense)──► Pixhawk POWER │
                      │                                           │
              power distribution ──► 4 × ESC ◄─ signal ─ MAIN OUT 1-4
                      │
                  5V UBEC (3A) ──► Raspberry Pi
                                      │ UART (3.3V logic)
                                      ▼
              Pixhawk TELEM2 ◄── TX/RX crossed, GND shared, VCC NOT connected
```

### Motor outputs: ArduCopter Quad **X** (look from above, nose up)

| Output | Position | Spin |
|---|---|---|
| MAIN OUT 1 | front right | CCW |
| MAIN OUT 2 | rear left | CCW |
| MAIN OUT 3 | front left | CW |
| MAIN OUT 4 | rear right | CW |

Verify with the Motor Test in your ground station, **props off**, before the
first arm. Wrong order or wrong spin direction is the most common first-flight
flip.

Connect the red (5 V) lead of **at most one** ESC, or none, as your ESC and
power-module documentation says. Never tie several ESC BECs together.

### Companion link: TELEM2

The TELEM2 port is a 6-pin DF13 connector:

| Pin | Signal | Connect to the companion |
|---|---|---|
| 1 | VCC (5 V) | **Nothing.** Leave it unconnected. |
| 2 | TX (out of the Pixhawk) | companion **RX** |
| 3 | RX (into the Pixhawk) | companion **TX** |
| 4 | CTS | optional, hardware flow control |
| 5 | RTS | optional, hardware flow control |
| 6 | GND | companion **GND** (mandatory) |

Both sides are 3.3 V logic, so no level shifter should be needed. Confirm that
for your clone with a multimeter.

On a Raspberry Pi, the hardware UART on the 40-pin header needs enabling and,
on some models, has to be freed from Bluetooth or the serial console; the
overlay names differ between Pi 4 and Pi 5, so follow the current Raspberry Pi
documentation. **For the first bring-up use the USB-serial adapter**
(`/dev/ttyUSB0`) and move to the GPIO UART once everything else works.

### Other ports

| Port | What |
|---|---|
| RC IN | Receiver (PPM-sum or S.BUS) |
| GPS | GPS 6-pin; compass on the I2C 4-pin from the same module |
| POWER | Power module |
| TELEM1 | SiK telemetry radio (optional) |
| USB | Configuration and flashing; also a handy way to bench-test MAVROS |

## Build order that avoids tears

1. Bench: FC + USB + power module on a bench supply. Flash firmware, calibrate.
2. Bench: add companion, bring up MAVROS, see `/mavros/state` connected.
3. Motors and ESCs on the frame, **props off**, motor test.
4. Props on only for a tethered or netted hover.
5. See [preflight.md](preflight.md) before every flight.
