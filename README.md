# AGVFlex

Same mecanum line-following AGV as [AGVDispatch](https://github.com/nilkanthagholap/AGVDispatch), but the **track map lives in the web app**, not in Arduino flash.

Change stations, nodes, headings, bay vs trunk-end, and dwell times in the Map Editor. The page stores the layout in `localStorage` and, on connect (or when you hit **Sync to Robot**), pushes it over Bluetooth/USB. You do not re-upload a sketch to rearrange the track.

The Arduino sketch is three lines. Navigation, PID, and docking are the same state machine as AGVDispatch.

**Live controller:** [https://nilkanthagholap.github.io/AGVFlex/](https://nilkanthagholap.github.io/AGVFlex/)

Chrome or Edge, desktop or Android. Not iOS Safari (no Web Bluetooth / Web Serial there).

---

## When to use which

| | AGVDispatch | AGVFlex |
|---|---|---|
| Track defined in | Arduino `stations[]` | Browser Map Editor |
| Change the layout | Re-upload the sketch | Edit + Sync to Robot |
| Sketch | Track table + `begin()` / `update()` | `begin()` / `update()` only |
| RAM on the Uno | Names live in flash | Up to 20 stations, no names on the MCU |

Use AGVDispatch if the track is fixed and you want the robot to be the source of truth. Use AGVFlex if you iterate on layouts from a laptop or phone.

---

## Hardware

Identical to AGVDispatch:

- Arduino Uno (AVR)
- Mecanum chassis with PCA9685 PWM motor driver over I2C
- 8 digital line sensors on pins 5–12
- Hall-effect junction magnet sensor on pin 2 (interrupt)
- Bluetooth UART module on pins 3 (RX) and 4 (TX) at 9600 baud — HM-10, DX-BT24, or HC-05

```
DX-BT24 / HM-10 / HC-05     Uno
VCC                     ->  5V
GND                     ->  GND
TX                      ->  pin 3   (library RX)
RX                      ->  pin 4   (library TX)
STATE, EN                   leave unconnected
```

Pins 3/4 are SoftwareSerial, so USB uploads still work with the module plugged in. Change the pin constants in `src/AGVFlex.h` if your wiring differs.

Also install **Adafruit PWM Servo Driver Library** from the Arduino Library Manager.

---

## Install

1. Download this repository (Code → Download ZIP) and unzip it.
2. Rename the top-level folder to `AGVFlex` if it isn't already.
3. Move it into your Arduino sketchbook's `libraries` folder:
   - Windows: `Documents\Arduino\libraries\AGVFlex`
   - macOS: `~/Documents/Arduino/libraries/AGVFlex`
   - Linux: `~/Arduino/libraries/AGVFlex`
4. Restart the Arduino IDE.
5. Open **File → Examples → AGVFlex → FlexDispatch**.

The example sketch is:

```cpp
#include <AGVFlex.h>
AGVFlex robot;
void setup() { robot.begin(); }
void loop()  { robot.update(); }
```

Optional tuning (same knobs as AGVDispatch) can be called from `setup()`:

```cpp
robot.setSpeeds(0.32, 0.6);
robot.setLineFollowGains(0.18, 0.60);
robot.setTimings(500, 1000, 400, 800, 2000, 1200);
```

---

## Web app

Open the [live page](https://nilkanthagholap.github.io/AGVFlex/) (or `docs/index.html` locally — Web Bluetooth/Serial need a [secure context](https://developer.mozilla.org/en-US/docs/Web/Security/Secure_Contexts), so localhost or HTTPS).

### Dispatch tab

1. **Connect BLE** (DX-BT24 / HM-10, service `0xFFE0` / characteristic `0xFFE1`) or **Connect USB** (cable to the Uno, or a paired HC-05). USB serial resets the Uno; the page waits ~2 s, then syncs the map.
2. Set **Current position** to wherever the robot is actually parked.
3. Tap a station to go there, or build a multi-stop route and **Send Route**.

Dispatch buttons stay disabled until the robot is connected, calibrated, **and** the map has been synced.

### Map Editor tab

Define stations here. The layout is saved to this browser automatically.

| Field | Meaning |
|---|---|
| Name | Label in the Dispatch tab. Names stay in the browser; the Uno only stores numbers. |
| Node ID | Trunk junction, counting `0, 1, 2…` from the first active magnet. A home south of node 0 can be `-1`. |
| Direction | Parked heading: N / E / S / W. |
| Type | **Bay (Turn)** = 90° off the trunk. **Trunk (In-line)** = stop on the trunk itself. |
| Dwell (ms) | Pause during a multi-stop route. `0` uses the library default (2000 ms). |

**Sync to Robot** sends `C` (clear) then one `M,...` line per station. Do this after any edit, and it also runs automatically on connect.

---

## Protocol

Newline-terminated ASCII. The Arduino also echoes everything on USB Serial at 9600.

### App → robot

| Command | Meaning |
|---|---|
| `C` | Clear the in-memory station table. Replies `ACK,C`. |
| `M,<id>,<node>,<N\|E\|S\|W>,<B\|T>,<dwellMs>` | Upsert one station. Replies `ACK,M,<id>`. Max 20 stations (`id` 0–19). |
| `P<id>` | Calibrate: “you are parked at station `id`.” Required before any move. |
| `G<id>` | Go to one station. |
| `R<id>,<id>,…` | Multi-stop route (max 12). |

There is no `Q` (query map). The app owns the layout.

Moves are rejected with `ERR,NOT_CALIBRATED` until a `P` has been accepted, and with `ERR,BUSY` while a leg is in progress.

### Robot → app

| Line | Meaning |
|---|---|
| `ACK,C` / `ACK,M,<id>` | Map write accepted. |
| `POS,<id>` | Confirmed parked position. |
| `LEG,<from>,<to>` | Started a leg. |
| `ARR,<id>` | Arrived. |
| `DONE` | Route finished. |
| `ERR,NOT_CALIBRATED` / `ERR,BUSY` | Command rejected. |
| `DBG,ARR,MAGNET\|DEADRECK,<ms>` | Trunk-end arrival diagnostic (safe to ignore). |

---

## Track model (same as AGVDispatch)

A single north–south trunk. Each magnet junction can have a west and/or east bay. Trunk-end stations can sit past their magnet; arrival still waits for the line to run out (`LINE_GONE_MS`, default 300 in `AGVFlex.h`) so the robot has room to U-turn.

`junctionSettleMs` (default 1200, last argument of `setTimings`) ignores junction and line-loss right after merging onto the trunk, so the magnet you just used is not counted twice.

---

## License

MIT. See [LICENSE](LICENSE).
