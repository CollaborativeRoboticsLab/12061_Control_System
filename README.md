# Control Systems 12061 / 12601 — Inverted Pendulum Lab

Teaching repository for the laboratory programme of **Control Systems 12061 /
12601**, Bachelor of Engineering (Hons) — Intelligent Robotics, University of
Canberra.

It contains two halves that talk to each other over one serial link:

| Half | What it is | Where it runs |
|---|---|---|
| **Firmware** | STM32 C code: the 200 Hz balance loop plus an ASCII command layer added for teaching | on the rig |
| **MATLAB_Code** | Weekly lab scripts and a helper toolbox that drive and record the rig | on the student's laptop |

**The control loop is not in MATLAB** 

---

## Hardware

| | |
|---|---|
| Rig | WHEELTEC linear inverted pendulum (cart on a belt-driven rail) |
| MCU | STM32F103C8T6 @ 72 MHz, 64 KB Flash / 20 KB RAM |
| Angle | potentiometer on the rod pivot → ADC on PA3, ~2080 counts over 180° |
| Cart | quadrature encoder on TIM4, **relative** — see the note below |
| Motor | TB6612 driver, TIM3 PWM at 10 kHz, clamped to ±6900 |
| Link | USB-serial at **128000 baud** (non-standard, set by `uart_init(72,128000)`) |
| Flashing | ST-Link V2 over SWD (only needed when the firmware changes) |

**The cart encoder is relative.** Reset writes the count to 10000 wherever the
cart happens to be, and opening the serial port resets the board. Until the host
sends `HOME`, the firmware's absolute limits (centre 7925, rail ends 5900 /
9900) mean nothing, so it skips them and caps every motor command to a nudge.
`homeCart(s)` does the handshake: you slide the cart to mid-rail by hand, it
sends `HOME`, and the firmware declares the count to be 7925.

---

## Layout

```
Control_System_lab/
├── platformio.ini          PlatformIO project: env "pendulum", ST-Link upload
├── BALANCE/
│   ├── CONTROL/            control.c — the 5 ms TIM1 ISR, angle PID, cart PD
│   ├── CHECK/              swing-up and protection logic
│   └── show/               OLED display
├── HARDWARE/               ADC, ENCODER, MOTOR, KEY, LED, OLED, TIMER, EXTI,
│                           DataScope_DP (binary telemetry frames)
├── SYSTEM/
│   ├── usart/              usart.c, and usart_cmd.c — the ASCII command layer
│   ├── sys/                CMSIS headers, clock and NVIC helpers
│   └── delay/
├── USER/Minibalance.c      main(): init, then the 50 ms housekeeping loop
├── boot/                   GNU startup file and linker script
├── tools/build_gcc.sh      bare arm-none-eabi build, for CI / quick checks
└── MATLAB_Code/
    ├── setupPath.m         run once per session; also warns about shadowing
    ├── pendulum/           the working toolbox for this rig
    ├── Labs/               one folder per teaching week  ← students start here
    ├── experiments/        bench tests: motor, deadband, step response
    ├── diagnostics/        only when something is wrong
    ├── bluepill/           older toolkit for a different board; do not mix
    └── simulink/           model builders (not used in the weekly labs)
```

### `MATLAB_Code/pendulum/` — the toolbox

| Function | Purpose |
|---|---|
| `connectPendulum(port)` | open, wait out the reset, mute telemetry, handshake, check firmware version |
| `homeCart(s)` | declare the cart's position and arm the rail limits |
| `pendulumCommand(s,cmd)` | raw-byte transport (`readline` does not work on this link) |
| `readStatusWheeltec(s)` | one round trip returns angle, cart, PWM, battery, flags |
| `readAngleWheeltec` / `readPositionWheeltec` / `readVoltage` | single readings |
| `readGains(s)` / `setGain(s,name,v)` | read and change Kp, Kd, Ki, PKp, PKd live |
| `resetIntegrator(s)` | clear the integral accumulator between trials |
| `setBalance(s,tf)` / `swingUp(s)` / `setZero(s)` / `setStream(s,tf)` | mode control |
| `setMotorPWM_wheeltec(s,pwm,ms)` | open-loop pulse with a firmware watchdog |

---

## Running a lab

```matlab
cd <repo>/MATLAB_Code
setupPath                       % once per MATLAB session
cd Labs/week5                   % the week you are in
open Control_System_Lab3.m
```

Then **run one section at a time with Ctrl+Enter**. Never press F5 / Run All —
several sections need you to be holding the rod when they start.

Each lab script is self-contained and follows the same conventions:

- **Section 0** is the only part you edit: `cfg.PORT`, `cfg.GROUP_ID`, and in
  Lab 4 also `cfg.MODE` (`"sim"` or `"rig"`).
- Boxed comments mark what you do — `EDIT THIS`, `PREDICT FIRST`,
  `STUDENT ACTION`, `SAFETY`, `FOR YOUR REPORT`. The SAFETY box is drawn in
  `!!!!` so it cannot be scrolled past.
- Captures are saved to `Labs/weekN/data/` as `.mat` and `.csv`, named with
  `cfg.GROUP_ID`. The path is resolved from the script's own location, so it
  does not matter where MATLAB is pointing.

A typical session begins:

```matlab
s = connectPendulum("COM4");    % must report Firmware : R4-5
homeCart(s);                    % slide the cart to mid-rail, press Enter
```

On Linux the port is `/dev/ttyUSB0` or `/dev/ttyACM0`, not `COM4`.

### Week ↔ lab map

| Week | Script | Topic |
|---|---|---|
| `week3` | `Control_System_Lab1.m` | link and command check |
| `week4` | `Control_System_Lab2.m` | signals, scaling, time response |
| `week5` | `Control_System_Lab3.m` | open-loop instability; measure the unstable pole |
| `week6` | `Control_System_Lab4.m` | PID control, sim + rig |
| `week7`, `week8`, `week10`–`week13` | — | placeholders |

**There is no `week9`** — that week is class-free. Do not create it.

---

## Firmware

Only needed when the C code changes; the board arrives programmed. Open
**this folder** in VS Code with the PlatformIO extension, then
<kbd>Ctrl</kbd>+<kbd>Alt</kbd>+<kbd>U</kbd>. Wait for `Verified OK`.
See `PlatformIO_Quick_Guide.pdf` (one folder up) for the full procedure and the
two failure modes worth telling apart:

- `Error: open failed` — the ST-Link itself was not opened. Driver / USB / not plugged in.
- `Error: init mode failed` — the adapter is fine, the chip is not reachable.
  **Swap SWDIO and SWCLK first**; that has been the cause every time so far.

### Serial command set (firmware R4-5)

Host sends `<CMD>\r`, board replies `<REPLY>\r\n`.

| Command | Reply | Notes |
|---|---|---|
| `PING` / `VER` | `PONG` / `VER=R4-5` | version is checked on connect |
| `A` / `P` / `V` | `ANGLE=` / `POS=` / `VOLT=` | volts arrive in hundredths |
| `STATUS` | nine integers | one round trip for the whole state |
| `M,<pwm>[,<ms>]` | `PWM=<v>` | open loop, with a watchdog; capped before `HOME` |
| `HOME` / `HOMED` | `HOME=7925` / `HOMED=0\|1` | declare the cart frame, arm the limits |
| `BAL,0\|1` / `SWING` / `STOP` | | mode control |
| `GAIN,<name>,<v>` | `GAIN=<name>,<v>` | `BKP BKD BKI PKP PKD` |
| `K` | five gains | `bkp,bkd,bki,pkp,pkd` |
| `IRESET` | `IRESET=0` | clear the integral accumulator |
| `STREAM,0\|1` | | binary DataScope frames |

The board streams binary telemetry out of reset; the first valid ASCII command
mutes it. `usart_cmd.c` carries the full protocol notes and revision history.

---

## Before you run

- **Charge the battery.** Below ~7 V the firmware refuses to drive the motor,
  and the driver has no supply on USB power alone.
- **Close the PlatformIO serial monitor before starting MATLAB.** Windows gives
  a COM port to one program at a time; this is the usual cause of "port exists
  but is busy".
- **Known issue:** on the current rig the hanging rod reads 988–1006 ADC counts
  against the 1020 ± 10 the firmware expects. Balancing is unlikely to work
  until the angle sensor is mechanically calibrated — see
  `摆杆角度传感器校准_作业指导书.pdf` / `Pendulum_Angle_Calibration_QuickGuide.pdf`
  one folder up.

---

## Credits

The firmware under `BALANCE/`, `HARDWARE/`, `SYSTEM/` and `USER/` derives from
the WHEELTEC / 平衡小车之家 reference code shipped with the rig, **used with the
vendor's permission**. Their attribution headers are left in place. The ASCII
command layer (`SYSTEM/usart/usart_cmd.c`), the `HOME` encoder-framing
mechanism, the angle-loop integral term, and everything under `MATLAB_Code/`
were written for this unit.
