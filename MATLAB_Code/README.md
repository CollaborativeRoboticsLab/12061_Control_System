# MATLAB toolbox — 12061 inverted pendulum

The host side of the rig: the toolbox that drives the pendulum, and the
weekly lab scripts. The firmware it talks to lives in `../source_code/`.

A staff working copy may also carry `bluepill/` — an older toolkit for a
**different board**, kept out of the repository on purpose. **The two must not
be mixed up**: using the wrong connect function gives a timeout, not an error
message, which is exactly how a whole afternoon gets lost.

```matlab
setupPath        % once per session, or put it in startup.m
```

---

## Which set do I want?

| | **WHEELTEC inverted pendulum** | **bluepill test board** |
|---|---|---|
| Folder | `pendulum/` | `bluepill/` |
| Connect | `connectPendulum("COM4")` | `connectSTM32("COM4",9600)` |
| Baud | 128000 | 9600 |
| Transport | `pendulumCommand` (raw bytes) | `sendSTM32Command` (readline) |
| PWM range | ±6900 | ±999 |
| Motor cmd | `setMotorPWM_wheeltec(s,pwm,ms)` | `setMotorPWM(s,pwm)` |
| Angle | `readAngleWheeltec(s)` | `readAngle(s)` |
| Status | `readStatusWheeltec(s)` | `readStatus(s)` |

The `*Wheeltec` suffixes are not decoration — they exist so both toolkits
can sit on the path at once without shadowing each other.

---

## Normal session

```matlab
setupPath
s = connectPendulum("COM4");     % check it prints  Firmware : R4-5
homeCart(s);                     % slide the cart to mid-rail, press Enter
readStatusWheeltec(s)            % RxErrors should be 0
```

`homeCart` is not optional. The encoder is **relative**: reset writes the
count to 10000 wherever the cart is, and connecting resets the board. Until
you home it, the firmware has no idea where the cart is, so it skips the
rail-end protection entirely and caps every motor command to PWM 2500 for
300 ms. `homeCart` sends `HOME`, which declares "the cart is mid-rail" and
writes the count to 7925 — that is what makes 5900/9900 real rail ends and
what makes the balance controller's setpoint correct. Details in the project
doc `12061-encoder-frame-and-home.md`.

Then whatever you came to do:

```matlab
setMotorPWM_wheeltec(s,2500,300)   % open-loop step, 300 ms
setBalance(s,true)                 % hand control to the on-board PD loop
swingUp(s)                         % automatic swing-up
setGain(s,"BKP",350)               % retune without reflashing
setGain(s,"BKI",150)               % integral term, added in firmware R4-5
resetIntegrator(s)                 % clear the accumulator between trials
```

---

## Folders

| Folder | What is in it |
|---|---|
| `pendulum/` | The working toolbox for the pendulum rig, including `homeCart` |
| `experiments/` | `lab_step_response` (student experiment), `testPendulumCommands`, `testPendulumMotor`, `measureDeadband` |
| `Labs/` | The weekly lab scripts — `week3` … `week13`, no `week9`. Students start here |
| `bluepill/` | **Not in the repository.** Older toolkit for a different board; staff only, see the warning above |
| `diagnostics/`, `simulink/` | **Not in the repository.** Staff debugging aids and model builders |

`slprj/` is MATLAB's Simulink build cache — generated, and gitignored. If you
cloned this repository you will not have the three folders marked above, and
nothing in the weekly labs needs them.

---

## Bring-up order for a rig that has never been talked to

1. Flash the firmware — open **`../source_code/`** in VS Code with the
   PlatformIO extension, not the repository root. The connect step verifies
   the board reports **R4-5**, and warns loudly if it does not
2. `testPendulumCommands(s)` — everything except the motor
3. Charge the battery. Below about 7 V the firmware refuses to drive the
   motor and the driver has no supply anyway
4. `homeCart(s)` — cart to mid-rail by hand, then Enter. The experiment
   scripts call this themselves, so this step is only for a bare session
5. Rod off, cart centred, hands clear → `testPendulumMotor(s)`
6. Calibrate the angle sensor if the hanging rod does not read 1010–1030
   (see `Pendulum_Angle_Calibration_QuickGuide.pdf`, kept with the lab
   materials, not in this repository)

## Known characteristics of this rig

- Opening the serial port **resets the board**; it is then silent for 2 s.
  `connectPendulum` waits it out — `connectSTM32` does not, which is why it
  always times out on this board.
- The board streams binary DataScope frames until the first ASCII command
  mutes them. `setStream(s,true)` turns them back on.
- The cart encoder has **no absolute reference and no index pulse**. Every
  absolute number in the firmware (7925, 5900, 9900, the swing-up waypoints)
  is measured from wherever the cart happened to be at the last reset, which
  is why `HOME` exists.
- Actuator deadband: PWM below roughly 900 moves the cart nowhere at all.
- DataScope frame rate is about 14 Hz, not the 20 Hz the 50 ms loop implies.
- Polling `readStatusWheeltec` in a loop samples at about **40 Hz**, measured
  on the rig in week 6. That is enough for recovery times, but it is 1/5 of
  the firmware's own 200 Hz control rate — which is why the control loop runs
  on the board and not here.
