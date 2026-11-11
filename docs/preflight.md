# Staged testing and pre-flight checklist

Do the stages in order. Do not skip one because the previous one "basically
worked".

## Stage 0: software only (no hardware)

- [ ] `python3 -m pytest` in `ros2_ws/src/robodrone_control` passes.
- [ ] ArduPilot SITL + MAVROS bring-up works ([ros2.md](ros2.md)).
- [ ] `offboard_demo` dry run succeeds, then flies the square **in SITL**,
      lands and exits 0.
- [ ] You flipped the SITL mode away from GUIDED mid-square and the demo
      stopped and stayed stopped.

## Stage 1: bench, props OFF

Take every propeller off. Physically. Not "I'll be careful".

- [ ] `/mavros/state` shows `connected: true` from the real Pixhawk.
- [ ] `status_monitor` shows battery voltage within 0.1 V of a multimeter.
- [ ] Motor test: each motor spins in the right direction and position.
- [ ] RC failsafe: power off the transmitter; the FC reacts as configured.
- [ ] Mode switch: every position selects the intended mode.
- [ ] Motor emergency stop switch stops the motors.
- [ ] Pull the companion's power: the FC keeps working normally.

## Stage 2: tethered hover, props ON

- [ ] Tether or net; open area; nobody within 10 m.
- [ ] Charged battery, secured; props tight, correct direction, undamaged.
- [ ] GPS 3D fix, 8+ satellites, no pre-arm warnings (never disable arming
      checks).
- [ ] Hover in `STABILIZE`/`ALT_HOLD` by hand at ~1 m. Watch for oscillation or
      drift. Land and fix before moving on (tuning is its own topic).

## Stage 3: autonomous demo

- [ ] Hover in `LOITER` was steady.
- [ ] Transmitter in hand, thumb near the mode and land switches.
- [ ] `offboard_demo` dry run OK on the real vehicle.
- [ ] Start with a **small** square and low altitude. The demo caps altitude at
      10 m and the square at 5 m in code; you should start well below that.

```bash
ros2 run robodrone_control offboard_demo --ros-args \
  -p enable_flight:=true -p takeoff_altitude:=1.5 -p square_side:=1.0
```

- [ ] Abort test in the air: flip the mode switch away from `GUIDED`; confirm
      you are flying the vehicle manually and the node has stopped.

## Every flight

- [ ] Battery voltage and cell balance, battery secure
- [ ] Props: tight, undamaged, right direction
- [ ] Frame, arms, motor screws
- [ ] Compass and GPS healthy, no pre-arm messages
- [ ] RC link and failsafe tested today
- [ ] Clear area, spectators back, local rules and permissions satisfied
- [ ] Know your abort: mode switch, land, motor kill

Fly within your local regulations. Register and get permissions as required
where you fly; they vary a lot by country and by weight class.
