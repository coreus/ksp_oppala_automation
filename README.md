# KSP Oppala — Autonomous Rocket Automation Framework

An autonomous SpaceX Starship like flight controller for **Kerbal Space Program**, powered by [kRPC](https://krpc.github.io/krpc/python.html). The project combines classical control theory, orbital mechanics, and machine learning to fully automate rocket missions — from launch to powered landing.

---

## Demo Videos

### Test Flight — Takeoff & Landing (no orbit)

[![KSP Takeoff & Landing Test](https://img.youtube.com/vi/KK4u8fWZ6EQ/maxresdefault.jpg)](https://www.youtube.com/watch?v=4nODeOr5-kY)

https://www.youtube.com/watch?v=4nODeOr5-kY

---

### Full Mission — Launch, Orbit, Rendezvous, Refueling, Deorbit & Landing


[![KSP Full Mission](https://img.youtube.com/vi/QMxKuYUfXSc/maxresdefault.jpg)](https://www.youtube.com/watch?v=QMxKuYUfXSc)

https://www.youtube.com/watch?v=QMxKuYUfXSc

---

## Features

### Mission Profiles

Run missions from the command line:

```bash
python takeoff.py <plan> <vesselName> [target]
```

| Plan | Description |
|---|---|
| `test <altitude>` | Vertical test launch to a given altitude, measures fuel consumption |
| `orbit <targetAlt>` | Full gravity-turn ascent and circularization burn at apoapsis |
| `rendezvous <ship> <targeted_ship>` | Complete autonomous refueling loop (see below) |
| `landing` | Reentry and landing from current position |
| `reentry` | Atmospheric reentry only |
| `testorbit` | Orbit + landing test cycle |

### Autonomous Rendezvous & Refueling Loop

The `rendezvous` mission plan executes the following sequence autonomously, repeating until the tanker is 90% full:

1. **Launch** to 80 km orbit with gravity turn
2. **Calculate launch window** — waits for the target tanker to reach the correct orbital position, warps time accordingly
3. **Rendezvous** with the tanker vessel
4. **Dock** and **transfer propellant** (Liquid Fuel + Oxidizer)
5. **Undock** and perform a **deorbit burn**
6. **Reentry** with adaptive aerodynamic flap control
7. **Bellyflop maneuver** and **powered vertical landing** back around KSC
8. **Repeat** until the tanker is refueled

### Ascent & Orbital Mechanics

- Gravity turn launch from 1,100 m to 48,000 m altitude
- Automatic stage separation on fuel depletion
- Circularization burn using maneuver nodes at apoapsis


### Landing System

**Landing guidance** (`landing_guidance.py`) — physics-based, computes ignition altitude from thrust, mass, and vertical velocity with a configurable safety margin. Bellyflop duration calculation based on basic linear regression

### Reentry — Adaptive Flap Control

During atmospheric reentry, flap angles are tuned in a closed-loop fashion:

- `FlapTuningV2` predicts pitch at zero velocity using a polynomial derivative model (jerk series)
- Independently adjusts front and back flap angles to follow the target pitch profile

### Control Systems

- **AbstractTuning** (`abstract_tuning.py`) — base class for adaptive control, uses polynomial prediction to anticipate overshoot and auto-adjust output
- **FlapTuning** (`flaptuning.py`) — closed-loop wing angle optimization during reentry

### Telemetry & Data Collection

- Real-time kRPC stream creation for altitude, fuel, apoapsis, velocity, pitch, g-forces, atmospheric data, and more
- CSV logging for flight telemetry used to build training datasets


---

## Project Structure

```
├── takeoff.py                   # Main mission controller
├── reload.py                    # Quick KSP quickload utility
├── config.ini                   # Connection and mission parameters
├── reentry_plan.csv             # Reentry flap/angle schedule
├── libs/
│   ├── flightplan.py            # Core flight controller (launch, reentry, landing)
│   ├── landing_guidance.py      # Landing guidance
│   ├── rendezvous.py            # Rendezvous, docking, and fuel transfer
│   ├── flaptuning.py            # Adaptive flap angle tuning
│   ├── abstract_tuning.py       # Base class for adaptive control
│   ├── telemetry.py             # kRPC stream helpers
│   ├── dataset.py               # CSV flight data logger
│   ├── connect.py               # kRPC connection
│   ├── helper.py                # Vector math utilities
│   ├── derivative.py            # Numerical derivative helper
│   └── logger.py                # Rich terminal logger
└── data/                        # Training CSV datasets
```

---

## Requirements

```bash
pip install -r requirements.txt
```

Requires a running instance of **Kerbal Space Program** with the [kRPC mod](https://krpc.github.io/krpc/) installed and the server started.

---

## TODO

- [ ] **Pinpoint landing** — guide the rocket to land on a specific target coordinates, combining GPS-based lateral corrections with the existing descent guidance
