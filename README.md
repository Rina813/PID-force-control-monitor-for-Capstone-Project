# PID Force Control Monitor

A Python desktop monitor for Arduino-based force control experiments.

It provides a live plot of force vs target, serial command controls, automatic CSV logging for each run, persistent PID saving, and offline plotting for previously recorded trial files.

## Features

- Live serial connection to Arduino
- GO / STOP / Tare / Params / Help controls
- Default PID values:
  - Kp = 10
  - Ki = 0.5
  - Kd = 0
- Saved PID persistence in `trials/stable_pid.json`
- Automatic CSV logging on trial start
- Save plot button for the current graph
- Offline CSV plotting with `--graph`

## Project structure

```text
pid_force_control_monitor/
├── pid_monitor.py
├── requirements.txt
├── README.md
├── tests/
│   ├── test_csv_analysis.py
│   └── data/
└── trials/
```

## Installation

Create and activate a virtual environment, then install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run live monitor

```bash
python pid_monitor.py
```

Or specify the port manually:

```bash
python pid_monitor.py --port /dev/cu.usbmodem14101
```

## Run offline graphing

```bash
python pid_monitor.py --graph trials/trial_xyz.csv
```

## Default PID behavior

If `trials/stable_pid.json` does not exist, the program uses:

- `Kp=10`
- `Ki=0.5`
- `Kd=0`

These values are shown in the PID status bar in the display.

If you type new PID values in the command box and click **Save PID**, they are saved for future sessions.

## Expected serial data format

The monitor expects Arduino lines like:

```text
DATA,time_ms,raw_adc,voltage_v,force_lbs,target_lbs,kp,ki,kd
STATUS,message
HEADER,...
SET,...
ERROR,...
```

Example:

```text
DATA,23145,96.0,0.4710,49.08,50.00,10.0000,0.5000,0.0000
```

## Tests

A placeholder test file is included in `tests/test_csv_analysis.py`.

When you have your CSV files ready, place them in:

```text
tests/data/
```

Then the tests can be expanded to check:
- CSV columns exist
- time is monotonic
- target is read correctly
- settling time behavior
- plotting input loads correctly

## Notes

This repo is set up as a simple single-file desktop app for easy GitHub upload. Later, it can be refactored into modules if you want a cleaner package structure.

