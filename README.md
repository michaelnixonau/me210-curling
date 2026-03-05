# ME 210 Final Project

A curling robot.

Requires [PlatformIO](https://platformio.org/).

## Project structure

- `apps/`: All programs
- `lib/`: Reusable libraries (subsystem logic)

## Uploading

- Build main firmware: `pio run -e main`
- Upload main firmware `pio  run -e main -t upload`

Use the `-e <environment>` flag to select the desired environment.

We'll use this to independently test/operate subsystems and integrations.

## Robot localisation visualizer

There is a live GUI visualizer for the `robot_localisation` app:

- Script: `scripts/robot_localisation_visualizer.py`
- Default source: `socket://192.168.4.1:328`

Run it with:

```bash
python3 scripts/robot_localisation_visualizer.py
```

Optional flags:

```bash
python3 scripts/robot_localisation_visualizer.py --url socket://192.168.4.1:328 --trail-size 300
```

## Robot localisation diagnostics

Capture a telemetry log from the robot:

```bash
python3 scripts/robot_localisation_diagnostics.py capture --duration 90
```

Analyze a captured log:

```bash
python3 scripts/robot_localisation_diagnostics.py analyze logs/robot_localisation_diag_YYYYMMDD_HHMMSS.csv
```

The diagnostics script reports:
- stationary drift rate and jitter
- heading disagreement (`heading_arena_deg` vs magnetometer heading)
- gyro sign consistency check
- half-selection consistency (north vs south solve comparison)
