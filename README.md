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