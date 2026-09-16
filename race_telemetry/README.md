# Race Telemetry

A development-only telemetry recorder for an existing
`aichallenge-racingkart` installation.

It can be cloned anywhere. It does **not** modify the official challenge
repository.

## Assumption

The official challenge repository already exists, normally at:

```text
~/aichallenge-racingkart
```

## Normal use

Start this tool first:

```bash
cd ~/aichallenge-tools
bash race_telemetry/run.sh
```

It waits for the `autoware` Docker container.

Then, in another terminal, start the challenge exactly as usual:

```bash
cd ~/aichallenge-racingkart
make dev
```

The recorder detects Autoware automatically and starts collecting ROS data.

When the run is finished, press `Ctrl+C` in the telemetry terminal. The CSV
and HTML report are copied back into:

```text
aichallenge-tools/race_telemetry/runs/run_YYYYMMDD_HHMMSS/
```

## If the challenge repository is elsewhere

Pass its path:

```bash
bash race_telemetry/run.sh /path/to/aichallenge-racingkart
```

or:

```bash
AICHALLENGE_REPO=/path/to/aichallenge-racingkart \
  bash race_telemetry/run.sh
```

## Baseline speed

Default:

```text
32 km/h
```

Override it like this:

```bash
TELEMETRY_BASE_SPEED=35 bash race_telemetry/run.sh
```

## Recorded topics

- `/localization/kinematic_state`
- `/vehicle/status/velocity_status`
- `/vehicle/status/steering_status`
- `/control/command/actuation_cmd`

## Report

The generated HTML includes:

- speed map relative to the baseline speed
- acceleration/deceleration map
- steering intensity map
- brake-command map
- distance-based speed/acceleration/steering/brake plots
- run summary and estimated time loss versus the baseline
