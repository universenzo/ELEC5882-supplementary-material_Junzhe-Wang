# Final CAN/INA226 three-mode synthesis

## Scope and method

- Included modes: current PID without NTC feed-forward, current PID
  with NTC feed-forward, and fixed-DAC open loop.
- The temperature-only limiter was intentionally excluded because it
  did not have a defensible matched-comparison rationale.
- Dataset: 2 voltage groups x 3 retained modes x 3 repeats = 18 runs.
- Repeats are aligned to the first active state=5 sample. Plotted solid
  curves are local-quadratic descriptive trends; shaded bands are +/-1
  sample SD across repeats. All quantitative metrics use raw samples.
- Open-loop current-versus-NTC fits are calculated separately for each
  repeat and then summarized. Repeated time samples are not treated as
  statistically independent replicates.

## Data-quality checks

- 12 closed-loop runs completed approximately 600 s without
  a fault; all 6 open-loop runs reached the overcurrent latch.
- The minimum per-run valid INA226 telemetry coverage was 96.99%.
- `current_ma` and `ina_current_ma` are two CAN messages derived from
  the same INA226 path, so their agreement checks host-frame pairing
  only; it is not an independent current-sensor accuracy validation.

## Main quantitative evidence

- At 10 V / 50 mA, feed-forward reduced mean peak current from
  63.33 to 56.27 mA (53.0% reduction in overshoot) and reduced
  post-60 s MAE from 4.04 to 1.11 mA
  (72.4% reduction).
- At 15 V / 100 mA, feed-forward reduced mean peak current from
  129.67 to 112.63 mA (57.4% reduction in overshoot) and reduced
  post-60 s MAE from 5.61 to 0.78
  mA (86.1% reduction).
- Fixed-DAC open loop reached the overcurrent latch in
  65.71 +/- 2.30 s
  at 10 V and 26.18 +/- 0.79 s at 15 V.
- The per-run open-loop thermal slopes were
  1.91 +/- 0.11
  mA/degC at 10 V and 4.13 +/- 0.13
  mA/degC at 15 V.

## Defensible interpretation

The open-loop results establish a repeatable electrothermal-drift
problem under both supplies. Current-only PID eventually returns the
final-minute mean close to target, but it permits a substantially larger
thermal transient and larger cumulative regulation error. Adding NTC
feed-forward reduces both peak excursion and post-startup MAE in both
voltage groups while all six feed-forward runs complete without a fault.
This cross-voltage replication is the strongest evidence for the chosen
smart-bias architecture.

## Limits on the claim

- n=3 supports repeatability but not a broad population claim; no
  p-values or significance claims are made.
- NTC readings lag junction temperature and are package-adjacent rather
  than direct junction measurements.
- The MOSFET/load bench is a low-voltage DC proxy, not an RF PA.
- DAC telemetry is the commanded voltage, not a measured VGS value.
