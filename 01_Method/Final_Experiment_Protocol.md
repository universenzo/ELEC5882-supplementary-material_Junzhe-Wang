# Final CAN + INA226 matrix: completed protocol and evidence

Updated 23 July 2026.

## Completion status

The final matrix is complete:

```text
2 voltage groups x 3 retained modes x 3 independent repeats = 18 runs
```

Retained modes are current PID without feed-forward, current PID with NTC
feed-forward and fixed-DAC open loop. Temperature-only limiter data are
excluded from the final synthesis.

## Locked conditions actually used

| Group | Base | PID gains | FF gain | Hard trip | Physical PSU limit |
|---|---:|---|---:|---|---:|
| 10 V / 50 mA | 1934 mV | 0.82 / 0.035 / 0.08 | 0 or 3.0 mV/degC | 80 mA / 70 degC | 90 mA |
| 15 V / 100 mA | 2062 mV | 1.20 / 0.06 / 0.10 | 0 or 4.0 mV/degC | 150 mA / 100 degC | <=140 mA |

The 15 V final condition supersedes the intermediate 2040 mV `_016` plan for
the final CSV analysis. Historical files remain unchanged.

## Data layout

```text
lab_data/final_can_ina/
  10V_50mA/
    10V_50mA_01_PID_NO_FF_R1..R3.csv
    10V_50mA_03_PID_WITH_NTC_FF_R1..R3.csv
    10V_50mA_04_OPEN_LOOP_R1..R3.csv
  15V_100mA/
    15V_100mA_01_PID_NO_FF_R1..R3.csv
    15V_100mA_03_PID_WITH_NTC_FF_R1..R3.csv
    15V_100mA_04_OPEN_LOOP_R1..R3.csv
```

Each CSV records time, mode, state, faults, DAC command, current, NTC
temperature, INA validity, bus voltage, INA current and power.

## Reproducibility command

```powershell
python tools/can_iv_host/waveshare_can_a.py --port COM5 --configure-adapter `
  matrix-demo 10 pid-ff --repeat 1 --seconds 600
```

Change only voltage, retained mode and repeat index. The host applies the exact
allow-list, waits for physical arming, records live data, saves CSV/PNG and
issues STOP during cleanup.

## Analysis

```powershell
python lab_data/final_can_ina/analyze_three_mode_results.py
python lab_data/final_can_ina/three_mode_analysis/make_ieee_word_figures.py
```

Authoritative outputs:

- `three_mode_analysis/run_metrics.csv`;
- `three_mode_analysis/group_summary.csv`;
- `three_mode_analysis/fit_results.csv`;
- `three_mode_analysis/analysis_summary.md`;
- `three_mode_analysis/ieee_word/`.

## Main data-quality outcomes

- 12/12 closed-loop runs completed approximately 600 s without fault;
- 6/6 fixed-DAC runs reached the overcurrent latch;
- minimum per-run INA-valid coverage: 96.99%;
- raw samples were used for metrics;
- local-quadratic curves are descriptive display trends only;
- repeated time samples were not treated as independent replicates.

## Reproduction safety

No further powered tests are planned. If a repeat is ever required:

- retain RL, common ground, NTC insulation and physical USER arming;
- use 90 mA PSU limit at 10 V and no more than 140 mA at 15 V;
- confirm the exact UART numeric basis, not only the firmware filename;
- preserve any faulted CSV without extending or editing it;
- return DAC below 0.1 V before touching the power path.
