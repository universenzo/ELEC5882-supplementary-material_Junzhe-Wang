# Final lab test protocol

Updated 23 July 2026.

The powered experiment programme is complete. This document records the final
method for reproducibility; it is not an instruction to collect additional
data before submission.

## Final matrix

| Group | Firmware/data basis | Modes retained | Repeats | Duration |
|---|---|---|---:|---:|
| 10 V / 50 mA | 1934 mV, `_013` | PID no FF; PID + 3 mV/degC FF; open loop | 3 each | 600 s or fault |
| 15 V / 100 mA | 2062 mV, 23 July `_018` build | PID no FF; PID + 4 mV/degC FF; open loop | 3 each | 600 s or fault |

The temperature-only limiter was excluded from the final analysis because its
derating objective is not matched to current tracking. Historical limiter
data and code remain available but are not part of the 18-run synthesis.

## Hardware and safety

1. PSU output off while changing firmware or wiring.
2. Verify 10 Ohm RL, MOSFET orientation, INA INPUT/OUTPUT polarity, NTC
   insulation and common ground.
3. Keep the physical PSU limit at 90 mA for 10 V or no more than 140 mA for
   15 V.
4. Confirm the correct UART banner, `CURRENT_SOURCE=INA226`, valid I2C probe,
   correct supply window and DAC below 0.1 V.
5. Start the host command before pressing USER; one physical press arms one
   run.
6. Stop on PSU constant-current indication, invalid measurement, unexpected
   heating, unstable current, fault or uncertain wiring.

Software limits are not substitutes for the physical PSU limit.

## Reproduction command

```powershell
python tools/can_iv_host/waveshare_can_a.py --port COM5 --configure-adapter `
  matrix-demo 10 pid-ff --repeat 1 --seconds 600
```

Valid final modes are `pid-no-ff`, `pid-ff` and `open-loop`. For the 15 V
group, use `15` only after flashing the correct source build and changing the
physical PSU setting. The host creates deterministic CSV and PNG names and
issues STOP during cleanup.

The host still exposes `temp-limiter` for firmware testing, but it is not a
final-analysis mode.

## Run acceptance

- correct voltage/mode/repeat filename;
- exact compiled target/base/FF tuple accepted by the MCU;
- physical USER arm recorded;
- active samples use state 5 and faults 0 until normal completion or trip;
- INA telemetry validity recorded;
- automatic STOP or fault returns DAC to zero;
- no manual extension of a faulted run;
- cold-start differences retained rather than edited out.

## Analysis rules

- align time to the first active state-5 sample;
- exclude pre-arm and repeated latched-zero rows;
- retain the first state-6 fault row;
- calculate all metrics from raw samples;
- use descriptive trend smoothing only for visualisation;
- report mean and sample SD across the three independent runs;
- do not treat repeated time samples as independent replicates.

Metrics:

- initial current, DAC and NTC temperature;
- peak current and overshoot;
- post-60 s MAE/RMSE;
- final-minute mean and standard deviation;
- maximum temperature and temperature rise;
- open-loop time/current/temperature at fault;
- open-loop current-versus-NTC slope;
- INA-valid telemetry percentage.

Analysis command:

```powershell
python lab_data/final_can_ina/analyze_three_mode_results.py
python lab_data/final_can_ina/three_mode_analysis/make_ieee_word_figures.py
```

Authorities:

- `three_mode_analysis/run_metrics.csv`;
- `three_mode_analysis/group_summary.csv`;
- `three_mode_analysis/fit_results.csv`;
- `three_mode_analysis/analysis_summary.md`.

## Historical-data separation

Do not merge the final dataset with:

- preliminary unequal-baseline 10 V runs;
- historical 1910 mV PA1/Rs 10 V matrix;
- historical 2015 mV unequal-repeat 15 V stress runs;
- intermediate 2040 mV `_014`/`_016` runs;
- untested 13 V design variants.

Those records remain useful for development history and critical reflection,
but the report's primary quantitative comparison is the final 1934/2062 mV
CAN/INA226 matrix.
