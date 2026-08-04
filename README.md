# Supplementary Material

This package contains the supporting evidence for the low-voltage smart-bias feasibility study. It is organised as follows.

## 01_Method

- `Final_Experiment_Protocol.md`: final 18-run matrix and locked conditions.
- `Lab_Test_Protocol.md`: test sequence, acceptance rules, safety limits and analysis rules.
- `Hardware_Reference.md`: final hardware configuration and wiring authority.
- `Firmware_Reference.md`: firmware identities and protocol details.

## 02_Code

- `firmware/`: selected application source for sensing, PID and temperature feed-forward control, protection, CAN communication and automated characterisation. Cube-generated startup, device support and HAL library files are not duplicated here.
- `host/`: the PC-side Waveshare USB-CAN-A utility and its protocol test.
- `analysis/`: scripts used to calculate the final metrics and prepare report-sized figures.

## 03_Raw_Data

- `10V_50mA/`: nine final CSV files covering three modes and three repeats.
- `15V_100mA/`: nine final CSV files covering three modes and three repeats.
- `Automated_Sweep/`: one measured fixed-supply DAC-command/drain-current sweep and its plot.

The final matrix contains 18 runs in total. The retained modes are PID without temperature feed-forward, PID with NTC feed-forward, and fixed-DAC open loop.

## 04_Processed_Results

- `tables/`: run-level metrics, group summaries, fitted slopes, averaged traces and the written analysis summary.
- `figures/`: the system diagram, current-response comparison and open-loop current-temperature relationship used in the report.

## Reproduction

The analysis can be repeated from the repository root with:

```powershell
python lab_data/final_can_ina/analyze_three_mode_results.py
python lab_data/final_can_ina/three_mode_analysis/make_ieee_word_figures.py
```

The supplied scripts in `02_Code/analysis` are copies of these analysis files. Relative paths are preserved in the main project repository rather than in this compact package.
