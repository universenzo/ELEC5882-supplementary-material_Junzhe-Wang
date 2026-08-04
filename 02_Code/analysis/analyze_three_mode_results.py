"""Synthesize the final three-mode CAN/INA226 experiment matrix.

The raw CSV files are never modified.  This script:

1. validates three repeats for each retained mode and voltage;
2. calculates run-level and group-level metrics from unsmoothed samples;
3. aligns repeats to the first active sample and draws local-quadratic
   descriptive trends with +/-1 sample SD bands;
4. fits open-loop current against NTC temperature rise separately for every
   repeat, avoiding a false independent-sample claim from autocorrelated rows.

Run from the repository root with the bundled or project Python:

    python lab_data/final_can_ina/analyze_three_mode_results.py
"""

from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
LAB_DATA_ROOT = ROOT.parent
OUTPUT = ROOT / "three_mode_analysis"
if str(LAB_DATA_ROOT) not in sys.path:
    sys.path.insert(0, str(LAB_DATA_ROOT))

from plot_style import (  # noqa: E402
    COLORS,
    add_fault_marker,
    add_limit_line,
    add_reference_line,
    apply_publication_style,
    draw_group_trend,
    save_figure,
    style_axis,
)


REQUIRED_COLUMNS = {
    "time_s",
    "experiment_mode",
    "state",
    "faults",
    "dac_mv",
    "current_ma",
    "temperature_c",
    "ina_valid",
    "ina_bus_v",
    "ina_current_ma",
    "ina_power_mw",
}
RESAMPLE_STEP_S = 0.5
CONTROL_COMPLETE_MIN_S = 590.0
ANALYSIS_VERSION = "three-mode-v1"


@dataclass(frozen=True)
class VoltageSpec:
    supply_v: int
    target_ma: float
    directory: Path
    startup_xlim_s: tuple[float, float]
    startup_ylim_ma: tuple[float, float]
    full_ylim_ma: tuple[float, float]
    hard_trip_ma: float

    @property
    def key(self) -> str:
        return f"{self.supply_v}V"

    @property
    def matrix_prefix(self) -> str:
        return f"{self.supply_v}V_{int(self.target_ma)}mA"


@dataclass(frozen=True)
class ModeSpec:
    key: str
    file_token: str
    label: str
    short_label: str
    color: str
    controlled: bool


VOLTAGES = (
    VoltageSpec(
        10,
        50.0,
        ROOT / "10V_50mA",
        (0.0, 75.0),
        (45.0, 82.5),
        (46.0, 65.0),
        80.0,
    ),
    VoltageSpec(
        15,
        100.0,
        ROOT / "15V_100mA",
        (0.0, 32.0),
        (88.0, 153.0),
        (94.0, 132.0),
        150.0,
    ),
)

MODES = (
    ModeSpec(
        "pid_no_ff",
        "01_PID_NO_FF",
        "Current PID, no NTC feed-forward",
        "PID, no FF",
        COLORS["pid_no_ff"],
        True,
    ),
    ModeSpec(
        "pid_ff",
        "03_PID_WITH_NTC_FF",
        "Current PID + NTC feed-forward",
        "PID + NTC FF",
        COLORS["pid_ff"],
        True,
    ),
    ModeSpec(
        "open_loop",
        "04_OPEN_LOOP",
        "Open loop, fixed DAC",
        "Open loop",
        COLORS["open_loop"],
        False,
    ),
)

MODE_BY_KEY = {mode.key: mode for mode in MODES}


def repeat_number(path: Path) -> int:
    match = re.search(r"_R(\d+)$", path.stem)
    if not match:
        raise ValueError(f"Cannot identify repeat number in {path.name}")
    return int(match.group(1))


def bool_series(series: pd.Series) -> pd.Series:
    if pd.api.types.is_bool_dtype(series):
        return series.fillna(False)
    return (
        series.astype(str)
        .str.strip()
        .str.lower()
        .isin({"true", "1", "yes"})
    )


def linear_fit(x: np.ndarray, y: np.ndarray) -> tuple[float, float, float]:
    finite = np.isfinite(x) & np.isfinite(y)
    x_fit = np.asarray(x[finite], dtype=float)
    y_fit = np.asarray(y[finite], dtype=float)
    if len(x_fit) < 3 or np.ptp(x_fit) <= np.finfo(float).eps:
        return np.nan, np.nan, np.nan
    slope, intercept = np.polyfit(x_fit, y_fit, 1)
    predicted = slope * x_fit + intercept
    residual_ss = float(np.sum((y_fit - predicted) ** 2))
    total_ss = float(np.sum((y_fit - y_fit.mean()) ** 2))
    r_squared = 1.0 - residual_ss / total_ss if total_ss > 0.0 else np.nan
    return float(slope), float(intercept), float(r_squared)


def average_duplicate_times(active: pd.DataFrame) -> pd.DataFrame:
    fields = [
        "current_ma",
        "temperature_c",
        "dac_mv",
        "ina_bus_v",
        "ina_current_ma",
        "ina_power_mw",
    ]
    return (
        active.groupby("relative_time_s", as_index=False)[fields]
        .mean()
        .sort_values("relative_time_s")
    )


def load_run(path: Path, voltage: VoltageSpec, mode: ModeSpec) -> dict[str, object]:
    raw = pd.read_csv(path)
    missing = REQUIRED_COLUMNS.difference(raw.columns)
    if missing:
        raise ValueError(f"{path.name} is missing columns: {sorted(missing)}")
    raw = raw.copy()
    raw["ina_valid"] = bool_series(raw["ina_valid"])
    raw = raw.sort_values("time_s", kind="stable").reset_index(drop=True)

    active = raw.loc[(raw["state"] == 5) & (raw["faults"] == 0)].copy()
    if active.empty:
        raise ValueError(f"{path.name} has no active state=5 samples")
    first_time_s = float(active.iloc[0]["time_s"])
    active["relative_time_s"] = active["time_s"] - first_time_s
    active_duration_s = float(active["relative_time_s"].max())

    fault_rows = raw.loc[
        ((raw["state"] == 6) | (raw["faults"] != 0))
        & (raw["time_s"] >= first_time_s)
    ]
    fault = fault_rows.iloc[0] if not fault_rows.empty else None
    fault_time_s = (
        float(fault["time_s"]) - first_time_s if fault is not None else np.nan
    )
    fault_current_ma = np.nan
    fault_temperature_c = np.nan
    if fault is not None:
        last_active_current_ma = float(active.iloc[-1]["current_ma"])
        if bool(fault["ina_valid"]) and pd.notna(fault["ina_current_ma"]):
            # The INA frame paired with the first fault-status frame can be
            # stale immediately after DAC shutdown.  The trip endpoint must
            # not be lower than the final active current sample.
            fault_current_ma = max(
                last_active_current_ma, float(fault["ina_current_ma"])
            )
        else:
            fault_current_ma = last_active_current_ma
        fault_temperature_c = float(fault["temperature_c"])

    trajectory = average_duplicate_times(active)
    if fault is not None and fault_time_s > trajectory["relative_time_s"].max():
        endpoint = pd.DataFrame(
            {
                "relative_time_s": [fault_time_s],
                "current_ma": [fault_current_ma],
                "temperature_c": [fault_temperature_c],
                "dac_mv": [float(active.iloc[-1]["dac_mv"])],
                "ina_bus_v": [float(fault["ina_bus_v"])],
                "ina_current_ma": [fault_current_ma],
                "ina_power_mw": [float(fault["ina_power_mw"])],
            }
        )
        trajectory = pd.concat([trajectory, endpoint], ignore_index=True)

    first_5 = active.loc[active["relative_time_s"] <= 5.0]
    last_60 = active.loc[
        active["relative_time_s"] >= max(0.0, active_duration_s - 60.0)
    ]
    after_60 = active.loc[active["relative_time_s"] >= 60.0]
    after_300 = active.loc[active["relative_time_s"] >= 300.0]
    completed = (
        mode.controlled
        and fault is None
        and active_duration_s >= CONTROL_COMPLETE_MIN_S
    )

    peak_current_ma = float(active["current_ma"].max())
    initial_current_ma = float(first_5["current_ma"].median())
    initial_temperature_c = float(first_5["temperature_c"].median())
    valid_ina = active["ina_valid"] & active["ina_current_ma"].notna()
    paired_difference = (
        active.loc[valid_ina, "ina_current_ma"]
        - active.loc[valid_ina, "current_ma"]
    )

    late_slope, late_intercept, late_r2 = (
        linear_fit(
            after_300["relative_time_s"].to_numpy(dtype=float),
            after_300["current_ma"].to_numpy(dtype=float),
        )
        if completed
        else (np.nan, np.nan, np.nan)
    )
    command_fit_source = active.loc[active["relative_time_s"] >= 30.0]
    command_temp_slope, command_temp_intercept, command_temp_r2 = (
        linear_fit(
            command_fit_source["temperature_c"].to_numpy(dtype=float),
            command_fit_source["dac_mv"].to_numpy(dtype=float),
        )
        if mode.controlled
        else (np.nan, np.nan, np.nan)
    )
    thermal_slope, thermal_intercept, thermal_r2 = (
        linear_fit(
            (
                active["temperature_c"].to_numpy(dtype=float)
                - initial_temperature_c
            ),
            active["current_ma"].to_numpy(dtype=float),
        )
        if not mode.controlled
        else (np.nan, np.nan, np.nan)
    )

    mae_after_60 = (
        float((after_60["current_ma"] - voltage.target_ma).abs().mean())
        if completed
        else np.nan
    )
    rmse_after_60 = (
        float(
            np.sqrt(
                np.mean(
                    (after_60["current_ma"].to_numpy() - voltage.target_ma) ** 2
                )
            )
        )
        if completed
        else np.nan
    )
    final_60_mean = (
        float(last_60["current_ma"].mean()) if completed else np.nan
    )
    final_60_std = (
        float(last_60["current_ma"].std(ddof=1)) if completed else np.nan
    )
    final_60_error = (
        final_60_mean - voltage.target_ma if completed else np.nan
    )

    metrics = {
        "analysis_version": ANALYSIS_VERSION,
        "supply_v": voltage.supply_v,
        "target_ma": voltage.target_ma,
        "mode": mode.key,
        "mode_label": mode.label,
        "repeat": repeat_number(path),
        "source_file": path.name,
        "active_samples": len(active),
        "active_duration_s": active_duration_s,
        "completed_600s": completed,
        "fault_flags": int(fault["faults"]) if fault is not None else 0,
        "initial_current_ma": initial_current_ma,
        "initial_temperature_c": initial_temperature_c,
        "initial_dac_mv": float(first_5["dac_mv"].median()),
        "peak_current_ma": peak_current_ma,
        "peak_overshoot_ma": peak_current_ma - voltage.target_ma,
        "peak_overshoot_percent": (
            (peak_current_ma - voltage.target_ma) / voltage.target_ma * 100.0
        ),
        "mae_after_60s_ma": mae_after_60,
        "mae_after_60s_percent": (
            mae_after_60 / voltage.target_ma * 100.0
            if np.isfinite(mae_after_60)
            else np.nan
        ),
        "rmse_after_60s_ma": rmse_after_60,
        "final_60s_current_mean_ma": final_60_mean,
        "final_60s_current_std_ma": final_60_std,
        "final_60s_error_ma": final_60_error,
        "final_60s_error_percent": (
            final_60_error / voltage.target_ma * 100.0
            if np.isfinite(final_60_error)
            else np.nan
        ),
        "maximum_temperature_c": float(active["temperature_c"].max()),
        "temperature_rise_c": (
            float(active["temperature_c"].max()) - initial_temperature_c
        ),
        "minimum_dac_mv": float(active["dac_mv"].min()),
        "final_60s_dac_mean_mv": (
            float(last_60["dac_mv"].mean()) if completed else np.nan
        ),
        "late_drift_ma_per_min": late_slope * 60.0,
        "late_drift_fit_r2": late_r2,
        "command_temp_slope_mv_per_c": command_temp_slope,
        "command_temp_fit_r2": command_temp_r2,
        "fault_time_s": fault_time_s,
        "fault_current_ma": fault_current_ma,
        "fault_temperature_c": fault_temperature_c,
        "open_loop_thermal_slope_ma_per_c": thermal_slope,
        "open_loop_thermal_intercept_ma": thermal_intercept,
        "open_loop_thermal_fit_r2": thermal_r2,
        "ina_valid_percent": float(valid_ina.mean() * 100.0),
        "paired_can_current_difference_ma": (
            float(paired_difference.abs().mean())
            if not paired_difference.empty
            else np.nan
        ),
        "median_sample_interval_s": float(
            active["relative_time_s"].drop_duplicates().diff().median()
        ),
    }
    return {
        "voltage": voltage,
        "mode": mode,
        "path": path,
        "raw": raw,
        "active": active,
        "trajectory": trajectory,
        "metrics": metrics,
    }


def discover_runs() -> list[dict[str, object]]:
    runs: list[dict[str, object]] = []
    for voltage in VOLTAGES:
        for mode in MODES:
            pattern = f"{voltage.matrix_prefix}_{mode.file_token}_R*.csv"
            paths = sorted(voltage.directory.glob(pattern), key=repeat_number)
            if len(paths) != 3:
                raise ValueError(
                    f"Expected 3 repeats for {voltage.key}/{mode.key}, "
                    f"found {len(paths)} using {pattern}"
                )
            repeats = [repeat_number(path) for path in paths]
            if repeats != [1, 2, 3]:
                raise ValueError(
                    f"Expected repeats [1, 2, 3] for "
                    f"{voltage.key}/{mode.key}; found {repeats}"
                )
            runs.extend(load_run(path, voltage, mode) for path in paths)
    return runs


def resample_group(runs: list[dict[str, object]]) -> pd.DataFrame:
    common_duration_s = min(
        float(run["trajectory"]["relative_time_s"].max()) for run in runs
    )
    grid_end = np.floor(common_duration_s / RESAMPLE_STEP_S) * RESAMPLE_STEP_S
    grid = np.arange(0.0, grid_end + RESAMPLE_STEP_S / 2.0, RESAMPLE_STEP_S)
    result = pd.DataFrame({"time_s": grid})
    output_names = {
        "current_ma": "current_mA",
        "temperature_c": "temperature_C",
        "dac_mv": "dac_target_mV",
    }
    for field, output_name in output_names.items():
        values: list[np.ndarray] = []
        for run in runs:
            trajectory = run["trajectory"]
            values.append(
                np.interp(
                    grid,
                    trajectory["relative_time_s"].to_numpy(dtype=float),
                    trajectory[field].to_numpy(dtype=float),
                )
            )
        matrix = np.vstack(values)
        result[f"{output_name}_mean"] = matrix.mean(axis=0)
        result[f"{output_name}_sd"] = matrix.std(axis=0, ddof=1)
    result["n"] = len(runs)
    return result


def build_group_averages(
    runs: list[dict[str, object]],
) -> dict[tuple[int, str], pd.DataFrame]:
    averages: dict[tuple[int, str], pd.DataFrame] = {}
    for voltage in VOLTAGES:
        for mode in MODES:
            group = [
                run
                for run in runs
                if run["voltage"] == voltage and run["mode"] == mode
            ]
            averages[(voltage.supply_v, mode.key)] = resample_group(group)
    return averages


def build_group_summary(run_metrics: pd.DataFrame) -> pd.DataFrame:
    numeric_columns = [
        column
        for column in run_metrics.columns
        if pd.api.types.is_numeric_dtype(run_metrics[column])
        and column
        not in {
            "supply_v",
            "target_ma",
            "repeat",
            "fault_flags",
            "active_samples",
        }
    ]
    rows: list[dict[str, object]] = []
    for voltage in VOLTAGES:
        for mode in MODES:
            group = run_metrics.loc[
                (run_metrics["supply_v"] == voltage.supply_v)
                & (run_metrics["mode"] == mode.key)
            ]
            row: dict[str, object] = {
                "supply_v": voltage.supply_v,
                "target_ma": voltage.target_ma,
                "mode": mode.key,
                "mode_label": mode.label,
                "repeats": len(group),
                "completed_runs": int(group["completed_600s"].sum()),
                "faulted_runs": int((group["fault_flags"] != 0).sum()),
            }
            for column in numeric_columns:
                values = group[column].dropna()
                row[f"{column}_mean"] = (
                    float(values.mean()) if not values.empty else np.nan
                )
                row[f"{column}_sd"] = (
                    float(values.std(ddof=1)) if len(values) > 1 else np.nan
                )
            rows.append(row)
    return pd.DataFrame(rows)


def build_fit_results(run_metrics: pd.DataFrame) -> pd.DataFrame:
    columns = [
        "supply_v",
        "target_ma",
        "mode",
        "repeat",
        "source_file",
        "open_loop_thermal_slope_ma_per_c",
        "open_loop_thermal_intercept_ma",
        "open_loop_thermal_fit_r2",
        "command_temp_slope_mv_per_c",
        "command_temp_fit_r2",
        "late_drift_ma_per_min",
        "late_drift_fit_r2",
    ]
    return run_metrics[columns].copy()


def group_row(
    summary: pd.DataFrame, supply_v: int, mode: str
) -> pd.Series:
    return summary.loc[
        (summary["supply_v"] == supply_v) & (summary["mode"] == mode)
    ].iloc[0]


def plot_current_comparison(
    averages: dict[tuple[int, str], pd.DataFrame],
    group_summary: pd.DataFrame,
) -> Path:
    apply_publication_style()
    figure, axes = plt.subplots(2, 2, figsize=(11.2, 7.7))
    for column, voltage in enumerate(VOLTAGES):
        startup_axis = axes[0, column]
        full_axis = axes[1, column]

        for mode in MODES:
            average = averages[(voltage.supply_v, mode.key)]
            draw_group_trend(startup_axis, average, mode, "current_mA")
            if mode.controlled:
                draw_group_trend(full_axis, average, mode, "current_mA")

        add_reference_line(startup_axis, voltage.target_ma)
        add_reference_line(full_axis, voltage.target_ma)
        add_limit_line(startup_axis, voltage.hard_trip_ma, fault=True)

        open_summary = group_row(
            group_summary, voltage.supply_v, "open_loop"
        )
        add_fault_marker(
            startup_axis,
            float(open_summary["fault_time_s_mean"]),
            float(open_summary["fault_current_ma_mean"]),
            xerr=float(open_summary["fault_time_s_sd"]),
        )

        startup_axis.set(
            xlim=voltage.startup_xlim_s,
            ylim=voltage.startup_ylim_ma,
            title=(
                f"({chr(97 + column)}) {voltage.supply_v} V startup "
                "and open-loop trip"
            ),
            xlabel="Time after activation (s)",
            ylabel="Drain current (mA)",
        )
        full_axis.set(
            xlim=(0.0, 600.0),
            ylim=voltage.full_ylim_ma,
            title=(
                f"({chr(99 + column)}) {voltage.supply_v} V closed-loop "
                "regulation"
            ),
            xlabel="Time after activation (s)",
            ylabel="Drain current (mA)",
        )
        style_axis(startup_axis)
        style_axis(full_axis)

    axes[0, 0].legend(loc="upper left", frameon=True)
    axes[1, 0].legend(loc="upper right", frameon=True)
    figure.suptitle(
        "Repeated current response: retained three-mode comparison",
        y=0.99,
        fontsize=12,
    )
    figure.text(
        0.5,
        0.012,
        "Solid: local-quadratic descriptive trend; faint: raw group mean; "
        "band: +/-1 SD across three repeats; red x: mean overcurrent trip "
        "(horizontal bar: +/-1 SD). Metrics use unsmoothed data.",
        ha="center",
        fontsize=8.1,
    )
    figure.tight_layout(rect=(0.0, 0.045, 1.0, 0.975))
    path = OUTPUT / "figure_1_three_mode_current_response.png"
    save_figure(figure, path)
    return path


def draw_repeated_metric(
    axis,
    run_metrics: pd.DataFrame,
    metric: str,
    modes: tuple[ModeSpec, ...],
    ylabel: str,
    title: str,
) -> None:
    base_positions = np.arange(len(VOLTAGES), dtype=float)
    offsets = np.linspace(-0.14, 0.14, len(modes))
    marker_offsets = np.array([-0.035, 0.0, 0.035])
    for mode, offset in zip(modes, offsets):
        for index, voltage in enumerate(VOLTAGES):
            values = run_metrics.loc[
                (run_metrics["supply_v"] == voltage.supply_v)
                & (run_metrics["mode"] == mode.key),
                metric,
            ].dropna().to_numpy(dtype=float)
            x = base_positions[index] + offset
            if len(values):
                jitter = marker_offsets[: len(values)]
                axis.scatter(
                    x + jitter,
                    values,
                    s=23,
                    color=mode.color,
                    alpha=0.55,
                    edgecolors="white",
                    linewidths=0.35,
                    zorder=3,
                )
                mean = float(values.mean())
                sd = float(values.std(ddof=1)) if len(values) > 1 else 0.0
                axis.errorbar(
                    x,
                    mean,
                    yerr=sd,
                    fmt="o",
                    color=mode.color,
                    markeredgecolor="white",
                    markeredgewidth=0.6,
                    markersize=6.5,
                    capsize=3,
                    linewidth=1.3,
                    label=mode.short_label if index == 0 else None,
                    zorder=4,
                )
    axis.set_xticks(base_positions, [f"{v.supply_v} V" for v in VOLTAGES])
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    style_axis(axis)


def plot_metric_summary(run_metrics: pd.DataFrame) -> Path:
    apply_publication_style()
    controlled_modes = tuple(mode for mode in MODES if mode.controlled)
    figure, axes = plt.subplots(1, 3, figsize=(11.2, 3.8))
    draw_repeated_metric(
        axes[0],
        run_metrics,
        "peak_overshoot_percent",
        controlled_modes,
        "Peak overshoot (% of target)",
        "(a) Peak transient",
    )
    draw_repeated_metric(
        axes[1],
        run_metrics,
        "mae_after_60s_percent",
        controlled_modes,
        "MAE after 60 s (% of target)",
        "(b) Regulation error",
    )
    draw_repeated_metric(
        axes[2],
        run_metrics,
        "fault_time_s",
        (MODE_BY_KEY["open_loop"],),
        "Time to overcurrent trip (s)",
        "(c) Open-loop stability",
    )
    axes[0].legend(loc="upper left", frameon=True)
    axes[2].legend(loc="upper right", frameon=True)
    figure.suptitle(
        "Repeat-level evidence and group mean +/-1 SD (n=3)",
        y=0.99,
        fontsize=12,
    )
    figure.text(
        0.5,
        0.01,
        "Small translucent points are individual runs; large points are "
        "group means. Closed-loop metrics use raw samples; open-loop MAE is "
        "not reported because the runs terminate at the safety trip.",
        ha="center",
        fontsize=8.1,
    )
    figure.tight_layout(rect=(0.0, 0.07, 1.0, 0.95))
    path = OUTPUT / "figure_2_repeat_level_metrics.png"
    save_figure(figure, path)
    return path


def plot_open_loop_fit(
    runs: list[dict[str, object]], run_metrics: pd.DataFrame
) -> Path:
    apply_publication_style()
    figure, axes = plt.subplots(1, 2, figsize=(10.8, 4.25))
    for axis, voltage in zip(axes, VOLTAGES):
        group = [
            run
            for run in runs
            if run["voltage"] == voltage
            and run["mode"] == MODE_BY_KEY["open_loop"]
        ]
        fit_rows = run_metrics.loc[
            (run_metrics["supply_v"] == voltage.supply_v)
            & (run_metrics["mode"] == "open_loop")
        ].sort_values("repeat")
        slopes = fit_rows["open_loop_thermal_slope_ma_per_c"].to_numpy(
            dtype=float
        )
        intercepts = fit_rows["open_loop_thermal_intercept_ma"].to_numpy(
            dtype=float
        )
        r_squared = fit_rows["open_loop_thermal_fit_r2"].to_numpy(dtype=float)

        max_delta = 0.0
        for run, (_, fit_row) in zip(group, fit_rows.iterrows()):
            active = run["active"]
            initial_temp = float(fit_row["initial_temperature_c"])
            delta_temp = (
                active["temperature_c"].to_numpy(dtype=float) - initial_temp
            )
            current = active["current_ma"].to_numpy(dtype=float)
            max_delta = max(max_delta, float(np.nanmax(delta_temp)))
            subsample = slice(None, None, 5)
            axis.plot(
                delta_temp[subsample],
                current[subsample],
                color=COLORS["open_loop"],
                linewidth=0.75,
                alpha=0.22,
            )

        fit_x = np.linspace(0.0, max_delta, 160)
        mean_slope = float(np.mean(slopes))
        mean_intercept = float(np.mean(intercepts))
        axis.plot(
            fit_x,
            mean_slope * fit_x + mean_intercept,
            color=COLORS["fault"],
            linestyle="--",
            linewidth=2.0,
            label="Mean repeat-level linear fit",
        )
        add_reference_line(axis, voltage.target_ma)
        add_limit_line(axis, voltage.hard_trip_ma, fault=True)
        axis.text(
            0.04,
            0.96,
            (
                f"slope = {mean_slope:.2f} +/- {np.std(slopes, ddof=1):.2f} "
                "mA/degC\n"
                f"R-squared = {np.mean(r_squared):.3f} "
                f"(range {np.min(r_squared):.3f}-{np.max(r_squared):.3f})"
            ),
            transform=axis.transAxes,
            va="top",
            fontsize=8.7,
            bbox={
                "boxstyle": "round,pad=0.3",
                "facecolor": "white",
                "edgecolor": "#B8B8B8",
                "alpha": 0.92,
            },
        )
        axis.set(
            xlabel="NTC temperature rise from initial median (degC)",
            ylabel="Drain current (mA)",
            title=f"{voltage.supply_v} V / {int(voltage.target_ma)} mA",
        )
        style_axis(axis)
        axis.legend(loc="lower right", frameon=True)

    figure.suptitle(
        "Open-loop electrothermal drift: descriptive repeat-level fits",
        y=0.99,
        fontsize=12,
    )
    figure.text(
        0.5,
        0.012,
        "Each faint trace is one run. Fits quantify association with the "
        "package-adjacent NTC reading; they are not a thermal-device model "
        "and do not remove NTC lag.",
        ha="center",
        fontsize=8.1,
    )
    figure.tight_layout(rect=(0.0, 0.06, 1.0, 0.95))
    path = OUTPUT / "figure_3_open_loop_thermal_fit.png"
    save_figure(figure, path)
    return path


def save_averaged_traces(
    averages: dict[tuple[int, str], pd.DataFrame]
) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for voltage in VOLTAGES:
        for mode in MODES:
            frame = averages[(voltage.supply_v, mode.key)].copy()
            frame.insert(0, "supply_v", voltage.supply_v)
            frame.insert(1, "target_ma", voltage.target_ma)
            frame.insert(2, "mode", mode.key)
            frame.insert(3, "mode_label", mode.label)
            frames.append(frame)
    result = pd.concat(frames, ignore_index=True)
    result.to_csv(
        OUTPUT / "averaged_traces.csv", index=False, float_format="%.5f"
    )
    return result


def write_analysis_notes(
    run_metrics: pd.DataFrame, group_summary: pd.DataFrame
) -> Path:
    def mean(supply: int, mode: str, metric: str) -> float:
        return float(group_row(group_summary, supply, mode)[f"{metric}_mean"])

    def sd(supply: int, mode: str, metric: str) -> float:
        return float(group_row(group_summary, supply, mode)[f"{metric}_sd"])

    ten_noff_peak = mean(10, "pid_no_ff", "peak_current_ma")
    ten_ff_peak = mean(10, "pid_ff", "peak_current_ma")
    ten_noff_mae = mean(10, "pid_no_ff", "mae_after_60s_ma")
    ten_ff_mae = mean(10, "pid_ff", "mae_after_60s_ma")
    fifteen_noff_peak = mean(15, "pid_no_ff", "peak_current_ma")
    fifteen_ff_peak = mean(15, "pid_ff", "peak_current_ma")
    fifteen_noff_mae = mean(15, "pid_no_ff", "mae_after_60s_ma")
    fifteen_ff_mae = mean(15, "pid_ff", "mae_after_60s_ma")
    ten_fault = mean(10, "open_loop", "fault_time_s")
    fifteen_fault = mean(15, "open_loop", "fault_time_s")
    ten_thermal = mean(
        10, "open_loop", "open_loop_thermal_slope_ma_per_c"
    )
    fifteen_thermal = mean(
        15, "open_loop", "open_loop_thermal_slope_ma_per_c"
    )
    ina_min = float(run_metrics["ina_valid_percent"].min())
    completed = int(run_metrics["completed_600s"].sum())
    faulted = int((run_metrics["fault_flags"] != 0).sum())

    ten_mae_reduction = (ten_noff_mae - ten_ff_mae) / ten_noff_mae * 100.0
    fifteen_mae_reduction = (
        (fifteen_noff_mae - fifteen_ff_mae)
        / fifteen_noff_mae
        * 100.0
    )
    ten_peak_reduction = (
        (ten_noff_peak - 50.0) - (ten_ff_peak - 50.0)
    ) / (ten_noff_peak - 50.0) * 100.0
    fifteen_peak_reduction = (
        (fifteen_noff_peak - 100.0) - (fifteen_ff_peak - 100.0)
    ) / (fifteen_noff_peak - 100.0) * 100.0

    lines = [
        "# Final CAN/INA226 three-mode synthesis",
        "",
        "## Scope and method",
        "",
        "- Included modes: current PID without NTC feed-forward, current PID",
        "  with NTC feed-forward, and fixed-DAC open loop.",
        "- The temperature-only limiter was intentionally excluded because it",
        "  did not have a defensible matched-comparison rationale.",
        "- Dataset: 2 voltage groups x 3 retained modes x 3 repeats = 18 runs.",
        "- Repeats are aligned to the first active state=5 sample. Plotted solid",
        "  curves are local-quadratic descriptive trends; shaded bands are +/-1",
        "  sample SD across repeats. All quantitative metrics use raw samples.",
        "- Open-loop current-versus-NTC fits are calculated separately for each",
        "  repeat and then summarized. Repeated time samples are not treated as",
        "  statistically independent replicates.",
        "",
        "## Data-quality checks",
        "",
        f"- {completed} closed-loop runs completed approximately 600 s without",
        f"  a fault; all {faulted} open-loop runs reached the overcurrent latch.",
        f"- The minimum per-run valid INA226 telemetry coverage was {ina_min:.2f}%.",
        "- `current_ma` and `ina_current_ma` are two CAN messages derived from",
        "  the same INA226 path, so their agreement checks host-frame pairing",
        "  only; it is not an independent current-sensor accuracy validation.",
        "",
        "## Main quantitative evidence",
        "",
        f"- At 10 V / 50 mA, feed-forward reduced mean peak current from",
        f"  {ten_noff_peak:.2f} to {ten_ff_peak:.2f} mA "
        f"({ten_peak_reduction:.1f}% reduction in overshoot) and reduced",
        f"  post-60 s MAE from {ten_noff_mae:.2f} to {ten_ff_mae:.2f} mA",
        f"  ({ten_mae_reduction:.1f}% reduction).",
        f"- At 15 V / 100 mA, feed-forward reduced mean peak current from",
        f"  {fifteen_noff_peak:.2f} to {fifteen_ff_peak:.2f} mA "
        f"({fifteen_peak_reduction:.1f}% reduction in overshoot) and reduced",
        f"  post-60 s MAE from {fifteen_noff_mae:.2f} to {fifteen_ff_mae:.2f}",
        f"  mA ({fifteen_mae_reduction:.1f}% reduction).",
        f"- Fixed-DAC open loop reached the overcurrent latch in",
        f"  {ten_fault:.2f} +/- {sd(10, 'open_loop', 'fault_time_s'):.2f} s",
        f"  at 10 V and {fifteen_fault:.2f} +/- "
        f"{sd(15, 'open_loop', 'fault_time_s'):.2f} s at 15 V.",
        f"- The per-run open-loop thermal slopes were",
        f"  {ten_thermal:.2f} +/- "
        f"{sd(10, 'open_loop', 'open_loop_thermal_slope_ma_per_c'):.2f}",
        f"  mA/degC at 10 V and {fifteen_thermal:.2f} +/- "
        f"{sd(15, 'open_loop', 'open_loop_thermal_slope_ma_per_c'):.2f}",
        "  mA/degC at 15 V.",
        "",
        "## Defensible interpretation",
        "",
        "The open-loop results establish a repeatable electrothermal-drift",
        "problem under both supplies. Current-only PID eventually returns the",
        "final-minute mean close to target, but it permits a substantially larger",
        "thermal transient and larger cumulative regulation error. Adding NTC",
        "feed-forward reduces both peak excursion and post-startup MAE in both",
        "voltage groups while all six feed-forward runs complete without a fault.",
        "This cross-voltage replication is the strongest evidence for the chosen",
        "smart-bias architecture.",
        "",
        "## Limits on the claim",
        "",
        "- n=3 supports repeatability but not a broad population claim; no",
        "  p-values or significance claims are made.",
        "- NTC readings lag junction temperature and are package-adjacent rather",
        "  than direct junction measurements.",
        "- The MOSFET/load bench is a low-voltage DC proxy, not an RF PA.",
        "- DAC telemetry is the commanded voltage, not a measured VGS value.",
    ]
    path = OUTPUT / "analysis_summary.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    runs = discover_runs()
    run_metrics = pd.DataFrame([run["metrics"] for run in runs])
    run_metrics = run_metrics.sort_values(
        ["supply_v", "mode", "repeat"]
    ).reset_index(drop=True)
    group_summary = build_group_summary(run_metrics)
    fit_results = build_fit_results(run_metrics)
    averages = build_group_averages(runs)

    run_metrics.to_csv(
        OUTPUT / "run_metrics.csv", index=False, float_format="%.6f"
    )
    group_summary.to_csv(
        OUTPUT / "group_summary.csv", index=False, float_format="%.6f"
    )
    fit_results.to_csv(
        OUTPUT / "fit_results.csv", index=False, float_format="%.6f"
    )
    averaged_traces = save_averaged_traces(averages)

    figures = (
        plot_current_comparison(averages, group_summary),
        plot_metric_summary(run_metrics),
        plot_open_loop_fit(runs, run_metrics),
    )
    notes = write_analysis_notes(run_metrics, group_summary)
    workbook_payload = {
        "run_metrics": json.loads(run_metrics.to_json(orient="records")),
        "group_summary": json.loads(group_summary.to_json(orient="records")),
        "fit_results": json.loads(fit_results.to_json(orient="records")),
        "averaged_traces": json.loads(
            averaged_traces.to_json(orient="records")
        ),
        "analysis_summary": notes.read_text(encoding="utf-8"),
    }
    (OUTPUT / "workbook_payload.json").write_text(
        json.dumps(workbook_payload, ensure_ascii=False),
        encoding="utf-8",
    )

    print(f"Analyzed {len(runs)} runs; source CSV files were not modified.")
    print(
        run_metrics[
            [
                "supply_v",
                "mode",
                "repeat",
                "active_duration_s",
                "peak_current_ma",
                "mae_after_60s_ma",
                "fault_time_s",
                "ina_valid_percent",
            ]
        ].to_string(index=False)
    )
    print("\nGenerated:")
    for path in (
        OUTPUT / "run_metrics.csv",
        OUTPUT / "group_summary.csv",
        OUTPUT / "fit_results.csv",
        OUTPUT / "averaged_traces.csv",
        OUTPUT / "workbook_payload.json",
        *figures,
        notes,
    ):
        print(f"  {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
