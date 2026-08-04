"""Create report-ready figures for the ELEC5882 IEEE-style Word template.

Template geometry measured from ELEC5882 Final Report Template.docx:
  - US Letter, left/right margins 0.65 in
  - two 3.50 in columns with a 0.20 in gutter
  - 7.20 in full text width
  - 8 pt figure-caption style

The output PNGs are generated at their intended physical insertion size and
600 dpi.  Large titles and explanatory footnotes are deliberately excluded;
those belong in the Word caption, not inside the plotting area.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
FINAL_DATA_ROOT = ROOT.parent
OUTPUT = ROOT / "ieee_word"
OUTPUT.mkdir(parents=True, exist_ok=True)

FULL_WIDTH_IN = 7.10
SINGLE_WIDTH_IN = 3.35
OUTPUT_DPI = 600

BLUE = "#0072B2"
GREEN = "#009E73"
ORANGE = "#D55E00"
BLACK = "#222222"
GRAY = "#666666"
LIGHT_GRAY = "#B7B7B7"
RED = "#C43C39"


@dataclass(frozen=True)
class ModeStyle:
    key: str
    label: str
    color: str
    linestyle: str
    marker: str


MODES = (
    ModeStyle("pid_no_ff", "PID, no FF", BLUE, "--", "o"),
    ModeStyle("pid_ff", "PID + NTC FF", GREEN, "-", "s"),
    ModeStyle("open_loop", "Open loop", BLACK, "-.", "D"),
)
MODE_BY_KEY = {mode.key: mode for mode in MODES}

TARGETS = {10: 50.0, 15: 100.0}
TRIPS = {10: 80.0, 15: 150.0}


def apply_ieee_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman"],
            "mathtext.fontset": "stix",
            "font.size": 7.5,
            "axes.titlesize": 8.0,
            "axes.labelsize": 7.5,
            "axes.linewidth": 0.65,
            "legend.fontsize": 6.7,
            "legend.framealpha": 0.95,
            "legend.edgecolor": "#A8A8A8",
            "xtick.labelsize": 6.8,
            "ytick.labelsize": 6.8,
            "xtick.direction": "out",
            "ytick.direction": "out",
            "xtick.major.width": 0.6,
            "ytick.major.width": 0.6,
            "grid.color": "#B8B8B8",
            "grid.linewidth": 0.45,
            "grid.alpha": 0.28,
            "lines.linewidth": 1.2,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
            "savefig.dpi": OUTPUT_DPI,
        }
    )


def local_quadratic(
    x: np.ndarray, y: np.ndarray, window_s: float = 15.0
) -> np.ndarray:
    x_values = np.asarray(x, dtype=float)
    y_values = np.asarray(y, dtype=float)
    result = y_values.copy()
    finite = np.isfinite(x_values) & np.isfinite(y_values)
    finite_indices = np.flatnonzero(finite)
    if len(finite_indices) < 3:
        return result
    half_window = window_s / 2.0
    for index in finite_indices:
        neighbours = finite & (
            np.abs(x_values - x_values[index]) <= half_window
        )
        count = int(neighbours.sum())
        if count < 3:
            nearest = finite_indices[
                np.argsort(
                    np.abs(x_values[finite_indices] - x_values[index])
                )[:3]
            ]
            neighbours = np.zeros(len(x_values), dtype=bool)
            neighbours[nearest] = True
            count = len(nearest)
        local_x = x_values[neighbours] - x_values[index]
        degree = min(2, count - 1)
        coefficients = np.polynomial.polynomial.polyfit(
            local_x, y_values[neighbours], degree
        )
        result[index] = coefficients[0]
    return result


def load_processed() -> tuple[pd.DataFrame, pd.DataFrame]:
    traces = pd.read_csv(ROOT / "averaged_traces.csv")
    metrics = pd.read_csv(ROOT / "run_metrics.csv")
    return traces, metrics


def group_trace(
    traces: pd.DataFrame, supply_v: int, mode: str
) -> pd.DataFrame:
    return traces.loc[
        (traces["supply_v"] == supply_v) & (traces["mode"] == mode)
    ].sort_values("time_s")


def plot_fitted_trace(
    axis,
    frame: pd.DataFrame,
    mode: ModeStyle,
    *,
    label: str | None = None,
) -> None:
    x = frame["time_s"].to_numpy(dtype=float)
    mean = frame["current_mA_mean"].to_numpy(dtype=float)
    sd = frame["current_mA_sd"].to_numpy(dtype=float)
    trend = local_quadratic(x, mean)
    smooth_sd = np.clip(local_quadratic(x, sd), 0.0, None)
    axis.fill_between(
        x,
        trend - smooth_sd,
        trend + smooth_sd,
        color=mode.color,
        alpha=0.13,
        linewidth=0,
        zorder=1,
    )
    axis.plot(
        x,
        trend,
        color=mode.color,
        linestyle=mode.linestyle,
        linewidth=1.25,
        label=label or mode.label,
        zorder=3,
    )


def style_axis(axis) -> None:
    axis.grid(True, axis="both")
    axis.set_axisbelow(True)
    for spine in axis.spines.values():
        spine.set_linewidth(0.65)


def panel_label(axis, label: str, title: str) -> None:
    axis.text(
        0.01,
        0.97,
        label,
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=7.8,
        fontweight="bold",
    )
    axis.text(
        0.50,
        0.97,
        title,
        transform=axis.transAxes,
        ha="center",
        va="top",
        fontsize=7.8,
    )


def fault_statistics(
    metrics: pd.DataFrame, supply_v: int
) -> tuple[float, float, float, float]:
    group = metrics.loc[
        (metrics["supply_v"] == supply_v)
        & (metrics["mode"] == "open_loop")
    ]
    return (
        float(group["fault_time_s"].mean()),
        float(group["fault_time_s"].std(ddof=1)),
        float(group["fault_current_ma"].mean()),
        float(group["fault_current_ma"].std(ddof=1)),
    )


def make_current_response(
    traces: pd.DataFrame, metrics: pd.DataFrame
) -> Path:
    apply_ieee_style()
    figure, axes = plt.subplots(
        2,
        2,
        figsize=(FULL_WIDTH_IN, 4.25),
        gridspec_kw={"hspace": 0.34, "wspace": 0.27},
    )
    startup_limits = {
        10: ((0.0, 75.0), (45.0, 85.0)),
        15: ((0.0, 32.0), (88.0, 157.0)),
    }
    full_limits = {
        10: ((0.0, 600.0), (46.0, 65.0)),
        15: ((0.0, 600.0), (94.0, 132.0)),
    }

    for column, supply_v in enumerate((10, 15)):
        startup = axes[0, column]
        full = axes[1, column]
        for mode in MODES:
            plot_fitted_trace(
                startup,
                group_trace(traces, supply_v, mode.key),
                mode,
            )
            if mode.key != "open_loop":
                plot_fitted_trace(
                    full,
                    group_trace(traces, supply_v, mode.key),
                    mode,
                )

        for axis in (startup, full):
            axis.axhline(
                TARGETS[supply_v],
                color=GRAY,
                linestyle=(0, (3, 2)),
                linewidth=0.8,
                zorder=0,
            )
            axis.set_ylabel("Drain current (mA)")
            style_axis(axis)

        startup.axhline(
            TRIPS[supply_v],
            color=RED,
            linestyle=":",
            linewidth=0.85,
            zorder=0,
        )
        fault_time, fault_time_sd, fault_current, _ = fault_statistics(
            metrics, supply_v
        )
        startup.errorbar(
            fault_time,
            fault_current,
            xerr=fault_time_sd,
            fmt="x",
            color=RED,
            markeredgewidth=1.2,
            markersize=5.2,
            capsize=2,
            linewidth=0.8,
            zorder=5,
        )
        startup.set_xlim(*startup_limits[supply_v][0])
        startup.set_ylim(*startup_limits[supply_v][1])
        full.set_xlim(*full_limits[supply_v][0])
        full.set_ylim(*full_limits[supply_v][1])
        startup.set_xlabel("Time after activation (s)")
        full.set_xlabel("Time after activation (s)")
        panel_label(
            startup,
            f"({chr(97 + column)})",
            f"{supply_v} V startup",
        )
        panel_label(
            full,
            f"({chr(99 + column)})",
            f"{supply_v} V closed-loop response",
        )

    legend_handles = [
        Line2D(
            [0],
            [0],
            color=mode.color,
            linestyle=mode.linestyle,
            linewidth=1.4,
            label=mode.label,
        )
        for mode in MODES
    ]
    legend_handles.extend(
        [
            Line2D(
                [0],
                [0],
                color=GRAY,
                linestyle=(0, (3, 2)),
                linewidth=0.9,
                label="Target",
            ),
            Line2D(
                [0],
                [0],
                color=RED,
                marker="x",
                linestyle="None",
                markeredgewidth=1.1,
                label="Trip mean",
            ),
        ]
    )
    figure.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.995),
        ncol=5,
        columnspacing=1.0,
        handlelength=2.3,
        frameon=True,
    )
    figure.subplots_adjust(
        left=0.085,
        right=0.985,
        bottom=0.10,
        top=0.91,
    )
    path = OUTPUT / "Fig1_current_response_2col_7p10in.png"
    figure.savefig(path, dpi=OUTPUT_DPI)
    plt.close(figure)
    return path


def make_current_response_single_column(
    traces: pd.DataFrame, metrics: pd.DataFrame
) -> Path:
    """Create a single-column Fig. 1 without shrinking the 2-by-2 layout.

    The four evidence views are stacked so every time axis retains the full
    3.35 in column width.  Labels in the compact legend are shortened only;
    the plotted modes, smoothing, SD bands, targets and trip statistics are
    identical to the double-column version.
    """

    apply_ieee_style()
    figure, axes = plt.subplots(
        4,
        1,
        figsize=(SINGLE_WIDTH_IN, 5.10),
        gridspec_kw={"hspace": 0.34},
    )
    startup_limits = {
        10: ((0.0, 75.0), (45.0, 85.0)),
        15: ((0.0, 32.0), (88.0, 157.0)),
    }
    full_limits = {
        10: ((0.0, 600.0), (46.0, 65.0)),
        15: ((0.0, 600.0), (94.0, 132.0)),
    }

    for group_index, supply_v in enumerate((10, 15)):
        startup = axes[group_index * 2]
        full = axes[group_index * 2 + 1]
        for mode in MODES:
            plot_fitted_trace(
                startup,
                group_trace(traces, supply_v, mode.key),
                mode,
            )
            if mode.key != "open_loop":
                plot_fitted_trace(
                    full,
                    group_trace(traces, supply_v, mode.key),
                    mode,
                )

        for axis in (startup, full):
            axis.axhline(
                TARGETS[supply_v],
                color=GRAY,
                linestyle=(0, (3, 2)),
                linewidth=0.8,
                zorder=0,
            )
            style_axis(axis)

        startup.axhline(
            TRIPS[supply_v],
            color=RED,
            linestyle=":",
            linewidth=0.85,
            zorder=0,
        )
        fault_time, fault_time_sd, fault_current, _ = fault_statistics(
            metrics, supply_v
        )
        startup.errorbar(
            fault_time,
            fault_current,
            xerr=fault_time_sd,
            fmt="x",
            color=RED,
            markeredgewidth=1.2,
            markersize=5.2,
            capsize=2,
            linewidth=0.8,
            zorder=5,
        )
        startup.set_xlim(*startup_limits[supply_v][0])
        startup.set_ylim(*startup_limits[supply_v][1])
        full.set_xlim(*full_limits[supply_v][0])
        full.set_ylim(*full_limits[supply_v][1])
        panel_label(
            startup,
            f"({chr(97 + group_index * 2)})",
            f"{supply_v} V startup",
        )
        panel_label(
            full,
            f"({chr(98 + group_index * 2)})",
            f"{supply_v} V closed-loop response",
        )

    legend_handles = [
        Line2D(
            [0],
            [0],
            color=MODE_BY_KEY["pid_no_ff"].color,
            linestyle=MODE_BY_KEY["pid_no_ff"].linestyle,
            linewidth=1.4,
            label="PID only",
        ),
        Line2D(
            [0],
            [0],
            color=MODE_BY_KEY["pid_ff"].color,
            linestyle=MODE_BY_KEY["pid_ff"].linestyle,
            linewidth=1.4,
            label="PID + NTC FF",
        ),
        Line2D(
            [0],
            [0],
            color=MODE_BY_KEY["open_loop"].color,
            linestyle=MODE_BY_KEY["open_loop"].linestyle,
            linewidth=1.4,
            label="Fixed DAC",
        ),
        Line2D(
            [0],
            [0],
            color=GRAY,
            linestyle=(0, (3, 2)),
            linewidth=0.9,
            label="Target",
        ),
        Line2D(
            [0],
            [0],
            color=RED,
            marker="x",
            linestyle="None",
            markeredgewidth=1.1,
            label="OC trip mean",
        ),
    ]
    figure.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.996),
        ncol=3,
        columnspacing=0.75,
        handlelength=2.0,
        handletextpad=0.4,
        fontsize=6.5,
        frameon=True,
    )
    figure.supylabel("Drain current (mA)", x=0.015, fontsize=7.5)
    figure.supxlabel("Time after activation (s)", y=0.012, fontsize=7.5)
    figure.subplots_adjust(
        left=0.19,
        right=0.975,
        bottom=0.085,
        top=0.895,
    )
    path = OUTPUT / "Fig1_current_response_1col_3p35in.png"
    figure.savefig(path, dpi=OUTPUT_DPI)
    plt.close(figure)
    return path


def draw_repeat_metric(
    axis,
    metrics: pd.DataFrame,
    metric: str,
    modes: tuple[ModeStyle, ...],
    ylim: tuple[float, float],
) -> None:
    base = np.array([0.0, 1.0])
    offsets = np.linspace(-0.11, 0.11, len(modes))
    jitter = np.array([-0.026, 0.0, 0.026])
    for mode, offset in zip(modes, offsets):
        for index, supply_v in enumerate((10, 15)):
            values = metrics.loc[
                (metrics["supply_v"] == supply_v)
                & (metrics["mode"] == mode.key),
                metric,
            ].dropna().to_numpy(dtype=float)
            x = base[index] + offset
            axis.scatter(
                x + jitter[: len(values)],
                values,
                s=9,
                facecolor=mode.color,
                edgecolor="white",
                linewidth=0.25,
                alpha=0.50,
                zorder=3,
            )
            axis.errorbar(
                x,
                float(values.mean()),
                yerr=float(values.std(ddof=1)),
                fmt=mode.marker,
                color=mode.color,
                markeredgecolor="white",
                markeredgewidth=0.35,
                markersize=3.9,
                capsize=1.8,
                linewidth=0.8,
                zorder=4,
            )
    axis.set_xlim(-0.35, 1.35)
    axis.set_ylim(*ylim)
    axis.set_xticks(base, ["10 V", "15 V"])
    axis.grid(True, axis="y")
    axis.set_axisbelow(True)


def make_metric_summary(metrics: pd.DataFrame) -> Path:
    apply_ieee_style()
    figure, axes = plt.subplots(
        3,
        1,
        figsize=(SINGLE_WIDTH_IN, 4.15),
        gridspec_kw={"hspace": 0.58},
    )
    controlled = (MODE_BY_KEY["pid_no_ff"], MODE_BY_KEY["pid_ff"])
    draw_repeat_metric(
        axes[0],
        metrics,
        "peak_overshoot_percent",
        controlled,
        (10.5, 32.0),
    )
    axes[0].set_ylabel("Peak overshoot\n(% target)")
    panel_label(axes[0], "(a)", "Peak transient")

    draw_repeat_metric(
        axes[1],
        metrics,
        "mae_after_60s_percent",
        controlled,
        (0.0, 8.8),
    )
    axes[1].set_ylabel("Post-60 s MAE\n(% target)")
    panel_label(axes[1], "(b)", "Regulation error")

    draw_repeat_metric(
        axes[2],
        metrics,
        "fault_time_s",
        (MODE_BY_KEY["open_loop"],),
        (20.0, 72.0),
    )
    axes[2].set_ylabel("Trip time (s)")
    panel_label(axes[2], "(c)", "Open-loop overcurrent trip")

    handles = [
        Line2D(
            [0],
            [0],
            marker=mode.marker,
            color=mode.color,
            linestyle="None",
            markersize=4.3,
            markeredgecolor="white",
            markeredgewidth=0.35,
            label=mode.label,
        )
        for mode in MODES
    ]
    figure.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.997),
        ncol=3,
        columnspacing=0.8,
        handletextpad=0.35,
        frameon=True,
    )
    figure.subplots_adjust(
        left=0.205,
        right=0.97,
        bottom=0.075,
        top=0.91,
    )
    path = OUTPUT / "Fig2_repeat_metrics_1col_3p35in.png"
    figure.savefig(path, dpi=OUTPUT_DPI)
    plt.close(figure)
    return path


def open_loop_paths(supply_v: int) -> list[Path]:
    target = int(TARGETS[supply_v])
    directory = FINAL_DATA_ROOT / f"{supply_v}V_{target}mA"
    return sorted(directory.glob(f"{supply_v}V_{target}mA_04_OPEN_LOOP_R*.csv"))


def make_thermal_fit(metrics: pd.DataFrame) -> Path:
    apply_ieee_style()
    figure, axis = plt.subplots(
        figsize=(SINGLE_WIDTH_IN, 2.65),
    )
    voltage_styles = {
        10: (BLUE, "o"),
        15: (ORANGE, "s"),
    }
    annotation_lines: list[str] = []

    for supply_v in (10, 15):
        color, marker = voltage_styles[supply_v]
        target = TARGETS[supply_v]
        group_metrics = metrics.loc[
            (metrics["supply_v"] == supply_v)
            & (metrics["mode"] == "open_loop")
        ].sort_values("repeat")
        max_delta = 0.0
        for path, (_, row) in zip(open_loop_paths(supply_v), group_metrics.iterrows()):
            data = pd.read_csv(path)
            active = data.loc[
                (data["state"] == 5) & (data["faults"] == 0)
            ].copy()
            initial_temp = float(row["initial_temperature_c"])
            delta = active["temperature_c"].to_numpy(dtype=float) - initial_temp
            normalized = active["current_ma"].to_numpy(dtype=float) / target * 100.0
            max_delta = max(max_delta, float(np.nanmax(delta)))
            axis.plot(
                delta[::5],
                normalized[::5],
                color=color,
                linewidth=0.55,
                alpha=0.20,
                zorder=1,
            )

        slopes = (
            group_metrics["open_loop_thermal_slope_ma_per_c"].to_numpy(
                dtype=float
            )
            / target
            * 100.0
        )
        intercepts = (
            group_metrics["open_loop_thermal_intercept_ma"].to_numpy(
                dtype=float
            )
            / target
            * 100.0
        )
        fit_x = np.linspace(0.0, max_delta, 160)
        axis.plot(
            fit_x,
            float(slopes.mean()) * fit_x + float(intercepts.mean()),
            color=color,
            linestyle="-" if supply_v == 10 else "--",
            linewidth=1.35,
            label=f"{supply_v} V fit",
            zorder=3,
        )
        r2 = group_metrics["open_loop_thermal_fit_r2"].to_numpy(dtype=float)
        annotation_lines.append(
            f"{supply_v} V: {slopes.mean():.2f} ± "
            f"{slopes.std(ddof=1):.2f} %/°C, "
            f"R²={r2.mean():.3f}"
        )

        delta_at_fault = (
            group_metrics["fault_temperature_c"].to_numpy(dtype=float)
            - group_metrics["initial_temperature_c"].to_numpy(dtype=float)
        )
        fault_percent = (
            group_metrics["fault_current_ma"].to_numpy(dtype=float)
            / target
            * 100.0
        )
        axis.errorbar(
            float(delta_at_fault.mean()),
            float(fault_percent.mean()),
            xerr=float(delta_at_fault.std(ddof=1)),
            yerr=float(fault_percent.std(ddof=1)),
            fmt=marker,
            color=color,
            markeredgecolor="white",
            markeredgewidth=0.35,
            markersize=4.0,
            capsize=1.8,
            linewidth=0.7,
            zorder=4,
        )

    axis.axhline(
        100.0,
        color=GRAY,
        linestyle=(0, (3, 2)),
        linewidth=0.8,
        zorder=0,
    )
    axis.set(
        xlim=(-0.25, 14.6),
        ylim=(90.0, 164.0),
        xlabel=r"NTC temperature rise, $\Delta T$ ($^\circ$C)",
        ylabel="Drain current (% target)",
    )
    style_axis(axis)
    panel_label(axis, "", "Open-loop thermal drift")
    axis.legend(
        loc="lower right",
        ncol=2,
        columnspacing=0.8,
        handlelength=2.0,
        frameon=True,
    )
    axis.text(
        0.025,
        0.88,
        "\n".join(annotation_lines),
        transform=axis.transAxes,
        ha="left",
        va="top",
        fontsize=6.4,
        bbox={
            "boxstyle": "round,pad=0.22",
            "facecolor": "white",
            "edgecolor": "#A8A8A8",
            "alpha": 0.94,
        },
    )
    figure.subplots_adjust(
        left=0.19,
        right=0.975,
        bottom=0.20,
        top=0.96,
    )
    path = OUTPUT / "Fig3_open_loop_thermal_fit_1col_3p35in.png"
    figure.savefig(path, dpi=OUTPUT_DPI)
    plt.close(figure)
    return path


def add_image_axes(
    figure,
    image_path: Path,
    x_in: float,
    y_in: float,
    width_in: float,
    height_in: float,
) -> None:
    axis = figure.add_axes(
        [
            x_in / 8.5,
            y_in / 11.0,
            width_in / 8.5,
            height_in / 11.0,
        ]
    )
    axis.imshow(mpimg.imread(image_path))
    axis.set_axis_off()


def make_layout_preview(
    fig1: Path,
    fig2: Path,
    fig3: Path,
) -> Path:
    apply_ieee_style()
    figure = plt.figure(figsize=(8.5, 11.0), dpi=150, facecolor="#D6D6D6")
    page = figure.add_axes([0, 0, 1, 1])
    page.set_xlim(0, 8.5)
    page.set_ylim(0, 11.0)
    page.set_axis_off()
    page.add_patch(
        Rectangle(
            (0.0, 0.0),
            8.5,
            11.0,
            facecolor="white",
            edgecolor="#777777",
            linewidth=0.7,
        )
    )
    page.plot(
        [0.65, 0.65],
        [0.70, 10.30],
        color="#D0D0D0",
        linewidth=0.5,
        linestyle=":",
    )
    page.plot(
        [7.85, 7.85],
        [0.70, 10.30],
        color="#D0D0D0",
        linewidth=0.5,
        linestyle=":",
    )
    page.plot(
        [4.15, 4.15],
        [0.70, 5.30],
        color="#D0D0D0",
        linewidth=0.5,
        linestyle=":",
    )

    add_image_axes(figure, fig1, 0.70, 5.85, 7.10, 4.25)
    figure.text(
        0.5,
        5.70 / 11.0,
        "Fig. 1. Repeated current response across the retained three-mode matrix.",
        ha="center",
        va="top",
        fontsize=8.0,
        family="Times New Roman",
    )

    add_image_axes(figure, fig2, 0.73, 1.30, 3.35, 4.15)
    figure.text(
        2.40 / 8.5,
        1.16 / 11.0,
        "Fig. 2. Repeat-level performance metrics (n=3).",
        ha="center",
        va="top",
        fontsize=8.0,
        family="Times New Roman",
    )

    add_image_axes(figure, fig3, 4.42, 2.78, 3.35, 2.65)
    figure.text(
        6.10 / 8.5,
        2.64 / 11.0,
        "Fig. 3. Open-loop current-temperature association.",
        ha="center",
        va="top",
        fontsize=8.0,
        family="Times New Roman",
    )

    for index in range(9):
        y = 2.34 - index * 0.17
        width = 3.18 if index not in {3, 7} else 2.35
        page.plot(
            [4.45, 4.45 + width],
            [y, y],
            color="#B9B9B9",
            linewidth=0.55,
        )
    page.text(
        0.65,
        10.48,
        "Suggested results-page figure placement at actual template scale",
        fontsize=8.5,
        family="Times New Roman",
        fontweight="bold",
        va="bottom",
    )
    page.text(
        7.85,
        0.42,
        "US Letter; margins 0.65/0.70 in; columns 3.50 in; gutter 0.20 in",
        fontsize=6.8,
        family="Times New Roman",
        ha="right",
    )

    path = OUTPUT / "IEEE_double_column_layout_preview.png"
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return path


def write_layout_guide(fig1: Path, fig2: Path, fig3: Path) -> Path:
    text = f"""# IEEE double-column figure placement

Template measurements:

- Full text width: 7.20 in
- Column width: 3.50 in
- Column gap: 0.20 in
- Template example figures: approximately 3.31-3.34 in wide
- Figure caption style: 8 pt

Recommended placement:

1. `{fig1.name}` - insert at 7.10 in wide as the only double-column
   figure. Use inline placement. Surround it with continuous section breaks:
   switch temporarily to one column, insert figure and caption, then return to
   two 3.50 in columns with a 0.20 in gap.
2. `{fig2.name}` - insert inline at 3.35 in wide in a normal column.
3. `{fig3.name}` - insert inline at 3.35 in wide in the adjacent or following
   column.

Do not add a Word text box or floating wrap. Keep each caption in the paragraph
immediately after its image using the template's `Figure Caption` style.

Suggested captions:

- Fig. 1. Repeated drain-current response at 10 and 15 V. Curves are
  local-quadratic trends of the aligned group means and bands show +/-1 SD
  across three repeats. Red crosses mark mean open-loop overcurrent trip time
  (+/-1 SD).
- Fig. 2. Repeat-level peak overshoot, post-60 s mean absolute error, and
  fixed-DAC time to overcurrent trip. Small points are individual runs and
  larger symbols show mean +/-1 SD (n=3).
- Fig. 3. Open-loop drain current versus package-adjacent NTC temperature rise.
  Faint lines are individual runs; fitted slopes are summarized across three
  runs. The relationship is descriptive and does not remove NTC thermal lag.
"""
    path = OUTPUT / "placement_guide.md"
    path.write_text(text, encoding="utf-8")
    return path


def main() -> None:
    traces, metrics = load_processed()
    fig1 = make_current_response(traces, metrics)
    fig1_single = make_current_response_single_column(traces, metrics)
    fig2 = make_metric_summary(metrics)
    fig3 = make_thermal_fit(metrics)
    preview = make_layout_preview(fig1, fig2, fig3)
    guide = write_layout_guide(fig1, fig2, fig3)

    print("Generated IEEE Word figures:")
    for path in (fig1, fig1_single, fig2, fig3, preview, guide):
        print(f"  {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
