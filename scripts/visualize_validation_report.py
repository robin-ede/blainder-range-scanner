#!/usr/bin/env python3
"""
Visualize a BLAINDER validation report JSON.

Generates up to three PNGs alongside the report:
  1. <stem>_rmse_comparison.png  — bar chart of RMSE across all pipeline stages
  2. <stem>_error_heatmap.png    — 2-D XY reprojection error heatmap (post-fusion)
  3. <stem>_depth_histogram.png  — synthetic vs reference depth distribution

Requirements:
  pip install matplotlib numpy

Usage:
  python scripts/visualize_validation_report.py path/to/*_report.json [--output-dir DIR]

Multiple reports may be passed; one set of PNGs is written per report.
"""

import argparse
import json
import os
import sys


# ---------------------------------------------------------------------------
# Matplotlib import guard
# ---------------------------------------------------------------------------


def _get_plt():
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        return plt
    except ImportError:
        sys.exit("matplotlib is required for visualisation: pip install matplotlib")


def _get_np():
    try:
        import numpy as np

        return np
    except ImportError:
        sys.exit("numpy is required: pip install numpy")


# ---------------------------------------------------------------------------
# Data helpers
# ---------------------------------------------------------------------------


def _load_report(path):
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


_STAGE_KEYS = [
    # (report_key_path, display_label, is_post_fusion)
    # is_post_fusion controls bar colour: False→blue, True→green/red vs threshold
    (["pre_fusion_combined_metrics"], "Pre-fusion\n(raw hits)", False),
    (["post_processing_fusion", "metrics"], "Post-fusion\n(voxel)", True),
    (["medium_separated_reconstruction", "metrics"], "Medium-sep.\nreconstruct", True),
    (["medium_height_grid_reconstruction", "metrics"], "Grid\nreconstruct", True),
    (["alignment_correction", "metrics"], "Ref-corrected\npoints", True),
    (["blind_alignment_correction", "metrics"], "Blind ICP\npoints", True),
    (["corrected_grid_reconstruction", "metrics"], "Ref-corrected\ngrid", True),
    (["blind_corrected_grid_reconstruction", "metrics"], "Blind ICP\ngrid", True),
]


def _get_nested(d, keys):
    for k in keys:
        if not isinstance(d, dict):
            return None
        d = d.get(k)
    return d


def _rmse_stages(report):
    stages = []
    for key_path, label, post in _STAGE_KEYS:
        metrics = _get_nested(report, key_path)
        if isinstance(metrics, dict):
            rmse = metrics.get("rmse")
            if rmse is not None:
                stages.append((label, float(rmse), post))
    return stages


# ---------------------------------------------------------------------------
# Plot 1 — RMSE comparison bar chart
# ---------------------------------------------------------------------------


def plot_rmse_comparison(report, output_path, title=None):
    plt = _get_plt()

    stages = _rmse_stages(report)
    if not stages:
        print("  [skip] No RMSE data found — RMSE chart not generated.")
        return None

    threshold = float(report.get("acceptance", {}).get("rmse_threshold_m", 0.15))

    labels = [s[0] for s in stages]
    values = [s[1] for s in stages]
    is_post = [s[2] for s in stages]

    colors = []
    for v, post in zip(values, is_post):
        if not post:
            colors.append("#5b8db8")  # blue  — pre-fusion baseline
        elif v <= threshold:
            colors.append("#4caf50")  # green — passes threshold
        else:
            colors.append("#e57373")  # red   — fails threshold

    fig, ax = plt.subplots(figsize=(max(8, len(stages) * 1.5), 5))
    bars = ax.bar(
        range(len(labels)),
        values,
        color=colors,
        edgecolor="white",
        linewidth=0.8,
    )

    ax.axhline(
        y=threshold,
        color="#f39c12",
        linestyle="--",
        linewidth=1.5,
        label="Pass threshold (%.3f m)" % threshold,
    )

    y_top = max(values) * 1.18
    for bar, v in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2.0,
            v + max(values) * 0.012,
            "%.4f" % v,
            ha="center",
            va="bottom",
            fontsize=8,
        )

    ax.set_xticks(range(len(labels)))
    ax.set_xticklabels(labels, fontsize=8)
    ax.set_ylabel("RMSE (m)")
    ax.set_ylim(0, y_top)
    ax.set_title(title or "RMSE by Pipeline Stage")
    ax.legend(fontsize=8)

    # Error reduction annotation
    er = report.get("error_reduction", {})
    reduction_pct = er.get("error_reduction_pct")
    if reduction_pct is not None:
        meets = er.get("meets_80pct_reduction", False)
        ann_color = "#4caf50" if meets else "#e57373"
        ax.annotate(
            "Error reduction: %.1f%%  (%s)"
            % (reduction_pct, "PASS" if meets else "FAIL"),
            xy=(0.98, 0.97),
            xycoords="axes fraction",
            ha="right",
            va="top",
            fontsize=9,
            color=ann_color,
            bbox=dict(
                boxstyle="round,pad=0.3",
                facecolor="white",
                edgecolor=ann_color,
                alpha=0.85,
            ),
        )

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


# ---------------------------------------------------------------------------
# Plot 2 — Reprojection error heatmap
# ---------------------------------------------------------------------------


def plot_reprojection_heatmap(report, output_path, title=None):
    plt = _get_plt()
    np = _get_np()

    samples = _get_nested(report, ["post_processing_fusion", "spatial_error_samples"])
    if not samples:
        print(
            "  [skip] No spatial_error_samples in report.\n"
            "         Re-run the pipeline (the new generic.py now stores them automatically)."
        )
        return None

    arr = np.asarray(samples, dtype=float)  # shape (N, 4): x, y, z, distance
    xs, ys, ds = arr[:, 0], arr[:, 1], arr[:, 3]

    n_cells = 60
    x_edges = np.linspace(xs.min(), xs.max(), n_cells + 1)
    y_edges = np.linspace(ys.min(), ys.max(), n_cells + 1)

    grid_sum = np.zeros((n_cells, n_cells), dtype=float)
    grid_count = np.zeros((n_cells, n_cells), dtype=int)

    xi = np.clip(np.digitize(xs, x_edges) - 1, 0, n_cells - 1)
    yi = np.clip(np.digitize(ys, y_edges) - 1, 0, n_cells - 1)

    for i, j, d in zip(xi, yi, ds):
        grid_sum[j, i] += d
        grid_count[j, i] += 1

    with np.errstate(invalid="ignore"):
        grid_mean = np.where(grid_count > 0, grid_sum / grid_count, np.nan)

    threshold = float(report.get("acceptance", {}).get("rmse_threshold_m", 0.15))
    vmax = float(
        max(
            np.nanmax(grid_mean) if not np.all(np.isnan(grid_mean)) else threshold,
            threshold,
        )
    )

    fig, ax = plt.subplots(figsize=(7, 6))
    im = ax.imshow(
        grid_mean,
        origin="lower",
        extent=[float(xs.min()), float(xs.max()), float(ys.min()), float(ys.max())],
        aspect="auto",
        cmap="RdYlGn_r",
        vmin=0.0,
        vmax=vmax,
        interpolation="nearest",
    )

    cb = plt.colorbar(im, ax=ax)
    cb.set_label("Mean surface distance error (m)")
    # Mark threshold on colour bar
    threshold_y = threshold / vmax
    cb.ax.plot(
        [0.0, 1.0],
        [threshold_y, threshold_y],
        color="black",
        linestyle="--",
        linewidth=1.2,
        transform=cb.ax.transAxes,
    )
    cb.ax.text(
        1.08,
        threshold_y,
        "%.3f m" % threshold,
        transform=cb.ax.transAxes,
        va="center",
        fontsize=7,
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(
        title
        or "Reprojection Error Heatmap\n(post-fusion · mean surface distance per cell)"
    )

    ax.text(
        0.01,
        0.99,
        "%d points" % len(samples),
        transform=ax.transAxes,
        va="top",
        fontsize=7,
        color="white",
        bbox=dict(facecolor="black", alpha=0.5, pad=2),
    )

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


# ---------------------------------------------------------------------------
# Plot 3 — Depth distribution histogram
# ---------------------------------------------------------------------------


def plot_depth_histogram(report, output_path, title=None):
    plt = _get_plt()

    dist_data = report.get("depth_distribution_comparison")
    if not dist_data or dist_data.get("status") != "computed":
        print(
            "  [skip] No depth_distribution_comparison data — histogram not generated."
        )
        return None

    syn_hist = dist_data.get("synthetic_histogram", {})
    ref_hist = dist_data.get("reference_histogram", {})
    syn_density = syn_hist.get("density", [])
    ref_density = ref_hist.get("density", [])
    bin_edges = syn_hist.get("bin_edges", [])

    if not syn_density or not bin_edges:
        print("  [skip] Incomplete histogram data — histogram not generated.")
        return None

    bin_centers = [
        (bin_edges[i] + bin_edges[i + 1]) / 2.0 for i in range(len(bin_edges) - 1)
    ]
    width = bin_edges[1] - bin_edges[0]

    fig, ax = plt.subplots(figsize=(9, 5))

    ax.bar(
        bin_centers,
        syn_density,
        width=width * 0.45,
        align="center",
        label="Synthetic",
        color="#5b8db8",
        alpha=0.85,
        edgecolor="white",
        linewidth=0.5,
    )

    if ref_density:
        shifted = [c + width * 0.45 for c in bin_centers]
        ax.bar(
            shifted,
            ref_density,
            width=width * 0.45,
            align="center",
            label="Reference (real-world)",
            color="#ff7043",
            alpha=0.85,
            edgecolor="white",
            linewidth=0.5,
        )

    ax.set_xlabel("Depth (m)")
    ax.set_ylabel("Density")
    ax.set_title(title or "Depth Distribution: Synthetic vs Reference")
    ax.legend(fontsize=9)

    # KS annotation
    ks = dist_data.get("ks_statistic")
    ks_thresh = float(dist_data.get("ks_threshold", 0.15))
    passes_ks = dist_data.get("passes_ks_threshold")
    max_dev = dist_data.get("max_bin_deviation")
    passes_bd = dist_data.get("passes_bin_deviation_threshold")

    if ks is not None:
        ks_color = "#4caf50" if passes_ks else "#e57373"
        lines = [
            "KS stat = %.4f  (threshold %.2f)  — %s"
            % (ks, ks_thresh, "PASS" if passes_ks else "FAIL")
        ]
        if max_dev is not None:
            lines.append(
                "Max bin deviation = %.3f  (threshold 0.10)  — %s"
                % (max_dev, "PASS" if passes_bd else "FAIL")
            )
        ax.annotate(
            "\n".join(lines),
            xy=(0.98, 0.96),
            xycoords="axes fraction",
            ha="right",
            va="top",
            fontsize=8,
            color=ks_color,
            bbox=dict(
                boxstyle="round,pad=0.35",
                facecolor="white",
                edgecolor=ks_color,
                alpha=0.85,
            ),
        )

    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return output_path


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------


def visualize_report(report_path, output_dir=None):
    report = _load_report(report_path)

    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(report_path))
    os.makedirs(output_dir, exist_ok=True)

    stem = os.path.splitext(os.path.basename(report_path))[0]
    scenario_name = report.get("scenario", {}).get("data_file_name", stem)

    print("Visualizing: %s" % report_path)
    generated = []

    rmse_path = os.path.join(output_dir, "%s_rmse_comparison.png" % stem)
    result = plot_rmse_comparison(
        report,
        rmse_path,
        title="RMSE by Stage — %s" % scenario_name,
    )
    if result:
        print("  RMSE chart  : %s" % result)
        generated.append(result)

    heatmap_path = os.path.join(output_dir, "%s_error_heatmap.png" % stem)
    result = plot_reprojection_heatmap(
        report,
        heatmap_path,
        title="Error Heatmap — %s" % scenario_name,
    )
    if result:
        print("  Heatmap     : %s" % result)
        generated.append(result)

    hist_path = os.path.join(output_dir, "%s_depth_histogram.png" % stem)
    result = plot_depth_histogram(
        report,
        hist_path,
        title="Depth Distribution — %s" % scenario_name,
    )
    if result:
        print("  Histogram   : %s" % result)
        generated.append(result)

    return generated


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Generate RMSE comparison chart, reprojection error heatmap, "
            "and depth distribution histogram from one or more BLAINDER "
            "validation report JSON files."
        )
    )
    parser.add_argument(
        "reports",
        nargs="+",
        help="Path(s) to *_report.json file(s).",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help=("Directory for output PNGs. Defaults to each report's own directory."),
    )
    args = parser.parse_args()

    all_generated = []
    for path in args.reports:
        if not os.path.isfile(path):
            print("Warning: report not found — %s" % path)
            continue
        generated = visualize_report(path, output_dir=args.output_dir)
        all_generated.extend(generated)

    print("\n%d image(s) written." % len(all_generated))


if __name__ == "__main__":
    main()
