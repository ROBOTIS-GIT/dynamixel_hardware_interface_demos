#!/usr/bin/env python3

import argparse
import csv
import html
import math
import os
import statistics
from dataclasses import dataclass
from typing import Dict, List


COLORS = [
    "#2563eb",
    "#dc2626",
    "#16a34a",
    "#9333ea",
    "#ea580c",
    "#0891b2",
    "#be123c",
    "#0f766e",
    "#4f46e5",
    "#ca8a04",
    "#db2777",
    "#475569",
]


@dataclass
class Sample:
    channel: int
    latency_us: float
    deadline_miss: bool
    comm_result: int
    dxl_error: int


@dataclass
class Series:
    label: str
    path: str
    samples: List[Sample]


def percentile(values, percent):
    if not values:
        return 0.0
    ordered = sorted(values)
    rank = (percent / 100.0) * (len(ordered) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return ordered[lo]
    weight = rank - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def fmt(value):
    return f"{value:.2f}"


def read_series(path, label):
    samples = []
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            samples.append(
                Sample(
                    channel=int(row["channel"]),
                    latency_us=float(row["latency_us"]),
                    deadline_miss=bool(int(row["deadline_miss"])),
                    comm_result=int(row["comm_result"]),
                    dxl_error=int(row["dxl_error"]),
                )
            )
    if not samples:
        raise RuntimeError(f"no samples found in {path}")
    return Series(label=label, path=path, samples=samples)


def summarize(series):
    latencies = [sample.latency_us for sample in series.samples]
    channels = sorted({sample.channel for sample in series.samples})
    return {
        "channels": ",".join(str(channel) for channel in channels),
        "samples": len(latencies),
        "avg": statistics.fmean(latencies),
        "p50": percentile(latencies, 50.0),
        "p95": percentile(latencies, 95.0),
        "p99": percentile(latencies, 99.0),
        "max": max(latencies),
        "miss": sum(1 for sample in series.samples if sample.deadline_miss),
        "comm_error": sum(1 for sample in series.samples if sample.comm_result != 0),
        "dxl_error": sum(1 for sample in series.samples if sample.dxl_error != 0),
    }


def svg_text(x, y, text, size=14, anchor="start", weight="400", fill="#111827"):
    safe = html.escape(str(text), quote=False)
    return (
        f'<text x="{x}" y="{y}" font-family="Arial, sans-serif" '
        f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" '
        f'fill="{fill}">{safe}</text>'
    )


def polyline(points):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def choose_x_max(summaries, explicit_limit):
    if explicit_limit > 0:
        return explicit_limit, False
    max_latency = max(summary["max"] for summary in summaries.values())
    global_p95 = max(summary["p95"] for summary in summaries.values())
    global_p99 = max(summary["p99"] for summary in summaries.values())
    focused_limit = max(global_p99 * 1.35, global_p95 * 2.0, 1.0)
    if max_latency > focused_limit * 1.15:
        return focused_limit, True
    return max(max_latency * 1.05, 1.0), False


def generate_svg(series_list, output, title, generated_at, x_limit_us):
    summaries: Dict[str, dict] = {series.label: summarize(series) for series in series_list}
    x_max, clipped = choose_x_max(summaries, x_limit_us)

    width = 1300
    table_row_h = 28
    table_h = 72 + table_row_h * len(series_list)
    height = 660 + table_row_h * len(series_list)
    margin_left = 86
    margin_right = 58
    margin_top = 112
    plot_h = 405
    plot_w = width - margin_left - margin_right
    plot_bottom = margin_top + plot_h
    table_top = plot_bottom + 74

    def x_of(value):
        return margin_left + (min(value, x_max) / x_max) * plot_w

    def y_cdf(percent):
        return margin_top + plot_h - (percent / 100.0) * plot_h

    subtitle = []
    if generated_at:
        subtitle.append(f"generated={generated_at}")
    subtitle.append(f"x-axis={'clipped ' if clipped else ''}limit {fmt(x_max)} us")

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        svg_text(margin_left, 44, title, 24, weight="700"),
        svg_text(margin_left, 72, "  ".join(subtitle), 14, fill="#374151"),
        f'<rect x="{margin_left}" y="{margin_top}" width="{plot_w}" height="{plot_h}" '
        'fill="#f9fafb" stroke="#d1d5db"/>',
    ]

    for i in range(6):
        value = x_max * i / 5.0
        x = x_of(value)
        svg.append(
            f'<line x1="{x:.2f}" y1="{margin_top}" x2="{x:.2f}" '
            f'y2="{plot_bottom}" stroke="#e5e7eb"/>'
        )
        svg.append(svg_text(x, plot_bottom + 28, f"{value:.0f}", 12, "middle", fill="#4b5563"))

    for percent in (0, 25, 50, 75, 100):
        y = y_cdf(percent)
        svg.append(
            f'<line x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + plot_w}" '
            f'y2="{y:.2f}" stroke="#e5e7eb"/>'
        )
        svg.append(svg_text(margin_left - 12, y + 4, str(percent), 12, "end", fill="#4b5563"))

    svg.append(svg_text(margin_left + plot_w / 2, plot_bottom + 55, "latency (us)", 14, "middle"))
    y_label = margin_top + plot_h / 2
    svg.append(
        f'<text x="28" y="{y_label}" font-family="Arial, sans-serif" font-size="14" '
        f'text-anchor="middle" fill="#111827" transform="rotate(-90 28,{y_label})">'
        "CDF percent</text>"
    )

    legend_x = margin_left + plot_w - 260
    legend_y = margin_top + 24
    for index, series in enumerate(series_list):
        color = COLORS[index % len(COLORS)]
        latencies = sorted(sample.latency_us for sample in series.samples)
        points = []
        if len(latencies) == 1:
            points.append((x_of(latencies[0]), y_cdf(100.0)))
        else:
            for sample_index, latency in enumerate(latencies):
                percent = 100.0 * sample_index / (len(latencies) - 1)
                points.append((x_of(latency), y_cdf(percent)))
        svg.append(
            f'<polyline points="{polyline(points)}" fill="none" '
            f'stroke="{color}" stroke-width="2.4" opacity="0.94"/>'
        )
        summary = summaries[series.label]
        p99_x = x_of(summary["p99"])
        svg.append(
            f'<line x1="{p99_x:.2f}" y1="{margin_top}" x2="{p99_x:.2f}" '
            f'y2="{plot_bottom}" stroke="{color}" stroke-width="1.2" '
            'stroke-dasharray="4 5" opacity="0.7"/>'
        )
        ly = legend_y + index * 25
        svg.append(
            f'<line x1="{legend_x}" y1="{ly - 5}" x2="{legend_x + 30}" '
            f'y2="{ly - 5}" stroke="{color}" stroke-width="3"/>'
        )
        svg.append(svg_text(legend_x + 40, ly, f"{series.label} p99={fmt(summary['p99'])} us", 13))

    svg.append(
        f'<rect x="{margin_left}" y="{table_top}" width="{plot_w}" height="{table_h}" '
        'fill="#ffffff" stroke="#d1d5db"/>'
    )

    columns = [
        ("label", margin_left + 16),
        ("ch", margin_left + 170),
        ("samples", margin_left + 260),
        ("avg", margin_left + 350),
        ("p50", margin_left + 440),
        ("p95", margin_left + 530),
        ("p99", margin_left + 620),
        ("max", margin_left + 710),
        ("miss", margin_left + 800),
        ("comm", margin_left + 880),
        ("dxl", margin_left + 960),
    ]
    svg.append(svg_text(margin_left + 16, table_top + 28, "Summary", 16, weight="700"))
    header_y = table_top + 55
    for label, x in columns:
        svg.append(svg_text(x, header_y, label, 12, weight="700", fill="#374151"))
    svg.append(
        f'<line x1="{margin_left}" y1="{header_y + 10}" x2="{margin_left + plot_w}" '
        f'y2="{header_y + 10}" stroke="#e5e7eb"/>'
    )

    for index, series in enumerate(series_list):
        color = COLORS[index % len(COLORS)]
        summary = summaries[series.label]
        y = header_y + 32 + index * table_row_h
        if index % 2:
            svg.append(
                f'<rect x="{margin_left + 1}" y="{y - 17}" width="{plot_w - 2}" '
                f'height="{table_row_h}" fill="#f9fafb"/>'
            )
        row = [
            series.label,
            summary["channels"],
            summary["samples"],
            fmt(summary["avg"]),
            fmt(summary["p50"]),
            fmt(summary["p95"]),
            fmt(summary["p99"]),
            fmt(summary["max"]),
            summary["miss"],
            summary["comm_error"],
            summary["dxl_error"],
        ]
        svg.append(
            f'<circle cx="{margin_left + 8}" cy="{y - 4}" r="4" fill="{color}"/>'
        )
        for value, (_, x) in zip(row, columns):
            svg.append(svg_text(x, y, value, 12, fill="#111827"))

    svg.append("</svg>\n")
    with open(output, "w") as svg_file:
        svg_file.write("\n".join(svg))


def parse_labels(csv_paths, label_arg):
    if label_arg:
        labels = [label.strip() for label in label_arg.split(",") if label.strip()]
        if len(labels) != len(csv_paths):
            raise RuntimeError("--labels count must match CSV count")
        return labels
    return [os.path.splitext(os.path.basename(path))[0] for path in csv_paths]


def main():
    parser = argparse.ArgumentParser(description="Compare E2D2 latency CSV files in one SVG.")
    parser.add_argument("csv", nargs="+", help="CSV files produced by e2d2_read_latency_test --csv")
    parser.add_argument("-o", "--output", default="e2d2_latency_compare.svg", help="Output SVG path")
    parser.add_argument("--labels", default="", help="Comma-separated labels matching CSV order")
    parser.add_argument("--title", default="E2D2 Read Latency TCP vs UDP")
    parser.add_argument("--generated-at", default="", help="Timestamp to show in the SVG")
    parser.add_argument("--x-limit-us", type=float, default=0.0, help="Manual latency axis limit in us")
    args = parser.parse_args()

    labels = parse_labels(args.csv, args.labels)
    series_list = [read_series(path, label) for path, label in zip(args.csv, labels)]
    generate_svg(series_list, args.output, args.title, args.generated_at, args.x_limit_us)
    print(args.output)


if __name__ == "__main__":
    main()
