#!/usr/bin/env python3

import argparse
import csv
import math
import statistics
from collections import defaultdict


COLORS = [
    "#2563eb",
    "#16a34a",
    "#dc2626",
    "#9333ea",
    "#ea580c",
    "#0891b2",
]


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


def read_csv(path):
    by_channel = defaultdict(list)
    misses_by_channel = defaultdict(int)
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            channel = int(row["channel"])
            latency = float(row["latency_us"])
            by_channel[channel].append(latency)
            if int(row["deadline_miss"]):
                misses_by_channel[channel] += 1
    return dict(sorted(by_channel.items())), misses_by_channel


def make_histogram(values, bins, x_max):
    counts = [0] * bins
    if not values or x_max <= 0.0:
        return counts
    width = x_max / bins
    for value in values:
        index = min(bins - 1, max(0, int(value / width)))
        counts[index] += 1
    return counts


def polyline(points):
    return " ".join(f"{x:.2f},{y:.2f}" for x, y in points)


def svg_text(x, y, text, size=14, anchor="start", weight="400", fill="#111827"):
    return (
        f'<text x="{x}" y="{y}" font-family="Arial, sans-serif" '
        f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" '
        f'fill="{fill}">{text}</text>'
    )


def generate_svg(by_channel, misses_by_channel, output, bins, title, generated_at):
    aggregate = [latency for values in by_channel.values() for latency in values]
    if not aggregate:
        raise RuntimeError("no latency samples found")

    width = 1200
    height = 760
    margin_left = 86
    margin_right = 56
    margin_top = 92
    margin_bottom = 92
    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    p95 = percentile(aggregate, 95.0)
    p99 = percentile(aggregate, 99.0)
    max_latency = max(aggregate)
    avg = statistics.fmean(aggregate)
    x_max = max(max_latency * 1.05, p99 * 1.15, 1.0)

    hist = make_histogram(aggregate, bins, x_max)
    y_max_count = max(hist) if hist else 1

    def x_of(value):
        return margin_left + (value / x_max) * plot_w

    def y_hist(count):
        return margin_top + plot_h - (count / y_max_count) * plot_h

    def y_cdf(percent):
        return margin_top + plot_h - (percent / 100.0) * plot_h

    summary = (
        f"samples={len(aggregate)}  avg={fmt(avg)} us  "
        f"p95={fmt(p95)} us  p99={fmt(p99)} us  max={fmt(max_latency)} us"
    )
    if generated_at:
        summary += f"  generated={generated_at}"

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        svg_text(margin_left, 42, title, 24, weight="700"),
        svg_text(
            margin_left,
            68,
            summary,
            14,
            fill="#374151",
        ),
        f'<rect x="{margin_left}" y="{margin_top}" width="{plot_w}" height="{plot_h}" '
        'fill="#f9fafb" stroke="#d1d5db"/>',
    ]

    for i in range(0, 6):
        value = x_max * i / 5.0
        x = x_of(value)
        svg.append(
            f'<line x1="{x:.2f}" y1="{margin_top}" x2="{x:.2f}" '
            f'y2="{margin_top + plot_h}" stroke="#e5e7eb"/>'
        )
        svg.append(svg_text(x, margin_top + plot_h + 28, f"{value:.0f}", 12, "middle", fill="#4b5563"))

    for percent in (0, 25, 50, 75, 100):
        y = y_cdf(percent)
        svg.append(
            f'<line x1="{margin_left}" y1="{y:.2f}" x2="{margin_left + plot_w}" '
            f'y2="{y:.2f}" stroke="#e5e7eb"/>'
        )
        svg.append(svg_text(margin_left - 12, y + 4, f"{percent}", 12, "end", fill="#4b5563"))

    svg.append(svg_text(margin_left + plot_w / 2, height - 26, "latency (us)", 14, "middle", fill="#111827"))
    y_label = margin_top + plot_h / 2
    svg.append(
        f'<text x="28" y="{y_label}" font-family="Arial, sans-serif" font-size="14" '
        f'text-anchor="middle" fill="#111827" transform="rotate(-90 28,{y_label})">'
        "CDF percent</text>"
    )

    bin_w = plot_w / bins
    for index, count in enumerate(hist):
        x = margin_left + index * bin_w
        y = y_hist(count)
        bar_h = margin_top + plot_h - y
        svg.append(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{max(1.0, bin_w - 1):.2f}" '
            f'height="{bar_h:.2f}" fill="#9ca3af" opacity="0.35"/>'
        )

    legend_x = margin_left + plot_w - 250
    legend_y = margin_top + 24
    for idx, (channel, values) in enumerate(by_channel.items()):
        color = COLORS[idx % len(COLORS)]
        ordered = sorted(values)
        points = []
        if len(ordered) == 1:
            points.append((x_of(ordered[0]), y_cdf(100.0)))
        else:
            for sample_index, value in enumerate(ordered):
                percent = 100.0 * sample_index / (len(ordered) - 1)
                points.append((x_of(value), y_cdf(percent)))
        svg.append(
            f'<polyline points="{polyline(points)}" fill="none" '
            f'stroke="{color}" stroke-width="2" opacity="0.9"/>'
        )
        ly = legend_y + idx * 22
        svg.append(
            f'<line x1="{legend_x}" y1="{ly - 5}" x2="{legend_x + 28}" '
            f'y2="{ly - 5}" stroke="{color}" stroke-width="3"/>'
        )
        svg.append(
            svg_text(
                legend_x + 36,
                ly,
                (
                    f"ch{channel}: p99={fmt(percentile(values, 99.0))} us, "
                    f"miss={misses_by_channel[channel]}"
                ),
                13,
                fill="#111827",
            )
        )

    markers = [(p95, "p95", "#0369a1"), (p99, "p99", "#7c3aed"), (max_latency, "max", "#dc2626")]
    for value, label, color in markers:
        x = x_of(value)
        svg.append(
            f'<line x1="{x:.2f}" y1="{margin_top}" x2="{x:.2f}" '
            f'y2="{margin_top + plot_h}" stroke="{color}" stroke-width="1.5" '
            'stroke-dasharray="5 5"/>'
        )
        svg.append(svg_text(x + 5, margin_top + 16, f"{label} {fmt(value)} us", 12, fill=color))

    svg.append("</svg>\n")
    with open(output, "w") as svg_file:
        svg_file.write("\n".join(svg))


def main():
    parser = argparse.ArgumentParser(description="Plot E2D2 latency CSV as an SVG histogram/CDF.")
    parser.add_argument("csv", help="CSV produced by e2d2_read_latency_test --csv")
    parser.add_argument("-o", "--output", default="e2d2_latency.svg", help="Output SVG path")
    parser.add_argument("--bins", type=int, default=80, help="Histogram bins")
    parser.add_argument("--title", default="E2D2 Read Latency Distribution")
    parser.add_argument("--generated-at", default="", help="Timestamp to show in the SVG")
    args = parser.parse_args()

    by_channel, misses_by_channel = read_csv(args.csv)
    generate_svg(by_channel, misses_by_channel, args.output, args.bins, args.title, args.generated_at)
    print(args.output)


if __name__ == "__main__":
    main()
