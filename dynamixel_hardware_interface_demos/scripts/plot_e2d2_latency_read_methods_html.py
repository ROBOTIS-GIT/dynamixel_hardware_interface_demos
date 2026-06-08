#!/usr/bin/env python3

import argparse
import csv
import html
import json
import math
import os
import statistics
from collections import Counter, defaultdict


DEFAULT_READ_METHODS = ["basic", "sync", "bulk", "fast-sync", "fast-bulk"]


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


def read_summary(path, label, transport, mode, read_method):
    values = []
    channels = set()
    deadline_misses = 0
    comm_errors = 0
    dxl_errors = 0
    comm_counts = Counter()
    dxl_counts = Counter()
    per_channel = defaultdict(list)

    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            channel = int(row["channel"])
            latency = float(row["latency_us"])
            comm_result = int(row["comm_result"])
            dxl_error = int(row["dxl_error"])

            channels.add(channel)
            values.append(latency)
            per_channel[channel].append(latency)
            if int(row["deadline_miss"]):
                deadline_misses += 1
            if comm_result != 0:
                comm_errors += 1
            if dxl_error != 0:
                dxl_errors += 1
            comm_counts[comm_result] += 1
            dxl_counts[dxl_error] += 1

    if not values:
        return None

    per_channel_p99 = {
        f"ch{channel}": percentile(channel_values, 99.0)
        for channel, channel_values in sorted(per_channel.items())
    }
    q1 = percentile(values, 25.0)
    q3 = percentile(values, 75.0)
    iqr = q3 - q1
    lower_fence = q1 - 1.5 * iqr
    upper_fence = q3 + 1.5 * iqr
    sorted_values = sorted(values)
    outliers = [
        value for value in sorted_values
        if value < lower_fence or value > upper_fence
    ]
    inliers = [
        value for value in sorted_values
        if lower_fence <= value <= upper_fence
    ]
    if not inliers:
        inliers = sorted_values

    return {
        "label": label,
        "transport": transport.upper(),
        "mode": mode,
        "read": read_method,
        "channels": ",".join(str(channel) for channel in sorted(channels)),
        "samples": len(values),
        "avg": statistics.fmean(values),
        "q1": q1,
        "p50": percentile(values, 50.0),
        "q3": q3,
        "iqr": iqr,
        "lower_fence": lower_fence,
        "upper_fence": upper_fence,
        "whisker_low": min(inliers),
        "whisker_high": max(inliers),
        "p95": percentile(values, 95.0),
        "p99": percentile(values, 99.0),
        "max": max(values),
        "outlier_count": len(outliers),
        "outliers": outliers,
        "miss": deadline_misses,
        "comm": comm_errors,
        "dxl": dxl_errors,
        "comm_counts": dict(sorted(comm_counts.items())),
        "dxl_counts": dict(sorted(dxl_counts.items())),
        "per_channel_p99": per_channel_p99,
        "path": path,
    }


def collect_rows(out_root, channels_tag, duration, read_methods):
    rows = []
    for transport in ("tcp", "udp"):
        for read_method in read_methods:
            path = os.path.join(
                out_root,
                transport,
                f"parallel_{read_method}_{channels_tag}_{transport}_{duration}s.csv",
            )
            if os.path.exists(path):
                row = read_summary(
                    path,
                    f"{transport.upper()} parallel {read_method}",
                    transport,
                    "parallel",
                    read_method,
                )
                if row:
                    rows.append(row)

    for transport in ("tcp", "udp"):
        for mode, label_mode in (("epoll", "epoll"), ("epoll-sdk", "epoll-sdk")):
            path = os.path.join(
                out_root,
                transport,
                f"{mode}_basic_{channels_tag}_{transport}_{duration}s.csv",
            )
            if os.path.exists(path):
                row = read_summary(
                    path,
                    f"{transport.upper()} {label_mode} basic",
                    transport,
                    mode,
                    "basic",
                )
                if row:
                    rows.append(row)

    mode_order = {"parallel": 0, "epoll": 1, "epoll-sdk": 2}
    read_order = {name: idx for idx, name in enumerate(read_methods)}
    rows.sort(
        key=lambda item: (
            item["transport"],
            mode_order.get(item["mode"], 9),
            read_order.get(item["read"], 9),
        )
    )
    return rows


def fmt(value):
    return f"{value:.2f}"


def make_boxplot_svg(rows, title):
    if not rows:
        return ""

    ordered_rows = sorted(rows, key=lambda row: row["p99"])
    x_max = max(
        max(row["whisker_high"], row["p99"], max(row["outliers"]) if row["outliers"] else row["max"])
        for row in ordered_rows
    )
    x_min = min(
        min(row["whisker_low"], row["q1"], min(row["outliers"]) if row["outliers"] else row["whisker_low"])
        for row in ordered_rows
    )
    span = max(1.0, x_max - x_min)
    x_min = max(0.0, x_min - span * 0.04)
    x_max = x_max + span * 0.06

    width = 1280
    row_h = 44
    margin_left = 230
    margin_right = 62
    margin_top = 70
    margin_bottom = 58
    plot_w = width - margin_left - margin_right
    height = margin_top + margin_bottom + row_h * len(ordered_rows)
    box_h = 18

    def x_of(value):
        return margin_left + ((value - x_min) / (x_max - x_min)) * plot_w

    def esc(value):
        return html.escape(str(value), quote=True)

    items = [
        f'<svg class="boxplot-svg" xmlns="http://www.w3.org/2000/svg" '
        f'viewBox="0 0 {width} {height}" role="img" '
        f'aria-label="{esc(title)} boxplot">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{margin_left}" y="30" font-family="Arial, sans-serif" '
        f'font-size="20" font-weight="700" fill="#111827">Latency boxplot with Tukey outliers</text>',
        f'<text x="{margin_left}" y="52" font-family="Arial, sans-serif" '
        f'font-size="13" fill="#4b5563">Box = Q1..Q3, center line = median, whiskers = non-outlier range, dots = outliers outside 1.5*IQR fences</text>',
    ]

    for tick in range(6):
        value = x_min + (x_max - x_min) * tick / 5.0
        x = x_of(value)
        items.append(
            f'<line x1="{x:.2f}" y1="{margin_top - 12}" x2="{x:.2f}" '
            f'y2="{height - margin_bottom + 10}" stroke="#e5e7eb"/>'
        )
        items.append(
            f'<text x="{x:.2f}" y="{height - 22}" font-family="Arial, sans-serif" '
            f'font-size="12" text-anchor="middle" fill="#4b5563">{value:.0f}</text>'
        )

    items.append(
        f'<text x="{margin_left + plot_w / 2:.2f}" y="{height - 4}" '
        f'font-family="Arial, sans-serif" font-size="13" text-anchor="middle" '
        f'fill="#111827">latency (us)</text>'
    )

    for index, row in enumerate(ordered_rows):
        y = margin_top + index * row_h + row_h / 2.0
        color = "#2563eb" if row["transport"] == "TCP" else "#16a34a"
        soft = "#dbeafe" if row["transport"] == "TCP" else "#dcfce7"
        label = esc(row["label"])
        x_w0 = x_of(row["whisker_low"])
        x_w1 = x_of(row["whisker_high"])
        x_q1 = x_of(row["q1"])
        x_q3 = x_of(row["q3"])
        x_med = x_of(row["p50"])

        if index % 2:
            items.append(
                f'<rect x="0" y="{y - row_h / 2:.2f}" width="{width}" '
                f'height="{row_h}" fill="#f8fafc"/>'
            )
        items.append(
            f'<text x="20" y="{y + 4:.2f}" font-family="Arial, sans-serif" '
            f'font-size="12" fill="#111827">{label}</text>'
        )
        items.append(
            f'<line x1="{x_w0:.2f}" y1="{y:.2f}" x2="{x_w1:.2f}" y2="{y:.2f}" '
            f'stroke="{color}" stroke-width="2"/>'
        )
        items.append(
            f'<line x1="{x_w0:.2f}" y1="{y - 8:.2f}" x2="{x_w0:.2f}" '
            f'y2="{y + 8:.2f}" stroke="{color}" stroke-width="2"/>'
        )
        items.append(
            f'<line x1="{x_w1:.2f}" y1="{y - 8:.2f}" x2="{x_w1:.2f}" '
            f'y2="{y + 8:.2f}" stroke="{color}" stroke-width="2"/>'
        )
        items.append(
            f'<rect x="{x_q1:.2f}" y="{y - box_h / 2:.2f}" '
            f'width="{max(1.0, x_q3 - x_q1):.2f}" height="{box_h}" '
            f'fill="{soft}" stroke="{color}" stroke-width="2"/>'
        )
        items.append(
            f'<line x1="{x_med:.2f}" y1="{y - box_h / 2:.2f}" '
            f'x2="{x_med:.2f}" y2="{y + box_h / 2:.2f}" '
            f'stroke="#111827" stroke-width="2"/>'
        )

        for outlier_index, value in enumerate(row["outliers"]):
            jitter = ((outlier_index % 9) - 4) * 1.65
            x = x_of(value)
            items.append(
                f'<circle cx="{x:.2f}" cy="{y + jitter:.2f}" r="2.2" '
                f'fill="#dc2626" opacity="0.58">'
                f'<title>{label} outlier {value:.2f} us</title></circle>'
            )

        items.append(
            f'<text x="{width - 16}" y="{y + 4:.2f}" font-family="Arial, sans-serif" '
            f'font-size="12" text-anchor="end" fill="#4b5563">'
            f'p99 {row["p99"]:.2f} us, outliers {row["outlier_count"]}</text>'
        )

    items.append("</svg>")
    return "\n".join(items)


def make_html(rows, title, svg_path, channels, length, duration):
    data_json = json.dumps(rows)
    svg_html = ""
    if svg_path:
        safe_svg = html.escape(svg_path)
        svg_html = f"""
        <details class="plot-panel" id="cdfPanel">
          <summary>CDF plot</summary>
          <div class="plot-help">Drag inside the plot to zoom into a region. Use reset to return to the full image.</div>
          <div class="plot-toolbar">
            <button type="button" id="cdfReset">Reset zoom</button>
          </div>
          <div class="plot-stage cdf-stage" id="cdfStage">
            <img id="cdfImage" src="{safe_svg}" alt="{html.escape(title)} SVG">
            <div class="zoom-selection" id="cdfSelection" hidden></div>
          </div>
        </details>
        """

    best = min(rows, key=lambda row: row["p99"]) if rows else None
    worst = max(rows, key=lambda row: row["p99"]) if rows else None
    cards = ""
    if best and worst:
        cards = f"""
        <section class="cards">
          <div class="card"><span>Best p99</span><strong>{html.escape(best['label'])}</strong><b>{fmt(best['p99'])} us</b></div>
          <div class="card"><span>Highest p99</span><strong>{html.escape(worst['label'])}</strong><b>{fmt(worst['p99'])} us</b></div>
          <div class="card"><span>Deadline misses</span><strong>{sum(row['miss'] for row in rows)}</strong><b>total</b></div>
          <div class="card"><span>Comm errors</span><strong>{sum(row['comm'] for row in rows)}</strong><b>total</b></div>
        </section>
        """

    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{html.escape(title)}</title>
  <style>
    :root {{
      color-scheme: light;
      --text: #111827;
      --muted: #4b5563;
      --line: #d1d5db;
      --soft: #f8fafc;
      --blue: #2563eb;
      --red: #dc2626;
      --green: #16a34a;
      --amber: #d97706;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: Arial, sans-serif;
      color: var(--text);
      background: #ffffff;
    }}
    header {{
      padding: 24px 28px 14px;
      border-bottom: 1px solid var(--line);
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 24px;
      line-height: 1.25;
      letter-spacing: 0;
    }}
    .meta {{
      margin: 0;
      color: var(--muted);
      font-size: 14px;
    }}
    main {{
      padding: 18px 28px 32px;
      display: grid;
      gap: 18px;
    }}
    .cards {{
      display: grid;
      grid-template-columns: repeat(4, minmax(150px, 1fr));
      gap: 10px;
    }}
    .card {{
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 12px;
      min-height: 92px;
      background: var(--soft);
    }}
    .card span {{
      display: block;
      color: var(--muted);
      font-size: 12px;
      margin-bottom: 8px;
    }}
    .card strong {{
      display: block;
      font-size: 14px;
      min-height: 34px;
    }}
    .card b {{
      display: block;
      margin-top: 6px;
      font-size: 18px;
    }}
    .chart {{
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: auto;
      background: #fff;
    }}
    .boxplot {{
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: auto;
      background: #fff;
    }}
    .boxplot-svg {{
      display: block;
      width: 100%;
      min-width: 1040px;
      height: auto;
    }}
    .chart img {{
      display: block;
      width: 100%;
      min-width: 980px;
      height: auto;
    }}
    .plot-panel {{
      border: 1px solid var(--line);
      border-radius: 8px;
      background: #fff;
      overflow: hidden;
    }}
    .plot-panel summary {{
      cursor: pointer;
      padding: 12px 14px;
      background: #f1f5f9;
      font-weight: 700;
      user-select: none;
    }}
    .plot-help {{
      color: var(--muted);
      font-size: 13px;
      padding: 10px 14px 0;
    }}
    .plot-toolbar {{
      display: flex;
      gap: 8px;
      align-items: center;
      flex-wrap: wrap;
      padding: 10px 14px;
    }}
    .plot-toolbar button {{
      border: 1px solid var(--line);
      border-radius: 6px;
      background: #ffffff;
      color: var(--text);
      padding: 7px 10px;
      font-size: 13px;
      cursor: pointer;
    }}
    .plot-stage {{
      position: relative;
      overflow: hidden;
      border-top: 1px solid #e5e7eb;
      background: #ffffff;
      touch-action: none;
    }}
    .boxplot-stage {{
      overflow-x: auto;
    }}
    .cdf-stage {{
      height: min(62vw, 680px);
      min-height: 360px;
    }}
    .cdf-stage img {{
      position: absolute;
      inset: 0;
      width: 100%;
      height: 100%;
      object-fit: contain;
      transform-origin: 0 0;
      transition: transform 120ms ease;
      user-select: none;
      -webkit-user-drag: none;
    }}
    .zoom-selection {{
      position: absolute;
      border: 1px solid var(--blue);
      background: rgba(37, 99, 235, 0.16);
      pointer-events: none;
      z-index: 4;
    }}
    .toolbar {{
      display: flex;
      gap: 12px;
      align-items: center;
      flex-wrap: wrap;
    }}
    .toolbar input {{
      width: min(420px, 100%);
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 9px 11px;
      font-size: 14px;
    }}
    .table-wrap {{
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: auto;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      min-width: 1160px;
      font-size: 13px;
    }}
    th, td {{
      padding: 9px 10px;
      border-bottom: 1px solid #e5e7eb;
      text-align: left;
      white-space: nowrap;
    }}
    th {{
      position: sticky;
      top: 0;
      background: #f1f5f9;
      cursor: pointer;
      user-select: none;
      z-index: 1;
    }}
    th[data-dir="asc"]::after {{ content: "  up"; color: var(--blue); }}
    th[data-dir="desc"]::after {{ content: "  down"; color: var(--blue); }}
    td.num, th.num {{
      text-align: right;
      font-variant-numeric: tabular-nums;
    }}
    tbody tr:hover {{
      background: #f8fafc;
    }}
    .pill {{
      display: inline-block;
      min-width: 58px;
      padding: 3px 7px;
      border-radius: 999px;
      text-align: center;
      font-size: 12px;
      color: #fff;
    }}
    .tcp {{ background: var(--blue); }}
    .udp {{ background: var(--green); }}
    .warn {{ color: var(--amber); font-weight: 700; }}
    .bad {{ color: var(--red); font-weight: 700; }}
    .details {{
      color: var(--muted);
      font-size: 12px;
      max-width: 320px;
      overflow: hidden;
      text-overflow: ellipsis;
    }}
    @media (max-width: 820px) {{
      header {{ padding: 18px 16px 12px; }}
      main {{ padding: 14px 16px 24px; }}
      .cards {{ grid-template-columns: repeat(2, minmax(140px, 1fr)); }}
      h1 {{ font-size: 20px; }}
    }}
    @media (max-width: 520px) {{
      .cards {{ grid-template-columns: 1fr; }}
    }}
  </style>
</head>
<body>
  <header>
    <h1>{html.escape(title)}</h1>
    <p class="meta">Channels {html.escape(channels)}. Read length {html.escape(str(length))} bytes. Duration {html.escape(str(duration))} s. Click a table header to toggle ascending or descending order.</p>
  </header>
  <main>
    {cards}
    <details class="plot-panel" id="boxplotPanel">
      <summary>Boxplot with outlier dots</summary>
      <div class="plot-help">The boxplot follows the current table sort and filter. Drag horizontally inside the boxplot to zoom the latency axis.</div>
      <div class="plot-toolbar">
        <button type="button" id="boxplotReset">Reset zoom</button>
      </div>
      <div class="plot-stage boxplot-stage" id="boxplotStage">
        <svg class="boxplot-svg" id="boxplotSvg" xmlns="http://www.w3.org/2000/svg"></svg>
        <div class="zoom-selection" id="boxplotSelection" hidden></div>
      </div>
    </details>
    {svg_html}
    <section class="toolbar">
      <input id="filter" type="search" placeholder="Filter rows">
    </section>
    <section class="table-wrap">
      <table id="summary">
        <thead>
          <tr>
            <th data-key="label">Case</th>
            <th data-key="transport">Transport</th>
            <th data-key="mode">Mode</th>
            <th data-key="read">Read</th>
            <th class="num" data-key="samples">Samples</th>
            <th class="num" data-key="avg">Avg us</th>
            <th class="num" data-key="p50">P50</th>
            <th class="num" data-key="p95">P95</th>
            <th class="num" data-key="p99">P99</th>
            <th class="num" data-key="max">Max</th>
            <th class="num" data-key="outlier_count">Outliers</th>
            <th class="num" data-key="miss">Miss</th>
            <th class="num" data-key="comm">Comm</th>
            <th class="num" data-key="dxl">DXL</th>
            <th data-key="per_channel_text">Per-channel p99</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>
  </main>
  <script>
    const rows = {data_json};
    let sortKey = "p99";
    let sortDir = "asc";
    let filterText = "";
    let boxZoom = null;
    let boxDomain = null;
    let currentVisibleRows = [];

    function formatNumber(value) {{
      if (typeof value !== "number") return value;
      return value.toLocaleString(undefined, {{ minimumFractionDigits: 2, maximumFractionDigits: 2 }});
    }}

    function rowText(row) {{
      return [
        row.label, row.transport, row.mode, row.read, row.channels,
        JSON.stringify(row.comm_counts), JSON.stringify(row.dxl_counts)
      ].join(" ").toLowerCase();
    }}

    function perChannelText(row) {{
      return Object.entries(row.per_channel_p99)
        .map(([channel, value]) => `${{channel}}:${{value.toFixed(1)}}`)
        .join(", ");
    }}

    function valueFor(row, key) {{
      if (key === "per_channel_text") return perChannelText(row);
      return row[key];
    }}

    function svgNode(name, attrs, text) {{
      const node = document.createElementNS("http://www.w3.org/2000/svg", name);
      for (const [key, value] of Object.entries(attrs || {{}})) {{
        node.setAttribute(key, value);
      }}
      if (text !== undefined) node.textContent = text;
      return node;
    }}

    function maxOutlier(row) {{
      return row.outliers.length ? Math.max(...row.outliers) : row.max;
    }}

    function minOutlier(row) {{
      return row.outliers.length ? Math.min(...row.outliers) : row.whisker_low;
    }}

    function computeBoxDomain(plotRows) {{
      if (!plotRows.length) return {{ min: 0, max: 1 }};
      let minValue = Infinity;
      let maxValue = -Infinity;
      for (const row of plotRows) {{
        minValue = Math.min(minValue, row.whisker_low, row.q1, minOutlier(row));
        maxValue = Math.max(maxValue, row.whisker_high, row.p99, maxOutlier(row));
      }}
      const span = Math.max(1, maxValue - minValue);
      return {{
        min: Math.max(0, minValue - span * 0.04),
        max: maxValue + span * 0.06,
      }};
    }}

    const boxSpec = {{
      width: 1280,
      rowH: 44,
      marginLeft: 230,
      marginRight: 62,
      marginTop: 70,
      marginBottom: 58,
      boxH: 18,
    }};

    function renderBoxplot(plotRows) {{
      const svg = document.querySelector("#boxplotSvg");
      if (!svg) return;
      svg.textContent = "";

      if (!plotRows.length) {{
        svg.setAttribute("viewBox", "0 0 1280 160");
        svg.appendChild(svgNode("text", {{
          x: 30, y: 78,
          "font-family": "Arial, sans-serif",
          "font-size": 16,
          fill: "#4b5563",
        }}, "No rows match the current filter."));
        return;
      }}

      const spec = boxSpec;
      const height = spec.marginTop + spec.marginBottom + spec.rowH * plotRows.length;
      const plotW = spec.width - spec.marginLeft - spec.marginRight;
      const domain = boxZoom || computeBoxDomain(plotRows);
      boxDomain = domain;
      const span = Math.max(1, domain.max - domain.min);

      function xOf(value) {{
        return spec.marginLeft + ((value - domain.min) / span) * plotW;
      }}

      svg.setAttribute("viewBox", "0 0 " + spec.width + " " + height);
      svg.appendChild(svgNode("rect", {{ width: "100%", height: "100%", fill: "#ffffff" }}));
      svg.appendChild(svgNode("text", {{
        x: spec.marginLeft, y: 30,
        "font-family": "Arial, sans-serif",
        "font-size": 20,
        "font-weight": 700,
        fill: "#111827",
      }}, "Latency boxplot with Tukey outliers"));
      svg.appendChild(svgNode("text", {{
        x: spec.marginLeft, y: 52,
        "font-family": "Arial, sans-serif",
        "font-size": 13,
        fill: "#4b5563",
      }}, "Box = Q1..Q3, center line = median, whiskers = non-outlier range, dots = outliers outside 1.5*IQR fences"));

      for (let tick = 0; tick <= 5; tick += 1) {{
        const value = domain.min + (domain.max - domain.min) * tick / 5;
        const x = xOf(value);
        svg.appendChild(svgNode("line", {{
          x1: x.toFixed(2), y1: spec.marginTop - 12,
          x2: x.toFixed(2), y2: height - spec.marginBottom + 10,
          stroke: "#e5e7eb",
        }}));
        svg.appendChild(svgNode("text", {{
          x: x.toFixed(2), y: height - 22,
          "font-family": "Arial, sans-serif",
          "font-size": 12,
          "text-anchor": "middle",
          fill: "#4b5563",
        }}, value.toFixed(0)));
      }}

      svg.appendChild(svgNode("text", {{
        x: (spec.marginLeft + plotW / 2).toFixed(2), y: height - 4,
        "font-family": "Arial, sans-serif",
        "font-size": 13,
        "text-anchor": "middle",
        fill: "#111827",
      }}, "latency (us)"));

      plotRows.forEach((row, index) => {{
        const y = spec.marginTop + index * spec.rowH + spec.rowH / 2;
        const color = row.transport === "TCP" ? "#2563eb" : "#16a34a";
        const soft = row.transport === "TCP" ? "#dbeafe" : "#dcfce7";
        const xW0 = xOf(row.whisker_low);
        const xW1 = xOf(row.whisker_high);
        const xQ1 = xOf(row.q1);
        const xQ3 = xOf(row.q3);
        const xMed = xOf(row.p50);

        if (index % 2) {{
          svg.appendChild(svgNode("rect", {{
            x: 0, y: (y - spec.rowH / 2).toFixed(2),
            width: spec.width, height: spec.rowH,
            fill: "#f8fafc",
          }}));
        }}

        svg.appendChild(svgNode("text", {{
          x: 20, y: (y + 4).toFixed(2),
          "font-family": "Arial, sans-serif",
          "font-size": 12,
          fill: "#111827",
        }}, row.label));
        svg.appendChild(svgNode("line", {{
          x1: xW0.toFixed(2), y1: y.toFixed(2),
          x2: xW1.toFixed(2), y2: y.toFixed(2),
          stroke: color, "stroke-width": 2,
        }}));
        svg.appendChild(svgNode("line", {{
          x1: xW0.toFixed(2), y1: (y - 8).toFixed(2),
          x2: xW0.toFixed(2), y2: (y + 8).toFixed(2),
          stroke: color, "stroke-width": 2,
        }}));
        svg.appendChild(svgNode("line", {{
          x1: xW1.toFixed(2), y1: (y - 8).toFixed(2),
          x2: xW1.toFixed(2), y2: (y + 8).toFixed(2),
          stroke: color, "stroke-width": 2,
        }}));
        svg.appendChild(svgNode("rect", {{
          x: xQ1.toFixed(2), y: (y - spec.boxH / 2).toFixed(2),
          width: Math.max(1, xQ3 - xQ1).toFixed(2), height: spec.boxH,
          fill: soft, stroke: color, "stroke-width": 2,
        }}));
        svg.appendChild(svgNode("line", {{
          x1: xMed.toFixed(2), y1: (y - spec.boxH / 2).toFixed(2),
          x2: xMed.toFixed(2), y2: (y + spec.boxH / 2).toFixed(2),
          stroke: "#111827", "stroke-width": 2,
        }}));

        row.outliers.forEach((value, outlierIndex) => {{
          const x = xOf(value);
          if (x < spec.marginLeft - 4 || x > spec.marginLeft + plotW + 4) return;
          const jitter = ((outlierIndex % 9) - 4) * 1.65;
          const dot = svgNode("circle", {{
            cx: x.toFixed(2), cy: (y + jitter).toFixed(2),
            r: 2.2,
            fill: "#dc2626",
            opacity: 0.58,
          }});
          dot.appendChild(svgNode("title", {{}}, row.label + " outlier " + value.toFixed(2) + " us"));
          svg.appendChild(dot);
        }});

        svg.appendChild(svgNode("text", {{
          x: spec.width - 16, y: (y + 4).toFixed(2),
          "font-family": "Arial, sans-serif",
          "font-size": 12,
          "text-anchor": "end",
          fill: "#4b5563",
        }}, "p99 " + row.p99.toFixed(2) + " us, outliers " + row.outlier_count));
      }});
    }}

    function setSelectionBox(selection, stage, start, end) {{
      const left = Math.min(start.x, end.x);
      const top = Math.min(start.y, end.y);
      const width = Math.abs(end.x - start.x);
      const height = Math.abs(end.y - start.y);
      selection.hidden = false;
      selection.style.left = (left + stage.scrollLeft) + "px";
      selection.style.top = (top + stage.scrollTop) + "px";
      selection.style.width = width + "px";
      selection.style.height = height + "px";
    }}

    function setupBoxplotDrag() {{
      const stage = document.querySelector("#boxplotStage");
      const selection = document.querySelector("#boxplotSelection");
      const svg = document.querySelector("#boxplotSvg");
      const reset = document.querySelector("#boxplotReset");
      if (!stage || !selection || !svg || !reset) return;

      let drag = null;
      stage.addEventListener("pointerdown", event => {{
        if (event.button !== 0) return;
        const rect = stage.getBoundingClientRect();
        drag = {{
          id: event.pointerId,
          start: {{ x: event.clientX - rect.left, y: event.clientY - rect.top }},
        }};
        stage.setPointerCapture(event.pointerId);
        selection.hidden = true;
      }});
      stage.addEventListener("pointermove", event => {{
        if (!drag) return;
        const rect = stage.getBoundingClientRect();
        setSelectionBox(selection, stage, drag.start, {{
          x: event.clientX - rect.left,
          y: event.clientY - rect.top,
        }});
      }});
      stage.addEventListener("pointerup", event => {{
        if (!drag) return;
        const rect = stage.getBoundingClientRect();
        const end = {{ x: event.clientX - rect.left, y: event.clientY - rect.top }};
        const width = Math.abs(end.x - drag.start.x);
        selection.hidden = true;
        stage.releasePointerCapture(drag.id);

        if (width > 12 && boxDomain) {{
          const svgRect = svg.getBoundingClientRect();
          const viewBox = svg.viewBox.baseVal;
          const toSvgX = clientX => (clientX - svgRect.left) * viewBox.width / svgRect.width;
          const x0 = Math.min(toSvgX(event.clientX), toSvgX(event.clientX - (end.x - drag.start.x)));
          const x1 = Math.max(toSvgX(event.clientX), toSvgX(event.clientX - (end.x - drag.start.x)));
          const plotStart = boxSpec.marginLeft;
          const plotEnd = boxSpec.width - boxSpec.marginRight;
          const left = Math.max(plotStart, Math.min(plotEnd, x0));
          const right = Math.max(plotStart, Math.min(plotEnd, x1));
          if (right - left > 6) {{
            const span = boxDomain.max - boxDomain.min;
            const min = boxDomain.min + ((left - plotStart) / (plotEnd - plotStart)) * span;
            const max = boxDomain.min + ((right - plotStart) / (plotEnd - plotStart)) * span;
            boxZoom = {{ min, max }};
            renderBoxplot(currentVisibleRows);
          }}
        }}
        drag = null;
      }});
      stage.addEventListener("pointercancel", () => {{
        drag = null;
        selection.hidden = true;
      }});
      reset.addEventListener("click", () => {{
        boxZoom = null;
        renderBoxplot(currentVisibleRows);
      }});
    }}

    function setupCdfDrag() {{
      const stage = document.querySelector("#cdfStage");
      const image = document.querySelector("#cdfImage");
      const selection = document.querySelector("#cdfSelection");
      const reset = document.querySelector("#cdfReset");
      if (!stage || !image || !selection || !reset) return;

      let drag = null;
      stage.addEventListener("pointerdown", event => {{
        if (event.button !== 0) return;
        const rect = stage.getBoundingClientRect();
        drag = {{
          id: event.pointerId,
          start: {{ x: event.clientX - rect.left, y: event.clientY - rect.top }},
        }};
        stage.setPointerCapture(event.pointerId);
        selection.hidden = true;
      }});
      stage.addEventListener("pointermove", event => {{
        if (!drag) return;
        const rect = stage.getBoundingClientRect();
        setSelectionBox(selection, stage, drag.start, {{
          x: event.clientX - rect.left,
          y: event.clientY - rect.top,
        }});
      }});
      stage.addEventListener("pointerup", event => {{
        if (!drag) return;
        const rect = stage.getBoundingClientRect();
        const end = {{ x: event.clientX - rect.left, y: event.clientY - rect.top }};
        const left = Math.min(drag.start.x, end.x);
        const top = Math.min(drag.start.y, end.y);
        const width = Math.abs(end.x - drag.start.x);
        const height = Math.abs(end.y - drag.start.y);
        selection.hidden = true;
        stage.releasePointerCapture(drag.id);

        if (width > 16 && height > 16) {{
          const scale = Math.min(rect.width / width, rect.height / height);
          image.style.transform = "translate(" + (-left * scale).toFixed(2) + "px, " +
            (-top * scale).toFixed(2) + "px) scale(" + scale.toFixed(3) + ")";
        }}
        drag = null;
      }});
      stage.addEventListener("pointercancel", () => {{
        drag = null;
        selection.hidden = true;
      }});
      reset.addEventListener("click", () => {{
        image.style.transform = "none";
      }});
    }}

    function compare(a, b) {{
      const av = valueFor(a, sortKey);
      const bv = valueFor(b, sortKey);
      let result = 0;
      if (typeof av === "number" && typeof bv === "number") {{
        result = av - bv;
      }} else {{
        result = String(av).localeCompare(String(bv), undefined, {{ numeric: true }});
      }}
      return sortDir === "asc" ? result : -result;
    }}

    function render() {{
      const tbody = document.querySelector("#summary tbody");
      tbody.textContent = "";
      const visible = rows
        .filter(row => rowText(row).includes(filterText))
        .sort(compare);
      currentVisibleRows = visible;

      for (const row of visible) {{
        const tr = document.createElement("tr");
        const transportClass = row.transport === "TCP" ? "tcp" : "udp";
        const cells = [
          row.label,
          `<span class="pill ${{transportClass}}">${{row.transport}}</span>`,
          row.mode,
          row.read,
          row.samples,
          row.avg,
          row.p50,
          row.p95,
          row.p99,
          row.max,
          row.outlier_count,
          row.miss,
          row.comm,
          row.dxl,
          perChannelText(row),
        ];
        cells.forEach((cell, index) => {{
          const td = document.createElement("td");
          if (index >= 4 && index <= 13) td.className = "num";
          if (index === 11 && row.miss > 0) td.classList.add("bad");
          if (index === 12 && row.comm > 0) td.classList.add("bad");
          if (index === 13 && row.dxl > 0) td.classList.add("warn");
          if (index === 14) td.className = "details";
          if (typeof cell === "number") {{
            td.textContent = Number.isInteger(cell) ? cell.toLocaleString() : formatNumber(cell);
          }} else if (String(cell).startsWith("<span")) {{
            td.innerHTML = cell;
          }} else {{
            td.textContent = cell;
          }}
          tr.appendChild(td);
        }});
        tbody.appendChild(tr);
      }}

      document.querySelectorAll("th").forEach(th => {{
        th.removeAttribute("data-dir");
        if (th.dataset.key === sortKey) th.dataset.dir = sortDir;
      }});
      renderBoxplot(visible);
    }}

    document.querySelectorAll("th").forEach(th => {{
      th.addEventListener("click", () => {{
        const key = th.dataset.key;
        if (sortKey === key) {{
          sortDir = sortDir === "asc" ? "desc" : "asc";
        }} else {{
          sortKey = key;
          sortDir = th.classList.contains("num") ? "asc" : "asc";
        }}
        render();
      }});
    }});

    document.querySelector("#filter").addEventListener("input", event => {{
      filterText = event.target.value.trim().toLowerCase();
      render();
    }});

    setupBoxplotDrag();
    setupCdfDrag();
    render();
  </script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="Generate a sortable HTML E2D2 latency report.")
    parser.add_argument("--out-root", default="/workspace/e2d2_latency_read_methods")
    parser.add_argument("--channels", default="1,2,3,4,6")
    parser.add_argument("--duration", default="10")
    parser.add_argument("--length", default="4")
    parser.add_argument("--read-methods", default=" ".join(DEFAULT_READ_METHODS))
    parser.add_argument("--output", default="e2d2_latency_read_methods.html")
    parser.add_argument("--svg", default="")
    parser.add_argument(
      "--title",
      default="E2D2 TCP/UDP Parallel and epoll Read Latency",
    )
    args = parser.parse_args()

    channels_tag = args.channels.replace(",", "-")
    read_methods = [item for item in args.read_methods.split() if item]
    rows = collect_rows(args.out_root, channels_tag, args.duration, read_methods)
    if not rows:
        raise RuntimeError(f"no CSV samples found under {args.out_root}")

    svg_path = args.svg
    if svg_path and os.path.isabs(svg_path):
        output_dir = os.path.dirname(os.path.abspath(args.output))
        try:
            svg_path = os.path.relpath(svg_path, output_dir)
        except ValueError:
            pass

    report = make_html(rows, args.title, svg_path, args.channels, args.length, args.duration)
    with open(args.output, "w") as output_file:
        output_file.write(report)
    print(args.output)


if __name__ == "__main__":
    main()
