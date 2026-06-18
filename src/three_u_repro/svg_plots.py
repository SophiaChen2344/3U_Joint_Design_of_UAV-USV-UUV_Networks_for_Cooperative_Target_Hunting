from __future__ import annotations

from pathlib import Path
import csv
from collections import defaultdict


def load_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def polyline(points: list[tuple[float, float]], color: str) -> str:
    joined = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    return f'<polyline fill="none" stroke="{color}" stroke-width="2" points="{joined}" />'


def render_line_svg(
    rows: list[dict[str, str]],
    *,
    case: str,
    x_field: str,
    y_field: str,
    title: str,
    output: Path,
) -> None:
    selected = [row for row in rows if row["case"] == case]
    grouped: dict[str, list[tuple[float, float]]] = defaultdict(list)
    for row in selected:
        grouped[row["algorithm"]].append((float(row[x_field]), float(row[y_field])))
    if not grouped:
        return

    x_values = [x for points in grouped.values() for x, _ in points]
    y_values = [y for points in grouped.values() for _, y in points]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    if abs(y_max - y_min) < 1e-9:
        y_max += 1.0
        y_min -= 1.0

    width, height = 720, 420
    left, right, top, bottom = 70, 30, 42, 62
    plot_w = width - left - right
    plot_h = height - top - bottom

    def sx(value: float) -> float:
        return left + (value - x_min) / max(x_max - x_min, 1e-9) * plot_w

    def sy(value: float) -> float:
        return top + (y_max - value) / max(y_max - y_min, 1e-9) * plot_h

    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]
    elements = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="24" text-anchor="middle" font-size="18">{title}</text>',
        f'<line x1="{left}" y1="{top + plot_h}" x2="{left + plot_w}" y2="{top + plot_h}" stroke="black"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_h}" stroke="black"/>',
        f'<text x="{width/2}" y="{height - 18}" text-anchor="middle" font-size="13">{x_field}</text>',
        f'<text x="18" y="{height/2}" transform="rotate(-90 18 {height/2})" text-anchor="middle" font-size="13">{y_field}</text>',
    ]

    for index in range(6):
        tx = left + index * plot_w / 5
        ty = top + index * plot_h / 5
        x_label = x_min + index * (x_max - x_min) / 5
        y_label = y_max - index * (y_max - y_min) / 5
        elements.append(f'<line x1="{tx:.2f}" y1="{top}" x2="{tx:.2f}" y2="{top + plot_h}" stroke="#eee"/>')
        elements.append(f'<line x1="{left}" y1="{ty:.2f}" x2="{left + plot_w}" y2="{ty:.2f}" stroke="#eee"/>')
        elements.append(f'<text x="{tx:.2f}" y="{top + plot_h + 18}" text-anchor="middle" font-size="11">{x_label:.1f}</text>')
        elements.append(f'<text x="{left - 8}" y="{ty + 4:.2f}" text-anchor="end" font-size="11">{y_label:.1f}</text>')

    for color, (algorithm, points) in zip(colors, sorted(grouped.items())):
        points = sorted(points)
        svg_points = [(sx(x), sy(y)) for x, y in points]
        elements.append(polyline(svg_points, color))
        for x, y in svg_points:
            elements.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3" fill="{color}"/>')
        legend_y = 46 + 18 * colors.index(color)
        elements.append(f'<rect x="{width - 170}" y="{legend_y - 10}" width="10" height="10" fill="{color}"/>')
        elements.append(f'<text x="{width - 154}" y="{legend_y}" font-size="12">{algorithm}</text>')

    elements.append("</svg>")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(elements), encoding="utf-8")


def render_standard_plots(csv_path: Path, output_dir: Path) -> list[Path]:
    rows = load_rows(csv_path)
    specs = [
        ("fig2a_fig3a_height", "h_m", "energy_kj", "Energy versus UAV height", "energy_vs_height.svg"),
        ("fig2b_fig3b_speed", "Vg_kn", "energy_kj", "Energy versus UUV speed", "energy_vs_speed.svg"),
        ("fig2a_fig3a_height", "h_m", "uav_usv_distance_m", "UAV-USV distance versus height", "uav_usv_distance_vs_height.svg"),
        ("fig2b_fig3b_speed", "Vg_kn", "usv_uuv_distance_m", "USV-UUV distance versus speed", "usv_uuv_distance_vs_speed.svg"),
        ("table_ii", "H_m", "success_rate", "Success rate versus target distance", "success_rate_vs_h.svg"),
    ]
    outputs: list[Path] = []
    for case, x_field, y_field, title, filename in specs:
        output = output_dir / filename
        render_line_svg(rows, case=case, x_field=x_field, y_field=y_field, title=title, output=output)
        if output.exists():
            outputs.append(output)
    return outputs
