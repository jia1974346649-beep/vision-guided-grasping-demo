"""Export only freshly generated synthetic simulation results and frames."""

from __future__ import annotations

import csv
import html
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


OUTPUT_NAMES = ("report.json", "trajectory.csv", "report.html", "preview.gif", "overview.png")


def _font(size=15):
    return ImageFont.load_default(size=size)


def _label(frame, stage):
    image = Image.new("RGB", (frame.width, frame.height + 56), "#102b2b")
    image.paste(frame.convert("RGB"), (0, 32))
    draw = ImageDraw.Draw(image)
    draw.text((12, 7), "SIMULATION / CONSTRAINT-ASSISTED", font=_font(13), fill="#d2eee4")
    draw.text((12, frame.height + 37), str(stage).upper(), font=_font(12), fill="#d2eee4")
    return image


def _stage_at(index, stages):
    stage = "initialize"
    for row in stages:
        if row["frame_index"] <= index:
            stage = row["stage"]
    return stage


def make_replay(frames, stages, destination):
    if not frames:
        raise ValueError("Recording requires at least one synthetic frame")
    stride = max(1, (len(frames) + 179) // 180)
    selected = list(range(0, len(frames), stride))
    if selected[-1] != len(frames) - 1:
        selected.append(len(frames) - 1)
    images = [_label(frames[index], _stage_at(index, stages)) for index in selected]
    # Stage transitions can add frames; this is a sampled explanatory replay,
    # not a wall-clock performance or precisely timed sensor recording.
    images[0].save(destination, save_all=True, append_images=images[1:],
                   duration=50 * stride, loop=0, optimize=False, disposal=2)


def make_overview(frames, stages, metrics, destination):
    if not frames:
        raise ValueError("Overview requires recorded frames")
    wanted = ("detect", "approach", "descend", "grasp", "lift", "hold")
    cells = []
    for name in wanted:
        matching = [i for i, stage in enumerate(stages) if stage["stage"] == name]
        if not matching:
            continue
        stage_index = matching[-1]
        stop = stages[stage_index + 1]["frame_index"] if stage_index + 1 < len(stages) else len(frames)
        index = min(len(frames) - 1, max(stages[stage_index]["frame_index"], stop - 1))
        cells.append((name, frames[index]))
    width, height = frames[0].size
    canvas = Image.new("RGB", (3 * width + 64, 2 * (height + 34) + 160), "#f1f5f0")
    draw = ImageDraw.Draw(canvas)
    draw.text((20, 17), "VISION-GUIDED GRASPING", font=_font(27), fill="#153c36")
    draw.text((20, 57), "Synthetic PyBullet scene | Fixed-constraint assisted grasp | No real robot", font=_font(15), fill="#46655c")
    for number, (stage, frame) in enumerate(cells):
        x, y = 16 + (number % 3) * (width + 16), 98 + (number // 3) * (height + 34)
        canvas.paste(frame, (x, y))
        draw.text((x + 3, y + height + 4), f"{number + 1:02d} / {stage.upper()}", font=_font(15), fill="#214b42")
    result = "COMPLETED" if metrics["success"] else "FAILED / " + metrics["failure_reason"]
    draw.text((20, canvas.height - 33), f"Seed {metrics['seed']} | {metrics['case']} | {result}", font=_font(15), fill="#214b42")
    canvas.save(destination)


def _number(value, digits=4):
    return "—" if value is None else f"{value:.{digits}f}"


def report_html(metrics):
    escape = html.escape
    status = "完成（辅助夹持）" if metrics["success"] else "按失败状态结束"
    events = "".join(f"<tr><td>{escape(str(row.get('stage', '')))}</td><td>{escape(str(row.get('step', '')))}</td></tr>" for row in metrics["stage_events"])
    raw = escape(json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False))
    return f"""<!doctype html>
<html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>视觉抓取仿真 · 匿名演示报告</title>
<link rel="icon" href="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'%3E%3Crect width='32' height='32' rx='8' fill='%23143c36'/%3E%3Ccircle cx='16' cy='16' r='7' fill='%23d1f1d9'/%3E%3C/svg%3E">
<style>
:root{{font:16px/1.8 system-ui,"Noto Sans CJK SC","Microsoft YaHei",sans-serif;color:#183d36;background:#f2f5ef}}*{{box-sizing:border-box}}body{{margin:0}}main{{max-width:1120px;margin:auto;padding:36px 24px}}h1{{font-size:32px;line-height:1.4;margin:8px 0 12px}}h2{{font-size:20px;margin:0 0 12px}}p{{margin:8px 0}}.eyebrow{{font-size:12px;letter-spacing:.15em;color:#48796c}}.notice{{background:#e5eee2;border-left:4px solid #42806b;padding:18px 22px;border-radius:8px;margin:24px 0}}.grid{{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}}section,.metric{{background:white;border:1px solid #dce5d8;border-radius:14px;padding:24px;margin-bottom:20px;min-width:0}}.metric strong{{display:block;font-size:25px}}.metric span{{font-size:14px;color:#627969}}.replay{{display:block;width:100%;max-width:640px;margin:auto;border-radius:9px}}.overview{{width:100%;height:auto}}a{{color:#156c56;text-underline-offset:3px}}nav{{display:flex;gap:20px;flex-wrap:wrap}}table{{border-collapse:collapse;width:100%}}td,th{{border-bottom:1px solid #e0e7da;text-align:left;padding:8px}}pre{{font:13px/1.7 monospace;white-space:pre-wrap;overflow-wrap:anywhere;background:#f4f7f0;padding:18px;border-radius:8px}}.muted{{color:#627969;font-size:14px}}@media(max-width:650px){{main{{padding:24px 16px}}.grid{{grid-template-columns:1fr}}h1{{font-size:27px}}section,.metric{{padding:18px}}}}
</style>
<main><p class="eyebrow">VISION → CONTROL → EVIDENCE</p><h1>视觉引导机器人抓取仿真</h1>
<p>固定种子 {metrics['seed']} · 场景 {escape(metrics['case'])} · {status}</p>
<div class="notice"><strong>这是 PyBullet 合成场景，使用固定约束辅助夹持。</strong><p>真实渲染图像用于定位，手写 DLS 与 PID 驱动机械臂。辅助约束承担附着，结果不表示接触摩擦抓取或真实机器人已经验证。</p></div>
<div class="grid"><div class="metric"><span>球心估计误差</span><strong>{_number(metrics['localization_error_m'] * 1000 if metrics['localization_error_m'] is not None else None, 2)} mm</strong></div><div class="metric"><span>相对初始提升高度</span><strong>{_number(metrics['lift_height_m'] * 100, 2)} cm</strong></div><div class="metric"><span>物理仿真时间</span><strong>{_number(metrics['simulated_seconds'], 2)} s</strong></div></div>
<section><h2>本次运行回放</h2><img class="replay" src="preview.gif" alt="合成场景中的视觉抓取阶段回放"><p class="muted">采样回放用于说明过程，包含额外阶段帧；播放时长不用于测量运行性能。</p></section>
<section><h2>阶段对照</h2><img class="overview" src="overview.png" alt="检测、接近、下降、夹持、提升和保持阶段"><table><thead><tr><th>状态</th><th>物理步数</th></tr></thead><tbody>{events}</tbody></table></section>
<section><h2>结果与边界</h2><p>状态原因：{escape(metrics['failure_reason'] or '已满足当前辅助夹持场景的提升与保持条件')}</p><p>相机正上方、已知球半径、固定光照和受限工作区是本演示的前提。场景真值用于误差评估及辅助约束建立，不用于运动目标规划。</p><nav><a href="report.json">原始运行指标 JSON</a><a href="trajectory.csv">采样轨迹 CSV</a></nav><details><summary>查看完整运行记录</summary><pre>{raw}</pre></details></section>
<p class="muted">独立匿名作品集演示 · 合成数据 · 模型由 PyBullet 数据依赖提供</p></main></html>"""


def write_report(result, output):
    metrics = result["metrics"]
    content = json.dumps(metrics, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    page = report_html(metrics)
    if not result["frames"]:
        raise ValueError("Report generation requires record=True")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    # Only these five documented result files are replaced by a new run.
    (output / "report.json").write_bytes(content.encode("utf-8"))
    with (output / "trajectory.csv").open("w", encoding="utf-8", newline="") as handle:
        rows = result["trajectory"]
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]) if rows else ["step", "time_s", "stage"])
        writer.writeheader()
        writer.writerows(rows)
    make_replay(result["frames"], result["stages"], output / "preview.gif")
    make_overview(result["frames"], result["stages"], metrics, output / "overview.png")
    (output / "report.html").write_bytes(page.encode("utf-8"))
    return output / "report.html"
