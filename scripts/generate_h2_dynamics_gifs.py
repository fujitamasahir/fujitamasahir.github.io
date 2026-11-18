#!/usr/bin/env python3
"""Create paired, source-verified H2 unseen-maze GIFs from recorded G1-proxy episodes.

The frames reproduce the recorded Experiment Review truth map, not a new
simulation or an imagined obstacle schedule. The same maze, model, seed and
schedule are used in both frequency conditions.
"""
from __future__ import annotations

from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from PIL import Image, ImageDraw, ImageFont

REPO = "fujitamasahir/humanoid-spatial-memory"
SOURCE_COMMIT = "3b208befeb268cb2ae789add7deaaf859abcfc5a"
MANIFEST_PATH = "results/phase2_mujoco/h2_comprehensive_benchmark_v1/combined_dataset/replay_index.json"
MODEL = "rdao_v1_footprint"
MAX_STEPS = 80
OUTPUT_DIR = Path("assets/spatial-memory")

COLORS = {
    "bg": "#0d1724", "panel": "#132232", "free": "#1c2d3c",
    "wall_r": "#355c88", "wall_bp": "#327d70", "wall_m": "#76529b",
    "object": "#f19a43", "object_detail": "#a85d25",
    "ped": "#f0444f", "robot": "#64b4ff", "robot_edge": "#c9ebff",
    "goal": "#ffd074", "event": "#fff2bb", "trail": "#6ba7d4",
    "white": "#f1f6fa", "muted": "#a9bccb", "border": "#4c6376",
    "success": "#8bdfbb", "dark": "#0a1420",
}


def raw(path: str) -> str:
    return f"https://raw.githubusercontent.com/{REPO}/{SOURCE_COMMIT}/{path}"


def download(path: str, expected_sha: str | None = None) -> bytes:
    request = Request(raw(path), headers={"User-Agent": "H2-verified-dynamics-figure/1.0"})
    for attempt in range(4):
        try:
            with urlopen(request, timeout=35) as response:
                data = response.read()
            if expected_sha and hashlib.sha256(data).hexdigest() != expected_sha:
                raise ValueError(f"SHA-256 mismatch: {path}")
            return data
        except (HTTPError, URLError, TimeoutError):
            if attempt == 3:
                raise
            time.sleep(1.5 * (attempt + 1))
    raise AssertionError("unreachable")


def read_episode(entry: dict) -> dict:
    if entry["payload_type"] != "replay":
        raise ValueError("Selected episode lacks full recorded replay")
    body = json.loads(gzip.decompress(download(entry["path"], entry["sha256"])))
    replay = body["replay"]
    row = body["row"]
    if (replay["schema_version"] != "1.0.0"
        or replay["episode_key"] != [entry["world_identity"], entry["schedule"], MODEL, 42]
        or row["world_identity"] != entry["world_identity"]
        or len(replay["steps"]) != row["elapsed_steps"] + 1
        or replay["grid"]["width"] != 25 or replay["grid"]["height"] != 25):
        raise ValueError(f"Invalid replay metadata: {entry['path']}")
    for i, frame in enumerate(replay["steps"]):
        if frame["step"] != i or frame["event"]["step"] != i:
            raise ValueError("Nonconsecutive recorded frames")
    return {"entry": entry, "body": body, "replay": replay, "row": row}


def trajectory_length(steps: list[dict]) -> float:
    positions = [f["robot_position_after_action_xy"] for f in steps]
    return sum(math.dist(a, b) for a, b in zip(positions, positions[1:]))


def score_pair(low: dict, high: dict) -> tuple[float, dict]:
    lf, hf = low["replay"]["steps"], high["replay"]["steps"]
    high_events = Counter(f["event"]["event_type"] for f in hf[1:])
    low_events = Counter(f["event"]["event_type"] for f in lf[1:])
    ped_count = [sum(bool(f.get("pedestrians_xy")) for f in fs) for fs in (lf, hf)]
    def near_ped(frames):
        return sum(any(math.dist(f["robot_position_after_action_xy"], p["cell"]) <= 6.5
                       for p in f.get("pedestrians_xy", [])) for f in frames)
    nearby = [near_ped(fs) for fs in (lf, hf)]
    travel = [trajectory_length(fs) for fs in (lf, hf)]
    full = min(len(lf), len(hf)) >= 54
    diverse = all(high_events.get(e, 0) >= 2 for e in ("addition", "relocation", "removal"))
    active_high = sum(high_events.get(e, 0) for e in ("addition", "relocation", "removal"))
    low_changes = sum(v for e, v in low_events.items() if e not in ("no_op", "no_change_in_saved_schedule"))
    qualifies = (full and diverse and ped_count[1] >= 20 and nearby[1] >= 5 and
                 min(travel) >= 12 and active_high >= 45 and low_changes >= 1)
    # Pedestrian encounters, visibly distinct dynamics, and both successful
    # runs improve the clarity of a paired teaching illustration.
    score = (1000 if qualifies else 0) + nearby[1] * 4 + ped_count[1] * .12 \
        + min(travel) * 1.5 + min(active_high, 70) * .3 + low_changes * 8 \
        + (80 if low["row"]["success"] and high["row"]["success"] else 0) \
        + (35 if low["replay"]["map_family"] == "R" else 0) \
        + (50 if low["entry"]["schedule"] == high["entry"]["schedule"] else 0) \
        - 1.5 * sum(high_events.get(e, 0) for e in ("no_op",))
    return score, {"qualifies": qualifies, "schedules": [low["entry"]["schedule"], high["entry"]["schedule"]], "pedestrian_frames": ped_count,
                   "nearby_pedestrian_frames": nearby, "robot_travel_cells": [round(v, 1) for v in travel],
                   "high_events": dict(high_events), "low_events": dict(low_events),
                   "success": [bool(low["row"]["success"]), bool(high["row"]["success"])],
                   "steps": [len(lf) - 1, len(hf) - 1]}


def choose_pair(entries: list[dict]) -> tuple[dict, dict, dict]:
    relevant = [e for e in entries if e["model_id"] == MODEL
                and e["condition"] in ("original", "high_frequency") and e["payload_type"] == "replay"]
    by_maze: dict[str, dict] = {}
    for e in relevant:
        by_maze.setdefault(e["world_identity"], {})[(e["condition"], e["schedule"])] = e
    candidates = {k: v for k, v in by_maze.items() if len(v) == 4}
    if len(candidates) != 36:
        raise ValueError(f"Expected 36 paired unseen mazes, found {len(candidates)}")
    print(f"Evaluating {len(candidates)} unseen mazes, model={MODEL}, two schedules each", flush=True)
    episodes = {}
    with ThreadPoolExecutor(max_workers=8) as pool:
        futures = {pool.submit(read_episode, e): (maze, cond, schedule)
                   for maze, pair in candidates.items() for (cond, schedule), e in pair.items()}
        for future in as_completed(futures):
            episodes[futures[future]] = future.result()
    ranked = []
    for maze in candidates:
        for low_schedule in (0, 1):
            for high_schedule in (0, 1):
                low = episodes[maze, "original", low_schedule]
                high = episodes[maze, "high_frequency", high_schedule]
                if (low["replay"]["static_walls_xy"] != high["replay"]["static_walls_xy"]
                    or low["replay"]["start_xy"] != high["replay"]["start_xy"]
                    or low["replay"]["goal_xy"] != high["replay"]["goal_xy"]):
                    raise ValueError(f"Paired map geometry differs: {maze}")
                score, metrics = score_pair(low, high)
                ranked.append((score, maze, low_schedule, high_schedule, metrics))
    ranked.sort(key=lambda item: (-item[0], item[1], item[2], item[3]))
    for score, maze, low_schedule, high_schedule, metrics in ranked[:5]:
        print(f"Candidate score={score:.1f} {maze} low=S{low_schedule} high=S{high_schedule}: {json.dumps(metrics)}", flush=True)
    winner = next((item for item in ranked if item[4]["qualifies"]), None)
    if winner is None:
        raise RuntimeError("No matched maze with low-frequency object changes and high-frequency pedestrians")
    _, maze, low_schedule, high_schedule, metrics = winner
    print(f"SELECTED {maze} low=S{low_schedule} high=S{high_schedule}: {json.dumps(metrics)}", flush=True)
    return episodes[maze, "original", low_schedule], episodes[maze, "high_frequency", high_schedule], metrics

def font(size: int, bold: bool = False):
    path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


W, H = 480, 538
CELL = 15
GX, GY = (W - 25 * CELL) // 2, 78


def center(c: list[int] | tuple[int, int]) -> tuple[float, float]:
    return GX + (c[0] + .5) * CELL, GY + (24 - c[1] + .5) * CELL


def draw_ring(draw: ImageDraw.ImageDraw, cell: list[int], color: str, radius: float = 6.6, width: int = 2):
    x, y = center(cell)
    draw.ellipse((x - radius, y - radius, x + radius, y + radius), outline=color, width=width)


def draw_panel(data: dict, step: int, label: str, number: int) -> Image.Image:
    replay = data["replay"]
    frames = replay["steps"]
    index = min(step, len(frames) - 1)
    frame = frames[index]
    im = Image.new("RGB", (W, H), COLORS["bg"])
    d = ImageDraw.Draw(im)
    d.rounded_rectangle((10, 8, W - 10, 68), radius=12, fill=COLORS["panel"])
    d.text((23, 15), label.upper(), fill=COLORS["white"], font=font(20, True))
    d.text((23, 43), f"MAZE {number:02d}  |  {replay['map_family']}  |  RDAO v1 Footprint", fill=COLORS["muted"], font=font(13))
    step_label = f"STEP {index:02d}/{MAX_STEPS}"
    box = d.textbbox((0, 0), step_label, font=font(13, True))
    d.text((W - 23 - (box[2] - box[0]), 18), step_label, fill=COLORS["goal"], font=font(13, True))

    d.rectangle((GX - 5, GY - 5, GX + 25 * CELL + 4, GY + 25 * CELL + 4), fill=COLORS["dark"], outline=COLORS["border"], width=1)
    walls = {tuple(c) for c in replay["static_walls_xy"]}
    objects = {tuple(o["cell"]) for o in frame["objects_after"]}
    wall_color = COLORS[{"R": "wall_r", "BP": "wall_bp", "M": "wall_m"}[replay["map_family"]]]
    for y in range(25):
        for x in range(25):
            yy = GY + (24 - y) * CELL
            xx = GX + x * CELL
            color = wall_color if (x, y) in walls else COLORS["free"]
            d.rectangle((xx + 1, yy + 1, xx + CELL - 1, yy + CELL - 1), fill=color)
    for o in frame["objects_after"]:
        x, y = center(o["cell"])
        d.rounded_rectangle((x - 6.1, y - 6.1, x + 6.1, y + 6.1), radius=2, fill=COLORS["object"])
        d.rectangle((x - 2.1, y - 2.1, x + 2.1, y + 2.1), fill=COLORS["object_detail"])

    # Record-accurate event rings: no invented intermediate obstacle positions.
    ev = frame["event"]
    event_type = ev.get("event_type", "no_op")
    event_cells = []
    for c in ev.get("changes", []):
        for key in ("cell", "from_cell", "to_cell", "to"):
            if c.get(key) is not None:
                event_cells.append(c[key])
    for key in ("from_cell", "to_cell"):
        if ev.get(key) is not None:
            event_cells.append(ev[key])
    for cell in event_cells:
        draw_ring(d, cell, COLORS["event"], radius=7.5, width=2)
    if ev.get("from_cell") and ev.get("to_cell"):
        d.line([center(ev["from_cell"]), center(ev["to_cell"])], fill=COLORS["event"], width=2)

    # The blue path is the *recorded* G1-proxy trajectory up to the current step.
    trajectory = [center(f["robot_position_after_action_xy"]) for f in frames[:index + 1]]
    if len(trajectory) >= 2:
        d.line(trajectory, fill=COLORS["trail"], width=3, joint="curve")
    draw_ring(d, replay["start_xy"], COLORS["robot_edge"], radius=5.4, width=2)
    draw_ring(d, replay["goal_xy"], COLORS["goal"], radius=7.3, width=3)
    x, y = center(replay["goal_xy"])
    d.ellipse((x - 2.4, y - 2.4, x + 2.4, y + 2.4), fill=COLORS["goal"])

    # Pedestrians are red moving discs, drawn above the obstacles and trail.
    for pedestrian in frame.get("pedestrians_xy", []):
        x, y = center(pedestrian["cell"])
        d.ellipse((x - 5.8, y - 5.8, x + 5.8, y + 5.8), fill=COLORS["ped"], outline=COLORS["white"], width=1)

    rx, ry = center(frame["robot_position_after_action_xy"])
    yaw = float(frame["robot_heading_after_action_rad"])
    ux, uy = math.cos(yaw), -math.sin(yaw)
    vx, vy = -uy, ux
    points = [
        (rx + 8.7 * ux, ry + 8.7 * uy),
        (rx - 6.1 * ux + 5.2 * vx, ry - 6.1 * uy + 5.2 * vy),
        (rx - 3.1 * ux, ry - 3.1 * uy),
        (rx - 6.1 * ux - 5.2 * vx, ry - 6.1 * uy - 5.2 * vy),
    ]
    d.polygon(points, fill=COLORS["robot"])
    d.line(points + [points[0]], fill=COLORS["robot_edge"], width=2)
    d.text((rx + 9, ry - 17), "G1", fill=COLORS["white"], font=font(12, True), stroke_width=2, stroke_fill=COLORS["dark"])

    labels = {
        "initial_state": "INITIAL STATE", "addition": "OBJECT APPEARED",
        "relocation": "OBJECT MOVED", "removal": "OBJECT DISAPPEARED",
        "no_op": "NO CHANGE", "no_change_in_saved_schedule": "NO CHANGE",
        "saved_schedule_changes": "OBJECTS CHANGED",
    }
    event_label = labels.get(event_type, event_type.replace("_", " ").upper())
    d.rounded_rectangle((11, 468, W - 11, 527), radius=11, fill=COLORS["panel"])
    d.text((23, 475), event_label, fill=COLORS["event"] if event_cells else COLORS["muted"], font=font(15, True))
    if index == len(frames) - 1 and frame.get("goal_reached"):
        d.text((W - 135, 477), "GOAL REACHED", fill=COLORS["success"], font=font(11, True))
    # Legend with shapes matching Experiment Review.
    ly = 511
    # Reuse the concave four-point wedge silhouette shown on the map.
    g1_points = [(26, ly), (13, ly - 6), (17, ly), (13, ly + 6)]
    d.polygon(g1_points, fill=COLORS["robot"])
    d.line(g1_points + [g1_points[0]], fill=COLORS["robot_edge"], width=1)
    d.text((31, ly - 7), "G1", fill=COLORS["muted"], font=font(11))
    d.rectangle((80, ly - 5, 89, ly + 4), fill=COLORS["object"])
    d.text((94, ly - 7), "Object", fill=COLORS["muted"], font=font(11))
    d.ellipse((165, ly - 5, 175, ly + 5), fill=COLORS["ped"])
    d.text((180, ly - 7), "Pedestrian", fill=COLORS["muted"], font=font(11))
    d.ellipse((288, ly - 6, 300, ly + 6), outline=COLORS["goal"], width=2)
    d.text((307, ly - 7), "Goal", fill=COLORS["muted"], font=font(11))
    d.ellipse((366, ly - 6, 378, ly + 6), outline=COLORS["event"], width=2)
    d.text((385, ly - 7), "Change", fill=COLORS["muted"], font=font(11))
    return im


def save_gif(data: dict, label: str, number: int, filename: str) -> dict:
    frames = [draw_panel(data, step, label, number) for step in range(MAX_STEPS + 1)]
    # A single stable palette prevents color flicker between frames.
    sample = frames[20].resize((240, 269))
    # Guarantee the pedestrian red survives the shared GIF palette even when
    # pedestrians are absent from the sample frame.
    palette_draw = ImageDraw.Draw(sample)
    for color_index, color in enumerate(COLORS.values()):
        x = color_index * 11
        palette_draw.rectangle((x, 0, x + 10, 11), fill=color)
    palette = sample.quantize(colors=128, method=Image.Quantize.MEDIANCUT)
    paletted = [f.quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    dest = OUTPUT_DIR / filename
    paletted[0].save(dest, save_all=True, append_images=paletted[1:],
                     duration=180, loop=0, disposal=2, optimize=True)
    print(f"Wrote {dest} ({dest.stat().st_size:,} bytes; {len(frames)} frames)", flush=True)
    return {"file": str(dest), "frames": len(frames), "bytes": dest.stat().st_size}


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    manifest = json.loads(download(MANIFEST_PATH))
    if manifest["schema_version"] != "h2_comprehensive_replay_index_v1":
        raise ValueError("Unexpected source manifest")
    low, high, metrics = choose_pair(manifest["entries"])
    maze = low["entry"]["world_identity"]
    if high["entry"]["world_identity"] != maze:
        raise ValueError("Not the same unseen maze")
    if metrics["schedules"][0] == metrics["schedules"][1]:
        print("Selected a matched schedule with both object changes and pedestrians", flush=True)
    else:
        print("NOTE: schedules differ to show low-frequency object changes and high-frequency pedestrians; these are illustrative, not a controlled pair", flush=True)
    catalog = json.loads(Path("games/beat-the-humanoid/h2/index.json").read_text())
    number = next(m["number"] for m in catalog["maps"] if m["identity"] == maze)
    low_result = save_gif(low, "Low-frequency changes", number, "h2-unseen-low-frequency.gif")
    high_result = save_gif(high, "High-frequency changes", number, "h2-unseen-high-frequency.gif")
    # One lightweight static side-by-side proof image for source/layout review.
    preview = Image.new("RGB", (2 * W + 12, H), COLORS["bg"])
    preview.paste(draw_panel(low, 24, "Low-frequency changes", number), (0, 0))
    preview.paste(draw_panel(high, 24, "High-frequency changes", number), (W + 12, 0))
    preview_path = OUTPUT_DIR / "h2-unseen-dynamics-preview.png"
    preview.save(preview_path, optimize=True)
    metadata = {
        "description": "Paired real H2 Experiment Review replays of the kinematic Unitree G1 proxy",
        "source_repo": REPO, "source_commit": SOURCE_COMMIT,
        "maze_identity": maze, "display_maze_number": number,
        "model": MODEL, "schedules": {"low": low["entry"]["schedule"], "high": high["entry"]["schedule"]}, "seed": 42,
        "selection": metrics,
        "low": {"entry": low["entry"], "result": low_result},
        "high": {"entry": high["entry"], "result": high_result},
        "animation": {"step_range": [0, MAX_STEPS], "duration_ms": 180,
                      "notes": "After the episode ends, its final recorded frame is held; no extra states are simulated."},
    }
    metadata_path = OUTPUT_DIR / "h2-unseen-dynamics-source.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Saved {preview_path} and {metadata_path}", flush=True)


if __name__ == "__main__":
    main()
