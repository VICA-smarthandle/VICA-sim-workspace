#!/usr/bin/env python3
"""Animate width-trial runs side by side, so a pass and a failure can be watched.

    ros2 run vica_description animate_run --width 1.20 \\
        --left  /tmp/vica_width_20kg/dwb_infl0.55 \\
        --right /tmp/vica_width_20kg/mppi_infl0.55 \\
        --out /tmp/vica_1.20

Writes <out>.gif and three stills: the start, the moment of divergence, and
the end.

The animation is built from the pose track each trial already records, not
from a camera. That is not a compromise. A camera shows a robot near a wall;
this draws the nav2 footprint polygon -- the shape that actually has to fit,
padding included -- against the wall boxes the course was built from. Whether
it fit is then visible rather than inferred.

Isaac runs headless here, so there is no viewport to capture, and rviz2 needs
a display it does not have. GIF is written through Pillow. MP4 needs ffmpeg,
which is not installed.

The track is recorded as spawn plus odom, and the trial's own start pose comes
from AMCL, which are not the same thing to within a few centimetres. The track
is shifted so its first sample sits on the AMCL start; without that the drawn
robot begins beside the lane mouth rather than in it, and the picture argues
for a conclusion the numbers do not support.
"""

import argparse
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Polygon, Rectangle  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Noto Sans CJK carries Hangul; DejaVu does not, and matplotlib's fallback for
# a missing glyph is a hollow box rather than an error. A figure captioned in
# boxes is worse than one captioned in English, so the family is checked here
# rather than assumed.
_have_cjk = any(
    "CJK" in f.name
    for f in matplotlib.font_manager.fontManager.ttflist)
matplotlib.rcParams["font.family"] = (
    ["Noto Sans CJK JP", "DejaVu Sans"] if _have_cjk else ["DejaVu Sans"])
matplotlib.rcParams["axes.unicode_minus"] = False
KO = _have_cjk

FOOTPRINT = [
    (0.305, 0.2275), (0.305, -0.2275), (-0.305, -0.2275),
    (-0.595, -0.035), (-0.595, 0.035), (-0.305, 0.2275),
]
PAD = 0.05
WALL_MIN_TOP = 0.15
ROBOT_SUBTREE = "/World/VICA"

COLOUR = {"reached": "#1a9850", "stuck": "#d73027",
          "timeout": "#fc8d59", "no-start": "#762a83"}


def footprint_at(x, y, yaw, pad=PAD):
    """nav2 pads per axis, not radially. See plot_run for why that matters."""
    pts = []
    for fx, fy in FOOTPRINT:
        gx = fx + (-pad if fx < 0 else pad)
        gy = fy + (-pad if fy < 0 else pad)
        pts.append((x + gx * math.cos(yaw) - gy * math.sin(yaw),
                    y + gx * math.sin(yaw) + gy * math.cos(yaw)))
    return pts


def read_course(stage_path):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(stage_path)
    stage.Load()
    walls = []
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(ROBOT_SUBTREE):
            continue
        if not prim.IsA(UsdGeom.Cube):
            continue
        m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        size = UsdGeom.Cube(prim).GetSizeAttr().Get() or 1.0
        w = m.GetRow3(0).GetLength() * size
        h = m.GetRow3(1).GetLength() * size
        d = m.GetRow3(2).GetLength() * size
        if t[2] + d / 2 < WALL_MIN_TOP:
            continue
        walls.append((t[0] - w / 2, t[1] - h / 2, w, h))
    return walls


def load_runs(directory, width):
    """Every repeat of one width, newest-looking first."""
    out = []
    for path in sorted(glob.glob(os.path.join(directory, f"lane_{width}_*.json"))):
        d = json.load(open(path))
        for r in d.get("records", []):
            track = r.get("track") or []
            if not track:
                continue
            start = r.get("from")
            if start:
                dx = start[0] - track[0][1]
                dy = start[1] - track[0][2]
                track = [[t, x + dx, y + dy, a] for t, x, y, a in track]
            out.append({
                "label": os.path.basename(path).replace(".json", ""),
                "controller": d.get("controller", "?"),
                "result": r.get("result", "?"),
                "moved": r.get("moved_m"),
                "goal": r.get("goal"),
                "clear_min": r.get("clearance_min"),
                "track": track,
            })
    return out


def pick(runs, prefer):
    """One run per side: the clearest example of what that controller did."""
    if not runs:
        return None
    for want in prefer:
        for r in runs:
            if r["result"] == want:
                return r
    return max(runs, key=lambda r: len(r["track"]))


def _view_for(args, xs, ys, goal, sides):
    """The rectangle the frame covers.

    The default fits the whole run in, which is the right choice for reading a
    path and the wrong one for reading the robot. A corner trial approaches
    along 27 m of corridor and turns inside 3 m of it; framed on the track, the
    robot is four pixels across and the corner it is negotiating is a smudge in
    one edge. --focus goal --window 8 puts the corner across the frame with the
    robot large enough to see which way its handle is pointing.

    Nothing is cropped in either case. The axes keep an equal aspect, so a
    window narrower than the figure is padded rather than cut.
    """
    if args.focus == "track":
        return (min(xs) - 2.0,
                max(xs) - min(xs) < 4 and min(xs) + 4.0 or max(xs) + 2.0,
                min(min(ys), goal[1]) - 1.5,
                max(max(ys), goal[1]) + 1.5)

    w = args.window if args.window else 8.0
    if args.focus == "goal":
        cx, cy = goal[0], goal[1]
    elif args.focus == "start":
        cx, cy = sides[0]["track"][0][1], sides[0]["track"][0][2]
    else:
        if not args.focus_xy:
            raise SystemExit("--focus point 에는 --focus-xy 'x,y' 가 필요합니다")
        cx, cy = (float(v) for v in args.focus_xy.split(","))
    return (cx - w / 2.0, cx + w / 2.0, cy - w / 2.0, cy + w / 2.0)


def draw_course(ax, walls, view):
    for x, y, w, h in walls:
        ax.add_patch(Rectangle((x, y), w, h, facecolor="#444", edgecolor="none",
                               zorder=1))
    ax.set_xlim(view[0], view[1])
    ax.set_ylim(view[2], view[3])
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.15, zorder=0)
    ax.set_xlabel("x (m)")


def _title(width):
    if KO:
        return f"레인 {width} m — 다각형은 padding 포함 nav2 footprint"
    return f"lane {width} m — polygon is the nav2 footprint, padding included"


def storyboard(sides, walls, view, width, out, cols=6, dpi=170):
    """A filmstrip: one row per controller, time running left to right.

    A video cannot be watched over a remote shell, and a single still cannot
    show a robot getting stuck -- being stuck is a thing that takes time. The
    grid puts both runs on one clock so the column where they part company is
    visible at a glance.

    Sampled on elapsed sim time rather than track index, because the runs have
    different sample counts and a stalled run's samples pile up in one place.
    """
    span = max(s["track"][-1][0] - s["track"][0][0] for s in sides)
    fig, axes = plt.subplots(len(sides), cols,
                             figsize=(2.5 * cols, 3.4 * len(sides)),
                             squeeze=False)
    for row, s in enumerate(sides):
        t0 = s["track"][0][0]
        c = COLOUR.get(s["result"], "#666")
        for col in range(cols):
            ax = axes[row][col]
            want = t0 + span * col / (cols - 1)
            k = min(range(len(s["track"])),
                    key=lambda i: abs(s["track"][i][0] - want))
            for x, y, w, h in walls:
                ax.add_patch(Rectangle((x, y), w, h, facecolor="#555",
                                       edgecolor="none", zorder=1))
            seg = s["track"][:k + 1]
            ax.plot([p[1] for p in seg], [p[2] for p in seg], "-", lw=1.4,
                    color=c, zorder=4)
            t, x, y, a = s["track"][k]
            ax.add_patch(Polygon(footprint_at(x, y, a), closed=True, fill=False,
                                 edgecolor=c, lw=1.8, zorder=6))
            if s["goal"]:
                ax.plot(s["goal"][0], s["goal"][1], "*", ms=11,
                        color="#2166ac", zorder=5)
            ax.set_xlim(view[0], view[1])
            ax.set_ylim(view[2], view[3])
            ax.set_aspect("equal")
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_edgecolor("#bbb")
            ax.set_title(f"{t - t0:.0f} s", fontsize=10, color="#333", pad=3)
            if col == 0:
                cm = s.get("clear_min")
                tail = f"\n최소여유 {cm:.2f} m" if cm else ""
                ax.set_ylabel(f"{s['controller'].upper()}\n{s['result']}  "
                              f"({s['tally']}){tail}", fontsize=11, color=c)
    fig.suptitle(_title(width), fontsize=13)
    fig.tight_layout()
    path = f"{out}_storyboard.png"
    fig.savefig(path, dpi=dpi, bbox_inches="tight")
    plt.close(fig)
    print(f"  wrote {path}")
    return path


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--width", required=True, help='lane width as written in the filename, e.g. "1.20"')
    ap.add_argument("--left", required=True, help="results directory for the left panel")
    ap.add_argument("--right", default=None, help="results directory for the right panel")
    ap.add_argument("--stage", default=None)
    ap.add_argument("--out", required=True, help="output path without extension")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--dpi", type=int, default=200,
                    help="still resolution; 200 is print quality")
    ap.add_argument("--focus", default="track",
                    choices=("track", "goal", "start", "point"),
                    help="what the frame is built around. track fits the whole "
                         "run in, which is right for seeing a path and wrong "
                         "for seeing the robot: over a 27 m approach it becomes "
                         "a smear. goal or start centres a window on one end, "
                         "point takes --focus-xy.")
    ap.add_argument("--window", type=float, default=None,
                    help="metres across, for the focused views. Default 8, "
                         "which holds a corner cell, a crossing or a dead end "
                         "with the 0.83 m robot still large enough to read.")
    ap.add_argument("--focus-xy", default=None,
                    help='"x,y" for --focus point')
    ap.add_argument("--stride", type=int, default=8,
                    help="use every Nth track sample; tracks are ~1400 long")
    args = ap.parse_args()

    stage = args.stage
    if stage is None:
        from ament_index_python.packages import get_package_share_directory
        stage = os.path.join(get_package_share_directory("vica_description"),
                             "isaac_vica_assets", "vica_widthcourse.usd")
    walls = read_course(stage)

    sides = []
    for d in (args.left, args.right):
        if not d:
            continue
        runs = load_runs(d, args.width)
        # Show the pass if there is one, otherwise the most instructive failure.
        r = pick(runs, ("reached", "stuck", "timeout", "no-start"))
        if r is None:
            print(f"  {d}: {args.width} m 결과에 track 이 없습니다", file=sys.stderr)
            continue
        passes = sum(1 for x in runs if x["result"] == "reached")
        r["tally"] = f"{passes}/{len(runs)}"
        sides.append(r)
    if not sides:
        return 1

    xs = [p[1] for s in sides for p in s["track"]]
    ys = [p[2] for s in sides for p in s["track"]]
    goal = sides[0]["goal"] or [0, 0]
    view = _view_for(args, xs, ys, goal, sides)

    storyboard(sides, walls, view, args.width, args.out)

    fig, axes = plt.subplots(1, len(sides), figsize=(6.2 * len(sides), 8.4),
                             squeeze=False)
    axes = axes[0]
    arts = []
    frames = max(len(s["track"]) // args.stride for s in sides)

    for ax, s in zip(axes, sides):
        draw_course(ax, walls, view)
        if s["goal"]:
            ax.plot(s["goal"][0], s["goal"][1], "*", ms=18, color="#2166ac",
                    zorder=5, label="goal")
        c = COLOUR.get(s["result"], "#666")
        trail, = ax.plot([], [], "-", lw=1.6, color=c, alpha=0.85, zorder=4)
        poly = Polygon(footprint_at(0, 0, 0), closed=True, fill=False,
                       edgecolor=c, lw=2.0, zorder=6)
        ax.add_patch(poly)
        clock = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top",
                        fontsize=10, family="monospace")
        cm = s.get("clear_min")
        tail = f"   최소여유 {cm:.2f} m" if cm else ""
        ax.set_title(f"{s['controller'].upper()}   {args.width} m   "
                     f"{s['result']}   ({s['tally']}){tail}", fontsize=12, color=c)
        ax.set_ylabel("y (m)")
        arts.append((s, trail, poly, clock))

    def draw(indices):
        """indices[j] is the track sample index for panel j."""
        out = []
        for (s, trail, poly, clock), k in zip(arts, indices):
            k = max(0, min(k, len(s["track"]) - 1))
            seg = s["track"][:k + 1]
            trail.set_data([p[1] for p in seg], [p[2] for p in seg])
            t, x, y, a = s["track"][k]
            poly.set_xy(footprint_at(x, y, a))
            el = t - s["track"][0][0]
            clock.set_text(f"{el:5.1f} s\n{s['moved']:5.2f} m moved")
            out += [trail, poly, clock]
        return out

    def update(i):
        # Animation: every panel at the same instant. Two controllers on the
        # same clock is what a viewer expects, and the divergence shows as one
        # robot leaving the other behind.
        return draw([i * args.stride] * len(arts))

    def nearest_index(s, y):
        """Where this run's robot came closest to that y."""
        return min(range(len(s["track"])), key=lambda k: abs(s["track"][k][2] - y))

    def draw_at_y(y):
        # Stills: every panel at the same *place*, which is a different
        # question from the same instant and the one a comparison figure has
        # to answer. At a shared timestamp the fast run is already past the
        # gap and the stalled one has not reached it, so the picture shows two
        # robots in different corridors rather than two attempts at the same
        # one.
        return draw([nearest_index(s, y) for s in sides])

    fig.suptitle(_title(args.width),
                 fontsize=13)
    fig.tight_layout()

    anim = FuncAnimation(fig, update, frames=frames, interval=1000 / args.fps,
                         blit=False)
    gif = args.out + ".gif"
    anim.save(gif, writer=PillowWriter(fps=args.fps))
    print(f"  wrote {gif}  ({frames} 프레임)")

    # MP4 when ffmpeg is there, and a plain sentence when it is not. Writing
    # only the GIF and saying nothing would leave someone looking for an mp4
    # that was never going to appear.
    import matplotlib.animation as _anim
    if _anim.writers.is_available("ffmpeg"):
        mp4 = args.out + ".mp4"
        anim.save(mp4, writer=_anim.FFMpegWriter(
            fps=args.fps, bitrate=2400,
            extra_args=["-pix_fmt", "yuv420p"]))
        print(f"  wrote {mp4}")
    else:
        print("  mp4 는 건너뜀 -- ffmpeg 이 없습니다 (sudo apt install ffmpeg)")

    # Stills for a report, at print resolution.
    #
    # The middle frame is taken where the lane is narrowest rather than at the
    # halfway mark. Half of a track that stopped after two metres is the robot
    # still in the approach, which shows nothing; half of a track that finished
    # is wherever the clock happened to be. The narrow part is the question the
    # course was built to ask, so that is the frame worth printing.
    # The frame worth printing is where the runs part company: the y at which
    # the worst of them stopped. Both panels are then drawn at that same y, so
    # the figure reads as one place and two outcomes rather than two pictures.
    stall = min(s["track"][-1][2] for s in sides)
    narrow = _narrowest_y(walls, sides)

    for name, y in (("start", min(s["track"][0][2] for s in sides)),
                    ("diverge", stall),
                    ("narrow", narrow if narrow is not None else stall)):
        draw_at_y(y)
        # Keep the handles and remove them; ax.lines is read-only from
        # matplotlib 3.5, and assigning to it raises rather than filtering.
        guides = [ax.axhline(y, color="#2166ac", lw=0.9, ls="--", alpha=0.6,
                             zorder=3) for ax in axes]
        png = f"{args.out}_{name}.png"
        fig.savefig(png, dpi=args.dpi, bbox_inches="tight")
        print(f"  wrote {png}   (y={y:.2f} m)")
        for g in guides:
            g.remove()

    # And the end of each run in its own right, which is where a failure's
    # final pose actually is.
    draw([len(s["track"]) - 1 for s in sides])
    png = f"{args.out}_end.png"
    fig.savefig(png, dpi=args.dpi, bbox_inches="tight")
    print(f"  wrote {png}")
    return 0


def _narrowest_y(walls, sides):
    """y of the lane's tightest point: where the two side walls are closest."""
    xs = [p[1] for s in sides for p in s["track"]]
    lo, hi = min(xs) - 1.5, max(xs) + 1.5
    best_y, best_gap = None, 1e9
    for y in [v / 20.0 for v in range(-40, 140)]:
        spans = sorted(
            (x, x + w) for x, wy, w, h in walls
            if wy <= y <= wy + h and lo <= x + w and x <= hi)
        for (a0, a1), (b0, b1) in zip(spans, spans[1:]):
            gap = b0 - a1
            if 0.3 < gap < best_gap:
                best_gap, best_y = gap, y
    return best_y


def _frame_nearest_y(sides, y, stride, frames):
    """The frame where the first side's robot is closest to that y."""
    if y is None:
        return frames // 2
    s = sides[0]
    best, bi = 1e9, frames // 2
    for i in range(frames):
        k = min(i * stride, len(s["track"]) - 1)
        d = abs(s["track"][k][2] - y)
        if d < best:
            best, bi = d, i
    return bi


if __name__ == "__main__":
    sys.exit(main())
