#!/usr/bin/env python3
"""Pull labelled stills out of a replay_render frame directory.

    python3 make_stills.py <frames_dir> <out.jpg> --caption "..." [--at last|N|0.75]

replay_render writes one PNG per track sample -- 1262 of them for a single
60-second run. A report needs one, and which one is a judgement about what the
run was: for a robot that stopped, the last frame is the answer, because the
last frame is where it stopped and stayed. For a robot that got through, the
last frame is the goal and shows nothing; the interesting moment is mid-gap.

So the frame is chosen explicitly rather than defaulted, and the caption is
burnt in. A still that travels into a slide deck or an artifact loses whatever
context sat around it in the report, and an unlabelled picture of a robot in a
corridor cannot be told from any other picture of a robot in a corridor.

palette.png is skipped: it is a 16x16 GIF palette that replay_render leaves in
the same directory, and sorting by name puts it last, where the final frame is
expected.
"""

import argparse
import glob
import os
import sys

from PIL import Image, ImageDraw, ImageFont


def pick(frames, at):
    if at == "last":
        return frames[-1]
    if at == "first":
        return frames[0]
    v = float(at)
    # A fraction of the run, or an absolute index. 0.75 means three quarters
    # through; 940 means frame 940. Anything below 1.0 is read as a fraction,
    # which makes frame 0 unreachable by number -- use "first" for that.
    idx = int(v * (len(frames) - 1)) if v < 1.0 else int(v)
    return frames[max(0, min(idx, len(frames) - 1))]


def font(size):
    """A font with Korean glyphs first, because the captions are Korean.

    DejaVu was first here and it has no Hangul: every caption rendered as a row
    of empty boxes, which is worse than no caption because it looks like the
    renderer failed rather than the font. The CJK collections are .ttc files
    holding several faces, and the Korean one is not index 0 -- SC, TC, JP and
    KR share the file. Pillow takes the index separately.
    """
    for path, index in (
            ("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 1),
            ("/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf", 0),
            ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 0)):
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size, index=index)
            except OSError:
                continue
    return ImageFont.load_default()


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("frames_dir")
    ap.add_argument("out")
    ap.add_argument("--caption", default="")
    ap.add_argument("--at", default="last")
    ap.add_argument("--width", type=int, default=800)
    ap.add_argument("--quality", type=int, default=82)
    a = ap.parse_args(argv if argv is not None else sys.argv[1:])

    frames = sorted(f for f in glob.glob(os.path.join(a.frames_dir, "*.png"))
                    if os.path.basename(f) != "palette.png")
    if not frames:
        print(f"프레임이 없습니다: {a.frames_dir}", file=sys.stderr)
        return 2

    src = pick(frames, a.at)
    im = Image.open(src).convert("RGB")
    h = round(im.height * a.width / im.width)
    im = im.resize((a.width, h), Image.LANCZOS)

    if a.caption:
        d = ImageDraw.Draw(im, "RGBA")
        f = font(max(15, a.width // 40))
        pad = a.width // 60
        box = d.textbbox((0, 0), a.caption, font=f)
        bh = box[3] - box[1] + pad * 2
        d.rectangle([0, h - bh, a.width, h], fill=(0, 0, 0, 190))
        d.text((pad, h - bh + pad - box[1]), a.caption, font=f, fill=(255, 255, 255))

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    im.save(a.out, quality=a.quality, optimize=True)
    print(f"  {os.path.basename(src)} ({len(frames)} 프레임 중) -> {a.out} "
          f"{os.path.getsize(a.out) // 1024} KB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
