#!/usr/bin/env python3
"""Stream a rendered replay to Foxglove as an image topic.

    replay_to_foxglove.py media/replay/dwb_lane_0.90_r2
    replay_to_foxglove.py media/replay/dwb_lane_0.90_r2 --fps 5 --result results/...json

[TEST ONLY] Publishes /replay/image/compressed, /replay/info, and with
--raw-width also /replay/image_raw. Nothing else. It sends no goal, no
/initialpose and no velocity command, so it cannot move a robot, real or
simulated. Safe to run beside a live nav2.

Why this exists
---------------
replay_render.py leaves PNGs on the machine that ran the measurement. Opening
them needs a file browser on that machine, and whoever is reading the result is
usually somewhere else. Foxglove is already the remote window into this
workspace, so the frames go where the rest of the telemetry already goes.

The frames are published as CompressedImage. --encode png sends the file's own
bytes untouched, which is the honest default in the sense that what the reader
sees is the file rather than a re-render of it.

It is not the default here. A viewer that receives png and cannot decode it
draws nothing and reports nothing, and that failure looks exactly like a dead
publisher from the reading end -- an hour was spent proving the publisher was
alive when the bytes had been arriving the whole time. jpeg is decoded by every
build, so it is what goes out unless asked otherwise. The re-encode is lossy
and this stream is for looking at, not measuring; the measurements are in the
JSON the frames were rendered from.

Both topics are RELIABLE and TRANSIENT_LOCAL, because that is what
foxglove_bridge subscribes with. Textbook QoS for a video stream is VOLATILE
and BEST_EFFORT -- drop frames rather than queue them -- and both were tried
here. Either one is refused outright:

    New subscription discovered on topic '/replay/image/compressed',
    requesting incompatible QoS. No messages will be sent to it.
    Last incompatible policy: DURABILITY

A publisher weaker than its subscriber does not match, and silence is a worse
failure than a stale frame. So the QoS is chosen to match the bridge, not to
be right in the abstract.

The cost is real and worth knowing: a retained sample outlives the publisher
that made it. Start a different run and the first frame a newly connected
viewer sees may be the last frame of the previous one. /replay/info carries
the run's name and frame number, so the caption is what says which run is on
screen -- not the picture.

Frame size is where the stream is actually kept healthy. Reliable delivery of
1.1 MB/s of png stalls a viewer; the same run as jpeg is 33 KB a frame, and at
5 fps that is 165 KB/s.

Playback is a loop by default. A 37 s run rendered at stride 27 is 52 frames,
which is under three seconds at 20 fps -- too short to watch once. --fps is
playback rate, unrelated to the rate the run was recorded at, so it is the
knob for "slow enough to see".
"""

import argparse
import json
import pathlib
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


def parse_args(argv):
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("frames", help="directory of frame_*.png from replay_render.py")
    ap.add_argument("--topic", default="/replay/image",
                    help="base image topic; /compressed is appended")
    ap.add_argument("--info-topic", default="/replay/info")
    ap.add_argument("--fps", type=float, default=8.0, help="playback rate")
    ap.add_argument("--once", action="store_true", help="play once, then hold")
    ap.add_argument("--hold", type=float, default=1.0,
                    help="seconds to hold the last frame before looping")
    ap.add_argument("--encode", choices=("jpeg", "png"), default="jpeg",
                    help="jpeg re-encodes for viewers that cannot decode png")
    ap.add_argument("--quality", type=int, default=88, help="jpeg quality")
    ap.add_argument("--raw-width", type=int, default=0,
                    help="also publish uncompressed Image at this width, 0 to skip")
    ap.add_argument("--frame-id", default="replay")
    ap.add_argument("--result", default=None,
                    help="run JSON, to caption the stream with what it measured")
    return ap.parse_args(argv)


def caption(path):
    """One line describing the run, or None when there is no JSON to read."""
    if not path:
        return None
    run = json.load(open(path))
    rec = (run.get("records") or [{}])[0]
    bits = [f"{run.get('controller', '?')}", f"폭 {run.get('width', '?')} m",
            f"{rec.get('result', '?')}"]
    if rec.get("moved_m") is not None:
        bits.append(f"이동 {rec['moved_m']} m")
    if rec.get("remaining_m") is not None:
        bits.append(f"잔여 {rec['remaining_m']} m")
    if rec.get("clearance_min") is not None:
        bits.append(f"최소여유 {rec['clearance_min']} m")
    if run.get("course_stamp"):
        bits.append(f"스탬프 {run['course_stamp']}")
    return " | ".join(bits)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    frames = sorted(pathlib.Path(args.frames).glob("frame_*.png"))
    if not frames:
        print(f"프레임이 없습니다: {args.frames}/frame_*.png", file=sys.stderr)
        return 2
    if args.encode == "png":
        blobs = [f.read_bytes() for f in frames]
    else:
        try:
            import io

            from PIL import Image as PILImage
        except ImportError:
            print("jpeg 로 보내려면 Pillow 가 필요합니다. --encode png 를 쓰십시오.",
                  file=sys.stderr)
            return 2
        blobs = []
        for f in frames:
            buf = io.BytesIO()
            PILImage.open(f).convert("RGB").save(buf, "JPEG", quality=args.quality)
            blobs.append(buf.getvalue())

    # An uncompressed Image needs no decoder at all. It is the fallback for a
    # viewer whose Image panel draws nothing from CompressedImage and says why
    # to nobody. Downscaled, because rgb8 at full size is 1.55 MB a frame.
    raws = []
    if args.raw_width:
        from PIL import Image as PILImage2
        for f in frames:
            im = PILImage2.open(f).convert("RGB")
            h = round(im.height * args.raw_width / im.width)
            im = im.resize((args.raw_width, h))
            raws.append((im.width, im.height, im.tobytes()))

    try:
        head = caption(args.result)
    except (OSError, ValueError, KeyError) as exc:
        print(f"결과 JSON 을 읽지 못했습니다 ({exc}). 캡션 없이 계속합니다.",
              file=sys.stderr)
        head = None

    # Depth 1: a viewer that falls behind should see the newest frame, not a
    # backlog of stale ones.
    qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

    rclpy.init()
    node = Node("replay_to_foxglove")
    pub = node.create_publisher(CompressedImage, args.topic + "/compressed", qos)
    info = node.create_publisher(String, args.info_topic, qos)
    raw_pub = (node.create_publisher(Image, args.topic + "_raw", qos)
               if raws else None)

    name = pathlib.Path(args.frames).name
    avg = sum(len(b) for b in blobs) // len(blobs)
    print(f"  {name}: {len(blobs)} 프레임 -> {args.topic}/compressed "
          f"@ {args.fps} fps, {args.encode} 평균 {avg // 1024} KB"
          f"{chr(10) + '  ' + args.topic + '_raw: rgb8 ' + str(raws[0][0]) + 'x' + str(raws[0][1]) if raws else ''}"
          f"{'' if not args.once else ' (1회)'}", flush=True)
    if head:
        print(f"  {head}", flush=True)
    print("  Foxglove: Image 패널을 열고 위 토픽을 고르십시오. Ctrl-C 로 종료합니다.",
          flush=True)

    state = {"i": 0, "done": False}

    def tick():
        if state["done"]:
            return
        i = state["i"]
        msg = CompressedImage()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = args.frame_id
        msg.format = args.encode
        msg.data = blobs[i]
        pub.publish(msg)

        if raw_pub is not None:
            w, h, buf = raws[i]
            r = Image()
            r.header = msg.header
            r.height, r.width = h, w
            r.encoding = "rgb8"
            r.is_bigendian = 0
            r.step = w * 3
            r.data = buf
            raw_pub.publish(r)

        line = f"{name}  {i + 1}/{len(blobs)}"
        info.publish(String(data=f"{line}\n{head}" if head else line))

        state["i"] = i + 1
        if state["i"] >= len(blobs):
            if args.once:
                state["done"] = True
                print("  1회 재생 완료. 마지막 프레임을 유지합니다.", flush=True)
            else:
                state["i"] = 0

    node.create_timer(1.0 / max(args.fps, 0.1), tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
