#!/usr/bin/env python3
"""Where to spawn the robot for a given lane's trial.

    lane_spawn.py <course.json> <width>   ->   "x y"

A file rather than a python -c one-liner. The one-liner went through bash
double quotes, where \n is a literal backslash-n, so it was a syntax error that
printed to stderr and left the spawn empty -- which seeded AMCL with a blank
pose and stopped nav2 from ever coming up. Two trials were recorded as
"nav2 down" before anyone read the stderr.
"""
import json
import sys

# 0.8 m short of the lane mouth: long enough to be a real approach, short
# enough to be identical for every lane. Driving to each lane from one end of
# the course made the approach the variable instead of the lane.
RUN_IN = 0.8

spec = json.load(open(sys.argv[1]))
want = float(sys.argv[2])
lane = min(spec["lanes"], key=lambda l: abs(l["width"] - want))
print(f'{lane["entry"][0]:.4f} {lane["entry"][1] - RUN_IN:.4f}')
