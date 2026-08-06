#!/usr/bin/env python3
"""Write the verified stamp, in a process that never played the stage.

    python3 stamp_verified.py <stage.usd>

Separate from verify_stage for one reason. verify_stage plays the stage, and
saving a layer after playing bakes whatever PhysX wrote back into it: the width
course came out of its own verification with base_link at (0.869, 8.703, 1.674)
instead of its spawn, which put the robot inside the 0.70 m lane, jammed against
a wall, and made every trial fail with "Start occupied" in 0.1 s.

A gate that corrupts what it is checking is worse than no gate. So the stamp is
written here, by a process that only ever opened the file.
"""
import sys
import time

from pxr import Sdf

path = sys.argv[1]
layer = Sdf.Layer.FindOrOpen(path)
if layer is None:
    raise SystemExit(f"could not open {path}")
data = dict(layer.customLayerData)
data["vica_verified"] = time.strftime("%Y-%m-%d %H:%M:%S")
layer.customLayerData = data
layer.Save()
print(f"  stamped verified: {data['vica_verified']}")
