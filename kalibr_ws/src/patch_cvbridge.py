#!/usr/bin/env python3
"""Insert `import cv2` before the first `import cv_bridge` in every kalibr
python tool source that uses cv_bridge.

Reason: on this arm64 image, cv_bridge's C++ module init calls
PyImport_ImportModule("cv2") from inside the extension init, which fails unless
cv2 was already imported by the interpreter ("initialization of cv_bridge_boost
raised unreported exception"). Pre-importing cv2 first fixes it.
"""
import pathlib
import re

root = pathlib.Path("/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python")
pat = re.compile(r"^(import cv_bridge|from cv_bridge import)")

for p in root.rglob("*"):
    if not p.is_file():
        continue
    try:
        text = p.read_text()
    except Exception:
        continue
    if "cv_bridge" not in text:
        continue
    out, done = [], False
    for ln in text.splitlines(keepends=True):
        if not done and pat.match(ln):
            out.append("import cv2  # noqa: preload before cv_bridge (arm64 quirk)\n")
            done = True
        out.append(ln)
    if done:
        p.write_text("".join(out))
        print("patched:", p)
