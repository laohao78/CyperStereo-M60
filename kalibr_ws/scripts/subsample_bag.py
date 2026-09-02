#!/usr/bin/env python3
"""Subsample a ROS1 bag: keep 1 of every N messages per topic-prefix.
Messages pass through with their original ros1 serialization bytes.

Examples:
  # camera-only stage: keep every 8th /cam*, drop everything else
  python3 subsample_bag.py in.bag out.bag '{"default":8}' --prefixes /cam

  # imu-camera stage: /cam* every 3rd, /imu0 ALL
  python3 subsample_bag.py in.bag out.bag '{"default":3,"/imu0":1}' \
      --prefixes /cam /imu0
"""
import sys, json
from collections import Counter
from rosbags.rosbag1 import Reader, Writer

src, dst = sys.argv[1], sys.argv[2]
step_cfg = json.loads(sys.argv[3])          # topic-prefix -> step; "default" as fallback
prefixes = sys.argv[sys.argv.index("--prefixes") + 1:] if "--prefixes" in sys.argv else ["/cam"]

def step_for(topic):
    for p in prefixes:
        if topic.startswith(p):
            return step_cfg.get(p, step_cfg.get("default", 1))
    return None  # not a topic we keep

count, kept = Counter(), Counter()

with Reader(src) as r:
    keep = {c.topic: c for c in r.connections if step_for(c.topic) is not None}
    with Writer(dst) as w:
        newconn = {t: w.add_connection(
                        t, c.msgtype,
                        msgdef=(c.msgdef.data if hasattr(c.msgdef, "data") else c.msgdef),
                        md5sum=getattr(c, "digest", None),
                        callerid=c.ext.callerid, latching=c.ext.latching)
                   for t, c in keep.items()}
        for conn, ts, raw in r.messages():
            step = step_for(conn.topic)
            if step is None:
                continue
            count[conn.topic] += 1
            if step == 1 or count[conn.topic] % step == 1:  # step==1 => keep all
                w.write(newconn[conn.topic], ts, raw)
                kept[conn.topic] += 1
print("read:", dict(count))
print("kept:", dict(kept))
