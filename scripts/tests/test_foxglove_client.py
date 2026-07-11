#!/usr/bin/env python3
"""Protocol-level test for live_rerun.FoxgloveClient against a fake
foxglove_bridge — no robot needed.

    /usr/bin/python3 scripts/tests/test_foxglove_client.py
(or any python with `websockets` + `rosbags` installed)

The fake server advertises /tf and a compressed image channel, waits for the
client's subscribe request, then pushes CDR-encoded frames using the binary
MESSAGE_DATA layout. Asserts the client decodes them into rosbridge-shaped
dicts and that client-side throttling drops frames before decode.
"""
import json
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from websockets.sync.server import serve
from rosbags.typesys import Stores, get_typestore

import live_rerun  # noqa: E402  (scripts/ on sys.path)

TS = get_typestore(Stores.ROS2_JAZZY)


def make_tf_cdr(x: float) -> bytes:
    T = TS.types
    msg = T["tf2_msgs/msg/TFMessage"](transforms=[
        T["geometry_msgs/msg/TransformStamped"](
            header=T["std_msgs/msg/Header"](
                stamp=T["builtin_interfaces/msg/Time"](sec=7, nanosec=250),
                frame_id="odom"),
            child_frame_id="base_link",
            transform=T["geometry_msgs/msg/Transform"](
                translation=T["geometry_msgs/msg/Vector3"](x=x, y=-1.0, z=0.0),
                rotation=T["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0)),
        )])
    return bytes(TS.serialize_cdr(msg, "tf2_msgs/msg/TFMessage"))


def make_image_cdr(payload: bytes) -> bytes:
    import numpy as np
    T = TS.types
    msg = T["sensor_msgs/msg/CompressedImage"](
        header=T["std_msgs/msg/Header"](
            stamp=T["builtin_interfaces/msg/Time"](sec=8, nanosec=0),
            frame_id="camera_color_optical_frame"),
        format="jpeg",
        data=np.frombuffer(payload, dtype=np.uint8))
    return bytes(TS.serialize_cdr(msg, "sensor_msgs/msg/CompressedImage"))


def frame(sub_id: int, cdr: bytes) -> bytes:
    return b"\x01" + struct.pack("<I", sub_id) + struct.pack("<Q", 0) + cdr


def fake_bridge(ws):
    ws.send(json.dumps({"op": "serverInfo", "name": "fake", "capabilities": []}))
    ws.send(json.dumps({"op": "advertise", "channels": [
        {"id": 10, "topic": "/tf", "encoding": "cdr",
         "schemaName": "tf2_msgs/msg/TFMessage", "schema": ""},
        {"id": 11, "topic": "/camera/color/image_raw/compressed", "encoding": "cdr",
         "schemaName": "sensor_msgs/msg/CompressedImage", "schema": ""},
        {"id": 12, "topic": "/unrelated", "encoding": "cdr",
         "schemaName": "std_msgs/msg/String", "schema": ""},
    ]}))
    req = json.loads(ws.recv())
    assert req["op"] == "subscribe"
    sid = {s["channelId"]: s["id"] for s in req["subscriptions"]}
    assert set(sid) == {10, 11}, f"subscribed to wrong channels: {sid}"

    # 3 rapid TF frames (no throttle -> all delivered) ...
    for i in range(3):
        ws.send(frame(sid[10], make_tf_cdr(float(i))))
    # ... and 3 rapid image frames (min_period=0.5 -> only the first decoded).
    for _ in range(3):
        ws.send(frame(sid[11], make_image_cdr(b"JPEGDATA")))
    time.sleep(1.0)


def main() -> int:
    got_tf, got_img = [], []

    with serve(fake_bridge, "127.0.0.1", 0,
               subprotocols=["foxglove.websocket.v1"]) as server:
        threading.Thread(target=server.serve_forever, daemon=True).start()
        port = server.socket.getsockname()[1]
        client = live_rerun.FoxgloveClient("127.0.0.1", port)
        client.subscribe("/tf", got_tf.append)
        client.subscribe("/camera/color/image_raw/compressed", got_img.append,
                         min_period=0.5)
        t = threading.Thread(target=client.run_forever, daemon=True)
        t.start()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and (len(got_tf) < 3 or not got_img):
            time.sleep(0.05)

    assert len(got_tf) == 3, f"expected 3 tf msgs, got {len(got_tf)}"
    tf0 = got_tf[0]["transforms"][0]
    assert tf0["child_frame_id"] == "base_link"
    assert tf0["header"]["frame_id"] == "odom"
    assert tf0["header"]["stamp"] == {"sec": 7, "nanosec": 250}
    assert got_tf[2]["transforms"][0]["transform"]["translation"]["x"] == 2.0
    assert got_tf[0]["transforms"][0]["transform"]["translation"]["y"] == -1.0

    assert len(got_img) == 1, f"throttle failed: {len(got_img)} imgs decoded"
    assert got_img[0]["data"] == b"JPEGDATA"
    assert got_img[0]["format"] == "jpeg"
    assert got_img[0]["header"]["frame_id"] == "camera_color_optical_frame"

    print("PASS: FoxgloveClient decode + subscribe + throttle OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
