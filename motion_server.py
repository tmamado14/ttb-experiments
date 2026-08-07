#!/usr/bin/env python3
"""
TurtleBot3 Motion Server + Web GUI
==================================
Runs on the laptop, talks to the TurtleBot3 over ROS 2 (DDS).
Serves a web page where you can:
  - see the robot camera (MJPEG stream)
  - drive the robot (teleop) with on-screen buttons or the keyboard

The robot side only needs its normal bringup running (camera publisher +
cmd_vel subscriber). Nothing new has to be installed on the robot.

Usage:
    python3 motion_server.py
    python3 motion_server.py --image-topic /image --cmd-vel-topic /cmd_vel --port 8000

Open http://localhost:8000  (or http://<laptop-ip>:8000 from another device).
"""

import argparse
import json
import math
import re
import socket
import threading
import time
from typing import Literal, Optional

import cv2
import httpx
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from pydantic import BaseModel

from seek import SeekBehaviour, SeekConfig, SeekParser


def clamp(value, lo, hi):
    """Constrain value to [lo, hi]. Every velocity and duration goes through this."""
    return max(lo, min(hi, value))


# --------------------------------------------------------------------------- #
# ROS 2 node
# --------------------------------------------------------------------------- #
class MotionNode(Node):
    def __init__(self, image_topic, cmd_vel_topic, image_type="auto",
                 robot_ip=None, cmd_timeout=0.4, odom_topic="/odom",
                 scan_topic="/scan"):
        super().__init__("motion_server")
        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.odom_topic = odom_topic
        self.scan_topic = scan_topic
        self.cmd_timeout = cmd_timeout

        # Live network round-trip to the robot (dominant part of teleop latency).
        self.link_rtt_ms = None
        if robot_ip:
            threading.Thread(target=self._latency_loop, args=(robot_ip,),
                             daemon=True).start()

        # Latest camera frame (BGR) protected by a lock.
        self._frame_lock = threading.Lock()
        self._frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(self._frame, "waiting for camera...", (120, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        self.frames_received = 0

        # Laser scan, used by seek to turn "I can see it" into "it is 1.4 m away".
        self._scan_lock = threading.Lock()
        self._scan = None          # the LaserScan message itself
        self._scan_time = 0.0      # time.monotonic() when it arrived
        self.scan_received = 0
        self._scan_dt = 0.0

        # Target velocity + deadman timestamp.
        self._vel_lock = threading.Lock()
        self._target = Twist()
        self._last_cmd_time = time.time()

        # Odometry: where the robot thinks it is. Closed-loop distance/angle
        # goals measure progress from this instead of trusting a stopwatch.
        self._odom_lock = threading.Lock()
        self._odom = None          # (x, y, yaw_unwrapped)
        self._odom_time = 0.0      # time.monotonic() when it arrived
        self._yaw_raw = None       # previous atan2 yaw, for delta unwrapping
        self._yaw_acc = 0.0        # continuous yaw, free to run past +/-pi
        self.odom_received = 0
        self.odom_broken = False   # set on a pose discontinuity (odom reset)
        self._odom_dt = 0.0        # smoothed sample period, for the rate readout

        # Sensor data is usually best-effort; command output is reliable.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Determine whether the image topic is raw or compressed.
        if image_type == "compressed":
            msg_type = CompressedImage
        elif image_type == "raw":
            msg_type = Image
        else:
            msg_type = self._detect_image_type(image_topic)
        if msg_type is CompressedImage:
            self.create_subscription(
                CompressedImage, image_topic, self._compressed_cb, sensor_qos)
            self.get_logger().info(f"Subscribed to {image_topic} (CompressedImage)")
        else:
            self.create_subscription(
                Image, image_topic, self._raw_cb, sensor_qos)
            self.get_logger().info(f"Subscribed to {image_topic} (Image)")

        # Odometry. A BEST_EFFORT subscriber is compatible with the robot's
        # RELIABLE publisher, and it's what a control loop actually wants:
        # over Wi-Fi, RELIABLE means retransmits, and a late-but-complete pose
        # is worse than a fresh one. depth=1 keeps only the newest sample.
        self.create_subscription(Odometry, odom_topic, self._odom_cb, sensor_qos)
        self.get_logger().info(f"Subscribed to {odom_topic} (Odometry)")

        # Laser scan. The same BEST_EFFORT profile is not optional here: the
        # LDS-01 publishes BEST_EFFORT, so a default (RELIABLE) subscriber is
        # QoS-incompatible and receives *nothing* -- rclpy logs
        # "incompatible QoS ... No messages will be received" and otherwise
        # looks exactly like a robot with its lidar switched off.
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, sensor_qos)
        self.get_logger().info(f"Subscribed to {scan_topic} (LaserScan)")

        # Publish target velocity at 20 Hz; stop if no fresh command (deadman).
        self.create_timer(0.05, self._publish_cmd)
        self.get_logger().info(f"Publishing Twist on {cmd_vel_topic}")

    def _detect_image_type(self, topic):
        """Look at the ROS graph for a few seconds to find the topic's type."""
        for _ in range(20):
            for name, types in self.get_topic_names_and_types():
                if name == topic:
                    if any("CompressedImage" in t for t in types):
                        return CompressedImage
                    if any("sensor_msgs/msg/Image" in t for t in types):
                        return Image
            time.sleep(0.25)
        self.get_logger().warn(
            f"Topic {topic} not seen yet; assuming raw Image. "
            "Start the robot camera, or pass --image-type compressed.")
        return Image

    # --- camera callbacks ------------------------------------------------- #
    def _store(self, frame):
        with self._frame_lock:
            self._frame = frame
            self.frames_received += 1

    def _raw_cb(self, msg):
        try:
            self._store(self.bridge.imgmsg_to_cv2(msg, "bgr8"))
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"raw image decode failed: {e}")

    def _compressed_cb(self, msg):
        try:
            arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self._store(frame)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"compressed image decode failed: {e}")

    def get_jpeg(self):
        with self._frame_lock:
            frame = self._frame.copy()
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return buf.tobytes() if ok else None

    def get_frame(self):
        """(BGR copy, sequence number) for the detector.

        The sequence number is what lets the seek loop tick faster than the
        camera without paying for inference twice on one frame: the feed is
        ~7 Hz over Wi-Fi while the control loop wants to run at 10-20 Hz, and
        get_jpeg()-style access alone cannot tell a fresh frame from a repeat.
        Returns BGR rather than JPEG because the detector would only have to
        decode it again.
        """
        with self._frame_lock:
            return self._frame.copy(), self.frames_received

    def frame_width(self):
        """Width of the published frame, in pixels.

        Read rather than assumed: this camera publishes 480x640 portrait (the
        publisher rotates 90 degrees to correct a rotated mount), and centring
        divides by exactly this number.
        """
        with self._frame_lock:
            return self._frame.shape[1]

    # --- odometry --------------------------------------------------------- #
    # Largest yaw jump between samples we'll believe. At 20 Hz a Burger turning
    # flat out moves 2.84/20 = 0.14 rad per sample, and even after half a second
    # of dropped samples (our staleness limit) only 1.4 rad. Anything past 2.0
    # is the robot teleporting, not turning.
    MAX_YAW_STEP = 2.0

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)          # (-pi, pi]

        with self._odom_lock:
            # Unwrap here, at the source, rather than in the control loop: the
            # per-sample delta is tiny compared to pi, so the wrap correction is
            # never ambiguous. That turns yaw into one continuous signal any
            # consumer can snapshot and subtract from -- which is what lets a
            # "rotate 540 degrees" goal keep counting past half a turn instead
            # of folding back at pi and never converging.
            if self._yaw_raw is not None:
                d = yaw - self._yaw_raw
                if d > math.pi:
                    d -= 2.0 * math.pi
                elif d < -math.pi:
                    d += 2.0 * math.pi
                if abs(d) > self.MAX_YAW_STEP:
                    # An odom reset, not a rotation. Unwrapping it would quietly
                    # corrupt the accumulator, so flag it and let the control
                    # loop abort instead.
                    self.odom_broken = True
                else:
                    self._yaw_acc += d
            self._yaw_raw = yaw
            self._odom = (p.x, p.y, self._yaw_acc)
            now = time.monotonic()
            if self._odom_time:
                dt = now - self._odom_time
                self._odom_dt = dt if not self._odom_dt else \
                    0.9 * self._odom_dt + 0.1 * dt
            self._odom_time = now
            self.odom_received += 1

    def odom_stats(self):
        """(age_s | None, hz | None, count) for the /status readout.

        Exposed so the decision to keep the executor single-threaded stays a
        measured one: if odom age starts climbing, that's the signal to split
        the camera decode off its callback group.
        """
        with self._odom_lock:
            if self._odom is None:
                return None, None, self.odom_received
            age = time.monotonic() - self._odom_time
            hz = round(1.0 / self._odom_dt, 1) if self._odom_dt > 0 else None
            return age, hz, self.odom_received

    def get_odom(self):
        """(x, y, yaw_unwrapped, age_s), or None if odom has never arrived.

        Age is measured against time.monotonic() on this laptop, deliberately
        not the message header: the robot's clock isn't synced to ours, so a
        header-based age would be wrong by however far the two clocks drift.
        """
        with self._odom_lock:
            if self._odom is None:
                return None
            x, y, yaw = self._odom
            return x, y, yaw, time.monotonic() - self._odom_time

    # --- laser scan ------------------------------------------------------- #
    # Fraction of the sector we allow to be nearer than the reported range.
    # See front_range() for why this is not a median.
    SCAN_PCT = 0.20

    # Sectors to try, narrowest first, as (halfwidth_deg, minimum valid beams).
    # A second, wider tier because thin targets genuinely disappear from a tight
    # sector: centred on a chair 2.6 m away this robot read 0 valid beams out of
    # 11 at +/-5 degrees and 3 out of 21 at +/-10 -- the pedestal base is mostly
    # gaps at that range and everything behind it was past the 3.5 m limit, so a
    # single narrow sector refused to range a target sitting in plain view.
    #
    # Widening is safe in the direction that matters: the reading is a robust
    # MINIMUM, so a wider sector can only report something nearer, and erring
    # near means stopping early. It cannot cause a late stop. The cost is
    # precision -- at +/-20 degrees and 2.6 m the sector is about 1.8 m across,
    # so the range may describe a doorframe beside the target rather than the
    # target. Hence narrowest-first: the wide tier is a fallback, not the norm.
    SCAN_TIERS = ((10.0, 3), (20.0, 6))

    def _scan_cb(self, msg):
        with self._scan_lock:
            self._scan = msg
            now = time.monotonic()
            if self._scan_time:
                dt = now - self._scan_time
                self._scan_dt = dt if not self._scan_dt else \
                    0.9 * self._scan_dt + 0.1 * dt
            self._scan_time = now
            self.scan_received += 1

    def scan_stats(self):
        """(age_s | None, hz | None, count), mirroring odom_stats()."""
        with self._scan_lock:
            if self._scan is None:
                return None, None, self.scan_received
            age = time.monotonic() - self._scan_time
            hz = round(1.0 / self._scan_dt, 1) if self._scan_dt > 0 else None
            return age, hz, self.scan_received

    def front_range(self, tiers=None):
        """(range_m, age_s) straight ahead, or (None, age_s) if unreadable.

        Deliberately a robust MINIMUM (20th percentile of the valid beams in
        the sector), not a median. Measured on this robot, a front sector of
        +/-10 degrees read min 1.97 m against a median of 2.60 m: the median was
        describing the wall behind the object while something sat two thirds of
        a metre nearer. For "stop before you hit it" the nearest thing in the
        path is the only correct quantity, and a median would have driven us
        into it. A percentile rather than a bare min() because single spurious
        short returns are common and would stop the robot early.

        Invalid returns are dropped, not clamped. The LDS-01 reports a dropout
        as 0.0, and a live sample had 251/360 beams valid with 0.0 sitting at
        index 0 -- dead ahead -- so treating one beam as gospel, or reading a
        dropout as "zero metres away", are both real failure modes here.
        """
        with self._scan_lock:
            msg, t = self._scan, self._scan_time
        if msg is None:
            return None, None
        age = time.monotonic() - t

        n = len(msg.ranges)
        if not n:
            return None, age
        # index == degrees CCW from forward on this unit (angle_min 0.0,
        # increment 1 degree), but derive it from the header anyway so a
        # different lidar doesn't silently read a random direction.
        inc = msg.angle_increment or (2.0 * math.pi / n)
        zero = int(round(-msg.angle_min / inc))
        lo, hi = msg.range_min, msg.range_max

        for halfwidth_deg, min_beams in (tiers or self.SCAN_TIERS):
            span = int(round(math.radians(halfwidth_deg) / abs(inc)))
            vals = []
            for k in range(-span, span + 1):
                r = msg.ranges[(zero + k) % n]
                if math.isfinite(r) and lo <= r <= hi and r > 0.0:
                    vals.append(r)
            if len(vals) >= min_beams:
                vals.sort()
                return vals[int(self.SCAN_PCT * (len(vals) - 1))], age
        return None, age

    # --- teleop ----------------------------------------------------------- #
    def set_velocity(self, linear, angular):
        with self._vel_lock:
            self._target.linear.x = float(linear)
            self._target.angular.z = float(angular)
            self._last_cmd_time = time.time()

    def stop(self):
        self.set_velocity(0.0, 0.0)

    def _latency_loop(self, robot_ip, port=22, interval=2.0):
        """Measure network round-trip to the robot via a quick TCP connect."""
        while True:
            t0 = time.time()
            try:
                with socket.create_connection((robot_ip, port), timeout=1.5):
                    self.link_rtt_ms = round((time.time() - t0) * 1000, 1)
            except OSError:
                self.link_rtt_ms = None
            time.sleep(interval)

    def _publish_cmd(self):
        with self._vel_lock:
            if time.time() - self._last_cmd_time > self.cmd_timeout:
                # Deadman: no recent command -> hold still.
                self._target = Twist()
            self.cmd_pub.publish(self._target)


# --------------------------------------------------------------------------- #
# Natural-language command parsing (local LLM via ollama)
# --------------------------------------------------------------------------- #
ACTIONS = ("forward", "backward", "rotate_left", "rotate_right", "stop", "unknown")
MODES = ("duration", "distance", "angle")

# Which modes make sense for which actions. You cannot drive forward by a number
# of degrees, and you cannot rotate by a number of meters.
LINEAR_ACTIONS = ("forward", "backward")
ANGULAR_ACTIONS = ("rotate_left", "rotate_right")

# Units each mode measures in, for messages and the GUI readout.
MODE_UNIT = {"duration": "s", "distance": "m", "angle": "deg"}

# --------------------------------------------------------------------------- #
# Splitting a chained command into steps
# --------------------------------------------------------------------------- #
# A chain ("forward 0.5 m, turn right 90, forward 0.3 m") is split HERE, in
# Python, rather than by asking the model for an array of steps. Three reasons:
#
#   1. Failures stay attributable. Holding the fragment lets us say
#      'step 2 of 3 ("trun rihgt 90"): ...'. If the model chose the
#      decomposition there would be no span of the user's text to quote back.
#   2. The prompt and schema below stay byte-identical, so the single-command
#      behaviour they were verified against is provably unchanged, not
#      probably fine.
#   3. Under constrained decoding, array length is effectively unbounded
#      (maxItems support is inconsistent), which would make the step cap a
#      post-hoc check on whatever the model felt like emitting instead of a
#      property of the user's punctuation.
_STEP_SEP = re.compile(
    r"[;\n]"                                     # explicit separators
    r"|,"                                        # comma
    r"|\.(?=\s|$)"                               # sentence-final period. Safe
                                                 # without a lookbehind: a
                                                 # decimal point is never
                                                 # followed by whitespace, so
                                                 # "0.5" survives intact.
    r"|\band\s+then\b|\bthen\b|\bafter\s+that\b|\bnext\b",
    re.IGNORECASE)

# Deliberately NOT a separator: a bare "and". "go forward a meter and a half"
# is one of the few-shot examples below and parses correctly today; splitting
# on "and" would turn it into ["go forward a meter", "a half"] and break it.
# "and then" is a separator, "and" alone is not.
_MOTION_WORDS = re.compile(
    r"\b(forward|forwards|ahead|straight|backward|backwards|back|reverse|"
    r"turn|rotate|spin|pivot|left|right|stop|halt|go|drive|move|around)\b",
    re.IGNORECASE)


def split_steps(text):
    """Free text -> ordered list of single-clause step texts.

    Returns [] for empty input and a one-element list when there is no
    separator -- which is what keeps a plain single command on exactly the code
    path it has always been on.
    """
    parts = [p.strip(" \t,.;") for p in _STEP_SEP.split(text or "")]
    parts = [p for p in parts if p]
    if not parts:
        return []

    # A fragment with no motion word in it isn't a step, it's a qualifier that
    # belongs to its neighbour. Without this, "move forward 1 m, slowly" becomes
    # a two-step chain whose second step is "slowly", parses as unknown, and
    # gets the whole chain refused -- a regression on a phrase that works today.
    if len(parts) > 1 and not _MOTION_WORDS.search(parts[0]):
        parts[1] = parts[0] + ", " + parts[1]     # leading qualifier
        parts = parts[1:]
    steps = [parts[0]]
    for p in parts[1:]:
        if _MOTION_WORDS.search(p):
            steps.append(p)
        else:
            steps[-1] += ", " + p                 # trailing qualifier
    return steps

# Constrains the model's output at the decoder level, so it can only ever emit
# one of these actions. Values are still re-validated and clamped below --- the
# model is an intent parser, never a safety layer.
#
# One tagged number ("mode" says what unit "value" is in) rather than separate
# optional distance_m/angle_deg fields. Under constrained decoding an optional
# property is a coin flip, so all three would end up required --- and a 3B model
# handed three mutually exclusive numbers will dutifully fill in all three,
# inventing a duration for "move forward 1 meter" purely because the slot exists.
#
# Property order is load-bearing: the decoder emits properties in the order
# listed, so putting "mode" before "value" makes the model commit to the unit
# before it writes the number. Reversed, it picks a number blind and then
# rationalises a unit for it.
NL_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "mode": {"type": "string", "enum": list(MODES)},
        "value": {"type": "number"},
        "speed": {"type": "string", "enum": ["slow", "normal", "fast"]},
    },
    "required": ["action", "mode", "value", "speed"],
}

# Degrees rather than radians, and meters rather than centimeters, because those
# are the units the model has actually seen in this context. Asking a 3B model
# for 1.5708 invites failure; math.radians() on our side cannot fail.
NL_SYSTEM_PROMPT = """\
You control a TurtleBot3 robot. Convert the user's driving instruction into JSON.

action: forward | backward | rotate_left | rotate_right | stop | unknown
  Use 'unknown' ONLY if the text is not a driving instruction.
mode: how the amount is measured.
  duration - the user gave a time      -> value is SECONDS (max {max_duration})
  distance - the user gave a length    -> value is METERS  (max {max_distance})
  angle    - the user gave a turn size -> value is DEGREES (max {max_angle})
  If the user gave no amount at all, use duration with value 2.
value: one number, in the unit chosen by mode. Convert to that unit yourself:
  centimeters/cm -> meters (50 cm = 0.5), feet -> meters (1 ft = 0.3),
  "a half" = 0.5, a quarter turn = 90, a half turn = 180, a full turn = 360.
mode must match the action: forward and backward take duration or distance,
never angle. rotate_left and rotate_right take duration or angle, never distance.
speed: slow | normal | fast. Default normal.

Examples:
  "move forward"                  -> forward, duration, 2, normal
  "back up slowly"                -> backward, duration, 2, slow
  "rotate left 3 seconds"         -> rotate_left, duration, 3, normal
  "move forward 1 meter"          -> forward, distance, 1, normal
  "go forward a meter and a half" -> forward, distance, 1.5, normal
  "back up 50 cm"                 -> backward, distance, 0.5, normal
  "drive forward 2 feet"          -> forward, distance, 0.6, normal
  "rotate right 30 degrees"       -> rotate_right, angle, 30, normal
  "turn left 90 deg fast"         -> rotate_left, angle, 90, fast
  "spin right a quarter turn"     -> rotate_right, angle, 90, normal
  "turn around"                   -> rotate_left, angle, 180, normal
  "halt"                          -> stop, duration, 0, normal
"""


class Intent(BaseModel):
    """Validated result of a parse. Pydantic rejects anything off-enum.

    Types and enums only. Whether the mode makes sense for the action is checked
    in /nl, not here: a validator raising at this point would surface as "the
    language model returned something unusable", which is a baffling thing to
    tell someone who typed a perfectly clear sentence the model merely mis-tagged.
    """
    action: Literal["forward", "backward", "rotate_left",
                    "rotate_right", "stop", "unknown"]
    mode: Literal["duration", "distance", "angle"] = "duration"
    value: float = 0.0
    speed: Literal["slow", "normal", "fast"] = "normal"


class NLRequest(BaseModel):
    """Body of POST /nl."""
    text: str = ""


class NLParser:
    """Turns free text into an Intent using a local ollama model."""

    def __init__(self, url, model, max_duration, max_distance, max_angle,
                 timeout=30.0):
        self.url = url.rstrip("/")
        self.model = model
        self.max_duration = max_duration
        self.max_distance = max_distance
        self.max_angle = max_angle
        self._client = httpx.Client(timeout=timeout)
        self._system = NL_SYSTEM_PROMPT.format(max_duration=f"{max_duration:g}",
                                               max_distance=f"{max_distance:g}",
                                               max_angle=f"{max_angle:g}")

    def parse(self, text):
        """Return (Intent, None) on success or (None, error_message) on failure."""
        body = {
            "model": self.model,
            "stream": False,
            "keep_alive": "10m",          # avoid a ~16 s cold reload per command
            "options": {"temperature": 0},
            "format": NL_SCHEMA,
            "messages": [
                {"role": "system", "content": self._system},
                {"role": "user", "content": text},
            ],
        }
        try:
            r = self._client.post(f"{self.url}/api/chat", json=body)
            r.raise_for_status()
            return Intent(**json.loads(r.json()["message"]["content"])), None
        except httpx.HTTPError as e:
            return None, f"cannot reach the language model at {self.url} ({type(e).__name__})"
        except Exception as e:  # noqa: BLE001 - malformed/unparseable model output
            return None, f"the language model returned something unusable ({type(e).__name__})"


class MotionExecutor:
    """Runs one bounded, cancellable motion at a time.

    A natural-language command is a single discrete instruction, but the node's
    0.4 s deadman (see MotionNode._publish_cmd) stops the robot unless commands
    keep arriving. So this re-asserts the target velocity on a short cycle until
    the goal is met, then stops. The deadman stays in place underneath as the
    backstop: if this thread dies, the robot halts within 0.4 s.

    Three kinds of goal:
      duration  open-loop on the wall clock. The only one that works without
                odometry, and so the fallback when /odom is unavailable.
      distance  closed-loop on /odom position, in meters.
      angle     closed-loop on /odom unwrapped yaw, in degrees.

    The control loop deliberately lives on its own thread and touches the node
    only through set_velocity(). It must never move into the ROS timer callback:
    a bug in here would then wedge the deadman publisher itself.
    """

    SPEED_FRAC = {"slow": 0.4, "normal": 0.7, "fast": 1.0}

    # Closed-loop control period. Odometry arrives at 20 Hz and the command
    # publisher runs at 20 Hz, so 0.05 s adds no avoidable latency; at 0.1 s the
    # decision lag alone would cost ~11 degrees of overshoot on a normal-speed
    # turn. Anything above ~0.2 s would start tripping the 0.4 s deadman.
    TICK = 0.05
    TICK_DURATION = 0.1          # open-loop re-assert period (unchanged)

    # Speed floor. A Burger below roughly 0.01 m/s or 0.1 rad/s buzzes without
    # moving, so we never command less than about twice that.
    MIN_LIN = 0.02               # m/s
    MIN_ANG = 0.15               # rad/s

    TOL_LIN = 0.01               # m    "close enough to stop"
    TOL_ANG = math.radians(2.0)
    RAMP_LIN = 0.10              # m    start easing off this far out
    RAMP_ANG = math.radians(25.0)

    ODOM_STALE = 0.5             # s    abort if odom goes quiet this long
    STALL_WINDOW = 1.5           # s    progress watchdog window
    STALL_LIN = 0.002            # m    minimum progress within that window
    STALL_ANG = math.radians(1.0)
    TIMEOUT_FACTOR = 3.0         # x ideal time...
    TIMEOUT_MARGIN = 2.0         # s ...plus this

    # Pause between chained steps, so the next one reads a start pose the robot
    # is actually at. The robot itself stops in well under 50 ms from the ramp's
    # floor speed -- what this really waits out is odometry latency, since
    # get_odom() returns the last sample RECEIVED, up to a 20 Hz period plus
    # Wi-Fi transport old. 0.35 s lets several fresh samples land after the
    # robot is genuinely still, and stays under the 0.4 s deadman so a healthy
    # chain never enters deadman territory.
    SETTLE = 0.35

    def __init__(self, node: MotionNode, max_lin, max_ang, max_duration,
                 max_distance=2.0, max_angle=360.0, goal_timeout_max=60.0):
        self._node = node
        self._max_lin = max_lin
        self._max_ang = max_ang
        self._max_duration = max_duration
        self._max_distance = max_distance
        self._max_angle = max_angle
        self._goal_timeout_max = goal_timeout_max
        self._lock = threading.Lock()
        self._thread = None
        self._run_state = None
        self._active = None
        self._ends_at = 0.0
        self._progress = 0.0      # in display units (s, m or deg)
        self._last_result = None

    # --- lifecycle -------------------------------------------------------- #
    def _abort(self, stop_on_exit):
        """Cancel any running motion and wait briefly for the thread to exit."""
        with self._lock:
            state, thread = self._run_state, self._thread
        if state is not None:
            state["stop_on_exit"] = stop_on_exit
            state["cancel"].set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.5)

    def cancel(self, stop=True):
        """Abort the current motion. stop=False when the caller is about to
        issue its own velocity (manual teleop preempting a typed command)."""
        self._abort(stop_on_exit=stop)
        if stop:
            self._node.stop()

    @classmethod
    def _timeout_for(cls, goal, cruise, hard_max):
        """Wall-clock ceiling for a closed-loop goal.

        This is what keeps "every motion ends by itself" true once the stopping
        condition depends on the robot actually moving. Three times the ideal
        time plus a margin covers the ramp tail (crossed at roughly half cruise)
        and a robot running slow on carpet or a flat battery, while still
        bounding how long a stuck one can grind.
        """
        if goal <= 0.0 or cruise <= 0.0:
            return 0.0
        return clamp(goal / cruise * cls.TIMEOUT_FACTOR + cls.TIMEOUT_MARGIN,
                     0.0, hard_max)

    def _plan_step(self, action, speed, mode="duration", value=0.0):
        """Work out (info, spec) for one motion without starting anything.

        Split out from start() so the HTTP layer can price a whole chain against
        the exact numbers the executor will run, rather than a reimplementation
        that can drift.

        `value` is in the unit implied by `mode`: seconds, meters or degrees.
        """
        frac = self.SPEED_FRAC.get(speed, 0.7)
        lin = ang = 0.0
        if action == "forward":
            lin = self._max_lin * frac
        elif action == "backward":
            lin = -self._max_lin * frac
        elif action == "rotate_left":
            ang = self._max_ang * frac
        elif action == "rotate_right":
            ang = -self._max_ang * frac
        lin = clamp(lin, -self._max_lin, self._max_lin)
        ang = clamp(ang, -self._max_ang, self._max_ang)

        # Clamp the goal again even though /nl already did. Same belt-and-braces
        # the duration cap has always had -- this method is reachable from
        # anywhere, and a cap that only lives at the HTTP edge isn't a cap.
        if mode == "distance":
            shown = clamp(float(value), 0.0, self._max_distance)
            goal, cruise, floor, to_disp = shown, abs(lin), self.MIN_LIN, 1.0
        elif mode == "angle":
            shown = clamp(float(value), 0.0, self._max_angle)
            goal = math.radians(shown)
            cruise, floor, to_disp = abs(ang), self.MIN_ANG, 180.0 / math.pi
        else:
            mode = "duration"
            shown = clamp(float(value), 0.0, self._max_duration)
            goal, cruise, floor, to_disp = shown, 0.0, 0.0, 1.0

        timeout = (goal if mode == "duration"
                   else self._timeout_for(goal, cruise, self._goal_timeout_max))

        info = {"action": action, "speed": speed, "mode": mode,
                "goal": round(shown, 3), "unit": MODE_UNIT[mode],
                "lin": round(lin, 3), "ang": round(ang, 3),
                "duration_s": round(shown if mode == "duration" else 0.0, 3),
                "timeout_s": round(timeout, 2)}
        spec = {"mode": mode, "goal": goal, "cruise": cruise, "floor": floor,
                "timeout": timeout, "to_disp": to_disp, "lin": lin, "ang": ang}
        return info, spec

    def plan(self, steps):
        """[(action, speed, mode, value), ...] -> [(info, spec), ...]."""
        return [self._plan_step(*s) for s in steps]

    def start(self, action, speed, mode="duration", value=0.0):
        """Begin a single motion. Signature and return shape unchanged."""
        return self.start_planned(self.plan([(action, speed, mode, value)]))

    def start_planned(self, planned):
        """Run a chain of pre-planned steps on ONE thread under ONE cancel event.

        The whole chain deliberately shares a single cancel event, so the three
        existing preemption sites (/cmd, /stop, and a typed "stop") abort the
        entire sequence without knowing sequences exist. The alternatives --
        starting the next step from _run's finally, or a separate runner calling
        start() per step -- race _abort(): it sets cancel, then joins with a
        timeout, so a thread that spawns its successor can leave _abort
        returning with a brand new motion already running. That is the STOP
        button failing to stop the robot, so it is not a style choice.
        """
        infos = [dict(i, step=n + 1, steps=len(planned))
                 for n, (i, _) in enumerate(planned)]
        specs = [sp for _, sp in planned]

        # Replace any in-flight motion; don't stop, we're about to drive.
        self._abort(stop_on_exit=False)

        runnable = [sp["goal"] > 0.0 and (sp["mode"] == "duration"
                                          or sp["cruise"] > 0.0) for sp in specs]
        if not any(runnable):
            # Nothing to do. Don't spawn a thread just to have it exit at once.
            self._node.stop()
            return dict(infos[0], started=False)

        state = {"cancel": threading.Event(), "stop_on_exit": True,
                 # infos live in state, not on self: a thread that outlived its
                 # join must never read a newer chain's plan.
                 "infos": infos}
        thread = threading.Thread(target=self._run, args=(state, specs), daemon=True)
        with self._lock:
            self._run_state = state
            self._thread = thread
            self._active = dict(infos[0])
            self._ends_at = time.monotonic() + infos[0]["timeout_s"]
            self._progress = 0.0
        thread.start()
        return dict(infos[0], started=True)

    def _set_progress(self, state, shown):
        """Publish progress, but only while we're still the current motion.

        No step bookkeeping needed here: `state` is per-CHAIN, so this guard
        already means "still the current chain".
        """
        with self._lock:
            if self._run_state is state:
                self._progress = shown

    # --- hooks used by SeekBehaviour -------------------------------------- #
    # Public wrappers rather than letting seek.py reach into privates, so the
    # "only the current motion may touch shared state" guard is enforced in one
    # place no matter who is driving.
    set_progress = _set_progress

    def update_active(self, state, **fields):
        """Merge live behaviour detail into the status readout."""
        with self._lock:
            if self._run_state is state and self._active is not None:
                self._active.update(fields)

    def set_goal(self, state, goal):
        """Set the denominator of the progress bar once it becomes knowable.

        A seek cannot state its goal up front the way "forward 1 m" can -- how
        far it has to drive isn't known until the target has been found and
        ranged -- so the bar stays empty until the approach begins.
        """
        with self._lock:
            if self._run_state is state and self._active is not None:
                self._active["goal"] = round(goal, 3)
                self._progress = 0.0

    def rotate_relative(self, state, degrees, action, speed="slow"):
        """Turn by a fixed amount using the existing closed-loop angle goal.

        Reused rather than reimplemented so the search sweep inherits the ramp,
        the speed floor, the stall watchdog and the timeout that the typed
        "rotate left 25 degrees" already has.
        """
        _, spec = self._plan_step(action, speed, "angle", degrees)
        if spec["goal"] <= 0.0 or spec["cruise"] <= 0.0:
            return "done"
        return self._run_one(state, spec)

    def _begin_step(self, state, idx):
        """Hand the readout to the next step of a chain.

        _active is mutated, never nulled, so status()["running"] stays true for
        the whole chain and the GUI's fast poller -- which tears itself down the
        moment it sees running go false -- survives the step boundary.
        """
        info = state["infos"][idx]
        with self._lock:
            if self._run_state is not state:
                return
            self._active = dict(info)
            self._ends_at = time.monotonic() + info["timeout_s"]
            self._progress = 0.0

    def _run(self, state, specs):
        """Run each step in turn, abandoning the rest if one doesn't finish."""
        t_chain = time.monotonic()
        reason, last = "done", 0
        try:
            for idx, spec in enumerate(specs):
                if idx:
                    # Come to rest before the next step reads its start pose.
                    # Commanded explicitly rather than left to the deadman --
                    # waiting on that would mean 0.4 s of continued motion at
                    # the last velocity, which is the opposite of settling.
                    self._node.stop()
                    # wait(), never sleep(): a sleep here would be a window in
                    # which STOP does nothing while _abort's join still returns.
                    if state["cancel"].wait(self.SETTLE):
                        reason = "cancelled"
                        break
                    self._begin_step(state, idx)
                # Track the step that actually ran separately from the loop
                # index: if a cancel lands during the settle above, idx has
                # already moved to a step that never started, and reporting it
                # would pair that step's goal with the previous step's progress.
                last = idx
                reason = self._run_one(state, spec)
                if reason != "done":
                    # A step that ended any other way leaves the robot somewhere
                    # the remaining steps weren't written for -- a turn that
                    # stopped short means the next "forward" goes the wrong way.
                    break
        finally:
            spec = specs[last]
            to_disp = spec["to_disp"]
            if state["stop_on_exit"]:
                self._node.stop()
            with self._lock:
                # Only the current motion may touch shared state: an older
                # thread that outlived its join must not clobber a newer one.
                if self._run_state is state:
                    self._last_result = {
                        "reason": reason, "mode": spec["mode"],
                        "goal": round(spec["goal"] * to_disp, 3),
                        "progress": round(self._progress, 3),
                        "unit": MODE_UNIT[spec["mode"]],
                        "elapsed_s": round(time.monotonic() - t_chain, 2),
                        "step": last + 1, "steps": len(specs),
                        "action": state["infos"][last]["action"],
                    }
                    self._run_state = None
                    self._thread = None
                    self._active = None

    # --- seek ------------------------------------------------------------- #
    @property
    def max_ang(self):
        return self._max_ang

    @property
    def approach_cruise(self):
        """Forward speed while closing on a target.

        Half of maximum: the stopping decision depends on a lidar sample and a
        control tick, so the faster this is, the further the robot travels
        after the range says stop. At 0.11 m/s one 20 Hz tick is 5 mm.
        """
        return self._max_lin * 0.5

    def start_seek(self, behaviour, timeout_s):
        """Run a seek on the same one-thread, one-cancel-event footing as a chain.

        Everything that makes STOP work for typed motions -- _abort setting the
        cancel event and joining, /cmd and /stop preempting, the 0.4 s deadman
        underneath -- applies unchanged, because this is the same machinery.
        A seek running on its own thread beside the executor would be a second
        writer to set_velocity() and would break that guarantee.
        """
        info = {"action": "seek", "target": behaviour.target, "mode": "seek",
                "goal": 0.0, "unit": "m", "lin": 0.0, "ang": 0.0,
                "duration_s": 0.0, "timeout_s": round(timeout_s, 2),
                "seek_state": "starting", "step": 1, "steps": 1}

        self._abort(stop_on_exit=False)

        state = {"cancel": threading.Event(), "stop_on_exit": True,
                 "infos": [info]}
        thread = threading.Thread(target=self._run_seek,
                                  args=(state, behaviour), daemon=True)
        with self._lock:
            self._run_state = state
            self._thread = thread
            self._active = dict(info)
            self._ends_at = time.monotonic() + timeout_s
            self._progress = 0.0
        thread.start()
        return dict(info, started=True)

    def _run_seek(self, state, behaviour):
        t0 = time.monotonic()
        reason = "done"
        try:
            reason = behaviour.run(state, state["cancel"])
        except Exception as e:  # noqa: BLE001
            # A crash in perception must not leave the robot driving. The
            # deadman would catch it in 0.4 s regardless, but the finally below
            # stops it now and the reason reaches the GUI instead of a
            # silently-dead thread.
            reason = f"seek failed ({type(e).__name__})"
            self._node.get_logger().exception("seek behaviour raised")
        finally:
            if state["stop_on_exit"]:
                self._node.stop()
            with self._lock:
                if self._run_state is state:
                    active = self._active or {}
                    self._last_result = {
                        "reason": reason, "mode": "seek",
                        "action": "seek", "target": behaviour.target,
                        "goal": active.get("goal", 0.0),
                        "progress": round(self._progress, 3), "unit": "m",
                        "range_m": active.get("range_m"),
                        "detections": behaviour.detections,
                        "elapsed_s": round(time.monotonic() - t0, 2),
                        "step": 1, "steps": 1,
                    }
                    self._run_state = None
                    self._thread = None
                    self._active = None

    def _run_one(self, state, spec):
        """Drive one step to its goal. Returns the reason it ended."""
        mode, goal, to_disp = spec["mode"], spec["goal"], spec["to_disp"]
        lin, ang = spec["lin"], spec["ang"]
        t0 = time.monotonic()
        deadline = t0 + spec["timeout"]

        if mode == "duration":
            while not state["cancel"].is_set() and time.monotonic() < deadline:
                self._node.set_velocity(lin, ang)
                self._set_progress(state, min(goal, time.monotonic() - t0))
                state["cancel"].wait(self.TICK_DURATION)
            if state["cancel"].is_set():
                return "cancelled"
            self._set_progress(state, goal)
            return "done"

        snap = self._node.get_odom()
        if snap is None:
            return "no odometry"
        # The pose we measure FROM has to be fresh too, not just the ones we
        # measure against. Without this a stale start pose reads as instantly
        # "done" -- a narrow race for one command, but a chain can put tens of
        # seconds between the endpoint's freshness check and a later step.
        if snap[3] > self.ODOM_STALE:
            return "odometry stalled"
        x0, y0, yaw0, _ = snap
        # A past discontinuity stops mattering the moment we re-snapshot: the
        # yaw accumulator only has to be continuous WITHIN a motion. Without
        # this the flag is sticky and one jump disables closed-loop control
        # until the server restarts.
        self._node.odom_broken = False

        linear = (mode == "distance")
        cruise, floor = spec["cruise"], spec["floor"]
        tol = self.TOL_LIN if linear else self.TOL_ANG
        ramp = self.RAMP_LIN if linear else self.RAMP_ANG
        stall = self.STALL_LIN if linear else self.STALL_ANG
        sign = 1.0 if (lin if linear else ang) >= 0.0 else -1.0

        best, best_t = 0.0, t0
        while True:
            if state["cancel"].is_set():
                return "cancelled"
            od = self._node.get_odom()
            if od is None or od[3] > self.ODOM_STALE:
                return "odometry stalled"
            if self._node.odom_broken:
                return "odometry jumped"
            x, y, yaw, _ = od

            if linear:
                # Straight-line displacement from the starting pose. Exact
                # for forward/backward and immune to yaw drift. Summing
                # per-sample hypot() instead would be strictly worse: under a
                # square root the noise is always positive, so a parked robot
                # would slowly accrue phantom distance. This assumption
                # breaks the day someone adds a curved-arc action.
                done = math.hypot(x - x0, y - y0)
            else:
                # yaw is the unwrapped accumulator, so this keeps climbing
                # past half a turn instead of folding at pi.
                done = max(0.0, sign * (yaw - yaw0))
            self._set_progress(state, done * to_disp)

            err = goal - done
            if err <= tol:
                return "done"

            now = time.monotonic()
            if now > deadline:
                return "timed out"
            if done > best + stall:
                best, best_t = done, now
            elif now - best_t > self.STALL_WINDOW:
                # Wheels blocked, or odometry frozen while still arriving.
                return "no progress"

            # Ease proportionally into the goal. Without this the stop
            # decision lands ~150 ms late at cruise speed, which is 2 cm or
            # 17 degrees of overshoot; slowing to the floor speed first cuts
            # it to millimetres. Note it's the floor, not the tolerance, that
            # bounds overshoot -- and a floor set too low stalls short.
            mag = cruise * clamp(err / ramp, 0.0, 1.0)
            mag = clamp(max(mag, floor), 0.0, cruise)
            self._node.set_velocity(sign * mag if linear else 0.0,
                                    0.0 if linear else sign * mag)
            state["cancel"].wait(self.TICK)

    def status(self):
        with self._lock:
            active = dict(self._active) if self._active else None
            ends_at, progress = self._ends_at, self._progress
            last = dict(self._last_result) if self._last_result else None
        if active is not None:
            # For a closed-loop goal this is time left on the timeout backstop,
            # not an ETA -- a healthy motion finishes long before it.
            active["remaining_s"] = max(0.0, round(ends_at - time.monotonic(), 1))
            active["progress"] = round(progress, 3)
            goal = active.get("goal") or 0.0
            frac = clamp(progress / goal, 0.0, 1.0) if goal > 0 else 0.0
            active["progress_pct"] = int(frac * 100)
            # Progress across the whole chain. Monotonic, unlike a per-step bar
            # that would reset 100 -> 0 at each boundary (and the CSS width
            # transition would animate that reset as a backwards sweep).
            # Identical to progress_pct when there's only one step.
            n, k = active.get("steps", 1), active.get("step", 1)
            active["chain_pct"] = int(clamp((k - 1 + frac) / n, 0.0, 1.0) * 100)
        return {"running": active is not None, "motion": active,
                "last_result": last}


# --------------------------------------------------------------------------- #
# Web server
# --------------------------------------------------------------------------- #
def build_app(node: MotionNode, max_lin, max_ang,
              parser: Optional[NLParser] = None, nl_max_duration=10.0,
              nl_max_distance=2.0, nl_max_angle=360.0, goal_timeout_max=60.0,
              nl_max_steps=5, nl_max_chain_distance=3.0,
              nl_max_chain_angle=720.0, nl_max_chain_seconds=120.0,
              link_check=False, detector=None, seek_parser=None,
              seek_cfg=None):
    app = FastAPI()
    executor = MotionExecutor(node, max_lin, max_ang, nl_max_duration,
                              max_distance=nl_max_distance,
                              max_angle=nl_max_angle,
                              goal_timeout_max=goal_timeout_max)
    goal_caps = {"duration": nl_max_duration, "distance": nl_max_distance,
                 "angle": nl_max_angle}
    # Whole-chain budgets. Every one is a strict superset of its per-step
    # counterpart, which is what guarantees none of them can bind on a single
    # command -- the backward-compatibility property, expressed as arithmetic.
    chain_caps = {"distance": nl_max_chain_distance, "angle": nl_max_chain_angle}
    seek_cfg = seek_cfg or SeekConfig()

    @app.get("/", response_class=HTMLResponse)
    def index():
        return HTML_PAGE.replace("__NL_ENABLED__",
                                 "true" if parser is not None else "false")

    @app.get("/video")
    def video():
        def gen():
            while True:
                jpeg = node.get_jpeg()
                if jpeg is not None:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + jpeg + b"\r\n")
                time.sleep(0.05)  # ~20 fps cap
        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/cmd")
    def cmd(lin: float = 0.0, ang: float = 0.0):
        # Manual teleop always wins: drop any typed command still running.
        # stop=False because we immediately set our own velocity below.
        executor.cancel(stop=False)
        lin = clamp(lin, -max_lin, max_lin)
        ang = clamp(ang, -max_ang, max_ang)
        node.set_velocity(lin, ang)
        return JSONResponse({"lin": lin, "ang": ang})

    @app.post("/stop")
    def stop():
        executor.cancel(stop=True)
        node.stop()
        return JSONResponse({"stopped": True})

    def do_seek(text, dry_run=0):
        """Shared by /nl routing and the explicit /seek endpoint."""
        if seek_parser is None or detector is None:
            return JSONResponse(
                {"executed": False, "action": "seek",
                 "message": "going to objects is disabled "
                            "(start the server with --enable-seek)"},
                status_code=503)

        target, error = seek_parser.parse(text)
        if error is not None:
            return JSONResponse({"executed": False, "action": "seek",
                                 "message": error})

        result = {"action": "seek", "target": target,
                  "stop_distance_m": seek_cfg.stop_distance,
                  "max_travel_m": seek_cfg.max_travel}

        # Checked here rather than inside the behaviour so a dry run reports it
        # too -- the point of dry_run is to find out whether this would work.
        age, _, _ = node.scan_stats()
        if age is None or age > SeekBehaviour.SCAN_STALE:
            result.update({
                "executed": False,
                "message": (f"I can see well enough to look for a {target}, but "
                            "the lidar isn't reporting - I'd have no way to know "
                            "when to stop")})
            return JSONResponse(result)

        if dry_run:
            result.update({"executed": False, "dry_run": True,
                           "message": f"would look for a {target}, then drive to "
                                      f"{seek_cfg.stop_distance:g} m from it"})
            return JSONResponse(result)

        behaviour = SeekBehaviour(node, detector, executor, target, seek_cfg)
        started = executor.start_seek(behaviour, seek_cfg.total_timeout)
        result.update({"executed": bool(started["started"]),
                       "timeout_s": started["timeout_s"],
                       "message": f"looking for a {target}"})
        return JSONResponse(result)

    @app.post("/seek")
    def seek(req: NLRequest, dry_run: int = 0):
        """Go to an object named in plain English."""
        text = (req.text or "").strip()
        if not text:
            return JSONResponse({"executed": False, "action": "seek",
                                 "message": "say what to go to"})
        return do_seek(text, dry_run)

    @app.get("/detect")
    def detect(target: str = "bottle", all: int = 1):
        """Annotated snapshot of what the detector currently sees.

        The robot does not move. This is the first verification stage: a
        behaviour that drives at whatever the detector reports is only as good
        as the detector, so being able to check that standing still -- and to
        find the confidence a given object actually scores in this room's
        lighting -- comes before letting it move.
        """
        if detector is None:
            return JSONResponse({"error": "detection is disabled "
                                          "(start with --enable-seek)"},
                                status_code=503)
        frame, seq = node.get_frame()
        boxes = detector.detect_all(frame, target)
        annotated = detector.annotate(frame, boxes, target, best_only=not all)
        ok, buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return JSONResponse({"error": "could not encode frame"},
                                status_code=500)
        rng, _ = node.front_range()
        return StreamingResponse(
            iter([buf.tobytes()]), media_type="image/jpeg",
            headers={"X-Detections": str(len(boxes)),
                     "X-Best-Conf": "" if not boxes else f"{boxes[0].conf:.3f}",
                     "X-Infer-Ms": str(detector.last_ms),
                     "X-Front-Range-M": "" if rng is None else f"{rng:.3f}"})

    @app.get("/video/detect")
    def video_detect(target: str = "bottle"):
        """Camera stream with detections drawn on it. The robot does not move.

        Deliberately slower than /video. Inference is cheap (~19 ms) but this
        runs for as long as someone leaves the tab open, and a seek in progress
        wants the GPU more than a viewer does -- so it samples a few times a
        second rather than at frame rate, and skips outright when the frame
        hasn't changed.
        """
        if detector is None:
            return JSONResponse({"error": "detection is disabled "
                                          "(start with --enable-seek)"},
                                status_code=503)

        def gen():
            last_seq, jpeg = -1, None
            while True:
                frame, seq = node.get_frame()
                if seq != last_seq:
                    last_seq = seq
                    boxes = detector.detect_all(frame, target)
                    ok, buf = cv2.imencode(
                        ".jpg", detector.annotate(frame, boxes, target),
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
                    jpeg = buf.tobytes() if ok else None
                if jpeg is not None:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + jpeg + b"\r\n")
                time.sleep(0.2)      # ~5 fps
        return StreamingResponse(
            gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.post("/nl")
    def nl(req: NLRequest, dry_run: int = 0):
        """Parse a plain-English instruction and (unless dry_run) execute it."""
        if parser is None:
            return JSONResponse(
                {"executed": False, "action": "unknown",
                 "message": "natural-language control is disabled "
                            "(start the server with --enable-nl)"},
                status_code=503)

        text = (req.text or "").strip()
        if not text:
            return JSONResponse({"executed": False, "action": "unknown",
                                 "message": "say something first"})

        frags = split_steps(text)
        if not frags:
            return JSONResponse({"executed": False, "action": "unknown",
                                 "message": "say something first"})

        def at(i, msg):
            """Prefix a per-step message with its position -- but only for a
            real chain, so a single command's message is byte-identical."""
            return msg if len(frags) == 1 else \
                f'step {i + 1} of {len(frags)} ("{frags[i]}"): {msg}'

        # Route to the seek parser before the driving parser gets a look in.
        # Deterministic, and deliberately checked across every fragment: a seek
        # buried at step 2 must be refused, not quietly handed to the driving
        # parser, which would read "go to the bottle" as an unknown action and
        # abandon the rest of the chain with a confusing message.
        if any(SeekParser.looks_like_seek(f) for f in frags):
            if len(frags) > 1:
                return JSONResponse(
                    {"executed": False, "action": "seek", "steps": len(frags),
                     "message": "I can't combine going to an object with other "
                                "steps yet - ask for it on its own"})
            return do_seek(text, dry_run)

        # Refuse an over-long chain BEFORE parsing: no point paying five LLM
        # round-trips to reject something on a count we already know.
        if len(frags) > nl_max_steps:
            return JSONResponse(
                {"executed": False, "action": "unknown", "steps": len(frags),
                 "message": f"that's {len(frags)} steps - I take at most "
                            f"{nl_max_steps} at a time"})

        # Parse every fragment before anything moves. Discovering at step 3 that
        # it's nonsense, having already driven steps 1 and 2, is exactly the
        # failure this ordering exists to prevent. Fail fast on the first error
        # so a wedged model costs one timeout, not N.
        intents = []
        for i, frag in enumerate(frags):
            intent, error = parser.parse(frag)
            if error is not None:
                return JSONResponse({"executed": False, "action": "unknown",
                                     "failed_step": i + 1,
                                     "message": at(i, error)})
            intents.append(intent)

        # "go forward 1 m then stop" is natural, and a chain always ends stopped
        # anyway, so a trailing stop is simply dropped. Done before the lone-stop
        # check below so "halt, halt" collapses to the plain stop path.
        while len(intents) > 1 and intents[-1].action == "stop":
            intents.pop()
            frags.pop()

        odom = node.get_odom()
        chained = len(intents) > 1

        if not chained:
            intent = intents[0]
            mode = intent.mode
            unit = MODE_UNIT[mode]
            requested = float(intent.value)
            value = clamp(requested, 0.0, goal_caps[mode])
            capped = abs(value - requested) > 1e-6
            result = {"action": intent.action, "speed": intent.speed,
                      "mode": mode, "unit": unit,
                      "value": value, "requested_value": requested,
                      # Kept so anything parsing the old shape still works. For
                      # a closed-loop goal there is no meaningful duration up
                      # front.
                      "duration_s": value if mode == "duration" else 0.0,
                      "requested_duration_s": (requested if mode == "duration"
                                               else 0.0),
                      "capped": capped, "executed": False,
                      "steps": 1, "odom_ok": odom is not None}

            if intent.action == "stop":
                # stop ignores mode and value entirely, so a mis-tagged mode
                # must never be allowed to block it.
                if not dry_run:
                    executor.cancel(stop=True)
                result["executed"] = not dry_run
                result["message"] = "stopping"
                return JSONResponse(result)
        else:
            result = {"action": intents[0].action, "speed": intents[0].speed,
                      "executed": False, "steps": len(intents),
                      "odom_ok": odom is not None}

        # A stop in the MIDDLE is refused rather than executed-then-truncated:
        # truncating would discard the trailing steps only after the robot had
        # already moved, which is the same "found out too late" failure that
        # parsing everything up front exists to avoid. It's ambiguous anyway --
        # "pause here" or "cancel the rest"?
        for i, intent in enumerate(intents):
            if intent.action == "stop":
                return JSONResponse(dict(
                    result, failed_step=i + 1,
                    message=f'"{frags[i]}" in the middle of a sequence - say it '
                            "on its own to stop the robot"))

        # Per-step validation. Every step is checked before any of them runs.
        plan_args, totals = [], {"distance": 0.0, "angle": 0.0}
        for i, intent in enumerate(intents):
            mode = intent.mode
            if intent.action == "unknown":
                return JSONResponse(dict(
                    result, failed_step=i + 1,
                    message=at(i, f'"{frags[i]}" is not a driving command '
                                  "I understand")))
            # Mode has to make sense for the action. Refuse rather than repair:
            # "forward" plus "angle" could be a mis-tagged distance or an
            # intended rotate, and there's no way to tell -- guessing wrong
            # drives the robot somewhere it was never asked to go.
            if ((intent.action in LINEAR_ACTIONS and mode == "angle") or
                    (intent.action in ANGULAR_ACTIONS and mode == "distance")):
                return JSONResponse(dict(
                    result, failed_step=i + 1,
                    message=at(i, f'"{frags[i]}" mixes up moving and turning - '
                                  "forward and backward take meters or seconds, "
                                  "rotating takes degrees or seconds")))
            v = clamp(float(intent.value), 0.0, goal_caps[mode])
            if mode in totals:
                totals[mode] += v
            plan_args.append((intent.action, intent.speed, mode, v))

        planned = executor.plan(plan_args)

        # Whole-chain budgets. Refused, not clamped -- deliberately unlike the
        # per-step caps. A per-step cap has a repair the user can see in their
        # own phrase ("capped from 5m"); trimming a chain to fit is arbitrary
        # (which step loses out?) and lands the robot somewhere they cannot
        # predict from what they typed.
        if chained:
            for mode, total in totals.items():
                if total > chain_caps[mode] + 1e-6:
                    return JSONResponse(dict(
                        result, message=(
                            f"that's {total:g}{MODE_UNIT[mode]} of {mode} across "
                            f"{len(intents)} steps - the limit for one sequence "
                            f"is {chain_caps[mode]:g}{MODE_UNIT[mode]}")))
            budget = sum(i["timeout_s"] for i, _ in planned)
            if budget > nl_max_chain_seconds + 1e-6:
                return JSONResponse(dict(
                    result, message=(
                        f"that sequence could take up to {budget:g}s - the limit "
                        f"for one sequence is {nl_max_chain_seconds:g}s")))

        # Refuse to drive blind when the robot link is known to be down.
        if link_check and node.link_rtt_ms is None and not dry_run:
            result["message"] = "robot link is down - not sending a motion command"
            return JSONResponse(result)

        # A distance or angle goal is only as good as the odometry it measures
        # against. With no odom we refuse rather than silently falling back to
        # "distance / speed = time" -- a guess dressed up as a measurement is
        # exactly the surprise that makes a robot dangerous. Checked once for
        # the whole chain; each step re-checks staleness as it runs.
        closed = [m for _, _, m, _ in plan_args if m in ("distance", "angle")]
        if closed and not dry_run:
            mode = closed[0]
            if odom is None:
                result["message"] = (
                    f"no odometry from the robot yet - {mode} commands need "
                    "/odom (is the robot bringup running?)")
                return JSONResponse(result)
            if odom[3] > 1.0:
                result["message"] = (f"odometry is {odom[3]:.1f}s stale - "
                                     f"refusing a closed-loop {mode} command")
                return JSONResponse(result)

        def phrase_of(info):
            p = info["action"].replace("_", " ") + " "
            return p + (f"for {info['goal']:g}s" if info["mode"] == "duration"
                        else f"{info['goal']:g} {info['unit']}")

        if chained:
            phrase = f"{len(planned)} steps: " + "; then ".join(
                phrase_of(i) for i, _ in planned)
            note = ""
        else:
            note = f" (capped from {requested:g}{unit})" if capped else ""
            phrase = phrase_of(planned[0][0])

        result["plan"] = [dict(i) for i, _ in planned]

        if dry_run:
            result["message"] = (f"[dry run] {phrase} at "
                                 f"{intents[0].speed} speed{note}")
            return JSONResponse(result)

        started = executor.start_planned(planned)
        result.update({"executed": bool(started["started"]),
                       "timeout_s": started["timeout_s"],
                       "lin": started["lin"], "ang": started["ang"]})
        if not chained:
            result.update({"value": started["goal"],
                           "duration_s": started["duration_s"]})
        result["message"] = (f"{phrase} at {intents[0].speed} speed{note}"
                             if started["started"] else "nothing to do")
        return JSONResponse(result)

    @app.get("/nl/status")
    def nl_status():
        age, hz, count = node.odom_stats()
        s_age, _, _ = node.scan_stats()
        return JSONResponse({"enabled": parser is not None,
                             "model": parser.model if parser else None,
                             "max_duration_s": nl_max_duration,
                             "max_distance_m": nl_max_distance,
                             "max_angle_deg": nl_max_angle,
                             "max_steps": nl_max_steps,
                             "odom_ok": age is not None and age <= 1.0,
                             "odom_age_s": None if age is None else round(age, 2),
                             "seek_enabled": detector is not None
                                             and seek_parser is not None,
                             "seek_stop_distance_m": seek_cfg.stop_distance,
                             "scan_ok": s_age is not None
                                        and s_age <= SeekBehaviour.SCAN_STALE,
                             "scan_age_s": None if s_age is None else round(s_age, 2),
                             **executor.status()})

    @app.get("/status")
    def status():
        age, hz, count = node.odom_stats()
        s_age, s_hz, s_count = node.scan_stats()
        front, _ = node.front_range()
        return JSONResponse({
            "frames": node.frames_received,
            "image_topic": node.image_topic,
            "cmd_vel_topic": node.cmd_vel_topic,
            "odom_topic": node.odom_topic,
            "max_lin": max_lin,
            "max_ang": max_ang,
            "link_rtt_ms": node.link_rtt_ms,
            "cmd_cycle_ms": 50,  # 20 Hz command publisher
            # Exposed so the choice to keep one spin thread stays measurable:
            # if odom_age_ms starts climbing, the camera decode is crowding it.
            "odom_count": count,
            "odom_hz": hz,
            "odom_age_ms": None if age is None else round(age * 1000),
            "scan_topic": node.scan_topic,
            "scan_count": s_count,
            "scan_hz": s_hz,
            "scan_age_ms": None if s_age is None else round(s_age * 1000),
            # The metric the approach actually stops on, surfaced so it can be
            # checked against a tape measure before trusting it to stop.
            "front_range_m": None if front is None else round(front, 3),
        })

    return app


HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>TurtleBot3 Motion Server</title>
<style>
  body { font-family: system-ui, sans-serif; background:#111; color:#eee;
         margin:0; padding:16px; text-align:center; }
  h1 { font-size:20px; margin:8px 0; }
  .wrap { display:flex; flex-wrap:wrap; gap:24px; justify-content:center;
          align-items:flex-start; max-width:1200px; margin:0 auto; }
  .camwrap { flex:0 1 auto; }
  #cam { max-height:74vh; max-width:100%; width:auto; border:2px solid #333;
         border-radius:8px; background:#000; display:block; }
  .panel { flex:0 0 330px; max-width:340px; }
  .pad { display:grid; grid-template-columns:repeat(3,92px); grid-auto-rows:92px;
         gap:10px; justify-content:center; margin:14px auto; }
  button.k { border:0; border-radius:12px; background:#2a2a2a; color:#eee;
             cursor:pointer; user-select:none; touch-action:none;
             display:flex; flex-direction:column; align-items:center;
             justify-content:center; line-height:1.1; }
  button.k .arw { font-size:30px; }
  button.k .lbl { font-size:11px; color:#9aa; margin-top:3px; }
  button.k:active, button.k.on { background:#3d7eff; color:#fff; }
  button.k:active .lbl, button.k.on .lbl { color:#e8f0ff; }
  #stop { background:#c0392b; }
  #stop:active, #stop.on { background:#e74c3c; }
  #vel { font-size:15px; color:#6cf; margin:8px 0 2px;
         font-variant-numeric:tabular-nums; }
  #lat { font-size:12px; color:#d0a15e; margin:0 0 8px;
         font-variant-numeric:tabular-nums; }
  #prog { display:none; margin:0 0 8px; }
  #progbar { height:4px; background:#2a2a2a; border-radius:2px; overflow:hidden; }
  #progfill { height:100%; width:0%; background:#3d7eff; transition:width .12s linear; }
  .row { margin:10px auto; max-width:640px; font-size:14px; }
  input[type=range] { width:55%; vertical-align:middle; }
  .hint { color:#888; font-size:13px; margin-top:14px; }
  #status { color:#6c6; font-size:13px; }
  .nlwrap { margin-top:16px; border-top:1px solid #333; padding-top:12px;
            text-align:left; }
  .nlwrap h2 { font-size:14px; color:#9aa; margin:0 0 8px; font-weight:600; }
  .nlrow { display:flex; gap:8px; }
  #nltext { flex:1 1 auto; min-width:0; background:#2a2a2a; color:#eee;
            border:1px solid #3a3a3a; border-radius:8px; padding:9px 11px;
            font-size:14px; font-family:inherit; }
  #nltext:focus { outline:none; border-color:#3d7eff; }
  #nlsend { flex:0 0 auto; border:0; border-radius:8px; background:#3d7eff;
            color:#fff; padding:9px 16px; font-size:14px; cursor:pointer; }
  #nlsend:disabled { background:#33415e; color:#889; cursor:default; }
  #nllog { margin-top:10px; max-height:150px; overflow-y:auto; font-size:13px;
           line-height:1.45; }
  #nllog div { margin:3px 0; }
  #nllog .you { color:#eee; }
  #nllog .you::before { content:"› "; color:#3d7eff; }
  #nllog .ok { color:#6c6; padding-left:14px; }
  #nllog .no { color:#d0745e; padding-left:14px; }
  #nllog .pending { color:#888; padding-left:14px; font-style:italic; }
  .nlhint { margin-top:8px; font-size:12px; color:#7d8a8a; line-height:1.5; }
  .nlhint b { color:#9aa; font-weight:600; }
  .nlhint code { background:#242424; padding:1px 5px; border-radius:4px;
                 color:#8fb0d8; cursor:pointer; }
  .nlhint code:hover { background:#2f2f2f; color:#b9d2f0; }
  .vistoggle { margin-top:8px; display:none; align-items:center; gap:8px;
               justify-content:center; font-size:13px; color:#9aa; }
  .vistoggle input { width:130px; background:#2a2a2a; color:#eee; font-size:13px;
                     border:1px solid #3a3a3a; border-radius:6px; padding:4px 8px; }
  .vistoggle button { border:0; border-radius:6px; background:#2a2a2a; color:#eee;
                      cursor:pointer; padding:5px 10px; font-size:13px; }
  .vistoggle button.on { background:#3d7eff; color:#fff; }
</style>
</head>
<body>
  <h1>TurtleBot3 Motion Server</h1>
  <div class="wrap">
   <div class="camwrap">
    <img id="cam" src="/video" alt="camera stream"/>
    <div class="vistoggle" id="vistoggle">
      <button id="visbtn" title="Draw boxes on the camera view showing what the detector finds. The robot does not move.">show what it sees</button>
      <input id="vistarget" type="text" value="bottle" autocomplete="off"/>
    </div>
    <div id="status">connecting…</div>
   </div>

   <div class="panel">
    <div id="vel">v = 0.00 m/s &nbsp; ω = 0.00 rad/s</div>
    <div id="lat" title="Time from a teleop command to the robot acting on it: network link + 50 ms command cycle + motor response.">teleop latency: measuring…</div>
    <div id="prog"><div id="progbar"><div id="progfill"></div></div></div>
    <div class="pad">
      <button class="k" data-keys="wa"><span class="arw">↖</span><span class="lbl">Fwd-Left</span></button>
      <button class="k" data-keys="w"><span class="arw">↑</span><span class="lbl">Forward</span></button>
      <button class="k" data-keys="wd"><span class="arw">↗</span><span class="lbl">Fwd-Right</span></button>
      <button class="k" data-keys="a"><span class="arw">↺</span><span class="lbl">Turn Left</span></button>
      <button class="k" id="stop" data-stop="1"><span class="arw">■</span><span class="lbl">STOP</span></button>
      <button class="k" data-keys="d"><span class="arw">↻</span><span class="lbl">Turn Right</span></button>
      <button class="k" data-keys="sa"><span class="arw">↙</span><span class="lbl">Back-Left</span></button>
      <button class="k" data-keys="s"><span class="arw">↓</span><span class="lbl">Backward</span></button>
      <button class="k" data-keys="sd"><span class="arw">↘</span><span class="lbl">Back-Right</span></button>
    </div>

    <div class="row">
      Linear speed: <span id="linv">0.15</span> m/s
      <input id="lin" type="range" min="0.02" max="0.22" step="0.01" value="0.15"/>
    </div>
    <div class="row">
      Angular speed: <span id="angv">1.0</span> rad/s
      <input id="ang" type="range" min="0.1" max="2.8" step="0.1" value="1.0"/>
    </div>
    <p class="hint">Drive with the buttons above (press &amp; hold), or the keyboard:
       <b>W/A/S/D</b> or arrow keys · <b>Space</b> or <b>X</b> to stop.
       The robot stops the moment you release.</p>

    <div class="nlwrap" id="nlwrap" style="display:none">
      <h2>Say it in plain English</h2>
      <div class="nlrow">
        <input id="nltext" type="text" autocomplete="off"
               placeholder="e.g. move forward 1 meter — or: go to the bottle"/>
        <button id="nlsend">Send</button>
      </div>
      <div class="nlhint" id="nlhint"></div>
      <div id="nllog"></div>
    </div>
   </div>
  </div>

<script>
const lin = document.getElementById('lin');
const ang = document.getElementById('ang');
const linv = document.getElementById('linv');
const angv = document.getElementById('angv');
const velEl = document.getElementById('vel');
lin.oninput = () => linv.textContent = lin.value;
ang.oninput = () => angv.textContent = ang.value;

function showVel(l, a) {
  velEl.textContent = `v = ${l.toFixed(2)} m/s   ω = ${a.toFixed(2)} rad/s`;
}
function send(l, a) {
  fetch(`/cmd?lin=${l}&ang=${a}`, {method:'POST'});
  showVel(l, a);
}
function drive(dir) {
  const L = parseFloat(lin.value), A = parseFloat(ang.value);
  const map = {
    w:[ L, 0], s:[-L, 0], a:[0,  A], d:[0, -A],
    wa:[L, A], wd:[L,-A], sa:[-L,-A], sd:[-L, A]
  };
  const v = map[dir] || [0, 0];
  send(v[0], v[1]);
}
function stop() { fetch('/stop', {method:'POST'}); showVel(0, 0); }

const held = new Set();
function combo() {
  let d = '';
  if (held.has('w')) d += 'w'; else if (held.has('s')) d += 's';
  if (held.has('a')) d += 'a'; else if (held.has('d')) d += 'd';
  return d;
}
function refreshBtns() {
  const c = combo();
  document.querySelectorAll('button.k').forEach(b =>
    b.classList.toggle('on', c !== '' && b.dataset.keys === c));
}
function apply() {
  const c = combo();
  if (!c) stop(); else drive(c);
  refreshBtns();
}
const keymap = {arrowup:'w', arrowdown:'s', arrowleft:'a', arrowright:'d'};
// Ignore keys typed into a field, or typing "forward" would drive the robot.
function typing(e) {
  const t = e.target;
  return t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA');
}
window.addEventListener('keydown', e => {
  if (typing(e)) return;
  let k = e.key.toLowerCase(); k = keymap[k] || k;
  if (k === 'x' || k === ' ') { e.preventDefault(); held.clear(); apply(); return; }
  if ('wasd'.includes(k)) { e.preventDefault();
    if (!held.has(k)) { held.add(k); apply(); } }
});
window.addEventListener('keyup', e => {
  if (typing(e)) return;
  let k = e.key.toLowerCase(); k = keymap[k] || k;
  if (held.has(k)) { held.delete(k); apply(); }
});

// On-screen buttons (mouse + touch): press-and-hold to drive, release to stop.
document.querySelectorAll('button.k').forEach(b => {
  const down = e => {
    e.preventDefault();
    if (b.dataset.stop) { held.clear(); apply(); return; }
    [...b.dataset.keys].forEach(c => held.add(c)); apply();
  };
  const up = e => {
    e.preventDefault();
    if (b.dataset.stop) return;
    [...b.dataset.keys].forEach(c => held.delete(c)); apply();
  };
  b.addEventListener('mousedown', down);
  b.addEventListener('mouseup', up);
  b.addEventListener('mouseleave', up);
  b.addEventListener('touchstart', down, {passive:false});
  b.addEventListener('touchend', up, {passive:false});
  b.addEventListener('touchcancel', up, {passive:false});
});

// ---- natural-language control -------------------------------------------- //
const NL_ENABLED = __NL_ENABLED__;
const nlText = document.getElementById('nltext');
const nlSend = document.getElementById('nlsend');
const nlLog  = document.getElementById('nllog');
if (NL_ENABLED) document.getElementById('nlwrap').style.display = '';

// ---- seek ("go to the bottle") -------------------------------------------- //
// Whether seek is available is a server-side fact (it needs the detector AND a
// live lidar), so it is asked for rather than baked into the page: the same GUI
// is served with it on and off, and a text box that silently does nothing is
// worse than one that says what it can do.
const nlHint    = document.getElementById('nlhint');
const visToggle = document.getElementById('vistoggle');
const visBtn    = document.getElementById('visbtn');
const visTarget = document.getElementById('vistarget');
const camImg    = document.getElementById('cam');
let seekReady = false, visOn = false;

async function initSeek() {
  let s;
  try { s = await (await fetch('/nl/status')).json(); } catch (e) { return; }
  seekReady = !!s.seek_enabled;
  if (!seekReady) return;
  visToggle.style.display = 'flex';
  const stop = (s.seek_stop_distance_m ?? 0.35).toFixed(2);
  nlHint.innerHTML =
    `<b>Go to an object:</b> try <code>go to the bottle</code>, ` +
    `<code>find the red backpack</code> or <code>approach the chair</code>. ` +
    `I'll turn until I see it, drive to it and stop ${stop} m short. ` +
    `Only objects on the floor can be ranged.`;
  // Click an example to load it -- the feature is only discoverable if the
  // phrasing that works is in front of you.
  nlHint.querySelectorAll('code').forEach(c => {
    c.addEventListener('click', () => { nlText.value = c.textContent; nlText.focus(); });
  });
}

function setVision(on) {
  visOn = on;
  visBtn.classList.toggle('on', on);
  visBtn.textContent = on ? 'showing what it sees' : 'show what it sees';
  const t = (visTarget.value || 'bottle').trim();
  camImg.src = on ? `/video/detect?target=${encodeURIComponent(t)}`
                  : '/video';
}
visBtn.addEventListener('click', () => setVision(!visOn));
visTarget.addEventListener('change', () => { if (visOn) setVision(true); });
initSeek();

function logLine(cls, text) {
  const d = document.createElement('div');
  d.className = cls; d.textContent = text;
  nlLog.appendChild(d); nlLog.scrollTop = nlLog.scrollHeight;
  return d;
}
async function sendNL() {
  const t = nlText.value.trim();
  if (!t) return;
  logLine('you', t);
  nlText.value = '';
  nlSend.disabled = true;
  const pending = logLine('pending', 'thinking…');
  // Each step costs its own model round-trip, so a sequence sits here for
  // several seconds. Say so rather than looking hung.
  const slow = setTimeout(() => {
    pending.textContent = 'thinking… (multi-step commands take longer)';
  }, 4000);
  try {
    const r = await fetch('/nl', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify({text:t})});
    const d = await r.json();
    pending.remove();
    logLine(d.executed ? 'ok' : 'no', d.message || 'no response');
    if (d.executed) startProgressPoll();
  } catch (e) {
    pending.remove();
    logLine('no', 'server unreachable');
  } finally {
    clearTimeout(slow);
    nlSend.disabled = false;
    nlText.focus();
  }
}
if (NL_ENABLED) {
  nlSend.addEventListener('click', sendNL);
  nlText.addEventListener('keydown', e => {
    if (e.key === 'Enter') { e.preventDefault(); sendNL(); }
  });
}

const progEl   = document.getElementById('prog');
const progFill = document.getElementById('progfill');

// What each stage of a seek is called in the readout. The internal names are
// the wrong register for someone watching a robot cross a room.
const SEEK_WORDS = {
  starting: 'starting',
  search:   'looking around',
  center:   'turning to face it',
  aiming:   're-aiming',
  approach: 'driving to it',
  arrived:  'arrived',
};

function renderMotion(m) {
  if (!m) { progEl.style.display = 'none'; return; }

  // A seek reports where it has got to, not how far it has driven. Its goal
  // isn't even known until the target has been found and ranged, so the
  // distance-style readout below would show "0.00 of 0.00 m" for the whole
  // search -- which reads as broken rather than as busy.
  if (m.mode === 'seek') {
    let s = `${SEEK_WORDS[m.seek_state] || m.seek_state || 'seeking'}`;
    if (m.target) s += ` · ${m.target}`;
    if (m.range_m != null) s += ` · ${m.range_m.toFixed(2)} m away`;
    if (m.conf != null) s += ` · seen ${(m.conf * 100).toFixed(0)}%`;
    velEl.textContent = s;
    const goal = m.goal ?? 0;
    if (goal > 0) {
      progEl.style.display = '';
      progFill.style.width = (m.progress_pct ?? 0) + '%';
    } else {
      progEl.style.display = 'none';
    }
    return;
  }

  const chained = (m.steps ?? 1) > 1;
  let head = chained ? `step ${m.step}/${m.steps} · ` : '';
  head += m.action.replace('_',' ') + ' · ';
  if (m.mode && m.mode !== 'duration') {
    // Closed-loop: show measured progress. Deliberately not remaining_s --
    // that's the timeout backstop, and a healthy motion finishes long before
    // it, so showing it would read as a wildly pessimistic ETA.
    head += `${(m.progress ?? 0).toFixed(2)} of ${(m.goal ?? 0).toFixed(2)} ${m.unit}`;
  } else {
    head += `${(m.remaining_s ?? 0).toFixed(1)}s left`;
  }
  // For a chain, track the whole sequence: chain_pct only ever grows, whereas a
  // per-step bar would reset 100->0 at each boundary and the CSS width
  // transition would animate that reset as a backwards sweep. It also keeps the
  // bar on screen across a mixed chain, where a duration step would otherwise
  // hide it and the next closed-loop step bring it back.
  if (chained || (m.mode && m.mode !== 'duration')) {
    progEl.style.display = '';
    progFill.style.width = (chained ? (m.chain_pct ?? 0)
                                    : (m.progress_pct ?? 0)) + '%';
  } else {
    progEl.style.display = 'none';
  }
  velEl.textContent = head + ` (v = ${m.lin.toFixed(2)}  ω = ${m.ang.toFixed(2)})`;
}

// A 30° turn is over in about a quarter of a second, so the 1 Hz status poll
// would miss the whole motion. Poll fast, but only while one is running.
let progTimer = null;
function startProgressPoll() {
  if (progTimer) return;
  progTimer = setInterval(async () => {
    try {
      const s = await (await fetch('/nl/status')).json();
      renderMotion(s.motion);
      if (!s.running) {
        clearInterval(progTimer); progTimer = null;
        progEl.style.display = 'none';
        showVel(0, 0);
        // Say why it ended. A motion that timed out or hit the progress
        // watchdog stopped short, and without this the log still shows the
        // cheerful acceptance message as if all had gone to plan.
        const r = s.last_result;
        const where = (r && r.steps > 1)
          ? `step ${r.step}/${r.steps} (${r.action.replace('_',' ')}) ` : '';
        if (r && r.reason !== 'done' && r.reason !== 'cancelled')
          logLine('no', `${where}stopped: ${r.reason} — ${r.progress} of ${r.goal} ${r.unit}`);
        // A plain cancel stays silent -- you pressed STOP, you know. But that
        // you also discarded the steps after this one is news.
        else if (r && r.reason === 'cancelled' && r.steps > 1 && r.step < r.steps)
          logLine('no', `cancelled at step ${r.step} of ${r.steps}`);
      }
    } catch (e) { clearInterval(progTimer); progTimer = null; }
  }, 150);
}

async function poll() {
  try {
    const r = await fetch('/status'); const s = await r.json();
    document.getElementById('status').textContent =
      `camera frames: ${s.frames} · topic ${s.image_topic} → ${s.cmd_vel_topic}`;
    const lat = document.getElementById('lat');
    if (s.link_rtt_ms == null) {
      lat.textContent = 'teleop latency: robot link down';
    } else {
      // command→motion ≈ one-way network + command cycle + motor response (~30 ms)
      const oneway = s.link_rtt_ms / 2;
      const lo = Math.round(oneway);
      const hi = Math.round(oneway + s.cmd_cycle_ms + 30);
      lat.textContent =
        `teleop command→motion ≈ ${lo}–${hi} ms   (link RTT ${s.link_rtt_ms} ms)`;
    }
  } catch(e) {
    document.getElementById('status').textContent = 'server unreachable';
  }
  // Catch a motion started elsewhere (another tab, curl) and hand it to the
  // fast poller; while that's running it owns the readout and we stay out.
  if (NL_ENABLED && !progTimer) {
    try {
      if ((await (await fetch('/nl/status')).json()).running) startProgressPoll();
    } catch(e) {}
  }
}
setInterval(poll, 1000); poll();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------- #
# main
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="TurtleBot3 motion server + web GUI")
    ap.add_argument("--image-topic", default="/image")
    ap.add_argument("--image-type", default="auto",
                    choices=["auto", "raw", "compressed"])
    ap.add_argument("--cmd-vel-topic", default="/cmd_vel")
    ap.add_argument("--odom-topic", default="/odom",
                    help="odometry topic, used to measure distance and angle goals")
    ap.add_argument("--robot-ip", default="192.168.68.100",
                    help="robot IP, used to measure live teleop link latency "
                         "(set empty to disable)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--max-lin", type=float, default=0.22,
                    help="max linear speed (Burger=0.22, Waffle=0.26)")
    ap.add_argument("--max-ang", type=float, default=2.84,
                    help="max angular speed (Burger=2.84, Waffle=1.82)")
    ap.add_argument("--enable-nl", action="store_true",
                    help="enable natural-language driving via a local LLM")
    ap.add_argument("--llm-url", default="http://localhost:11434",
                    help="ollama base URL")
    ap.add_argument("--llm-model", default="qwen2.5:3b",
                    help="ollama model used to parse commands")
    ap.add_argument("--nl-max-duration", type=float, default=10.0,
                    help="hard cap (seconds) on any single typed motion")
    ap.add_argument("--nl-max-distance", type=float, default=2.0,
                    help="hard cap (meters) on any single typed distance goal")
    ap.add_argument("--nl-max-angle", type=float, default=360.0,
                    help="hard cap (degrees) on any single typed angle goal")
    ap.add_argument("--goal-timeout-max", type=float, default=60.0,
                    help="ceiling (seconds) on a closed-loop goal's timeout backstop")
    # Whole-sequence budgets. The per-step caps above are per motion, so without
    # these a 3-step chain of 2 m legs passes every check and drives 6 m.
    ap.add_argument("--nl-max-steps", type=int, default=5,
                    help="most steps allowed in one comma-separated sequence")
    ap.add_argument("--nl-max-chain-distance", type=float, default=3.0,
                    help="cap (meters) on total path length across a sequence")
    ap.add_argument("--nl-max-chain-angle", type=float, default=720.0,
                    help="cap (degrees) on total rotation across a sequence")
    ap.add_argument("--nl-max-chain-seconds", type=float, default=120.0,
                    help="cap (seconds) on a sequence's worst-case running time")
    # Seek: "go to the bottle". Needs the lidar as well as the camera.
    ap.add_argument("--enable-seek", action="store_true",
                    help="enable going to objects named in plain English")
    ap.add_argument("--scan-topic", default="/scan",
                    help="LaserScan topic used to range the target")
    ap.add_argument("--seek-weights", default="yolov8m-world.pt",
                    help="YOLO-World weights (open-vocabulary detector)")
    ap.add_argument("--seek-device", default=None,
                    help="torch device for detection (default: cuda if present)")
    ap.add_argument("--seek-conf", type=float, default=0.10,
                    help="detection confidence threshold (see vision.Detector "
                         "for the measurements behind this default)")
    ap.add_argument("--seek-stop-distance", type=float, default=0.35,
                    help="how far short of the target to stop (meters, min 0.25)")
    ap.add_argument("--seek-max-travel", type=float, default=2.5,
                    help="cap (meters) on how far one approach may drive")
    ap.add_argument("--seek-search-step", type=float, default=25.0,
                    help="degrees rotated between looks while searching")
    ap.add_argument("--seek-search-max", type=float, default=400.0,
                    help="degrees swept before giving up the search")
    ap.add_argument("--seek-timeout", type=float, default=60.0,
                    help="cap (seconds) on one whole seek")
    args = ap.parse_args()

    rclpy.init()
    node = MotionNode(args.image_topic, args.cmd_vel_topic, args.image_type,
                      robot_ip=(args.robot_ip or None),
                      odom_topic=args.odom_topic, scan_topic=args.scan_topic)

    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    parser = None
    if args.enable_nl:
        parser = NLParser(args.llm_url, args.llm_model, args.nl_max_duration,
                          args.nl_max_distance, args.nl_max_angle)
        print(f"  Natural language: {args.llm_model} via {args.llm_url} "
              f"(max {args.nl_max_duration:g}s, {args.nl_max_distance:g}m, "
              f"{args.nl_max_angle:g}deg per step; up to {args.nl_max_steps} "
              f"steps per sequence)")

    detector = seek_parser = seek_cfg = None
    if args.enable_seek:
        seek_cfg = SeekConfig(stop_distance=args.seek_stop_distance,
                              max_travel=args.seek_max_travel,
                              search_step_deg=args.seek_search_step,
                              search_max_deg=args.seek_search_max,
                              total_timeout=args.seek_timeout,
                              conf=args.seek_conf)
        from vision import Detector
        device = args.seek_device
        if device is not None and device.isdigit():
            device = int(device)
        detector = Detector(args.seek_weights, conf=args.seek_conf,
                            device=device)
        # Warm up before serving, not on the first command: the first inference
        # costs ~0.7 s against ~19 ms for the rest, and paid later that lands as
        # most of a second of the robot rotating without looking.
        warm = detector.warmup()
        seek_parser = SeekParser(args.llm_url, args.llm_model)
        print(f"  Seek: {args.seek_weights} on device {detector.device} "
              f"(warmup {warm:.1f}s, conf {args.seek_conf:g}); "
              f"stops {seek_cfg.stop_distance:g}m short, "
              f"max travel {seek_cfg.max_travel:g}m")
        if not args.enable_nl:
            print("  NOTE: --enable-seek without --enable-nl: use POST /seek "
                  "(the GUI's text box needs --enable-nl)")

    app = build_app(node, args.max_lin, args.max_ang, parser=parser,
                    nl_max_duration=args.nl_max_duration,
                    nl_max_distance=args.nl_max_distance,
                    nl_max_angle=args.nl_max_angle,
                    goal_timeout_max=args.goal_timeout_max,
                    nl_max_steps=args.nl_max_steps,
                    nl_max_chain_distance=args.nl_max_chain_distance,
                    nl_max_chain_angle=args.nl_max_chain_angle,
                    nl_max_chain_seconds=args.nl_max_chain_seconds,
                    link_check=bool(args.robot_ip),
                    detector=detector, seek_parser=seek_parser,
                    seek_cfg=seek_cfg)
    print(f"\n  Open the GUI at:  http://localhost:{args.port}\n")
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        node.stop()
        time.sleep(0.2)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
