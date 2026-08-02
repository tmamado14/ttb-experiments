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
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, JSONResponse
from pydantic import BaseModel


def clamp(value, lo, hi):
    """Constrain value to [lo, hi]. Every velocity and duration goes through this."""
    return max(lo, min(hi, value))


# --------------------------------------------------------------------------- #
# ROS 2 node
# --------------------------------------------------------------------------- #
class MotionNode(Node):
    def __init__(self, image_topic, cmd_vel_topic, image_type="auto",
                 robot_ip=None, cmd_timeout=0.4):
        super().__init__("motion_server")
        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.cmd_vel_topic = cmd_vel_topic
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

        # Target velocity + deadman timestamp.
        self._vel_lock = threading.Lock()
        self._target = Twist()
        self._last_cmd_time = time.time()

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

# Constrains the model's output at the decoder level, so it can only ever emit
# one of these actions. Values are still re-validated and clamped below --- the
# model is an intent parser, never a safety layer.
NL_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "duration_s": {"type": "number"},
        "speed": {"type": "string", "enum": ["slow", "normal", "fast"]},
    },
    "required": ["action", "duration_s", "speed"],
}

NL_SYSTEM_PROMPT = """\
You control a TurtleBot3 robot. Convert the user's driving instruction into JSON.

action: forward | backward | rotate_left | rotate_right | stop | unknown
  Use 'unknown' ONLY if the text is not a driving instruction.
duration_s: how many seconds to move. Default 2. Maximum {max_duration}.
speed: slow | normal | fast. Default normal.

Examples:
  "move forward"            -> forward, 2, normal
  "back up slowly"          -> backward, 2, slow
  "rotate left 3 seconds"   -> rotate_left, 3, normal
  "halt"                    -> stop, 0, normal
"""


class Intent(BaseModel):
    """Validated result of a parse. Pydantic rejects anything off-enum."""
    action: Literal["forward", "backward", "rotate_left",
                    "rotate_right", "stop", "unknown"]
    duration_s: float
    speed: Literal["slow", "normal", "fast"] = "normal"


class NLRequest(BaseModel):
    """Body of POST /nl."""
    text: str = ""


class NLParser:
    """Turns free text into an Intent using a local ollama model."""

    def __init__(self, url, model, max_duration, timeout=30.0):
        self.url = url.rstrip("/")
        self.model = model
        self.max_duration = max_duration
        self._client = httpx.Client(timeout=timeout)
        self._system = NL_SYSTEM_PROMPT.format(max_duration=max_duration)

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
    keep arriving. So this re-asserts the target velocity every 0.1 s for the
    requested duration, then stops. The deadman stays in place underneath as the
    backstop: if this thread dies, the robot halts within 0.4 s.
    """

    SPEED_FRAC = {"slow": 0.4, "normal": 0.7, "fast": 1.0}

    def __init__(self, node: MotionNode, max_lin, max_ang, max_duration):
        self._node = node
        self._max_lin = max_lin
        self._max_ang = max_ang
        self._max_duration = max_duration
        self._lock = threading.Lock()
        self._thread = None
        self._run_state = None
        self._active = None
        self._ends_at = 0.0

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

    def start(self, action, duration_s, speed):
        """Begin a motion. Returns (duration, lin, ang) after clamping."""
        duration = clamp(float(duration_s), 0.0, self._max_duration)
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

        # Replace any in-flight motion; don't stop, we're about to drive.
        self._abort(stop_on_exit=False)

        state = {"cancel": threading.Event(), "stop_on_exit": True}
        thread = threading.Thread(target=self._run, args=(state, lin, ang, duration),
                                  daemon=True)
        with self._lock:
            self._run_state = state
            self._thread = thread
            self._active = {"action": action, "duration_s": duration,
                            "speed": speed, "lin": round(lin, 3),
                            "ang": round(ang, 3)}
            self._ends_at = time.time() + duration
        thread.start()
        return duration, lin, ang

    def _run(self, state, lin, ang, duration):
        end = time.time() + duration
        try:
            while not state["cancel"].is_set() and time.time() < end:
                self._node.set_velocity(lin, ang)
                state["cancel"].wait(0.1)   # returns immediately once cancelled
        finally:
            if state["stop_on_exit"]:
                self._node.stop()
            with self._lock:
                if self._run_state is state:
                    self._run_state = None
                    self._thread = None
                    self._active = None

    def status(self):
        with self._lock:
            active = dict(self._active) if self._active else None
            ends_at = self._ends_at
        if active is not None:
            active["remaining_s"] = max(0.0, round(ends_at - time.time(), 1))
        return {"running": active is not None, "motion": active}


# --------------------------------------------------------------------------- #
# Web server
# --------------------------------------------------------------------------- #
def build_app(node: MotionNode, max_lin, max_ang,
              parser: Optional[NLParser] = None, nl_max_duration=10.0,
              link_check=False):
    app = FastAPI()
    executor = MotionExecutor(node, max_lin, max_ang, nl_max_duration)

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

        intent, error = parser.parse(text)
        if error is not None:
            return JSONResponse({"executed": False, "action": "unknown",
                                 "message": error})

        requested = intent.duration_s
        duration = clamp(float(requested), 0.0, nl_max_duration)
        capped = abs(duration - requested) > 1e-6

        result = {"action": intent.action, "speed": intent.speed,
                  "duration_s": duration, "requested_duration_s": requested,
                  "capped": capped, "executed": False}

        if intent.action == "unknown":
            result["message"] = f'"{text}" is not a driving command I understand'
            return JSONResponse(result)

        if intent.action == "stop":
            if not dry_run:
                executor.cancel(stop=True)
            result["executed"] = not dry_run
            result["message"] = "stopping"
            return JSONResponse(result)

        # Refuse to drive blind when the robot link is known to be down.
        if link_check and node.link_rtt_ms is None and not dry_run:
            result["message"] = "robot link is down - not sending a motion command"
            return JSONResponse(result)

        if dry_run:
            result["message"] = f"[dry run] {intent.action} for {duration:g}s ({intent.speed})"
            return JSONResponse(result)

        duration, lin, ang = executor.start(intent.action, duration, intent.speed)
        result.update({"executed": True, "duration_s": duration,
                       "lin": round(lin, 3), "ang": round(ang, 3)})
        note = f" (capped from {requested:g}s)" if capped else ""
        result["message"] = (f"{intent.action.replace('_', ' ')} for "
                             f"{duration:g}s at {intent.speed} speed{note}")
        return JSONResponse(result)

    @app.get("/nl/status")
    def nl_status():
        return JSONResponse({"enabled": parser is not None,
                             "model": parser.model if parser else None,
                             "max_duration_s": nl_max_duration,
                             **executor.status()})

    @app.get("/status")
    def status():
        return JSONResponse({
            "frames": node.frames_received,
            "image_topic": node.image_topic,
            "cmd_vel_topic": node.cmd_vel_topic,
            "max_lin": max_lin,
            "max_ang": max_ang,
            "link_rtt_ms": node.link_rtt_ms,
            "cmd_cycle_ms": 50,  # 20 Hz command publisher
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
</style>
</head>
<body>
  <h1>TurtleBot3 Motion Server</h1>
  <div class="wrap">
   <div class="camwrap">
    <img id="cam" src="/video" alt="camera stream"/>
    <div id="status">connecting…</div>
   </div>

   <div class="panel">
    <div id="vel">v = 0.00 m/s &nbsp; ω = 0.00 rad/s</div>
    <div id="lat" title="Time from a teleop command to the robot acting on it: network link + 50 ms command cycle + motor response.">teleop latency: measuring…</div>
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
               placeholder="e.g. move forward for 3 seconds"/>
        <button id="nlsend">Send</button>
      </div>
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
  try {
    const r = await fetch('/nl', {method:'POST',
      headers:{'Content-Type':'application/json'}, body: JSON.stringify({text:t})});
    const d = await r.json();
    pending.remove();
    logLine(d.executed ? 'ok' : 'no', d.message || 'no response');
  } catch (e) {
    pending.remove();
    logLine('no', 'server unreachable');
  } finally {
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
  if (NL_ENABLED) {
    try {
      const m = (await (await fetch('/nl/status')).json()).motion;
      if (m) velEl.textContent =
        `${m.action.replace('_',' ')} · ${m.remaining_s.toFixed(1)}s left ` +
        `(v = ${m.lin.toFixed(2)}  ω = ${m.ang.toFixed(2)})`;
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
    ap.add_argument("--robot-ip", default="192.168.68.100",
                    help="robot IP, used to measure live teleop link latency "
                         "(set empty to disable)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--max-lin", type=float, default=0.26,
                    help="max linear speed (Burger=0.22, Waffle=0.26)")
    ap.add_argument("--max-ang", type=float, default=1.82,
                    help="max angular speed (Burger=2.84, Waffle=1.82)")
    ap.add_argument("--enable-nl", action="store_true",
                    help="enable natural-language driving via a local LLM")
    ap.add_argument("--llm-url", default="http://localhost:11434",
                    help="ollama base URL")
    ap.add_argument("--llm-model", default="qwen2.5:3b",
                    help="ollama model used to parse commands")
    ap.add_argument("--nl-max-duration", type=float, default=10.0,
                    help="hard cap (seconds) on any single typed motion")
    args = ap.parse_args()

    rclpy.init()
    node = MotionNode(args.image_topic, args.cmd_vel_topic, args.image_type,
                      robot_ip=(args.robot_ip or None))

    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    parser = None
    if args.enable_nl:
        parser = NLParser(args.llm_url, args.llm_model, args.nl_max_duration)
        print(f"  Natural language: {args.llm_model} via {args.llm_url} "
              f"(max {args.nl_max_duration:g}s per command)")

    app = build_app(node, args.max_lin, args.max_ang, parser=parser,
                    nl_max_duration=args.nl_max_duration,
                    link_check=bool(args.robot_ip))
    print(f"\n  Open the GUI at:  http://localhost:{args.port}\n")
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        node.stop()
        time.sleep(0.2)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
