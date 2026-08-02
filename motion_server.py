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
import socket
import threading
import time

import cv2
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
# Web server
# --------------------------------------------------------------------------- #
def build_app(node: MotionNode, max_lin, max_ang):
    app = FastAPI()

    @app.get("/", response_class=HTMLResponse)
    def index():
        return HTML_PAGE

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
        # Clamp to configured maxima.
        lin = max(-max_lin, min(max_lin, lin))
        ang = max(-max_ang, min(max_ang, ang))
        node.set_velocity(lin, ang)
        return JSONResponse({"lin": lin, "ang": ang})

    @app.post("/stop")
    def stop():
        node.stop()
        return JSONResponse({"stopped": True})

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
window.addEventListener('keydown', e => {
  let k = e.key.toLowerCase(); k = keymap[k] || k;
  if (k === 'x' || k === ' ') { e.preventDefault(); held.clear(); apply(); return; }
  if ('wasd'.includes(k)) { e.preventDefault();
    if (!held.has(k)) { held.add(k); apply(); } }
});
window.addEventListener('keyup', e => {
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
    args = ap.parse_args()

    rclpy.init()
    node = MotionNode(args.image_topic, args.cmd_vel_topic, args.image_type,
                      robot_ip=(args.robot_ip or None))

    spin = threading.Thread(
        target=lambda: rclpy.spin(node), daemon=True)
    spin.start()

    app = build_app(node, args.max_lin, args.max_ang)
    print(f"\n  Open the GUI at:  http://localhost:{args.port}\n")
    try:
        uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    finally:
        node.stop()
        time.sleep(0.2)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
