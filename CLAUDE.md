# Working notes for this repo

A laptop-side ROS 2 + FastAPI app that drives a TurtleBot3 Burger over Wi-Fi:
teleop, a camera stream, natural-language driving, and "go to the bottle".

## Layout

| File | Role |
|---|---|
| `motion_server.py` | ROS 2 node + FastAPI + GUI. `MotionNode` (sensors, deadman publisher), `MotionExecutor` (one bounded cancellable motion), HTTP layer, `HTML_PAGE`. |
| `vision.py` | `Detector` — YOLO-World, open-vocabulary. Pure perception. |
| `seek.py` | `SeekParser` (sentence → target noun) and `SeekBehaviour` (search → centre → approach). |
| `scripts/robot_start.sh` / `robot_stop.sh` | Start/stop the robot's ROS nodes over SSH. |
| `robot/` | Read-only *snapshot* of robot-side code. Editing it does nothing — the robot builds from its own repo (`AI361-MEX3`). |

## Invariants — don't break these

- **One motion at a time, on one thread, under one cancel event.** `MotionExecutor`
  owns this. Seek runs *through* it (`start_seek`), not beside it. A second
  writer to `set_velocity()` breaks STOP.
- **The 0.4 s deadman is the backstop.** Any control loop must re-assert velocity
  faster than that. If a thread dies, the robot stops.
- **The LLM is an intent parser, never a control or safety element.** Every value
  it produces is clamped or refused in code. It never sees a pixel.
- **Caps are refused, not clamped**, for anything the user could be surprised by.
- **Refuse rather than fall back to open loop.** No `/odom` → no distance goal.
  No `/scan` → no approach. "I can't see anything ahead" ≠ "the way is clear".

## Hardware facts, all measured — re-measure, never assume

- **Camera latency dominates everything.** The publisher defaults to 30 FPS over a
  link carrying ~7; the backlog *is* the lag. Measured **2.10 s at 30 FPS vs
  0.38 s at 5 FPS**. `CAMERA_FPS` in `.env` fixes it. Any closed loop on vision
  must be slower than this latency implies — that includes `SETTLE_AFTER_STEP`
  and `CENTER_MAX_ANG`.
- **`/scan` needs BEST_EFFORT QoS** or you receive *nothing*, which looks exactly
  like a dead lidar. It runs at **~5 Hz, not 20**. Range 0.12–3.5 m. Typically
  only ~251 of 360 beams are valid; dropouts read `0.0`.
- **Lidar sees only its ~18 cm plane.** Objects on tables are detectable by camera
  and unrangeable. Floor-level objects only.
- **The camera is dim** (mean luma 63/255). A chair in plain view scores 0.13–0.24,
  so the stock 0.25 detector threshold finds *nothing*. Default is 0.10.
- **Detection is intermittent** even when stationary — a bag at 0.12 was missing
  from 5 of 12 consecutive frames. Grace periods are written in seconds but what
  they buy is *frames*; changing the camera FPS silently retunes all of them.

## Debugging playbook

- **Log the filtered value beside the raw one.** `/status` reports raw
  `front_range_m`; the seek reports its filtered `range_m`. Comparing them caught
  the worst bug in this repo in a single run.
- **`odom_age_ms` climbing while `scan_age_ms` stays normal** = `turtlebot3_node`
  has crashed. It publishes `/odom` and `/battery_state` and consumes `/cmd_vel`,
  but the lidar and camera are *separate processes* and keep publishing, so
  everything else looks healthy. Restart the robot's bringup. Usually follows
  battery sag; the crash itself (`stack smashing detected` after a Dynamixel
  read failure) is an upstream ROBOTIS bug.
- **Discovery takes up to ~60 s.** A partial `ros2 topic list` right after bringup
  is normal — retry past a minute before suspecting the network. A stale `ros2`
  daemon hides everything: use `--no-daemon`.
- **Remote `pkill -f` matches the shell running it.** Killing "ros2 launch" over
  SSH kills your own session mid-script. Filter candidates through `/proc` for a
  sentinel; `$$` is not enough, because command substitution forks subshells that
  inherit the parent's argv.

## Testing without risking the robot

- `GET /detect?target=X` — annotated frame, no motion. First thing to try in a new room.
- `POST /nl?dry_run=1` or `POST /seek?dry_run=1` — parse only.
- `--cmd-vel-topic /cmd_vel_test` — the robot doesn't subscribe, so the whole
  state machine runs with zero physical risk.
- The GUI's **"show what it sees"** toggle streams live detections, stationary.

## Style

Match the surrounding code: comments explain *why*, especially where a value was
chosen by measurement — record the measurement, not just the number. Several
constants here look arbitrary and are not; the comment is the evidence.
