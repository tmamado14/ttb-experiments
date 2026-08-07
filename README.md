# TurtleBot3 Motion Server

A web app that lets you **drive a TurtleBot3 from your browser** and **watch its
camera** live. It runs on a laptop and talks to the robot over Wi-Fi using ROS 2.

![Web GUI](docs/images/gui.png)

## What's in here

| File | Purpose |
|------|---------|
| `motion_server.py`  | The app: ROS 2 node + web server + GUI. |
| `vision.py`         | Open-vocabulary object detection (YOLO-World) for "go to the X". |
| `seek.py`           | The seek behaviour: search, centre, approach — plus its target parser. |
| `run_server.sh`     | Starts the server on the laptop using values from `.env`. |
| `scripts/robot_start.sh` | Starts the robot's ROS nodes (motors + camera) over SSH. |
| `scripts/robot_stop.sh`  | Stops the robot's ROS nodes (leaves it powered on). |
| `.env.example`      | Template for configuration. Copy to `.env` and edit. |
| `fastdds_unicast.xml` | Optional fallback for networks that block multicast. |
| `robot/`            | Reference snapshot of the **robot-side** files (camera node, `start_all.sh`). See `robot/README.md`. |
| `docs/`             | Full LaTeX documentation and the compiled **PDF**. |

> **Read `docs/turtlebot3_motion_server.pdf` for the complete, beginner-friendly
> guide** (assumes no ROS knowledge).

## Quick start

```bash
# 1. Configure
cp .env.example .env      # then edit .env with your robot IP, password, etc.

# 2. Start the robot's nodes (motors + camera)
./scripts/robot_start.sh

# 3. Start the web server on the laptop
./run_server.sh

# 4. Open the GUI
#    http://localhost:8000
```

Drive with the on-screen buttons or **W/A/S/D** / arrow keys. Release to stop.

## ROS 2 topics

The laptop and robot talk over **topics** — named channels carrying typed
messages. Topics are *not* declared in config files; a node creates one in code
and it appears on the network. Everything below is live when the robot is up
(`ROS_DOMAIN_ID=203`).

This app uses three of them: it **reads** `/image/compressed` and `/odom`, and
**writes** `/cmd_vel`. The rest are published by the standard bringup and are
available if you want to extend things.

| Topic | Message type | Direction | Published by |
|-------|--------------|-----------|--------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | laptop → robot | **this app** (drives the motors) |
| `/image/compressed` | `sensor_msgs/msg/CompressedImage` | robot → laptop | `image_publisher` (custom, see `robot/`) |
| `/odom` | `nav_msgs/msg/Odometry` | robot → laptop | **`diff_drive_controller`** (this app reads it to measure distance and angle) |
| `/scan` | `sensor_msgs/msg/LaserScan` | robot → laptop | `hlds_laser_publisher` (LDS-01 lidar) |
| `/imu` | `sensor_msgs/msg/Imu` | robot → laptop | `turtlebot3_node` |
| `/magnetic_field` | `sensor_msgs/msg/MagneticField` | robot → laptop | `turtlebot3_node` |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | robot → laptop | `turtlebot3_node` |
| `/joint_states` | `sensor_msgs/msg/JointState` | robot → laptop | `turtlebot3_node` |
| `/sensor_state` | `turtlebot3_msgs/msg/SensorState` | robot → laptop | `turtlebot3_node` |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | robot → laptop | `robot_state_publisher` |
| `/robot_description` | `std_msgs/msg/String` | robot → laptop | `robot_state_publisher` |

`/cmd_vel` carries a `Twist`, of which the robot uses two fields: `linear.x`
(forward, m/s) and `angular.z` (turn, rad/s). A Burger tops out at 0.22 m/s and
2.84 rad/s — the `MAX_LIN` / `MAX_ANG` values in `.env`.

There is also one **service** (request/response, not a topic) defined in
`robot/turtlebot3_image_motion/srv/Motion.srv`. It belongs to the upstream
Jetson stack and is unused by this app.

### Checking topics yourself

```bash
export ROS_DOMAIN_ID=203
ros2 topic list                  # what's out there
ros2 topic type /cmd_vel         # its message type
ros2 topic echo /odom            # watch messages live
```

> **If `ros2 topic list` shows only `/parameter_events` and `/rosout`**, it's
> almost always a stale ROS daemon, *not* a network problem. Run
> `ros2 daemon stop`, or add `--no-daemon` to the command. Note that
> `ros2 topic hz` does not accept `--no-daemon` on Humble. Discovery can also
> take 20-30 s to settle after launch.

## Natural-language control

Instead of holding a key, you can **type what you want** into the box under the
control panel. You can say how *long* to move, how *far*, or how far to *turn*:

```
move forward for 3 seconds      back up slowly
move forward 1 meter            go forward a meter and a half
back up 50 cm                   drive forward 2 feet
rotate right 30 degrees         turn left 90 deg fast
spin right a quarter turn       turn around
halt
```

You can also **chain steps with commas** (or `then`, or `;`). Each one starts
only after the previous has finished:

```
move forward 0.5 m, turn right 90 degrees, move forward 0.3 m
go forward 1 meter then turn left 90 degrees
forward 1 m, right 90, forward 1 m, right 90
```

The text is parsed by a **local LLM** (ollama, default `qwen2.5:3b`) — nothing
leaves your laptop. Enable it in `.env`:

```bash
ENABLE_NL=1
LLM_URL=http://localhost:11434
LLM_MODEL=qwen2.5:3b
NL_MAX_DURATION=10              # seconds  } per step
NL_MAX_DISTANCE=2.0             # meters   }
NL_MAX_ANGLE=360                # degrees  }
NL_MAX_STEPS=5                  # steps         } per sequence
NL_MAX_CHAIN_DISTANCE=3.0       # meters total  }
NL_MAX_CHAIN_ANGLE=720          # degrees total }
NL_MAX_CHAIN_SECONDS=120        # worst case    }
```

Requires `ollama serve` running with the model pulled (`ollama pull qwen2.5:3b`).

### Distance and angle are measured, not timed

A time-based command is open-loop: it drives for N seconds and hopes. A
**distance or angle command is closed-loop** — the server subscribes to `/odom`
and watches how far the robot has actually gone, easing off as it approaches the
goal and stopping when it arrives. Battery level, floor surface and motor
deadband stop mattering, because nothing is being guessed from a stopwatch.

Two consequences worth knowing:

- **Distance and angle commands need `/odom`.** If the robot's bringup isn't
  running, they are *refused* with a message saying so. They never silently fall
  back to "distance ÷ speed = time" — a guess presented as a measurement is
  exactly how a robot surprises you. Time-based commands still work, since they
  need no odometry.
- **Expect ±2 cm and ±5°** on a hard floor. Carpet is worse. That figure is
  measured against the robot's *own wheel odometry*, which itself drifts 1–3 %,
  so it is not a guarantee about absolute heading — over a full 360° spin the
  true heading can be off by several degrees even when odometry reports exactly
  360.0.

### How it stays safe

The LLM only **interprets** your words. It never touches the motors directly,
and every value it produces is re-checked in code before anything moves:

| Guard | Effect |
|-------|--------|
| Duration cap | Clamped to `NL_MAX_DURATION` (10 s). "Drive forward for 3 hours" becomes 10 s |
| Distance cap | Clamped to `NL_MAX_DISTANCE` (2 m). "Move forward 5 meters" becomes 2 m |
| Angle cap | Clamped to `NL_MAX_ANGLE` (360°). "Rotate 1000 degrees" becomes 360° |
| Chain budget | A sequence is **refused** (not trimmed) if it exceeds 5 steps, 3 m of path, 720°, or 120 s worst case. The caps above are per *step*, so without this three 2 m legs would pass every check and drive 6 m |
| Chain aborts on failure | If any step ends for any reason other than reaching its goal, the remaining steps are abandoned — a turn that stopped short would leave every later step pointing the wrong way |
| Mid-sequence stop | Refused. "forward 1 m, halt, turn left" is ambiguous, and truncating after the robot already moved is the failure that validating everything up front exists to prevent |
| Speed clamp | Velocities clamped to `MAX_LIN` / `MAX_ANG`, same limits as the D-pad |
| Fixed action list | Only forward, backward, rotate left/right, stop. Anything else is refused |
| Auto-stop | Every motion ends by itself — there is no "drive forever" (see below) |
| Goal timeout | A distance/angle goal also carries a wall-clock deadline (≈3× the ideal time + 2 s, ceiling `GOAL_TIMEOUT_MAX`). Slipping wheels can't turn "1 meter" into forever |
| Progress watchdog | Less than 2 mm or 1° of progress for 1.5 s aborts the motion. Blocked wheels stop it in under 2 s rather than waiting out the timeout |
| Odometry watchdog | If `/odom` goes quiet for 0.5 s mid-motion, the robot stops |
| No odometry | Distance/angle commands are refused outright, never estimated open-loop |
| STOP wins | The STOP button, `Space` and `X` abort a typed motion instantly |
| Manual override | Pressing W/A/S/D takes control away from a running command |
| Deadman backstop | If the server dies mid-motion, the robot halts within 0.4 s |
| LLM down | Falls back to a refusal — never to uncommanded motion |

A closed-loop goal is the one thing that could break the "auto-stop" promise: a
motion that stops when the robot *arrives* will never stop if the robot can't
get there. That's why a distance or angle command carries three independent
endings — reaching the goal, the progress watchdog, and the timeout — with the
0.4 s deadman underneath all of them.

> **One gap, stated plainly:** none of this catches a robot that has been
> **picked up**. TurtleBot3 odometry comes from wheel encoders, so wheels
> spinning in the air report perfect progress, and a lifted robot will happily
> "complete" a 1 metre move. The watchdogs catch *blocked* and *odometry-dead*,
> not *airborne*. (This is also what makes testing on blocks so convenient.)

> **Chaining is sequential convenience, not a path-accurate trajectory.**
> Each step is measured from where the robot *thinks* it is when that step
> begins, so errors compound. A 5° heading error left over from step 2 rotates
> the whole of step 3's displacement — on a 0.3 m leg that lands you ~2.6 cm to
> the side, and it grows with the length of the chain. Four 90° turns will not
> reliably close a square. If you need a path the robot actually follows, you
> need a navigation stack with a map and a localiser; this is dead reckoning
> with good manners.

Expect **2–4 seconds** between pressing Enter and the robot moving; that's the
model parsing your text locally.

### Testing without moving the robot

`?dry_run=1` parses and clamps but never sends a motion command:

```bash
curl -s -X POST 'http://localhost:8000/nl?dry_run=1' \
     -H 'Content-Type: application/json' \
     -d '{"text":"drive forward for 3 hours"}'
# -> {"action":"forward","mode":"duration","value":10.0,"capped":true,...}

curl -s -X POST 'http://localhost:8000/nl?dry_run=1' \
     -H 'Content-Type: application/json' \
     -d '{"text":"move forward 5 meters"}'
# -> {"action":"forward","mode":"distance","value":2.0,"unit":"m","capped":true,...}
```

Dry runs work with the robot switched off, so you can develop phrasings without
it. A second server on a topic the robot doesn't subscribe to is also inert:

```bash
python3 motion_server.py --cmd-vel-topic /cmd_vel_test --port 8001 \
        --image-topic /nonexistent --image-type raw --enable-nl
```

Nothing moves, but `/odom` is real — so every closed-loop goal ends in the
progress watchdog, which is a direct test that the backstops fire.

`GET /nl/status` reports whether a motion is running, how far it has got
(`progress` / `goal` / `unit`), which step of a sequence it's on (`step` /
`steps`), and `last_result` — why the previous motion ended (`done`,
`cancelled`, `timed out`, `no progress`, `odometry stalled`) and **at which
step**.

A neat property of the dummy-topic instance: an **all-duration chain is
open-loop, so it runs to completion with the robot stationary**. That gives a
real end-to-end test of the sequencing — including that the settle pause between
steps actually happens — without a wheel turning. Mix in one distance or angle
step and you get a genuine mid-chain failure to check that the steps after it
are abandoned.

### Where a chained command can surprise you

Clauses are split on punctuation before the model ever sees them, one clause per
model call. That keeps failures attributable to a specific fragment, but it has
two consequences worth knowing:

- **A fragment with no movement word in it is treated as a qualifier and glued
  to its neighbour.** That's what makes "move forward 1 m, slowly" work as one
  command — but it also means "forward 1 m, banana, turn left" silently absorbs
  the `banana` rather than complaining about it.
- **Elliptical clauses don't carry context.** "forward 1 m, then the same again"
  has no amount in step 2, so it falls back to the 2-second default; "turn right
  90, then left" gives you a 2-second left turn, not a 90° one. Spell each step
  out in full.

## Going to an object: "go to the bottle"

Set `ENABLE_SEEK=1` in `.env` (you also need a working `/scan` and YOLO-World
weights at `SEEK_WEIGHTS`). Then, **in the GUI's "Say it in plain English" box**,
type **"go to the bottle"**, "find the red backpack", or "approach the chair".

The robot rotates in place until the camera finds the object, turns to face it,
drives at it, and stops short of it.

It is the same text box the driving commands use — routing happens on the server,
so there is nothing to switch between "drive" and "go to". When seek is enabled
the box shows clickable example phrases underneath, and the readout above reports
the stage in plain words as it runs:

```
looking around · bottle
turning to face it · bottle · seen 44%
driving to it · bottle · 0.47 m away · seen 40%
arrived · bottle · 0.35 m away
```

**"show what it sees"** under the camera swaps the live view for the same stream
with detection boxes drawn on it, plus the centre line the robot aims at. Type
any object name beside it. The robot does not move while you look — this is the
quickest way to find out whether an object is detectable in your lighting before
asking the robot to drive at it.

### The language model does not do the seeing

It extracts the object's **name** from your sentence and nothing else. That is
the same rule the driving commands follow — the model is an intent parser and
never a control or safety element.

Finding the object is a real object detector (YOLO-World), because a
vision-language model answers in prose rather than pixel coordinates and takes
seconds per frame; the robot is moving the whole time it would be thinking.
Detection here is ~19 ms/frame on a GPU.

**How far away it is comes from the lidar, not the camera.** A single camera
cannot measure distance without knowing how big the object is supposed to be.

### Bearing from vision, range from the lidar

The two are deliberately kept apart:

1. **Search** — turn `SEEK_SEARCH_STEP` degrees, stop, look. Step-and-look
   rather than a continuous sweep, because the camera feed is ~7 Hz over Wi-Fi
   and a moving frame is a blurred one.
2. **Centre** — rotate until the object sits on the middle of the frame.
3. **Approach** — drive forward, steering to keep it centred, watching the
   range straight ahead. Stop at `SEEK_STOP_DISTANCE`.

Centring first is what removes any need to calibrate the camera's field of
view: once the object is dead ahead, "how far away is it" is just "what is in
front of me", which the lidar answers directly.

### Why the range is a minimum and not an average

`front_range()` reports the **20th percentile** of the valid beams in a ±10°
sector — a robust minimum. Measured on this robot, that sector read a *minimum*
of 1.97 m against a *median* of 2.60 m: the median was describing the wall
behind the object while something sat two thirds of a metre nearer. For "stop
before you hit it", the nearest thing in the path is the only correct quantity.
A percentile rather than a bare minimum because single spurious short returns
are common — a live scan had only 251 of 360 beams valid, with a `0.0` dropout
sitting dead ahead.

### How it stays safe

Everything the typed motions already do (20 Hz publisher, 0.4 s deadman, one
cancellable motion at a time, STOP preempting) applies unchanged — the seek runs
on the *same* motion thread under the *same* cancel event, precisely so that
STOP keeps meaning stop. On top of that:

- **No lidar, no seek.** If `/scan` is missing or stale it refuses to start, and
  aborts mid-approach if the lidar goes quiet. It never falls back to open loop.
- **No range, no driving.** If the front sector returns nothing readable for
  0.6 s, it stops. "I cannot see anything ahead" is not the same claim as "the
  way is clear."
- **Caps are refused, not trimmed**: `SEEK_MAX_TRAVEL` of approach,
  `SEEK_SEARCH_MAX` of sweep, `SEEK_TIMEOUT` overall.
- **Two of three frames** must contain the target before the robot will drive at
  it. Open-vocabulary detection will happily put a low-confidence box on a noun
  that isn't there.

### Check the detector before you trust it

`/detect` runs detection on the current frame and returns it annotated, **without
moving the robot**. This is the first thing to try in a new room:

```bash
curl -s -D - -o out.jpg "http://localhost:8000/detect?target=bottle" | grep -i x-
# X-Detections: 1   X-Best-Conf: 0.412   X-Infer-Ms: 19   X-Front-Range-M: 1.987
```

The confidence it reports is the number `SEEK_CONF` has to sit below. It is very
lighting-dependent: in this room a chair in plain view scored 0.13–0.24, so the
usual 0.25 threshold would have found *nothing*, while objects that were absent
stayed under 0.07. Re-measure rather than assuming the default fits.

To dry-run the language half without moving:

```bash
curl -s -X POST "localhost:8000/seek?dry_run=1" -H 'Content-Type: application/json' \
     -d '{"text":"go to the red backpack"}'
# -> {"action":"seek","target":"red backpack","executed":false,"dry_run":true,...}
```

### Limits worth knowing before you rely on it

- **The lidar only sees its own scan plane, ~18 cm up.** A bottle on a table is
  detected by the camera and returns no range, so the approach refuses to drive.
  Floor-level objects are what this is built for.
- **The LDS-01 only sees 3.5 m at all**, which is why the travel cap is 2.5 m.
- **The camera's horizontal field of view is its narrow axis** (it is mounted
  rotated), so the search steps are small. Turning `SEEK_SEARCH_STEP` up makes
  searching quicker and makes it walk straight past things.
- **A seek can't be chained** with other steps yet — "forward 0.5 m, go to the
  bottle" is refused rather than half-executed.
- **Nothing here avoids obstacles.** It drives at the target in a straight line
  and stops for whatever is nearest in front — which may not be the target.

## Security

Real secrets (the robot SSH password) live in `.env`, which is **git-ignored**
and never pushed. Only `.env.example` (placeholders) is committed.
