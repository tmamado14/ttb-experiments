# TurtleBot3 Motion Server

A web app that lets you **drive a TurtleBot3 from your browser** and **watch its
camera** live. It runs on a laptop and talks to the robot over Wi-Fi using ROS 2.

![Web GUI](docs/images/gui.png)

## What's in here

| File | Purpose |
|------|---------|
| `motion_server.py`  | The whole app: ROS 2 node + web server + GUI (one file). |
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

This app uses only two of them: it **reads** `/image/compressed` and **writes**
`/cmd_vel`. The rest are published by the standard bringup and are available if
you want to extend things.

| Topic | Message type | Direction | Published by |
|-------|--------------|-----------|--------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | laptop → robot | **this app** (drives the motors) |
| `/image/compressed` | `sensor_msgs/msg/CompressedImage` | robot → laptop | `image_publisher` (custom, see `robot/`) |
| `/odom` | `nav_msgs/msg/Odometry` | robot → laptop | `diff_drive_controller` |
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
control panel:

```
move forward for 3 seconds
back up slowly
rotate left
spin right for 5 sec
halt
```

The text is parsed by a **local LLM** (ollama, default `qwen2.5:3b`) — nothing
leaves your laptop. Enable it in `.env`:

```bash
ENABLE_NL=1
LLM_URL=http://localhost:11434
LLM_MODEL=qwen2.5:3b
NL_MAX_DURATION=10
```

Requires `ollama serve` running with the model pulled (`ollama pull qwen2.5:3b`).

### How it stays safe

The LLM only **interprets** your words. It never touches the motors directly,
and every value it produces is re-checked in code before anything moves:

| Guard | Effect |
|-------|--------|
| Duration cap | Clamped to `NL_MAX_DURATION` (10 s). "Drive forward for 3 hours" becomes 10 s |
| Speed clamp | Velocities clamped to `MAX_LIN` / `MAX_ANG`, same limits as the D-pad |
| Fixed action list | Only forward, backward, rotate left/right, stop. Anything else is refused |
| Auto-stop | Every motion ends by itself — there is no "drive forever" |
| STOP wins | The STOP button, `Space` and `X` abort a typed motion instantly |
| Manual override | Pressing W/A/S/D takes control away from a running command |
| Deadman backstop | If the server dies mid-motion, the robot halts within 0.4 s |
| LLM down | Falls back to a refusal — never to uncommanded motion |

Expect **2–4 seconds** between pressing Enter and the robot moving; that's the
model parsing your text locally.

### Testing without moving the robot

`?dry_run=1` parses and clamps but never sends a motion command:

```bash
curl -s -X POST 'http://localhost:8000/nl?dry_run=1' \
     -H 'Content-Type: application/json' \
     -d '{"text":"drive forward for 3 hours"}'
# -> {"action":"forward","duration_s":10.0,"capped":true,...}
```

`GET /nl/status` reports whether a motion is running and how long is left.

## Security

Real secrets (the robot SSH password) live in `.env`, which is **git-ignored**
and never pushed. Only `.env.example` (placeholders) is committed.
