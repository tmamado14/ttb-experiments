# Robot-side files (reference snapshot)

These files run **on the TurtleBot3**, not on the laptop. They are copied here so
this repo documents the full picture — but they are **not the source of truth**.

> **Upstream:** <https://github.com/SuperMadee/AI361-MEX3>
> **Snapshot of commit:** `54fb8a2` (2026-03-21)
> Package path on the robot: `~/ros2_ws/src/AI361-MEX3/turtlebot3_image_motion`

**Edit them upstream, not here.** If you change a file in this folder, the robot
will not pick it up — the robot builds from its own clone in `~/ros2_ws`.
Re-copy the snapshot after upstream changes rather than editing in place.

Only the parts this teleop repo actually depends on were copied. The upstream
package also contains `mcp_bridge.py`, `web_controller.py`, `motion_server.py`
and `cmd_vel_watchdog.py` — the Jetson-side AI/MCP stack (~15k lines), which is
unrelated to browser teleop and is deliberately **not** duplicated here.

## What's here

| File | Purpose |
|------|---------|
| `turtlebot3_image_motion/turtlebot3_image_motion/image_publisher.py` | The camera node. Reads `/dev/video0` with OpenCV, rotates 90° CW, JPEG-encodes, publishes `/image/compressed`. |
| `turtlebot3_image_motion/srv/Motion.srv` | A ROS 2 **service** definition (not a topic) — used by the upstream Jetson stack, not by this repo's web GUI. |
| `turtlebot3_image_motion/package.xml` | Package manifest (dependencies, build type). |
| `turtlebot3_image_motion/CMakeLists.txt` | Build file — also generates the `Motion.srv` interface. |
| `start_all.sh` | Lives at `~/start_all.sh` on the robot. Kills stale nodes, then launches bringup + the camera node, both detached. |

## The camera node

`image_publisher.py` is the only robot-side file this repo genuinely depends on
— it produces the video the GUI displays. Its parameters (declared at
`image_publisher.py:21-27`) and their current values:

| Parameter | Default | Notes |
|-----------|---------|-------|
| `camera_index` | `0` | Opens `/dev/video0`. |
| `fps` | `30.0` | 30 fps at the camera; ~7 Hz is what survives the Wi-Fi hop. |
| `image_topic` | `/image/compressed` | Must match `IMAGE_TOPIC` in the laptop's `.env`. |
| `width` / `height` | `640` / `480` | Capture resolution, before rotation. |
| `jpeg_quality` | `70` | Lower = less bandwidth, more artifacts. |
| `rotate_90` | `true` | Corrects the physical camera mounting. |

It tries the V4L2 backend first and falls back to OpenCV's default backend, then
auto-reconnects if the camera drops out.

## Running it by hand

`scripts/robot_start.sh` (in the repo root) writes and runs `start_all.sh` over
SSH, so you normally never touch this. To do it manually on the robot:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=203 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-01
ros2 launch turtlebot3_bringup robot.launch.py    # motors, lidar, odometry
ros2 run turtlebot3_image_motion image_publisher  # camera
```

Both must be launched **detached** (`nohup setsid … &`) — the SSH connection
drops the moment `ros2 launch` starts flooding DDS. The robot itself is fine;
only the SSH session dies.
