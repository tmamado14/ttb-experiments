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

## Security

Real secrets (the robot SSH password) live in `.env`, which is **git-ignored**
and never pushed. Only `.env.example` (placeholders) is committed.
