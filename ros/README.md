# ROS 2 Rover Control (Docker)

This directory contains a ROS 2 workspace that bridges `/cmd_vel` to the existing rover control API running in `main.py`.

## Layout

- `ws/`: ROS 2 workspace
- `run_ros_docker.sh`: Start a ROS Jazzy container from this directory
- `start_rover_bridge.sh`: Build workspace and launch the rover bridge

## 1) Start rover server on the Pi host

From project root:

```bash
python3 main.py
```

`start_rover_bridge.sh` auto-detects the Pi IP and keeps a live endpoint file at `ros/.server_url`.

## 2) Start bridge in Docker

From this `ros/` directory:

```bash
./start_rover_bridge.sh
```

This will:

1. Start container `rover_ros2`
2. Build `/ros_ws`
3. Launch `rover_control/rover_cmd_vel.launch.py`
4. Keep `ros/.server_url` refreshed (default every 2s), so the bridge can switch if host IP changes

If you want to force a specific rover API endpoint:

```bash
ROVER_SERVER_URL=http://192.168.29.210:5000 ./start_rover_bridge.sh
```

Tune endpoint refresh interval:

```bash
ROVER_IP_REFRESH_SEC=1 ./start_rover_bridge.sh
```

## 3) Send test commands

From another host terminal, open a shell in the running container:

```bash
docker exec -it rover_ros2 bash
```

Then run:

```bash
source /opt/ros/jazzy/setup.bash
source /ros_ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.35}, angular: {z: 0.0}}" -r 10
```

Stop command:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" -1
```

## Optional launch parameters

```bash
ros2 launch rover_control rover_cmd_vel.launch.py max_speed_pct:=50 min_speed_pct:=20 forward_gear:=1
```

## Notes

- This bridge uses the existing HTTP control endpoints (`/gear`, `/steer`, `/forward`, `/stop`, `/speed_limit_enable/<speed>`).
- The bridge watches `server_url_file` (`/app/ros/.server_url`) and fails over dynamically if the host IP changes.
- On first motion command, the bridge auto-arms control by calling `/engine/start` and `/emergency_brake/off`.
- Safety timeout stops the rover if `/cmd_vel` is not received for `0.6s`.

## Quick troubleshooting

- If `ros2 topic pub` shows messages but wheels don't move, restart the host server once to load latest endpoints:

```bash
cd /home/pi/rpi_car
pkill -f "python3 main.py" || true
python3 main.py
```

- You can verify control endpoints manually:

```bash
curl http://127.0.0.1:5000/engine/start
curl http://127.0.0.1:5000/emergency_brake/off
curl http://127.0.0.1:5000/gear/1
curl http://127.0.0.1:5000/speed_limit_enable/35
curl http://127.0.0.1:5000/forward
```
