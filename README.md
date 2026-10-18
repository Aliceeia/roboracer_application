# AutoDRIVE RoboRacer Wall Follower

A ROS 2 wall-following node for the AutoDRIVE RoboRacer simulator.

- **Video:** TODO_YOUTUBE_LINK
- **Fastest lap:** TODO_LAP_TIME s

## How it works

The node is `autodrive_devkit/autodrive_roboracer/wall_follow_node.py`. It subscribes to
`/autodrive/roboracer_1/lidar` and publishes `std_msgs/Float32` to
`/autodrive/roboracer_1/steering_command` and `/autodrive/roboracer_1/throttle_command`.

- **Wall geometry:** two LiDAR beams per side (perpendicular, and 30° further forward) give the
  distance to each wall and the car's heading relative to it.
- **Steering:** `-(kp * offset + kh * heading)`. With both walls visible the car holds the middle of
  the track; the heading term damps the response so the car doesn't overshoot. With only one wall
  visible (a corner opening), the car only steers away from that wall when it gets too close.
- **Corners:** when the wall straight ahead is closer than 2 m, extra steering is added toward the
  more open side.
- **Side guard:** steers away when a wall beside or behind the car is closer than 0.4 m.
- **Speed:** throttle drops as the car leaves the center line, steers harder, or approaches a wall
  ahead.

## Run

From the AutoDRIVE-RoboRacer-Sim-Racing repo, with this `autodrive_devkit/` in place of the original:

```bash
cd /home/autodrive_devkit
colcon build
source install/setup.bash
ros2 launch autodrive_roboracer bringup_headless.launch.py
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/autodrive_devkit/install/setup.bash
ros2 run autodrive_roboracer wall_follow_node
```

Connect the simulator to `127.0.0.1:4567` and set Driving Mode to Autonomous.
