# Sassy 2 wheel lidar navigation

Voice-controlled indoor navigation for the two-wheel differential-drive robot.

Sassy is based on the structure and lessons from `cub-queso`, but uses the
current Uno firmware protocol in `firmware/sassy_drive.ino` as its hardware
contract. The target platform is an Orin Nano/Jetson running ROS 2 Humble and
an Arduino Uno motor controller.

## Current status

- Audio input and microphone: working (reported 2026-09-08)
- Uno motor sketch: uploaded and captured in `firmware/sassy_drive.ino`
- Serial bridge: scaffolded for `C`, `D`, `S`, and `H` frames
- D6 LiDAR and OAK-D Lite live dashboard: working on the Orin Nano
- Supervised 5-second obstacle-avoidance trial: turns away from a measured wall,
  then resumes forward travel when the forward arc clears
- House mapping, localization, and voice navigation: next milestones

## Live sensors and motor trials

On the Orin Nano, start `python3 tools/lidar_live_web.py` and open port 8080.
The dashboard displays LiDAR, OAK RGB, and stereo depth. Its guarded trial
button currently runs for 5 seconds. LiDAR bearing and range choose the turn;
OAK depth is displayed and logged as supporting information.

The current hardware uses `/dev/ttyTHS1` for the D6 LiDAR and `/dev/leia-uno`
for the Uno. The tested full-power motor commands are `D,255,0` for a right
pivot, `D,0,255` for a left pivot, and `D,255,255` for forward travel. The Uno
watchdog stops the motors after 250 ms without another command. The controller
refreshes commands about every 80 ms and sends `S` when the trial ends.

For a standalone motor check, run `python3 tests/motor_wiggle_test.py --run`.
For the four one-wheel turns, run `python3 tests/motor_turn_test.py` in a clear
area. Save one four-pane sensor image with
`python3 tools/capture_navigation_snapshot.py` while the dashboard is running.
Generated videos, images, and device crash dumps stay local under `artifacts/`
and `.cache/`.

## Architecture

```text
[Microphone] -> voice_nav -> /sassy/nav_goal -> navigation/controller
[2D lidar] -----------------> SLAM/localization -> obstacle-aware controller
                                                      |
                                              /sassy/drive_cmd
                                                      v
                                               serial_bridge -> Uno
```

The first navigation milestone should be teleoperation plus lidar recording;
autonomous house navigation should only be enabled after a map and emergency
stop behavior have been validated.

## Quick start

```bash
cd ~/sassy-2-wheel-lidar-navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run sassy_serial_bridge serial_bridge --ros-args -p serial_port:=/dev/ttyACM0
```

The Uno accepts newline-terminated frames at 115200 baud:

```text
C,150,0       # throttle, steering; -255..255
D,150,130     # direct left, right; -255..255
S             # immediate stop
H             # heartbeat
```

The watchdog stops active motors if no command arrives for 250 ms. The Jetson
bridge therefore sends commands at a steady heartbeat rate and always sends
`S` on shutdown.

## Next milestones

1. Confirm serial port and forward/turn polarity with the smoke test.
2. Add lidar driver and record a scan bag while manually driving.
3. Build a first-floor map and validate localization.
4. Add constrained voice intents: stop, go to named room, pause, resume.
5. Add navigation goals and a physical/software emergency stop.

See `docs/SAFETY_AND_OPERATING_PLAN.md`.
