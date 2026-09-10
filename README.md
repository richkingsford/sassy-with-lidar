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
- Lidar mapping, localization, and voice command execution: next milestones

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
