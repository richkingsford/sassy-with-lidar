#!/usr/bin/env python3
"""No-motion LiDAR/OAK diagnostic for a wall in Sassy's forward-left sector."""
import statistics
import time

import depthai as dai
import numpy as np

from lidar_obstacle_turn_trial import FRONT_DEG, MIN_VALID_DISTANCE_M, ScanReader, angle_error

SECONDS = 4.0
MAX_DISTANCE_M = 2.5


def lidar_sector(scan, low_deg, high_deg):
    values = [distance for angle, distance in scan
              if low_deg <= angle_error(angle, FRONT_DEG) <= high_deg
              and MIN_VALID_DISTANCE_M <= distance <= MAX_DISTANCE_M]
    return None if not values else (len(values), min(values), statistics.median(values))


def oak_sector_distances(pipeline):
    left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
    left_out = left.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=15)
    right_out = right.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=15)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
    left_out.link(stereo.left); right_out.link(stereo.right)
    return stereo.depth.createOutputQueue(maxSize=1, blocking=False)


def main():
    lidar = ScanReader(); lidar.start()
    pipeline = dai.Pipeline(); queue = oak_sector_distances(pipeline); pipeline.start()
    readings = {"left": [], "center": [], "right": []}
    try:
        deadline = time.monotonic() + SECONDS
        while time.monotonic() < deadline:
            frame = queue.tryGet()
            if frame is None:
                time.sleep(.01); continue
            depth = frame.getFrame(); height, width = depth.shape
            region = depth[height // 4: height * 9 // 10, :]
            for name, start, end in (("left", 0, width // 3), ("center", width // 3, 2 * width // 3), ("right", 2 * width // 3, width)):
                values = region[:, start:end]
                valid = values[(values >= int(MIN_VALID_DISTANCE_M * 1000)) & (values <= int(MAX_DISTANCE_M * 1000))]
                if valid.size >= 40:
                    readings[name].append(float(np.percentile(valid, 20)) / 1000.0)
        scan = lidar.scan()
        print(f"LiDAR points in latest 360 scan: {len(scan)}")
        # Negative bearing offsets are left of the robot-forward axis: north-west on the live map.
        for name, low, high in (("north-west / forward-left", -75, -15), ("north / forward", -15, 15), ("north-east / forward-right", 15, 75)):
            result = lidar_sector(scan, low, high)
            print(f"LiDAR {name}: " + ("no valid near return" if result is None else f"{result[0]} returns; min {result[1]:.2f}m; median {result[2]:.2f}m"))
        for name in ("left", "center", "right"):
            values = readings[name]
            print(f"OAK {name}: " + ("no valid near depth" if not values else f"{len(values)} frames; median {statistics.median(values):.2f}m; min {min(values):.2f}m"))
        northwest_lidar = lidar_sector(scan, -75, -15)
        northwest_oak = readings["left"]
        print("RESULT: LiDAR sees north-west wall" if northwest_lidar else "RESULT: LiDAR does not currently see a north-west wall")
        print("RESULT: OAK sees north-west wall" if northwest_oak else "RESULT: OAK does not currently see a north-west wall")
    finally:
        pipeline.stop(); lidar.stop.set(); lidar.join(2)


if __name__ == "__main__":
    main()
