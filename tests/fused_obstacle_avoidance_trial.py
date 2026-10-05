#!/usr/bin/env python3
"""Prepared five-second LiDAR + OAK-D Lite obstacle-avoidance trial.

LiDAR supplies the 360-degree bearing.  OAK stereo depth independently detects
forward obstacles and provides a fallback when LiDAR has no valid return.
Run only with `--run`, in a clear supervised area; stop the live dashboard first
because it owns both sensor devices.
"""
import argparse
import threading
import time

import depthai as dai
import numpy as np
import serial

from lidar_obstacle_turn_trial import (
    COMMAND_PERIOD_S, FRONT_CENTER_DEADBAND_DEG, FRONT_DEG, HIT_M,
    INITIAL_CRAWL_S, LIDAR, LIDAR_BAUD, MIN_VALID_DISTANCE_M, UNO, UNO_BAUD,
    ScanReader, angle_error, forward_command, front_cluster,
)

TRIAL_S = 5.0
OAK_FPS = 15
CAMERA_MIN_M = 0.30
CAMERA_MIN_POINTS = 40


class OakDepthReader(threading.Thread):
    """Continuously publish the latest metric stereo-depth frame from the OAK-D Lite."""
    def __init__(self):
        super().__init__(daemon=True)
        self.depth = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.ready = threading.Event()
        self.error = None

    def run(self):
        # A device disconnect/crash is recoverable: create a fresh pipeline once.
        for attempt in (1, 2):
            pipeline = None
            try:
                pipeline = dai.Pipeline()
                left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
                left_out = left.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=OAK_FPS)
                right_out = right.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=OAK_FPS)
                stereo = pipeline.create(dai.node.StereoDepth)
                stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.ROBOTICS)
                left_out.link(stereo.left); right_out.link(stereo.right)
                queue = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
                pipeline.start(); self.ready.set()
                while pipeline.isRunning() and not self.stop_event.is_set():
                    frame = queue.tryGet()
                    if frame is not None:
                        with self.lock:
                            self.depth = frame.getFrame().copy()
                    time.sleep(.005)
                return
            except Exception as exc:
                self.error = f"OAK attempt {attempt}: {exc}"
                if self.stop_event.is_set() or attempt == 2:
                    return
                time.sleep(.75)
            finally:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
        self.ready.set()

    def snapshot(self):
        with self.lock:
            return None if self.depth is None else self.depth.copy()


def camera_obstacle(depth_mm):
    """Return (side, distance_m), derived from stereo geometry, or None.

    We discard the ceiling/floor margins, divide the remaining image into equal
    left/centre/right viewing sectors, and require a substantial valid-depth
    cluster.  No room-specific positions or objects are encoded here.
    """
    if depth_mm is None:
        return None
    height, width = depth_mm.shape
    region = depth_mm[height // 4: height * 9 // 10, :]
    sectors = (("left", 0, width // 3), ("center", width // 3, 2 * width // 3), ("right", 2 * width // 3, width))
    candidates = []
    for side, start, end in sectors:
        values = region[:, start:end]
        valid = values[(values >= int(CAMERA_MIN_M * 1000)) & (values <= int(HIT_M * 1000))]
        if valid.size >= CAMERA_MIN_POINTS:
            candidates.append((side, float(np.percentile(valid, 20)) / 1000.0))
    return min(candidates, key=lambda item: item[1]) if candidates else None


def adaptive_turn_command(turn_right, distance_m, bearing_offset_deg):
    """Near, frontal hazards turn sharply; farther/side hazards turn gently."""
    closeness = max(0.0, min(1.0, 1.0 - distance_m / HIT_M))
    frontalness = max(0.0, math_cos_deg(bearing_offset_deg))
    speed = round(85 + 145 * max(closeness, frontalness))
    return f"D,{speed},0" if turn_right else f"D,0,{speed}"


def math_cos_deg(degrees):
    return __import__("math").cos(__import__("math").radians(degrees))


def choose_turn(lidar_hit, camera_hit):
    """Fuse geometry: lidar bearing wins; camera protects its three forward sectors."""
    if lidar_hit:
        offset = angle_error(lidar_hit[0], FRONT_DEG)
        return offset <= FRONT_CENTER_DEADBAND_DEG, lidar_hit[1], offset, "lidar"
    if camera_hit:
        side, distance = camera_hit
        # Centre defaults right; left/right sectors turn away from the measured side.
        return side != "right", distance, {"left": -35, "center": 0, "right": 35}[side], "oak-depth"
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", action="store_true")
    args = parser.parse_args()
    if not args.run:
        print("Prepared only. Stop the dashboard, then add --run for a supervised 5-second trial.")
        return
    lidar = ScanReader(); oak = OakDepthReader(); lidar.start(); oak.start()
    try:
        with serial.Serial(UNO, UNO_BAUD, timeout=.1) as uno:
            uno.write(b"S\n"); uno.flush(); time.sleep(1.5)
            deadline = time.monotonic() + 6
            while (not lidar.scan() or oak.snapshot() is None) and oak.error is None and time.monotonic() < deadline:
                time.sleep(.05)
            if oak.error:
                raise RuntimeError(oak.error)
            if not lidar.scan() or oak.snapshot() is None:
                raise RuntimeError("LiDAR or OAK depth did not become ready")
            print("both sensors ready; starting 5-second fused trial")
            started = time.monotonic()
            while time.monotonic() - started < TRIAL_S:
                elapsed = time.monotonic() - started
                lidar_hit = front_cluster(lidar.scan())
                oak_hit = camera_obstacle(oak.snapshot())
                choice = choose_turn(lidar_hit, oak_hit)
                if elapsed < INITIAL_CRAWL_S:
                    command = forward_command()
                elif choice is None:
                    command = forward_command()
                else:
                    turn_right, distance, offset, source = choice
                    command = adaptive_turn_command(turn_right, distance, offset)
                    print(f"{source} obstacle {distance:.2f}m, offset {offset:+.0f}deg -> {'right' if turn_right else 'left'}")
                uno.write((command + "\n").encode()); uno.flush(); time.sleep(COMMAND_PERIOD_S)
    finally:
        try:
            with serial.Serial(UNO, UNO_BAUD, timeout=.1) as uno:
                uno.write(b"S\n"); uno.flush()
        except serial.SerialException:
            pass
        lidar.stop.set(); oak.stop_event.set()
        lidar.join(2); oak.join(8)
        if oak.is_alive():
            print("warning: OAK reader did not stop within 8 seconds")
        elif oak.error:
            print(f"warning: {oak.error}")
        print("trial complete; stop sent")


if __name__ == "__main__":
    main()
