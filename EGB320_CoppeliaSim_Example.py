"""Minimal EGB320 MazeBot example.

Open ``EGB320_search_and_rescue_2026.ttt`` in CoppeliaSim before running this file.
Press Ctrl+C in the terminal to stop.
"""

import time

from mazebot_lib import MazeBot


FORWARD_SPEED = 0.08  # metres per second


def main():
    # Create the robot interface using the default robot and maze settings.
    # CoppeliaSim must already be open with the supplied scene loaded.
    robot = MazeBot()
    robot.StartSimulator()

    try:
        # Send one movement command. The robot continues using this command until a
        # different velocity is sent.
        robot.SetTargetVelocities(FORWARD_SPEED, 0.0)
        print("The robot is driving forward. Press Ctrl+C to stop.")

        while True:
            # Refresh the internal object positions used to calculate detections.
            robot.UpdateObjectPositions()

            # Read the left, front and right proximity sensors. A value of None means
            # that the sensor did not detect anything within its range.
            walls = robot.GetWallDistances()

            # DEBUG: Display both layers of mobility feedback. Encoders are cumulative
            # hardware-like counts; odometry is a local pose estimate integrated from
            # those counts and starts at (0, 0, 0). Comment out these reads and prints
            # when they are no longer useful to your navigation-system debugging.
            encoders = robot.GetWheelEncoders()
            odometry = robot.GetOdometry()

            # GetDetections() uses the low-resolution detection camera, but rendering
            # that camera still slows the simulation. If detections are not yet needed
            # by your navigation system, leave `detections` as None and comment out the
            # GetDetections() line until you need marker or victim detections.
            detections = None
            detections = robot.GetDetections()

            print(f"wall distances: {walls}")
            print(f"DEBUG wheel encoders: {encoders}")
            print(f"DEBUG local odometry: {odometry}")
            print(f"visible objects: {detections}\n")

            # OPTIONAL: Read the full colour image from the VisionSensor camera.
            # This transfers much more data, so only call it when an image is needed.
            # resolution, image_data = robot.GetCameraImage()

            # STUDENTS: Add your navigation code here. For example, use `walls` to
            # decide when to drive forward or turn, then call SetTargetVelocities().

            # A short delay prevents this example from requesting data unnecessarily.
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        # Always stop the motors and simulation, including after an error or Ctrl+C.
        robot.SetTargetVelocities(0.0, 0.0)
        robot.StopSimulator()


if __name__ == '__main__':
    main()
