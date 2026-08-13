"""Drive the EGB320 MazeBot with W/A/S/D on Windows.

Controls:
    W/S   drive forward/backward
    A/D   turn left/right
    Space collect or release a victim
    Q     quit
"""

import math
import time

from keyboard_control import KeyboardController, clear_console
from mazebot_lib import MazeBot, RobotParameters


FORWARD_SPEED = 0.03  # m/s
TURN_SPEED = 0.30     # rad/s
DISPLAY_PERIOD = 0.10


def format_distance(distance):
    """Convert a distance-sensor result into text for the status display."""
    return 'none' if distance is None else f'{distance:.3f} m'


def collect_or_release(robot):
    """Perform one Space-key action and return a short status message."""
    if robot.HasVictim():
        success, label = robot.ReleaseVictim()
        return f'Released victim {label}.' if success else 'Release failed.'

    success, label, distance = robot.CollectVictim()
    if success:
        return f'Collected victim {label} at {distance:.3f} m.'
    return 'No victim is close enough to collect.'


def show_status(command, walls, encoders, odometry, detections, collection_status):
    """Display the latest commands and sensor readings in the terminal."""
    clear_console()
    print('EGB320 MazeBot keyboard control')
    print('W/S: drive  A/D: turn  Space: collect/release  Q: quit\n')
    print(f'command: forward={command[0]:.2f} m/s, turn={command[1]:.2f} rad/s')
    print(
        'wall distances: '
        f"left={format_distance(walls['left'])}, "
        f"front={format_distance(walls['front'])}, "
        f"right={format_distance(walls['right'])}")
    print(
        'DEBUG wheel encoders: '
        f"left={encoders['left_ticks']} ticks, "
        f"right={encoders['right_ticks']} ticks, "
        f"time={encoders['time']:.2f} s")
    print(
        'DEBUG local odometry: '
        f"x={odometry['x']:.3f} m, "
        f"y={odometry['y']:.3f} m, "
        f"heading={math.degrees(odometry['heading']):.1f} degrees")

    if detections is None:
        print('wall markers: detection camera disabled')
        print('yellow victim: detection camera disabled')
    else:
        visible_markers = [
            name for name in ('base', 'victim', 'rubble_victim', 'hazard')
            if detections[name]
        ]
        print('wall markers:', ', '.join(visible_markers) or 'none')

        if detections['victim_object']:
            range_m, bearing_rad = detections['victim_object'][0]
            print(
                f'yellow victim: {range_m:.3f} m, '
                f'{math.degrees(bearing_rad):.1f} degrees')
        else:
            print('yellow victim: none')

    print('collection:', collection_status)


def main():
    # Only settings that differ from the defaults need to be specified.
    parameters = RobotParameters()
    parameters.cameraDistanceFromRobotCenter = 0.0
    parameters.cameraTilt = -0.1
    # Choose 'legacy' (stable default) or 'opengl3' for the ObjectDetector renderer.
    parameters.objectDetectorRenderer = 'legacy'

    # Connect to the supplied scene and generate the default maze.
    robot = MazeBot(parameters)
    keyboard = KeyboardController()
    robot.StartSimulator()

    last_command = None
    collection_status = 'No collection attempted.'
    next_display_time = 0.0

    try:
        while not keyboard.is_down('q'):
            # Convert the held W/A/S/D keys into forward and turning velocities.
            # A new command is sent only when the key state changes. Releasing the
            # movement keys therefore sends one (0, 0) stop command.
            command = keyboard.drive_command(FORWARD_SPEED, TURN_SPEED)
            if command != last_command:
                robot.SetTargetVelocities(*command)
                last_command = command

            # Perform one collection action when Space changes from up to down.
            if keyboard.was_pressed('space'):
                collection_status = collect_or_release(robot)

            current_time = time.monotonic()
            if current_time >= next_display_time:
                # Refresh object positions, then read the distance and object sensors.
                robot.UpdateObjectPositions()
                walls = robot.GetWallDistances()

                # DEBUG: Read both the raw mobility feedback and the local pose derived
                # from it. These values do not expose the simulator's global robot pose.
                # Comment out these reads and the matching show_status() lines when the
                # extra display is no longer useful.
                encoders = robot.GetWheelEncoders()
                odometry = robot.GetOdometry()

                # GetDetections() uses the low-resolution detection camera, but rendering
                # that camera still slows the simulation. If detections are not yet needed
                # by your navigation system, leave `detections` as None and comment out the
                # GetDetections() line until you need marker or victim detections.
                detections = None
                detections = robot.GetDetections()

                # OPTIONAL: Read the full colour image from the VisionSensor camera.
                # This is slower than GetDetections(), so only use it when needed.
                # resolution, image_data = robot.GetCameraImage()

                # STUDENTS: This is a useful place to inspect or process sensor data.
                # For an autonomous program, replace the keyboard-command section above
                # with code that uses `walls` and `detections` to choose velocities.
                show_status(
                    command, walls, encoders, odometry, detections, collection_status)
                next_display_time = current_time + DISPLAY_PERIOD

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave the robot and simulator stopped when the program exits.
        robot.SetTargetVelocities(0.0, 0.0)
        robot.StopSimulator()


if __name__ == '__main__':
    main()
