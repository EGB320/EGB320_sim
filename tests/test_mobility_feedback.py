import math
import unittest

from mazebot_lib import MazeBot, RobotParameters


class FakeSim:
    """Return deterministic Lua function results without starting CoppeliaSim."""

    def __init__(self, results):
        self.results = {name: list(samples) for name, samples in results.items()}
        self.calls = []

    def callScriptFunction(self, name, script_handle):
        self.calls.append((name, script_handle))
        return self.results[name].pop(0)


def make_robot(results):
    """Build only the small portion of MazeBot needed by the mobility API."""
    robot = object.__new__(MazeBot)
    robot.robotParameters = RobotParameters()
    robot.scriptHandle = 17
    robot.sim = FakeSim(results)
    robot._odometryPose = [0.0, 0.0, 0.0]
    robot._odometryLastEncoderCounts = None
    return robot


class WheelEncoderTests(unittest.TestCase):
    def test_encoder_angles_are_quantized_to_signed_cumulative_ticks(self):
        robot = make_robot({
            'getWheelEncoderData': [(1.25, 2.0 * math.pi, -math.pi)],
        })

        sample = robot.GetWheelEncoders()

        self.assertEqual(sample, {
            'time': 1.25,
            'left_ticks': 360,
            'right_ticks': -180,
        })

    def test_reset_encoders_preserves_odometry_and_rebases_counts(self):
        robot = make_robot({
            'resetWheelEncoders': [(3.0, 0.0, 0.0)],
        })
        robot._odometryPose = [1.0, 2.0, 0.3]
        robot._odometryLastEncoderCounts = (120, 118)

        robot.ResetWheelEncoders()

        self.assertEqual(robot._odometryPose, [1.0, 2.0, 0.3])
        self.assertEqual(robot._odometryLastEncoderCounts, (0, 0))

    def test_encoder_resolution_must_be_a_positive_integer(self):
        robot = make_robot({
            'getWheelEncoderData': [(0.0, 0.0, 0.0)],
        })
        robot.robotParameters.encoderCountsPerRevolution = 0

        with self.assertRaisesRegex(ValueError, 'positive integer'):
            robot.GetWheelEncoders()


class OdometryTests(unittest.TestCase):
    def test_straight_motion_uses_wheel_circumference(self):
        robot = make_robot({
            'getWheelEncoderData': [
                (0.0, 0.0, 0.0),
                (1.0, 2.0 * math.pi, 2.0 * math.pi),
            ],
        })
        robot.robotParameters.wheelRadius = 0.1

        robot.ResetOdometry()
        odometry = robot.GetOdometry()

        self.assertAlmostEqual(odometry['x'], 2.0 * math.pi * 0.1)
        self.assertAlmostEqual(odometry['y'], 0.0)
        self.assertAlmostEqual(odometry['heading'], 0.0)

    def test_opposite_wheel_motion_turns_without_translation(self):
        robot = make_robot({
            'getWheelEncoderData': [
                (0.0, 0.0, 0.0),
                (1.0, -0.5 * math.pi, 0.5 * math.pi),
            ],
        })
        robot.robotParameters.wheelRadius = 0.1
        robot.robotParameters.wheelBase = 0.2

        robot.ResetOdometry()
        odometry = robot.GetOdometry()

        self.assertAlmostEqual(odometry['x'], 0.0)
        self.assertAlmostEqual(odometry['y'], 0.0)
        self.assertAlmostEqual(odometry['heading'], 0.5 * math.pi)

    def test_reset_odometry_uses_current_counts_as_the_new_origin(self):
        robot = make_robot({
            'getWheelEncoderData': [(4.0, math.pi, 0.5 * math.pi)],
        })
        robot._odometryPose = [2.0, -1.0, 0.7]

        robot.ResetOdometry()

        self.assertEqual(robot._odometryPose, [0.0, 0.0, 0.0])
        self.assertEqual(robot._odometryLastEncoderCounts, (180, 90))


if __name__ == '__main__':
    unittest.main()
