# EGB320 MazeBot simulator

This repository contains the 2026 EGB320 search-and-rescue maze simulation. The Python
library can drive the robot, read its sensors, detect markers and victims, and collect or
release victims.

## Start here

1. Install [CoppeliaSim Edu](https://www.coppeliarobotics.com/) and Python 3.
2. Install the CoppeliaSim Python client:

   ```bash
   pip install coppeliasim-zmqremoteapi-client
   ```

3. Open `EGB320_search_and_rescue_2026.ttt` in CoppeliaSim.
4. Run one of the examples:

   ```bash
   python EGB320_CoppeliaSim_Example.py
   python EGB320_CoppeliaSim_Example_keyboard.py
   ```

The keyboard example requires Windows. Its controls are W/A/S/D, Space to collect or
release a victim, and Q to quit.

## Smallest useful program

```python
import time

from mazebot_lib import MazeBot

robot = MazeBot()
robot.StartSimulator()

try:
    robot.SetTargetVelocities(0.08, 0.0)

    while True:
        robot.UpdateObjectPositions()

        distances = robot.GetWallDistances()
        detections = robot.GetDetections()
        time.sleep(0.1)
finally:
    robot.SetTargetVelocities(0.0, 0.0)
    robot.StopSimulator()
```

Use `try/finally` so the simulator is stopped even if your program encounters an error.

## Main robot commands

### Movement

```python
robot.SetTargetVelocities(forward_velocity, turn_velocity)
```

- Forward velocity is in metres per second.
- Turn velocity is in radians per second.
- Positive turn velocity turns left.
- Send `(0.0, 0.0)` to stop.

Examples:

```python
robot.SetTargetVelocities(0.10, 0.0)   # drive forward
robot.SetTargetVelocities(0.0, 0.30)   # turn left
robot.SetTargetVelocities(0.0, 0.0)    # stop
```

### Robot pose

```python
pose, obstacle_positions = robot.UpdateObjectPositions()
```

`pose` is `[x, y, heading]`, with position in metres and heading in radians. Call this
before requesting range and bearing detections.

### Wall-distance sensors

```python
distances = robot.GetWallDistances()

left = distances['left']
front = distances['front']
right = distances['right']
```

Each value is a distance in metres or `None` when the sensor detects nothing. The names
are relative to the robot, not the maze.

### Markers and yellow victims

```python
detections = robot.GetDetections()
```

The result contains five separate object types:

```python
{
    'base': [],
    'victim': [],          # cyan victim marker on a wall
    'rubble_victim': [],
    'hazard': [],
    'victim_object': [],   # yellow victim on the ground
}
```

Each non-empty list contains one `[range, bearing]` pair. Range is in metres and bearing
is in radians, relative to the camera centreline.

```python
yellow_victims = detections['victim_object']
if yellow_victims:
    range_m, bearing_rad = yellow_victims[0]
    print(range_m, bearing_rad)
```

`GetDetectedVictims()` is also available when only the yellow victim is needed.

### Camera image

```python
resolution, image_data = robot.GetCameraImage()
```

`resolution` is `[width, height]`. Reading the full camera image is more expensive than
the small object detector, so avoid calling it when it is not needed.

## Victim collection

Attempt collection only after navigating close to a victim:

```python
success, label, distance = robot.CollectVictim()
```

Collection succeeds when the victim is within the permitted collection distance. The
default is 0.10 m.

```python
if robot.HasVictim():
    success, label = robot.ReleaseVictim()
```

Released victims are placed on the floor in front of the robot.

## Changing parameters

The default parameters are suitable for the supplied scene. Change only the settings your
experiment needs:

```python
from mazebot_lib import MazeBot, RobotParameters, SceneParameters

robot_parameters = RobotParameters()
robot_parameters.maximumLinearSpeed = 0.20
robot_parameters.cameraTilt = -0.10

scene_parameters = SceneParameters()
scene_parameters.robotStartingPosition = [0.0, 0.0, 0.0]

robot = MazeBot(robot_parameters, scene_parameters)
```

The default maze is 7 by 7 cells and contains three victim objects. Each victim is placed
in a dead end with its wall marker directly beyond it on the terminal wall, allowing an
approaching robot to see both at once. Maze walls and victims are regenerated whenever
`StartSimulator()` is called.

The base station is itself a dead end. The robot begins facing a four-cell straight
corridor whose first junction is at `(0, 2)`: continuing straight enters the wider maze,
while the first right turn leads directly into the L1 victim's dead end at `(1, 2)`.
The physically open routes to L2 and L3 avoid the remaining central diagonal-wall cells.

## Optional object selection

Most student code should use `GetDetections()`. `GetDetectedObjects()` is available
when selecting optional obstacles or one particular detection class:

```python
from mazebot_lib import MazeObject

obstacles = robot.GetDetectedObjects([MazeObject.obstacles])
victim_markers = robot.GetDetectedObjects([MazeObject.victimMarker])
yellow_victims = robot.GetDetectedObjects([MazeObject.victims])
```

## Files students need

- `EGB320_search_and_rescue_2026.ttt`: CoppeliaSim scene.
- `mazebot_lib.py`: Python robot library.
- `EGB320_CoppeliaSim_Example.py`: minimal starting example.
- `EGB320_CoppeliaSim_Example_keyboard.py`: manual driving and sensor demonstration.
- `keyboard_control.py`: keyboard helper used by the manual-driving example.

The Lua files are source copies of scripts already embedded in the `.ttt` scene. Students
do not need to paste them into CoppeliaSim.

## Troubleshooting

If Python cannot connect:

- Confirm CoppeliaSim is open.
- Confirm `EGB320_search_and_rescue_2026.ttt` is loaded.
- Confirm the ZeroMQ Remote API is listening on port 23000.
- Stop any other Python script that may already be controlling the simulator.

If detection returns empty lists, call `UpdateObjectPositions()` first and check that the
object is visible to the robot camera.

## License

See [LICENSE](LICENSE).
