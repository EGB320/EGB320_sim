# EGB320 MazeBot Library

Python support for the 2026 EGB320 search-and-rescue maze simulation in CoppeliaSim. The
library generates the maze, controls the robot, reads its camera and distance sensors, and
provides software-triggered victim collection and release.

## Installation

Install CoppeliaSim and the required Python packages:

```bash
pip install coppeliasim-zmqremoteapi-client opencv-python numpy
```

Load `EGB320_search_and_rescue_2026.ttt` in CoppeliaSim, then run either example:

```bash
python EGB320_CoppeliaSim_Example.py
python EGB320_CoppeliaSim_Example_keyboard.py
```

## Quick start

```python
from mazebot_lib import COPPELIA_MazeRobot, RobotParameters, SceneParameters

robot_parameters = RobotParameters()
scene_parameters = SceneParameters()

robot = COPPELIA_MazeRobot(robot_parameters, scene_parameters)
robot.StartSimulator()

try:
    while True:
        robot.UpdateObjectPositions()
        distances = robot.GetWallDistances()

        if distances['front'] is not None and distances['front'] < 0.10:
            robot.SetTargetVelocities(0.0, 0.3)
        else:
            robot.SetTargetVelocities(0.08, 0.0)
except KeyboardInterrupt:
    robot.StopSimulator()
```

## Main classes

### `COPPELIA_MazeRobot`

```python
COPPELIA_MazeRobot(
    robotParameters,
    sceneParameters,
    coppelia_server_ip='127.0.0.1',
    port=23000,
)
```

The main CoppeliaSim interface.

### `RobotParameters`

Configures drive, camera, obstacle-detection and victim-collection behaviour. Important
settings include:

```python
parameters.maximumLinearSpeed = 0.25
parameters.driveSystemQuality = 1.0
parameters.cameraResolutionX = 640
parameters.cameraResolutionY = 480
parameters.victimCollectionDistance = 0.10
parameters.sync = False
```

### `SceneParameters`

Configures maze generation and starting positions. The defaults generate the approved
7-by-7 example maze with three victims.

```python
scene.mazeRows = 7
scene.mazeColumns = 7
scene.mazeCellSize = 0.280
scene.baseCell = (0, 6)
scene.victimCells = {
    'L1': (1, 5),
    'L2': (4, 3),
    'L3': (6, 0),
}
```

## Robot control

### `StartSimulator()`

Stops a running simulation if necessary, regenerates the maze while stopped, places the
robot and starts the simulation.

### `StopSimulator()`

Stops the CoppeliaSim simulation.

### `SetTargetVelocities(x_dot, theta_dot)`

Commands forward velocity in metres per second and angular velocity in radians per second.

```python
robot.SetTargetVelocities(0.10, 0.0)
robot.SetTargetVelocities(0.0, 0.3)
robot.SetTargetVelocities(0.0, 0.0)
```

## Sensors

### `GetWallDistances()`

Returns robot-relative proximity readings in metres:

```python
distances = robot.GetWallDistances()
left = distances['left']
front = distances['front']
right = distances['right']
```

A value is `None` when the corresponding sensor has no detection. Despite the method name,
the proximity sensors can detect any detectable object in their sensing volumes.

### `GetCameraImage()`

Returns `(resolution, image_data)`. Resolution is `[width, height]`; image data is `None`
when no image is available.

```python
resolution, image_data = robot.GetCameraImage()
```

### `GetDetectedObjects(objects=None)`

Returns optional obstacle detections as `[range, bearing]` pairs. An empty list means no
requested obstacle was detected.

```python
from mazebot_lib import mazeObjects

obstacles = robot.GetDetectedObjects([mazeObjects.obstacles])
for range_m, bearing_rad in obstacles:
    print(range_m, bearing_rad)
```

Available selectors are `mazeObjects.obstacle0`, `mazeObjects.obstacle1`,
`mazeObjects.obstacle2`, and `mazeObjects.obstacles`.

### `UpdateObjectPositions()`

Refreshes cached robot, camera and optional obstacle positions. It returns
`(robotPose, obstaclePositions)`.

```python
robot_pose, obstacle_positions = robot.UpdateObjectPositions()
```

`robotPose` is `[x, y, theta]` in metres and radians.

### `SetCameraResolution(x_res, y_res)`

Sets the onboard camera resolution and returns `True` when successful.

## Victim collection

### `CollectVictim()`

Attempts to collect the nearest generated victim. Collection succeeds only when the
shortest horizontal clearance between the robot model and victim is no greater than
`victimCollectionDistance`, which defaults to 0.10 m.

```python
success, victim_label, distance = robot.CollectVictim()
```

On success, the victim is attached to `/Robot/VictimCarryPoint`. The library uses the
dummy's scene-authored position and orientation without moving the dummy.

### `HasVictim()`

Returns `True` while a victim is attached to the robot.

### `ReleaseVictim()`

Places the carried victim on the maze floor 0.15 m in front of the robot and returns
`(success, victim_label)`.

```python
if robot.HasVictim():
    success, victim_label = robot.ReleaseVictim()
```

## Keyboard example

The keyboard example uses held-key input on Windows:

- W/S: drive forward or backward
- A/D: rotate left or right
- Space: collect a nearby victim or release the carried victim
- Q: stop and exit

Movement commands are sent only when the held-key state changes. Releasing the movement
keys sends one zero-velocity command.

## Troubleshooting

If the Python client cannot connect, confirm that CoppeliaSim is open, the search-and-rescue
scene is loaded, and the ZeroMQ Remote API is listening on port 23000.

If an optional sensor is missing, the library reports it during initialization and keeps the
remaining functionality available. Required robot, motor, floor and maze-template objects
must use the paths expected by `mazebot_lib.py`.

## License

See [LICENSE](LICENSE).
