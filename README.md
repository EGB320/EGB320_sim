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

Returns requested obstacle or marker detections as `[range, bearing]` pairs. An empty list
means no requested object was detected. With no argument it continues to select all optional
obstacles for backwards compatibility.

```python
from mazebot_lib import mazeObjects

obstacles = robot.GetDetectedObjects([mazeObjects.obstacles])
for range_m, bearing_rad in obstacles:
    print(range_m, bearing_rad)

victim_markers = robot.GetDetectedObjects([mazeObjects.victimMarker])
```

Available selectors are `mazeObjects.obstacle0`, `mazeObjects.obstacle1`,
`mazeObjects.obstacle2`, `mazeObjects.baseStationMarker`, `mazeObjects.victimMarker`,
`mazeObjects.rubbleVictimMarker`, `mazeObjects.hazardMarker`, and `mazeObjects.victim`
(`mazeObjects.victimObject` is an equivalent descriptive alias).
The group selectors are `mazeObjects.obstacles`, `mazeObjects.markers`, and
`mazeObjects.victims`.

### `GetDetectedMarkers()`

Runs the low-resolution object detector once and retains the object type labels. It returns
zero-or-one `[range, bearing]` pair under each of the keys `base`, `victim`,
`rubble_victim`, `hazard`, and `victim_object`; a type that is not visible has an empty list.
The `victim` key represents the cyan wall marker, while `victim_object` represents the
separate yellow victim on the ground. When several objects share a class colour, the closest
matching object in the camera field of view is used.

```python
markers = robot.GetDetectedMarkers()
for marker_type, detections in markers.items():
    for range_m, bearing_rad in detections:
        print(marker_type, range_m, bearing_rad)
```

Each marker wall has two child planes. The textured `marker` remains white and is what students
see through `VisionSensor`. A second untextured `detector_marker` normally has no visible layer.
One simulator-side call briefly swaps the two planes, renders and processes `ObjectDetector`, and
restores them without changing CoppeliaSim's global visible-layer selection. Its class colours are blue for the base
station, cyan for an exposed victim, magenta for a rubble victim, and red for a hazard. The
structural wall and student-facing marker appearance therefore remain unchanged.

### `GetDetectedVictims()`

Runs the same low-resolution detector and returns zero-or-one `[range, bearing]` pair for
the closest visible victim. Each yellow visual victim owns a simple, untextured yellow
`victim_detector_proxy`. The proxy normally has no visible layer and follows the victim when
it is collected or released. During an object-detector render the Lua helper atomically hides
the detailed visual model and exposes only the proxy, so `VisionSensor` and the editor retain
the original victim appearance without adding a second complex mesh.

```python
victims = robot.GetDetectedVictims()
if victims:
    range_m, bearing_rad = victims[0]
```

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
