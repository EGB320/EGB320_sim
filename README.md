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
release a victim, and Q to quit. The flood-fill example is a staff-only reference
solution and is intentionally omitted from the student quick start.

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

### Wheel encoders and local odometry

The mobility system exposes both raw wheel feedback and a derived local pose:

```python
encoders = robot.GetWheelEncoders()
# {'time': 2.5, 'left_ticks': 1420, 'right_ticks': 1398}

odometry = robot.GetOdometry()
# {'time': 2.5, 'x': 0.59, 'y': -0.02, 'heading': -0.08}
```

Encoder counts are signed and cumulative from the last simulator start or
`ResetWheelEncoders()` call. The Lua robot script accumulates wheel rotation on every
simulation step, so slow Python loops do not lose complete wheel revolutions. On the
four-wheel robot, the available front and rear motor encoders are averaged on each side.

`GetOdometry()` integrates the encoder counts using `wheelRadius` and `wheelBase`.
Its `(x, y, heading)` frame begins at `(0, 0, 0)` when the simulation starts or when
`ResetOdometry()` is called. It is not the simulator's global pose, and wheel slip or
incorrect wheel parameters will produce normal odometry drift.

For this four-wheel skid-steer chassis, the default `wheelBase` is a calibrated effective
track width of 0.145 m. It is wider than the geometric left/right wheel spacing because
the tyres scrub sideways during turns. Students can recalibrate this parameter as part
of modelling their own mobility system.

The physical front and rear contact axles are placed 0.03 m either side of the chassis
centre. This symmetric, shortened spacing lets the four-wheel robot turn on the spot
without weakening one axle's traction or translating around one end of the robot. The
setting is exposed as `wheelAxleLongitudinalOffset` and is applied when `MazeBot` starts.

```python
robot.ResetWheelEncoders()  # Zero counts; preserve the current odometry pose.
robot.ResetOdometry()       # Zero local pose; preserve the current encoder counts.
```

The default encoder resolution is 360 counts per wheel revolution. It can be changed
before creating the robot:

```python
from mazebot_lib import MazeBot, RobotParameters

parameters = RobotParameters()
parameters.encoderCountsPerRevolution = 720
robot = MazeBot(parameters)
```

### Refreshing object positions

```python
robot.UpdateObjectPositions()
```

Call this before requesting range and bearing detections. It refreshes the simulator's
internal object-position cache and does not return global pose or object-position data.

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

#### ObjectDetector renderer

The low-resolution colour detector can use either CoppeliaSim renderer:

```python
from mazebot_lib import MazeBot, RobotParameters

parameters = RobotParameters()
parameters.objectDetectorRenderer = 'legacy'  # Stable default: Legacy OpenGL
# parameters.objectDetectorRenderer = 'opengl3'  # Optional simOpenGL3 renderer

robot = MazeBot(parameters)
```

Legacy OpenGL is recommended because the ObjectDetector only needs flat RGB colours.
OpenGL3 is available for comparison or systems where it is known to be stable, but its
GPU-driver/plugin path may be less reliable on some computers. An unknown value raises a
clear `ValueError` during robot initialization.

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

Released victims are placed on the floor in front of the robot. A completed base
delivery can be recorded with `ReleaseVictim(delivered=True)`. Delivered victims remain
visible but are excluded from later collection, colour detection and proximity sensing.
Optional `forward_offset` and `lateral_offset` arguments allow several victims to be
arranged in one drop zone.

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

The default maze is 7 by 7 cells and contains three victim objects. The number can be
set from 1 to 3; levels are included in order, so a value of 2 generates only L1 and L2:

```python
scene_parameters = SceneParameters()
scene_parameters.numberOfVictims = 2
```

Each generated victim is placed
in a dead end with its wall marker directly beyond it on the terminal wall, allowing an
approaching robot to see both at once. Maze walls and victims are regenerated whenever
`StartSimulator()` is called. A victim level omitted by `numberOfVictims` is no longer
placed; if its cell remains a dead end, that cell receives a hazard marker instead.

The original maze remains the default preset. To generate a challenge-compliant random
maze at simulator start, select random mode before constructing `MazeBot`:

```python
scene_parameters = SceneParameters()
scene_parameters.numberOfVictims = 2             # L1 and L2 only (valid: 1-3)
scene_parameters.mazeGenerationMode = 'random'  # or 'preset'
scene_parameters.randomMazeSeed = None          # a fresh maze on every start
# scene_parameters.randomMazeSeed = 2026         # repeatable for teaching/debugging

robot = MazeBot(robot_parameters, scene_parameters)
robot.StartSimulator()
print(scene_parameters.activeMazeSeed)           # replay a generated maze later
```

Random topology generation is implemented independently of CoppeliaSim in
`maze_generation.py`. It creates a connected axis-aligned maze, forces the base to have
one opening, and chooses one-entry victim cells using shortest-path distance from the
base: L1 from the short band, L2 from the medium band and, when configured, L3 as the
farthest selected victim. The marker beyond each victim is placed on the wall opposite
its only entrance.
Every remaining one-entry cell receives a hazard marker. The accepted seed, victim path
distances and hazard cells are printed when the scene is built and are also available as
`activeMazeSeed`, `mazeVictimDistances` and `mazeHazardCells`.

The base station is itself a dead end. The robot begins facing a four-cell straight
corridor whose first junction is at `(0, 2)`: continuing straight enters the wider maze,
while the first right turn leads directly into the L1 victim's dead end at `(1, 2)`.
All generated maze walls are axis-aligned to the cell grid; diagonal wall segments are
rejected by scene-parameter validation.

## Internal grid map, localisation and flood-fill

`maze_navigation.py` provides a navigation layer that uses only the public robot sensor
API. It never reads the robot's simulator-global pose or the generated wall list.

```python
from maze_navigation import Direction, MazeMapVisualizer, MazeNavigationSystem

navigation = MazeNavigationSystem(
    rows=robot.sceneParameters.mazeRows,
    columns=robot.sceneParameters.mazeColumns,
    cell_size=robot.sceneParameters.mazeCellSize,
    base_cell=robot.sceneParameters.baseCell,
    start_direction=Direction.from_world_yaw(robot.sceneParameters.baseYaw),
    landmark_sensor_forward_offset=(
        robot.robotParameters.cameraDistanceFromRobotCenter),
)
display = MazeMapVisualizer(navigation)

while True:
    state = navigation.update_from_robot(robot, read_landmarks=True)
    display.update()
    print(state['cell'], state['direction'])
```

The internal representation stores one confidence-weighted state (`unknown`, `open` or
`wall`) for every shared cell edge. Long proximity readings are matched to the correct
grid boundary, so a reading through one open cell marks that first boundary open and the
farther boundary closed. Scans are committed at encoder-confirmed cell-centre crossings;
camera and GUI work is performed only after stopping. This prevents wall posts seen while
crossing a boundary from being mistaken for a wall across the passage.
Wall discovery uses a three-frame majority/median consensus, accepts scans only close to a
cell centre and within 7.5 degrees of a cardinal heading, and rejects new discoveries when
even one known edge conflicts with the measured pattern. A single raw range hit remains
tentative. Later contradictory consensus scans reduce confidence and can demote an
incorrect wall back to `unknown`; conflict-repair scans may weaken existing beliefs but
cannot add new walls.
Emergency obstacle closures also require a stopped three-frame confirmation. Semantic
three-sided dead-end walls require the same marker in two camera updates plus a cardinal,
non-conflicting wall pattern.
An edge the robot has physically crossed is retained as confirmed open. The localiser:

- transforms local encoder odometry into continuous grid coordinates;
- uses distances to cardinal wall planes to correct within-cell position drift;
- anchors each encoder-confirmed one-cell move to the adjacent topological cell;
- rebases accumulated skid-steer encoder error after a validated cardinal turn;
- uses the unique blue base marker as an absolute return landmark; and
- learns the unique magenta rubble marker as a repeatable loop-closure landmark.

A close base, hazard, victim or rubble marker also identifies the terminal wall of one
of this scene's three-sided dead ends. The mapper closes the front/left/right sides and
keeps the traversed rear edge open, allowing the controller to reverse out instead of
trying to rotate the chassis inside the narrow cell.

Wall readings are not assigned to grid edges while the robot is between cardinal
headings or moving between cell centres. This avoids corrupting the map with diagonal
observations during turns. The current representation can be copied without the GUI
using `navigation.snapshot()` or
`navigation.map.to_dict(navigation.planner.values)`.

Flood-fill initially targets the maze centre. For an odd-sized maze this is one cell;
for an even-sized maze it is the usual four centre cells. Change the return goal with:

```python
navigation.set_goal_to_base()
direction = navigation.choose_next_direction()
```

`EGB320_CoppeliaSim_Example_floodfill.py` contains a conservative configurable rescue
mission for one to three victims. Set `VICTIMS_TO_RESCUE` near the top of that script.
It targets the configured Level 1 cell first. After delivering Level 1, it
flood-fills toward unvisited cells and diverts to victim cells inferred from camera
range/bearing detections or close cyan/magenta victim markers. Every collected victim is
returned to base before the next search begins; the robot backs into the base while facing
its opening, then releases the configured victims at separate lateral offsets. Its speed, turn
and safety constants are intentionally
grouped near the top so they can be retuned if the drive geometry, physics engine or
`driveSystemQuality` is changed. Before each transition it takes a median stopped range
scan. A known front wall provides a physical longitudinal reference: the controller moves
gently until the sensor-to-wall distance corresponds to the centre of the cell. One or two
known side walls provide lateral feedback during the following translation, and a large
lateral error automatically reduces forward speed. If no wall observes an axis, the
controller explicitly leaves that axis as odometry-only rather than applying a guessed
correction.
Straight-line lateral control uses a filtered proportional/derivative correction with a
2 mm deadband. Its correction limit and gain are higher than the encoder-heading loop, so
an observed side wall can pull the chassis back toward the corridor centre promptly while
the derivative term suppresses steering spikes from individual range jumps.

For classroom demonstrations, the motion-mode flag is near the top of the script:

```python
STOP_AT_EACH_CELL = True   # stop, scan and redraw after every grid cell
```

This is the default teaching mode and includes a short display pause at every centre so
students can follow the flood-fill decision one cell at a time. Set it to `False` to join
straight steps into the faster continuous rolling-corridor mode. Code that imports the
mission can also override it with
`run_floodfill_mission(..., stop_at_each_cell=False)`.

Teaching mode also opens a second **MazeBot continuous PD control** figure. Controller
samples are collected at the 25 ms feedback rate and rendered only after the robot stops,
so plotting cannot delay the motor loop. The stacked time plots show lateral wall error,
the zero lateral setpoint versus measured error, side-wall proportional and derivative
actions, encoder-heading action, desired versus encoder-derived actual angular velocity,
and desired versus actual heading. Dotted vertical markers identify changes between grid
cells. Linear action is intentionally omitted from this teaching figure.

With `STOP_AT_EACH_CELL = False`, straight flood-fill steps are joined into rolling
corridor runs. At each cell centre the
controller confirms one encoder cell length, anchors the topological transition and fuses
a range-only map sample without braking. It stops before a turn, a known victim/base goal,
a dead end, an inconsistent wall pattern, or any change in the planned direction. Camera
capture and Matplotlib redraws therefore do not interrupt straight-line feedback.
Planning time between rolling cells is included in the next cell's encoder distance rather
than becoming cumulative centre overshoot. The next flood-fill step is predicted before
entry so known turns use a slow final approach, while an unexpected re-plan invokes a short
low-speed encoder correction back to the nominal cell centre.

After a turn, the controller compares the measured left/front/right wall pattern with the
map before committing the scan. A distinctive conflicting pattern restores the inferred
heading and permits one controlled retry; an ambiguous conflict stops the robot. The live
figure shows discovered walls, unknown edges, visit history, the estimated robot pose and
heading, the current flood value, and lateral/front alignment errors in millimetres. It
redraws only while the robot is stopped at a cell centre; Matplotlib rendering is therefore
kept out of the time-critical odometry and motor-control loops.

## Optional object selection

Most student code should use `GetDetections()`. `GetDetectedObjects()` is available
when selecting optional obstacles or one particular detection class:

```python
from mazebot_lib import MazeObject

obstacles = robot.GetDetectedObjects([MazeObject.obstacles])
victim_markers = robot.GetDetectedObjects([MazeObject.victimMarker])
yellow_victims = robot.GetDetectedObjects([MazeObject.victims])
```

## Student-safe release allow-list

- `EGB320_search_and_rescue_2026.ttt`: CoppeliaSim scene.
- `mazebot_lib.py`: Python robot library.
- `EGB320_CoppeliaSim_Example.py`: minimal starting example.
- `EGB320_CoppeliaSim_Example_keyboard.py`: manual driving and sensor demonstration.
- `EGB320_CoppeliaSim_Example_navigation.py`: known-route odometry/navigation example.
- `keyboard_control.py`: keyboard helper used by the manual-driving example.

Teacher-only solution files—do not include these in a student release:

- `EGB320_CoppeliaSim_Example_floodfill.py`: complete mapping and flood-fill mission.
- `maze_navigation.py`: solution map, localiser, planner and visualiser.

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
