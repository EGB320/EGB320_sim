# EGB320 Warehouse Robot Library

A Python library for controlling autonomous warehouse robots in CoppeliaSim simulation environment. This library provides a comprehensive interface for robot navigation, object detection, item collection, and delivery operations.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [API Documentation](#api-documentation)
- [Examples](#examples)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## Prerequisites

- **CoppeliaSim** (Version 4.3.0 or later)
- **Python** 3.7 or later
- **Git** (for cloning the repository)

## Installation

### 1. Install CoppeliaSim

Download and install CoppeliaSim from the [official website](https://www.coppeliarobotics.com/downloads).

### 2. Clone the Repository

```bash
git clone https://github.com/EGB320/EGB320.github.io.git
cd EGB320.github.io/VREP_Sim/EGB320_VREP_Files/VREP_PythonCode
```

### 3. Install Python Dependencies

Install the required Python packages:

```bash
# Core dependency for CoppeliaSim communication
pip install coppeliasim-zmqremoteapi-client

# Additional dependencies for computer vision and GUI
pip install opencv-python pygame numpy
```

### 4. Verify Installation

Test your installation by running the example script:

```bash
python EGB320_CoppeliaSim_Example.py
```

## Quick Start

### Basic Setup

```python
from warehousebot_lib import *

# Configure robot parameters
robotParameters = RobotParameters()
robotParameters.maximumLinearSpeed = 0.25
robotParameters.driveSystemQuality = 1

# Configure scene parameters
sceneParameters = SceneParameters()
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl
sceneParameters.pickingStationContents[1] = warehouseObjects.mug
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle

# Initialize robot
robot = COPPELIA_WarehouseRobot(robotParameters, sceneParameters)
robot.StartSimulator()

# Main control loop
try:
    while True:
        # Update object positions (CRITICAL!)
        robot.UpdateObjectPositions()
        
        # Get object detections
        objects = robot.GetDetectedObjects([warehouseObjects.items, warehouseObjects.obstacles])
        items, _, obstacles, _, _, _ = objects
        
        # Simple navigation
        if len(obstacles) > 0:
            robot.SetTargetVelocities(0.0, 0.3)  # Turn away from obstacles
        elif len(items) > 0:
            robot.SetTargetVelocities(0.1, 0.0)  # Move towards items
        else:
            robot.SetTargetVelocities(0.0, 0.2)  # Search by rotating
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    robot.StopSimulator()
```

## API Documentation

### Core Classes

#### `COPPELIA_WarehouseRobot`

Main class for controlling the warehouse robot in CoppeliaSim.

**Constructor**
```python
COPPELIA_WarehouseRobot(robotParameters, sceneParameters, coppelia_server_ip='127.0.0.1', port=23000)
```

**Parameters:**
- `robotParameters` (RobotParameters): Robot configuration object
- `sceneParameters` (SceneParameters): Scene configuration object  
- `coppelia_server_ip` (str, optional): CoppeliaSim server IP address. Default: '127.0.0.1'
- `port` (int, optional): ZMQ Remote API port number. Default: 23000

---

### Simulation Control

#### `StartSimulator()`

Starts the CoppeliaSim simulation and initializes the scene.

**Returns:** None

**Raises:** SystemExit if simulation fails to start

**Example:**
```python
robot.StartSimulator()
```

---

#### `StopSimulator()`

Stops the CoppeliaSim simulation.

**Returns:** None

**Example:**
```python
robot.StopSimulator()
```

---

### Robot Movement

#### `SetTargetVelocities(x_dot, theta_dot)`

Sets the target velocities for robot movement.

**Parameters:**
- `x_dot` (float): Forward velocity in m/s
- `theta_dot` (float): Rotational velocity in rad/s

**Returns:** None

**Example:**
```python
robot.SetTargetVelocities(0.1, 0.0)    # Move forward at 0.1 m/s
robot.SetTargetVelocities(0.0, 0.5)    # Rotate at 0.5 rad/s
robot.SetTargetVelocities(0.1, 0.2)    # Move forward and turn
robot.SetTargetVelocities(0.0, 0.0)    # Stop
```

---

### Object Detection

#### `GetDetectedObjects(objects=None)`

Gets range and bearing to all detected objects in the camera's field of view.

**Parameters:**
- `objects` (list, optional): List of object types to detect. If None, detects all objects.

**Returns:** 
- `tuple`: (itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB, pickingStationRB)
  - `itemsRB` (list): Range and bearing to items [6-element list, one per item type]
  - `packingStationRB` (list): Range and bearing to main picking station
  - `obstaclesRB` (list): Range and bearing to obstacles
  - `rowMarkerRB` (list): Range and bearing to row markers [3-element list]
  - `shelfRB` (list): Range and bearing to shelves [6-element list]
  - `pickingStationRB` (list): Range and bearing to individual picking stations [3-element list]

**Data Format:**
Each detection is a `[range, bearing]` array where:
- `range`: Distance in meters (float)
- `bearing`: Angle in radians relative to robot's heading (float)

**Example:**
```python
# Detect items and obstacles
objects = robot.GetDetectedObjects([warehouseObjects.items, warehouseObjects.obstacles])
items, _, obstacles, _, _, _ = objects

# Check for detected items
if items[warehouseObjects.bowl] is not None:
    for detection in items[warehouseObjects.bowl]:
        range_m, bearing_rad = detection
        print(f"Bowl detected at {range_m:.2f}m, {math.degrees(bearing_rad):.1f}°")
```

---

#### `GetCameraImage()`

Gets the current camera image from the robot's vision sensor.

**Returns:**
- `tuple`: (resolution, image_data)
  - `resolution` (list): [width, height] of the image
  - `image_data` (list): Image pixel data, or None if no image available

**Example:**
```python
resolution, image_data = robot.GetCameraImage()
if image_data is not None:
    width, height = resolution
    print(f"Got image: {width}x{height} pixels")
```

---

#### `GetDetectedWallPoints()`

Gets range and bearing to wall points visible in the camera's field of view.

**Returns:**
- `list`: List of [range, bearing] arrays for visible wall points, or None if no walls detected

**Example:**
```python
wall_points = robot.GetDetectedWallPoints()
if wall_points:
    for point in wall_points:
        range_m, bearing_rad = point
        print(f"Wall point at {range_m:.2f}m, {math.degrees(bearing_rad):.1f}°")
```

---

### Item Collection and Delivery

#### `CollectItem(closest_picking_station=False)`

Attempts to collect items from picking stations.

**Parameters:**
- `closest_picking_station` (bool, optional): If True, collects from the closest picking station. Default: False

**Returns:**
- `tuple`: (success, station_number)
  - `success` (bool): True if item was collected successfully
  - `station_number` (int): Station number (1-3) where item was collected, or None if failed

**Example:**
```python
# Try to collect from closest picking station
success, station = robot.CollectItem(closest_picking_station=True)
if success:
    print(f"Collected item from station {station}")
else:
    print("Failed to collect item")
```

---

#### `DropItemInClosestShelfBay(max_drop_distance=0.5)`

Drops an item in the closest empty shelf bay.

**Parameters:**
- `max_drop_distance` (float, optional): Maximum distance to consider a shelf bay. Default: 0.5 meters

**Returns:**
- `tuple`: (success, shelf_info)
  - `success` (bool): True if item was dropped successfully
  - `shelf_info` (dict): Dictionary with shelf information, or None if failed
    - `shelf` (int): Shelf number (0-5)
    - `x` (int): X position in shelf (0-3)
    - `y` (int): Y position in shelf (0-2)
    - `distance` (float): Distance to shelf bay

**Example:**
```python
if robot.itemCollected():
    success, shelf_info = robot.DropItemInClosestShelfBay()
    if success:
        shelf = shelf_info['shelf']
        print(f"Item dropped at shelf {shelf}")
```

---

#### `itemCollected()`

Checks if the robot is currently carrying an item.

**Returns:**
- `bool`: True if robot is carrying an item, False otherwise

**Example:**
```python
if robot.itemCollected():
    print("Robot is carrying an item")
    # Try to drop it at a shelf
    robot.DropItemInClosestShelfBay()
```

---

#### `DropItem()`

Drops the currently held item at the robot's current location.

**Returns:** None

**Example:**
```python
if robot.itemCollected():
    robot.DropItem()
    print("Item dropped")
```

---

### Configuration

#### `UpdateObjectPositions()`

Updates the positions of all objects in the simulation. **This must be called in every control loop.**

**Returns:**
- `tuple`: (robotPose, itemPositions, obstaclePositions) for debugging purposes

**Example:**
```python
while True:
    # CRITICAL: Always call this in your main loop
    robot.UpdateObjectPositions()
    
    # Rest of your control logic
    # ...
```

---

#### `SetCameraResolution(x_res, y_res)`

Sets the camera resolution to specific width and height values.

**Parameters:**
- `x_res` (int): Camera width resolution in pixels
- `y_res` (int): Camera height resolution in pixels

**Returns:**
- `bool`: True if successful, False otherwise

**Example:**
```python
success = robot.SetCameraResolution(640, 480)
if success:
    print("Camera resolution set to 640x480")
```

---

### Object Types

#### `warehouseObjects` (Enum)

Enumeration of all object types in the warehouse simulation.

**Item Types:**
- `warehouseObjects.bowl` (0): Cereal bowls
- `warehouseObjects.mug` (1): Coffee mugs
- `warehouseObjects.bottle` (2): Water bottles
- `warehouseObjects.soccer` (3): Soccer balls
- `warehouseObjects.rubiks` (4): Rubik's cubes
- `warehouseObjects.cereal` (5): Cereal boxes

**Obstacle Types:**
- `warehouseObjects.obstacle0` (6): First obstacle
- `warehouseObjects.obstacle1` (7): Second obstacle
- `warehouseObjects.obstacle2` (8): Third obstacle

**Station Types:**
- `warehouseObjects.pickingStation` (9): Main picking station
- `warehouseObjects.pickingStation1` (19): Individual picking station 1
- `warehouseObjects.pickingStation2` (20): Individual picking station 2
- `warehouseObjects.pickingStation3` (21): Individual picking station 3

**Navigation Types:**
- `warehouseObjects.row_marker_1` (10): Row marker 1
- `warehouseObjects.row_marker_2` (11): Row marker 2
- `warehouseObjects.row_marker_3` (12): Row marker 3

**Storage Types:**
- `warehouseObjects.shelf_0` through `warehouseObjects.shelf_5` (13-18): Storage shelves

**Object Groups:**
- `warehouseObjects.items` (101): All collectible items
- `warehouseObjects.obstacles` (102): All obstacles
- `warehouseObjects.row_markers` (103): All row markers
- `warehouseObjects.shelves` (104): All shelves
- `warehouseObjects.PickingStationMarkers` (105): All individual picking stations

---

## Examples

### Example 1: Basic Navigation with Obstacle Avoidance

```python
from warehousebot_lib import *
import time

# Initialize robot
robotParameters = RobotParameters()
sceneParameters = SceneParameters()
robot = COPPELIA_WarehouseRobot(robotParameters, sceneParameters)
robot.StartSimulator()

try:
    while True:
        robot.UpdateObjectPositions()
        
        # Get obstacle detections
        objects = robot.GetDetectedObjects([warehouseObjects.obstacles])
        _, _, obstacles, _, _, _ = objects
        
        # Simple obstacle avoidance
        if obstacles and len(obstacles) > 0:
            print(f"Obstacle detected! Turning away...")
            robot.SetTargetVelocities(0.0, 0.5)  # Turn right
        else:
            print("Clear path, moving forward")
            robot.SetTargetVelocities(0.2, 0.0)  # Move forward
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    robot.StopSimulator()
```

### Example 2: Item Collection and Delivery

```python
from warehousebot_lib import *
import time

# Setup
robotParameters = RobotParameters()
sceneParameters = SceneParameters()
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl
robot = COPPELIA_WarehouseRobot(robotParameters, sceneParameters)
robot.StartSimulator()

try:
    while True:
        robot.UpdateObjectPositions()
        
        if not robot.itemCollected():
            # Try to collect items
            success, station = robot.CollectItem(closest_picking_station=True)
            if success:
                print(f"Collected item from station {station}")
        else:
            # Try to deliver item
            success, shelf_info = robot.DropItemInClosestShelfBay()
            if success:
                print(f"Delivered item to shelf {shelf_info['shelf']}")
            else:
                # Navigate to find a shelf
                robot.SetTargetVelocities(0.1, 0.1)
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    robot.StopSimulator()
```

### Example 3: Computer Vision Processing

```python
from warehousebot_lib import *
import cv2
import numpy as np

# Initialize robot
robotParameters = RobotParameters()
sceneParameters = SceneParameters()
robot = COPPELIA_WarehouseRobot(robotParameters, sceneParameters)
robot.StartSimulator()

try:
    while True:
        robot.UpdateObjectPositions()
        
        # Get camera image
        resolution, image_data = robot.GetCameraImage()
        
        if image_data is not None:
            # Convert to OpenCV format
            width, height = resolution
            image_array = np.array(image_data, dtype=np.uint8)
            image = image_array.reshape((height, width, 3))
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # Display image
            cv2.imshow('Robot Camera', image_bgr)
            cv2.waitKey(1)
        
        time.sleep(0.1)
        
except KeyboardInterrupt:
    cv2.destroyAllWindows()
    robot.StopSimulator()
```

## Troubleshooting

### Connection Issues

**Problem:** "Failed to connect to CoppeliaSim"
- **Solution:** Ensure CoppeliaSim is running and the correct scene is loaded
- **Check:** Verify ZMQ Remote API is enabled in CoppeliaSim
- **Port:** Default port is 23000, make sure it's not blocked

**Problem:** "Port 23000 connection failed"
- **Solution:** Check if another program is using the port
- **Alternative:** Try a different port number in the constructor

### Detection Issues

**Problem:** No objects detected
- **Solution:** Always call `UpdateObjectPositions()` in your main loop
- **Check:** Verify objects are within detection ranges
- **Debug:** Use `GetCameraImage()` to verify camera is working

**Problem:** Objects detected incorrectly
- **Solution:** Check robot parameter detection distances
- **Calibration:** Adjust camera pose and orientation settings

### Movement Issues

**Problem:** Robot not moving
- **Solution:** Check velocity values are within speed limits
- **Verify:** Ensure simulation is running (not paused)
- **Debug:** Test with small velocity values first

**Problem:** Robot moving erratically
- **Solution:** Reduce maximum speed parameters
- **Adjust:** Increase `driveSystemQuality` value
- **Check:** Ensure `UpdateObjectPositions()` is called regularly

### Installation Issues

**Problem:** "No module named 'coppeliasim_zmqremoteapi_client'"
- **Solution:** Install using pip: `pip install coppeliasim-zmqremoteapi-client`
- **Alternative:** Check if you're using the correct Python environment

**Problem:** ImportError with other dependencies
- **Solution:** Install all required packages: `pip install opencv-python pygame numpy`

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Support

For technical support:
- Check the [Troubleshooting](#troubleshooting) section
- Review the [Examples](#examples) for common use cases
- Contact course instructors for EGB320-specific questions

## Changelog

### Version 1.0.0
- Initial release with full warehouse robot functionality
- Support for CoppeliaSim ZMQ Remote API
- Comprehensive object detection and navigation capabilities
- Item collection and delivery system
