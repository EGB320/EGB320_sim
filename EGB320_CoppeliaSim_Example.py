#!/usr/bin/python

"""
EGB320 CoppeliaSim Warehouse Robot Example

GETTING STARTED:
===============
1. Open CoppeliaSim
2. Load the warehouse robot scene file
3. Run this Python script
4. The robot will connect to CoppeliaSim and start detecting objects

WHAT THIS EXAMPLE DOES:
======================
- Connects to CoppeliaSim simulation
- Sets up a warehouse scene with items at picking stations
- Continuously detects objects using the robot's camera
- Shows how to control robot movement
- Demonstrates item collection and dropping

STUDENT TASKS:
=============
- Modify the robot movement commands in the main loop
- Implement navigation algorithms using object detection data
- Add logic to collect items and deliver them to shelves
- Experiment with different robot parameters

For more information, see the documentation in warehousebot_lib.py
"""

# Import the warehouse bot library
from warehousebot_lib import *

# Import additional modules
import os

def clear_screen():
	"""Clear the terminal screen for better output readability"""
	os.system('cls' if os.name == 'nt' else 'clear')

# CONFIGURE SCENE PARAMETERS
sceneParameters = SceneParameters()

# Set which items appear at which picking stations (0=station 1, 1=station 2, 2=station 3)
# Set to -1 to leave a station empty
sceneParameters.pickingStationContents[0] = warehouseObjects.bowl    # Bowl at picking station 1
sceneParameters.pickingStationContents[1] = warehouseObjects.mug     # Mug at picking station 2  
sceneParameters.pickingStationContents[2] = warehouseObjects.bottle  # Bottle at picking station 3

# Set obstacle starting positions [x, y] in meters
# Use -1 to keep current CoppeliaSim position, None to disable obstacle
sceneParameters.obstacle0_StartingPosition = [-0.2, -0.25]
sceneParameters.obstacle1_StartingPosition = -1  # Use current position
sceneParameters.obstacle2_StartingPosition = -1  # Use current position

# CONFIGURE ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive system settings
robotParameters.driveType = 'differential'        # Type of drive system
robotParameters.minimumLinearSpeed = 0.0          # Minimum forward speed (m/s)
robotParameters.maximumLinearSpeed = 0.25         # Maximum forward speed (m/s)
robotParameters.driveSystemQuality = 1            # Drive quality (0-1, 1=perfect)

# Camera settings
robotParameters.cameraOrientation = 'landscape'   # Camera orientation
robotParameters.cameraDistanceFromRobotCenter = 0.1  # Distance from robot center (m)
robotParameters.cameraHeightFromFloor = 0.15      # Height above floor (m)
robotParameters.cameraTilt = 0.0                  # Camera tilt angle (radians)

# Object detection ranges (in meters)
robotParameters.maxItemDetectionDistance = 1.0         # Items
robotParameters.maxPackingBayDetectionDistance = 2.5   # Picking stations
robotParameters.maxObstacleDetectionDistance = 1.5     # Obstacles
robotParameters.maxRowMarkerDetectionDistance = 2.5    # Row markers

# Item collection settings
robotParameters.collectorQuality = 1              # Collector reliability (0-1)
robotParameters.maxCollectDistance = 0.15         # Maximum collection distance (m)

# Simulation settings
robotParameters.sync = False  # Use asynchronous mode (recommended)

# MAIN PROGRAM
if __name__ == '__main__':
	# Use try-except to handle Ctrl+C gracefully
	try:
		print("EGB320 CoppeliaSim Warehouse Robot Example")
		print("Press Ctrl+C to stop the simulation\n")
		
		# Enable/disable debug output
		show_debug_info = False

		# Create and initialize the warehouse robot
		print("Connecting to CoppeliaSim...")
		warehouseBotSim = COPPELIA_WarehouseRobot(robotParameters, sceneParameters, 
													coppelia_server_ip='127.0.0.1', port=23000)
		
		# Start the simulation
		warehouseBotSim.StartSimulator()

		# Main control loop
		print("Starting main control loop...")
		print("Robot is now ready for navigation commands.")
		
		while True:
			# Set robot movement (forward_velocity, rotation_velocity)
			# Students: Replace these values with your navigation algorithm
			warehouseBotSim.SetTargetVelocities(0.0, 0.0)  # Stop the robot

			# Get all detected objects in camera field of view
			objectsRB = warehouseBotSim.GetDetectedObjects([
				warehouseObjects.items,                # Items (bowls, mugs, etc.)
				warehouseObjects.shelves,              # Storage shelves
				warehouseObjects.row_markers,          # Navigation markers
				warehouseObjects.obstacles,            # Obstacles to avoid
				warehouseObjects.pickingStation,       # Main picking station
				warehouseObjects.PickingStationMarkers # Individual picking stations
			])

			# Unpack the detection results
			itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB, pickingStationRB = objectsRB

			# Optional: Get camera image for computer vision processing
			# resolution, image_data = warehouseBotSim.GetCameraImage()

			# Clear screen and show current status
			if show_debug_info:
				clear_screen()
				print("EGB320 Warehouse Robot - Object Detection Status")
				print("=" * 50)
				
				# Display detected objects
				print_debug_range_bearing("Items", itemsRB)
				print_debug_range_bearing("Packing Station", packingStationRB)
				print_debug_range_bearing("Obstacles", obstaclesRB)
				print_debug_range_bearing("Row Markers", rowMarkerRB)
				print_debug_range_bearing("Shelves", shelfRB)
				print_debug_range_bearing("Picking Stations", pickingStationRB)
				print("=" * 50)

			# Update object positions (required for accurate detection)
			warehouseBotSim.UpdateObjectPositions()
			
			# Students: Add your navigation and control logic here
			
			# Example item collection (uncomment to test)
			# success, station = warehouseBotSim.CollectItem(closest_picking_station=True)
			# if success:
			#     print(f"Collected item from station {station}")

			# Example item dropping (uncomment to test)
			# if warehouseBotSim.itemCollected():
			#     success, shelf_info = warehouseBotSim.DropItemInClosestShelfBay()
			#     if success:
			#         print(f"Dropped item at shelf {shelf_info['shelf']}")

	except KeyboardInterrupt:
		print("\nStopping simulation...")
		warehouseBotSim.StopSimulator()
		print("Simulation stopped successfully. Goodbye!")

	except Exception as e:
		print(f"\nAn error occurred: {e}")
		print("Stopping simulation...")
		try:
			warehouseBotSim.StopSimulator()
		except:
			pass
		print("Please check your CoppeliaSim setup and try again.")