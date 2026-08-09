#!/usr/bin/python

"""
EGB320 CoppeliaSim Search and Rescue Robot Example (2026, Phase 1)

GETTING STARTED:
===============
1. Open CoppeliaSim and load the search and rescue maze scene
2. Run this Python script
3. StartSimulator() deterministically (re)generates the maze (posts/walls/victims)
   while the simulation is stopped, then starts the simulation

WHAT THIS EXAMPLE DOES:
======================
- Connects to CoppeliaSim
- Generates the Phase 1 example maze (7x7 cells, wall posts, victims)
- Places the robot at the base cell
- Shows basic robot movement and prints the robot pose and victim positions

STUDENT TASKS:
=============
- Modify the robot movement commands in the main loop
- Implement navigation/mapping using object and wall detection data
- Implement victim detection and call CollectVictim() when your robot finds a victim

For more information, see the documentation in warehousebot_lib.py
"""

# Import the warehouse bot library
from warehousebot_lib import *

# Import additional modules
import os

def clear_screen():
	"""Clear the terminal screen for better output readability"""
	os.system('cls' if os.name == 'nt' else 'clear')

# CONFIGURE SCENE PARAMETERS (search and rescue maze)
sceneParameters = SceneParameters()

# The Phase 1 example maze (7x7 cells, 0.280 m cells, base cell (0,6), 3 victims) is set up
# by default in SceneParameters.__init__ - override any of these to experiment, e.g.:
# sceneParameters.placeRobotAtBase = False
# sceneParameters.clearGeneratedMaze = False  # leave a previously generated maze in place

# Diagnostic setup: clear old generated objects and run on a blank table.
sceneParameters.generateMazeObjects = False

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

# Victim collection setting from the assessment rules
robotParameters.victimCollectionDistance = 0.10  # Maximum horizontal clearance (m)

# Simulation settings
robotParameters.sync = False  # Use asynchronous mode (recommended)

# MAIN PROGRAM
if __name__ == '__main__':
	# Use try-except to handle Ctrl+C gracefully
	try:
		print("EGB320 CoppeliaSim Search and Rescue Robot Example")
		print("Press Ctrl+C to stop the simulation\n")
		
		# Enable/disable debug output
		show_debug_info = True

		# Create and initialize the warehouse robot
		print("Connecting to CoppeliaSim...")
		warehouseBotSim = COPPELIA_WarehouseRobot(robotParameters, sceneParameters, 
													coppelia_server_ip='127.0.0.1', port=23000)
		
		# Start the simulation (generates the maze, then starts the simulation)
		warehouseBotSim.StartSimulator()

		# Main control loop
		print("Starting main control loop...")
		print("Robot is being commanded forward at 0.08 m/s.")
		
		while True:
			# Set robot movement (forward_velocity, rotation_velocity)
			# Fixed command for testing SetTargetVelocities without keyboard input.
			warehouseBotSim.SetTargetVelocities(0.08, 0.0)

			# Optional: Get camera image for computer vision processing
			# This will slow down the sim
			#resolution, image_data = warehouseBotSim.GetCameraImage()

			# Update object positions (required for accurate detection)
			warehouseBotSim.UpdateObjectPositions()

			# Clear screen and show current status
			if show_debug_info:
				clear_screen()
				print("EGB320 Search and Rescue Robot - Status")
				print("=" * 50)

				if warehouseBotSim.robotPose is not None:
					print("Robot pose (x, y, theta): %0.3f, %0.3f, %0.3f" % tuple(warehouseBotSim.robotPose[:3]))

				# Ground-truth victim positions (victim detection is not implemented in this phase)
				for label, position in warehouseBotSim.victimPositions.items():
					print(f"Victim {label} position (x,y,z): {position[0]:0.3f}, {position[1]:0.3f}, {position[2]:0.3f}")

				print("=" * 50)

			# Students: Add your navigation, mapping and victim detection logic here

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
