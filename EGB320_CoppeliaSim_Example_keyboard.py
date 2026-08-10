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
- Generates the Phase 1 example maze (7x7 cells, wall posts and three victims)
- Places the robot at the base cell
- Lets you drive the robot around with the keyboard and attempt victim collection

KEYBOARD CONTROLS:
==================
W / S - drive forward / backward
A / D - rotate left / right on the spot
SPACE - collect a nearby victim, or release the carried victim
Q     - quit
Hold a movement key to drive. Releasing the key immediately commands zero velocity.

STUDENT TASKS:
=============
- Implement navigation/mapping using object and wall detection data
- Implement victim detection and call CollectVictim() when your robot finds a victim

For more information, see the documentation in mazebot_lib.py
"""

# Import the maze bot library
from mazebot_lib import *

# Import additional modules
import os
import ctypes
import time
import math

try:
	if os.name != 'nt':
		raise OSError("held-key polling is only available on Windows")
	_get_async_key_state = ctypes.windll.user32.GetAsyncKeyState
	_get_async_key_state.argtypes = [ctypes.c_int]
	_get_async_key_state.restype = ctypes.c_short
	_HAS_HELD_KEY_INPUT = True
except (AttributeError, OSError):
	_get_async_key_state = None
	_HAS_HELD_KEY_INPUT = False

_VIRTUAL_KEYS = {
	'w': 0x57,
	'a': 0x41,
	's': 0x53,
	'd': 0x44,
	'space': 0x20,
	'q': 0x51,
}

def clear_screen():
	"""Clear the terminal screen for better output readability"""
	os.system('cls' if os.name == 'nt' else 'clear')

def is_key_down(key):
	"""Return whether a keyboard key is currently held down."""
	if not _HAS_HELD_KEY_INPUT:
		return False
	return bool(_get_async_key_state(_VIRTUAL_KEYS[key]) & 0x8000)

def get_keyboard_command():
	"""Return the velocity command represented by the currently held keys."""
	if is_key_down('q'):
		raise KeyboardInterrupt

	linearVelocity = keyboardForwardSpeed * (int(is_key_down('w')) - int(is_key_down('s')))
	angularVelocity = keyboardTurnSpeed * (int(is_key_down('a')) - int(is_key_down('d')))
	return linearVelocity, angularVelocity

def format_distance(distance):
	"""Format an optional proximity reading for the terminal display."""
	return "no detection" if distance is None else f"{distance:0.3f} m"

# CONFIGURE SCENE PARAMETERS (search and rescue maze)
sceneParameters = SceneParameters()

# The Phase 1 example maze (7x7 cells, 0.280 m cells, base cell (0,6), 3 victims) is set up
# by default in SceneParameters.__init__ - override any of these to experiment, e.g.:
# sceneParameters.placeRobotAtBase = False
# sceneParameters.clearGeneratedMaze = False  # leave a previously generated maze in place

# CONFIGURE ROBOT PARAMETERS
robotParameters = RobotParameters()

# Drive system settings
robotParameters.driveType = 'differential'        # Type of drive system
robotParameters.minimumLinearSpeed = 0.0          # Minimum forward speed (m/s)
robotParameters.maximumLinearSpeed = 0.25         # Maximum forward speed (m/s)
robotParameters.driveSystemQuality = 1            # Drive quality (0-1, 1=perfect)

# Camera settings
robotParameters.cameraOrientation = 'landscape'   # Camera orientation
robotParameters.cameraDistanceFromRobotCenter = 0.0  # Distance from robot center (m)
robotParameters.cameraHeightFromFloor = 0.15      # Height above floor (m)
robotParameters.cameraTilt = -0.1                 # Camera tilt angle (radians)

# Victim collection setting from the assessment rules
robotParameters.victimCollectionDistance = 0.10  # Maximum horizontal clearance (m)

# Simulation settings
robotParameters.sync = False  # Use asynchronous mode (recommended)

# KEYBOARD TELEOPERATION SETTINGS
keyboardForwardSpeed = 0.03   # m/s applied when driving forward/backward (kept low to avoid abrupt starts)
keyboardTurnSpeed = 0.3       # rad/s applied when rotating left/right (kept low to avoid abrupt starts)

# MAIN PROGRAM
if __name__ == '__main__':
	# Use try-except to handle Ctrl+C gracefully
	try:
		print("EGB320 CoppeliaSim Search and Rescue Robot Example")
		print("Press Ctrl+C to stop the simulation\n")
		
		# Enable/disable debug output
		show_debug_info = True

		# Create and initialize the maze robot
		print("Connecting to CoppeliaSim...")
		mazeBotSim = COPPELIA_MazeRobot(robotParameters, sceneParameters,
											coppelia_server_ip='127.0.0.1', port=23000)
		
		# Start the simulation (generates the maze, then starts the simulation)
		mazeBotSim.StartSimulator()

		if not _HAS_HELD_KEY_INPUT:
			print("Warning: held-key input is unavailable - keyboard control is disabled on this platform.")

		# Main control loop
		print("Starting main control loop...")
		print("Use W/A/S/D to drive, SPACE to collect/release a victim, Q to quit.")

		lastSentCommand = (0.0, 0.0)
		spaceWasDown = False
		collectionStatus = "No victim collection attempted."
		markerDetections = {
			'base': [],
			'victim': [],
			'rubble_victim': [],
			'hazard': [],
			'victim_object': [],
		}
		nextMarkerDetectionTime = 0.0

		while True:
			# Send only when the held-key command changes. Releasing the movement keys changes
			# the command to (0, 0), so exactly one stop command is sent on release.
			keyboardCommand = get_keyboard_command()
			if keyboardCommand != lastSentCommand:
				mazeBotSim.SetTargetVelocities(*keyboardCommand)
				lastSentCommand = keyboardCommand

			# Trigger one collection/release action on the press edge. Holding Space does not
			# repeatedly call either API.
			spaceIsDown = is_key_down('space')
			if spaceIsDown and not spaceWasDown:
				if mazeBotSim.HasVictim():
					success, victimLabel = mazeBotSim.ReleaseVictim()
					if success:
						collectionStatus = f"Released victim {victimLabel} onto the maze floor."
					else:
						collectionStatus = "Victim release failed."
				else:
					success, victimLabel, collectionDistance = mazeBotSim.CollectVictim()
					if success:
						collectionStatus = (
							f"Collected victim {victimLabel} at {collectionDistance:.3f} m clearance.")
					else:
						collectionStatus = (
							f"Collection failed: no victim is within "
							f"{robotParameters.victimCollectionDistance:.3f} m.")
			spaceWasDown = spaceIsDown

			# Robot-relative proximity readings. None means no detectable object is in range.
			wallDistances = mazeBotSim.GetWallDistances()

			# Optional: Get camera image for computer vision processing
			# This will slow down the sim
			# resolution, image_data = mazeBotSim.GetCameraImage()

			# Update object positions (required for accurate detection)
			mazeBotSim.UpdateObjectPositions()

			# The detector is only 32x24, but throttle this example to 10 Hz so terminal
			# rendering and keyboard polling are not needlessly slowed down.
			currentTime = time.monotonic()
			if currentTime >= nextMarkerDetectionTime:
				markerDetections = mazeBotSim.GetDetectedMarkers()
				nextMarkerDetectionTime = currentTime + 0.1

			# Clear screen and show current status
			if show_debug_info:
				clear_screen()
				print("EGB320 Search and Rescue Robot - Status")
				print("=" * 50)
				print("Controls: W/S = forward/back, A/D = rotate, SPACE = collect/release, Q = quit")
				print(
					f"Commanded velocity (x_dot, theta_dot): {lastSentCommand[0]:0.2f} m/s, "
					f"{lastSentCommand[1]:0.2f} rad/s")

				if mazeBotSim.robotPose is not None:
					print("Robot pose (x, y, theta): %0.3f, %0.3f, %0.3f" % tuple(mazeBotSim.robotPose[:3]))

				print(
					"Wall Distance sensors: "
					f"left={format_distance(wallDistances['left'])}, "
					f"front={format_distance(wallDistances['front'])}, "
					f"right={format_distance(wallDistances['right'])}")

				print(f"Victim collection: {collectionStatus}")
				if mazeBotSim.HasVictim():
					print(f"Carrying victim: {mazeBotSim.carriedVictimLabel}")

				visibleMarkerTypes = [
					markerType
					for markerType in ('base', 'victim', 'rubble_victim', 'hazard')
					if markerDetections[markerType]
				]
				print(
					"Visible wall markers: " +
					(", ".join(visibleMarkerTypes) if visibleMarkerTypes else "none"))

				if markerDetections['victim_object']:
					victimRange, victimBearing = markerDetections['victim_object'][0]
					print(
						f"Visible yellow victim: range={victimRange:.3f} m, "
						f"bearing={victimBearing:.3f} rad "
						f"({math.degrees(victimBearing):.1f} deg)")
				else:
					print("Visible yellow victim: none")

				# Ground-truth victim positions for comparison with GetDetectedVictims().
				for label, position in mazeBotSim.victimPositions.items():
					print(f"Victim {label} position (x,y,z): {position[0]:0.3f}, {position[1]:0.3f}, {position[2]:0.3f}")

				print("=" * 50)

			# Students: Add your navigation, mapping and victim detection logic here

	except KeyboardInterrupt:
		print("\nStopping simulation...")
		mazeBotSim.StopSimulator()
		print("Simulation stopped successfully. Goodbye!")

	except Exception as e:
		print(f"\nAn error occurred: {e}")
		print("Stopping simulation...")
		try:
			mazeBotSim.StopSimulator()
		except:
			pass
		print("Please check your CoppeliaSim setup and try again.")
