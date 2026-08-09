# Import required Python modules
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import sys
from collections import deque
from enum import IntEnum

"""
EGB320 Search and Rescue MazeBot Library (2026)

This library provides a Python interface for controlling the EGB320 maze robot in CoppeliaSim.

As of this phase, the library targets the 2026 search and rescue maze challenge. It uses
the CoppeliaSim ZeroMQ Python remote API to deterministically generate a 7x7 cell maze
(walls, corner posts) and place three victim objects.

MAIN FUNCTIONS:
===========================

Robot Control:
- StartSimulator() / StopSimulator()    : Start/stop the simulation (generates the maze while stopped)
- SetTargetVelocities(x_dot, theta_dot) : Set robot movement velocities
- UpdateObjectPositions()               : Update object positions (call this every loop!)

Object Detection:
- GetDetectedObjects(objects)           : Get range/bearing to optional obstacle objects
- GetCameraImage()                      : Get camera image for computer vision
- GetWallDistances()                    : Get robot-relative left/front/right proximity readings

Victim Collection:
- CollectVictim()                       : Attempt to collect a victim within 0.10 m
- ReleaseVictim()                       : Place the carried victim on the ground
- HasVictim()                           : Check whether the robot is carrying a victim

Configuration:
- SetCameraResolution(x_res, y_res)    : Set camera resolution

EXAMPLE USAGE:
=============
# Initialize robot
robot = COPPELIA_MazeRobot(robotParameters, sceneParameters)
robot.StartSimulator()  # generates the maze, then starts the simulation

# Main control loop
while True:
    # Move robot
    robot.SetTargetVelocities(0.1, 0.0)  # Move forward at 0.1 m/s
    
    # Update positions (important!)
    robot.UpdateObjectPositions()

For more examples, see EGB320_CoppeliaSim_Example.py
"""


# Deterministic example maze layout (Phase 1): internal wall segments between grid
# intersections. Each entry is ((startColumn, startRow), (endColumn, endRow)) using the
# grid intersection convention described in SetScene/_grid_point_to_world. Intentionally
# not merged/optimised - this should always produce exactly 40 internal wall copies.
EXAMPLE_MAZE_SEGMENTS = [
	((4, 0), (4, 1)),

	((1, 1), (2, 1)),
	((2, 1), (2, 2)),

	((3, 1), (3, 2)),
	((3, 2), (3, 3)),
	((3, 1), (4, 2)),
	((4, 2), (5, 1)),

	((5, 1), (6, 1)),
	((6, 1), (6, 2)),
	((6, 2), (6, 3)),

	((5, 1), (5, 2)),
	((5, 2), (5, 3)),
	((5, 3), (5, 4)),

	((0, 2), (1, 2)),
	((1, 2), (2, 2)),

	((3, 2), (4, 2)),
	((4, 2), (5, 2)),

	((1, 3), (1, 4)),

	((2, 3), (3, 3)),
	((2, 3), (2, 4)),
	((2, 4), (2, 5)),
	((2, 5), (2, 6)),
	((2, 4), (3, 3)),

	((3, 4), (4, 3)),

	((1, 4), (2, 4)),

	((3, 4), (4, 4)),
	((4, 4), (5, 4)),

	((6, 4), (7, 4)),

	((3, 4), (3, 5)),

	((1, 5), (1, 6)),
	((1, 6), (1, 7)),

	((4, 5), (4, 6)),

	((5, 4), (5, 5)),
	((5, 5), (5, 6)),
	((5, 5), (6, 5)),

	((4, 6), (5, 6)),
	((5, 6), (6, 6)),

	((1, 6), (2, 7)),
	((2, 7), (3, 6)),
	((3, 6), (3, 7)),
]

# Small separation used when placing a victim on either the maze floor or robot chassis.
VICTIM_SURFACE_CLEARANCE = 0.001
VICTIM_CARRY_ORIENTATION = (0.0, math.pi / 2.0, 0.0)
VICTIM_RELEASE_FORWARD_OFFSET = 0.15



# Object types retained for the search-and-rescue challenge.
class mazeObjects(IntEnum):
	# Obstacle objects
	obstacle0 = 0
	obstacle1 = 1
	obstacle2 = 2

	# Obstacle group for detection
	obstacles = 100


################################
##### MAZE BOT CLASS #####
################################

class COPPELIA_MazeRobot(object):
	"""
	Main class for controlling the search-and-rescue maze robot in CoppeliaSim.
	This class provides robot navigation, sensing, maze generation and victim collection.
	"""
	
	####################################
	#### MAZE BOT INITIALIZATION ###
	####################################

	def __init__(self, robotParameters, sceneParameters, coppelia_server_ip='127.0.0.1', port=23000):
		"""
		Initialize the maze robot connection to CoppeliaSim.
		
		Args:
			robotParameters: RobotParameters object with robot configuration
			sceneParameters: SceneParameters object with scene configuration
			coppelia_server_ip: IP address of CoppeliaSim server (default: '127.0.0.1')
			port: Port number for ZMQ Remote API (default: 23000)
		"""
		print("Initializing maze robot connection...")
		
		# Store parameters
		self.robotParameters = robotParameters
		self.sceneParameters = sceneParameters
		self.port = port

		# Initialize wheel bias for drive system simulation
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# CoppeliaSim connection variables
		self.clientID = None
		self.client = None
		self.sim = None

		# CoppeliaSim object handles
		self.robotHandle = None
		self.scriptHandle = None
		self.cameraHandle = None
		self.objectDetectorHandle = None
		self.victimCarryPointHandle = None
		self.leftMotorHandle = None
		self.rightMotorHandle = None
		self.leftRearMotorHandle = None
		self.rightRearMotorHandle = None
		self.v60MotorHandle = None
		self.v180MotorHandle = None
		self.v300MotorHandle = None
		self.obstacleHandles = [None, None, None]
		self.proximityHandle = None
		self.distanceSensorHandles = {
			'left': None,
			'front': None,
			'right': None,
		}

		# Search and rescue maze scene object handles (essential for this phase)
		self.floorHandle = None
		self.tableWallHandles = []
		self.mazeWallTemplateHandle = None
		self.baseStationWallTemplateHandle = None
		self.victimWallTemplateHandle = None
		self.rubbleVictimWallTemplateHandle = None
		self.hazardWallTemplateHandle = None
		self.wallPostTemplateHandle = None
		self.victimTemplateHandle = None

		# Generated maze object bookkeeping (used for cleanup on regeneration)
		self.generatedSceneRootHandle = None
		self.generatedMazeWallHandles = []
		self.generatedMarkerWallHandles = []
		self.generatedWallPostHandles = []
		self.victimHandles = {}
		self.victimPositions = {}
		self.markerWallPlacements = []
		self.carriedVictimHandle = None
		self.carriedVictimLabel = None

		# Cached maze geometry (populated by _get_floor_info/_cache_template_geometry)
		self.floorCenter = None
		self.floorTopZ = None
		self.mazeXMinimum = None
		self.mazeYMaximum = None
		self.mazeWallTemplateSize = None
		self.wallTemplateGeometry = {}
		self.wallPostTemplateSize = None
		self.wallPostTemplateOrientation = None
		self.victimTemplateSize = None
		self.victimTemplateOrientation = None

		# Optional objects that could not be resolved during handle lookup (diagnostics only)
		self.missingOptionalObjects = []

		# Wheel bias simulation for imperfect drive systems
		if self.robotParameters.driveSystemQuality != 1:
			self.leftWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)
			self.rightWheelBias = np.random.normal(0, (1-self.robotParameters.driveSystemQuality)*0.2, 1)

		# Physical parameters
		self.obstacleSize = 0.18  # diameter of obstacle in meters

		# Object position variables
		self.robotPose = None
		self.cameraPose = None
		self.obstaclePositions = [None, None, None]

		# Connect to CoppeliaSim
		print("Connecting to CoppeliaSim...")
		self.OpenConnectionToZMQ(coppelia_server_ip, self.port)

		# Get object handles from the simulation
		print("Getting simulation object handles...")
		self.GetCOPPELIAObjectHandles()
		if self.missingOptionalObjects:
			print(f"Optional objects not found (safe to ignore for this phase): {', '.join(self.missingOptionalObjects)}")

		# Configure robot parameters
		print("Configuring robot parameters...")
		self.UpdateCOPPELIARobot()
		
		print("Maze robot initialization complete!")
	
	########################################
	##### MAIN API FUNCTIONS FOR STUDENTS #####
	########################################
	# These are the main functions students should use

	def StartSimulator(self):
		"""
		Starts the CoppeliaSim simulation.
		Can also be started manually by pressing the Play button in CoppeliaSim.

		The maze (posts, walls, victims) is generated deterministically while the
		simulation is stopped, so that starting/stopping/starting again never leaves
		duplicate generated objects in the scene.
		"""
		print('Starting CoppeliaSim simulation...')

		try:
			simState = self.sim.getSimulationState()
			if simState != self.sim.simulation_stopped:
				print('Simulation is currently running - stopping it to (re)generate the scene deterministically...')
				self.sim.stopSimulation()
				while self.sim.getSimulationState() != self.sim.simulation_stopped:
					time.sleep(0.05)
				print('Simulation stopped.')

			if self.robotParameters.sync:
				print('Setting synchronous mode (may cause issues - consider setting sync=False)')
				self.sim.setStepping(True)
		except Exception as e:
			print(f'Error checking/stopping simulation state: {e}')

		print('Preparing static scene (maze generation) while simulation is stopped...')
		self.SetScene()

		try:
			self.sim.startSimulation()
			print('CoppeliaSim simulation started successfully.')
		except Exception as e:
			print(f'Error starting simulation: {e}')
			print('Try starting the simulation manually by pressing Play in CoppeliaSim.')
			sys.exit(-1)

		time.sleep(1)
		self.GetObjectPositions()

	def StopSimulator(self):
		"""
		Stops the CoppeliaSim simulation.
		Can also be stopped manually by pressing the Stop button in CoppeliaSim.
		"""
		print('Stopping CoppeliaSim simulation...')
		try:
			self.sim.stopSimulation()
			print('CoppeliaSim simulation stopped successfully.')
		except Exception as e:
			print(f'Error stopping simulation: {e}')
			print('You can stop manually by pressing Stop in CoppeliaSim.')

	def _is_object_detected(self, objectsDetected, obj_index):
		"""Helper function to safely check detection array - expecting 1/0 values from Lua script"""
		return (isinstance(objectsDetected, (list, tuple)) and 
				len(objectsDetected) > obj_index and 
				obj_index >= 0 and 
				objectsDetected[obj_index] == 1)
	
	def _process_single_object_detection(self, position, detection_index, objectsDetected, max_detection_distance):
		"""Helper function to process detection of a single object at a position"""
		if position is not None and self._is_object_detected(objectsDetected, detection_index):
			if self.PointInsideArena(position):
				_valid, _range, _bearing = self.GetRBInCameraFOV(position)
				if _valid and _range < max_detection_distance:
					return [_range, _bearing]
		return None
	
	def GetDetectedObjects(self, objects=None):
		"""
		Gets the range and bearing to requested obstacles in the camera's field of view.
		
		Args:
			objects: List containing mazeObjects.obstacles and/or individual obstacle
				types. Defaults to all obstacles.
			
		Returns:
			list: Detected obstacles as [range, bearing] pairs. Returns an empty list when
				no requested obstacle is detected.
		"""
		# Default to detecting all obstacles if none are specified.
		if objects is None:
			objects = [mazeObjects.obstacles]

		requestedObjects = set(objects)
		obstacleTypes = (
			mazeObjects.obstacle0,
			mazeObjects.obstacle1,
			mazeObjects.obstacle2,
		)
		detectAllObstacles = mazeObjects.obstacles in requestedObjects
		if not detectAllObstacles and not any(obstacle in requestedObjects for obstacle in obstacleTypes):
			return []

		# Check if camera and detector data are available.
		if self.cameraPose is None or self.objectDetectorHandle is None:
			return []

		# Get object detection data from CoppeliaSim vision sensor
		try:
			result, data, packets = self.sim.handleVisionSensor(self.objectDetectorHandle)
			
			if result == -1 or not packets or len(packets) == 0:
				objectsDetected = []
			else:
				objectsDetected = packets
				
		except Exception:
			objectsDetected = []

		obstaclesRangeBearing = []
		if objectsDetected:
			# New maze-only detector scripts use indices 0-2. Accept the previous 6-8
			# layout as well so existing scene sensor scripts continue to work.
			detectorIndexOffset = 6 if len(objectsDetected) >= 9 else 0
			for index, obstaclePosition in enumerate(self.obstaclePositions):
				if not detectAllObstacles and obstacleTypes[index] not in requestedObjects:
					continue
				if obstaclePosition is not None:
					result = self._process_single_object_detection(
						obstaclePosition,
						detectorIndexOffset + index,
						objectsDetected,
						self.robotParameters.maxObstacleDetectionDistance)
					if result is not None:
						obstaclesRangeBearing.append(result)

		return obstaclesRangeBearing


	def GetCameraImage(self):
		"""
		Gets the current camera image from the robot's vision sensor.
		
		Returns:
			tuple: (resolution, image_data) where:
				- resolution: [width, height] of the image
				- image_data: Image pixel data as a list
		"""
		if self.cameraHandle is None:
			return None, None
	
		try:
			detectionCount, packet1, packet2 = self.sim.handleVisionSensor(self.cameraHandle)
			image, resolution = self.sim.getVisionSensorImg(self.cameraHandle)
			if image is not None:
				image_data = self.sim.unpackUInt8Table(image)
				return resolution, image_data
			else:
				return None, None
		except Exception as e:
			print(f"Error getting camera image: {e}")
			return None, None
	
	def GetDetectedWallPoints(self):
		"""
		Legacy compatibility stub; wall-point estimation is not supported for the 2026 maze.

		The previous boundary-only implementation intersected the camera field-of-view with
		four fixed, axis-aligned arena boundaries. It does not account for generated internal
		maze walls or diagonal wall segments and therefore produced misleading readings.
		Use GetWallDistances() for the robot-relative proximity sensors instead. A future
		version may replace this stub with geometry-based or vision-based wall detection.

		Returns:
			None: Always, while maze wall-point detection is unsupported.
		"""
		return None

	def GetWallDistances(self):
		"""
		Read the three robot-relative proximity sensors.

		Returns:
			dict: ``{'left': distance, 'front': distance, 'right': distance}``, where
				each detected distance is measured in metres from that sensor's origin. A value
				of ``None`` means that the sensor detected nothing within its configured range,
				or that the optional sensor is not present in the scene.

		The direction names are relative to the robot, not the world. Proximity sensors can
		detect any scene object configured as detectable, so a reading is not guaranteed to
		come from a wall.
		"""
		distances = {}
		for direction in ('left', 'front', 'right'):
			handle = self.distanceSensorHandles.get(direction)
			if handle is None:
				distances[direction] = None
				continue

			try:
				detectionState, distance, _point, _objectHandle, _normal = self.sim.readProximitySensor(handle)
			except Exception:
				distances[direction] = None
				continue

			if detectionState == 1 and distance is not None and math.isfinite(distance) and distance >= 0.0:
				distances[direction] = float(distance)
			else:
				distances[direction] = None

		return distances
		

	def SetTargetVelocities(self, x_dot, theta_dot):
		"""
		Set the target velocities for the robot.
		
		Args:
			x_dot: Forward velocity in m/s
			theta_dot: Rotational velocity in rad/s
		"""
		if not (math.isfinite(x_dot) and math.isfinite(theta_dot)):
			print(f"Warning: ignoring non-finite velocity command (x_dot={x_dot}, theta_dot={theta_dot}); stopping instead.")
			x_dot, theta_dot = 0.0, 0.0

		if self.robotParameters.driveType == 'differential':
			# Robot physical parameters (fixed for the simulation)
			self.robotParameters.wheelBase = 0.15
			self.robotParameters.wheelRadius = 0.03

			# Calculate speed limits
			maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

			# Calculate individual wheel speeds
			leftWheelSpeed = (x_dot - 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.leftWheelBias
			rightWheelSpeed = (x_dot + 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.rightWheelBias

			# Add noise if drive system quality is not perfect
			if self.robotParameters.driveSystemQuality != 1:
				leftWheelSpeed = np.random.normal(leftWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)[0]
				rightWheelSpeed = np.random.normal(rightWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)[0]

			# Limit wheel speeds to +/- maxWheelSpeed (both directions - forward and reverse)
			leftWheelSpeed = max(min(leftWheelSpeed, maxWheelSpeed), -maxWheelSpeed)
			rightWheelSpeed = max(min(rightWheelSpeed, maxWheelSpeed), -maxWheelSpeed)

			if not (math.isfinite(leftWheelSpeed) and math.isfinite(rightWheelSpeed)):
				print("Warning: computed non-finite wheel speeds; stopping motors instead.")
				leftWheelSpeed, rightWheelSpeed = 0.0, 0.0

			# Set motor speeds
			try:
				self.sim.setJointTargetVelocity(self.leftMotorHandle, leftWheelSpeed)
				self.sim.setJointTargetVelocity(self.rightMotorHandle, rightWheelSpeed)
				if self.leftRearMotorHandle is not None:
					self.sim.setJointTargetVelocity(self.leftRearMotorHandle, leftWheelSpeed)
				if self.rightRearMotorHandle is not None:
					self.sim.setJointTargetVelocity(self.rightRearMotorHandle, rightWheelSpeed)
			except Exception as e:
				print(f"Error setting motor velocities: {e}")

		elif self.robotParameters.driveType == 'holonomic':
			print('Holonomic drive not yet implemented')

	def HasVictim(self):
		"""Return True when a victim is currently attached to VictimCarryPoint."""
		return self.carriedVictimHandle is not None

	def _get_horizontal_robot_to_victim_distance(self, victimHandle, robotCollectionHandle):
		"""Return the shortest horizontal clearance between the robot model and one victim."""
		result, distanceData, _objectPair = self.sim.checkDistance(
			robotCollectionHandle, victimHandle, 0.0)
		if result <= 0 or distanceData is None or len(distanceData) < 7:
			return None
		return math.hypot(
			distanceData[3] - distanceData[0],
			distanceData[4] - distanceData[1])

	def _attach_victim_to_carry_point(self, victimHandle):
		"""Attach a victim at the carry dummy and lay its long axis horizontally."""
		originalParent = self.sim.getObjectParent(victimHandle)
		originalPosition = self.sim.getObjectPosition(victimHandle, -1)
		originalOrientation = self.sim.getObjectOrientation(victimHandle, -1)

		try:
			# Generated victims are static/non-respondable already. Re-apply those properties
			# defensively so the carried visual does not disturb the robot's dynamics.
			for shapeHandle in self.sim.getObjectsInTree(victimHandle, self.sim.object_shape_type, 0):
				self.sim.setObjectInt32Param(shapeHandle, self.sim.shapeintparam_static, 1)
				self.sim.setObjectInt32Param(shapeHandle, self.sim.shapeintparam_respondable, 0)

			self.sim.setObjectParent(victimHandle, self.victimCarryPointHandle, True)
			self.sim.setObjectPosition(victimHandle, self.victimCarryPointHandle, [0.0, 0.0, 0.0])
			self.sim.setObjectOrientation(
				victimHandle, self.victimCarryPointHandle, list(VICTIM_CARRY_ORIENTATION))
			return True
		except Exception as e:
			# Restore the victim if any part of the attachment operation failed.
			try:
				self.sim.setObjectParent(victimHandle, originalParent, True)
				self.sim.setObjectPosition(victimHandle, -1, originalPosition)
				self.sim.setObjectOrientation(victimHandle, -1, originalOrientation)
			except Exception:
				pass
			print(f"Error attaching victim to VictimCarryPoint: {e}")
			return False

	def CollectVictim(self):
		"""
		Attempt to collect the closest generated victim within the permitted distance.

		The distance test is performed only when this function is called. It uses the
		shortest horizontal distance between any shape in the robot model and the victim,
		matching the assessment's 0.10 m collection rule. Only one victim can be carried.

		Returns:
			tuple: (success, victim_label, distance) where victim_label and distance are
			None when no victim was collected. Distance is in metres and is reported only
			after a successful collection, so this API cannot be used as a victim sensor.
		"""
		if self.HasVictim():
			print(f"Collection failed: already carrying victim {self.carriedVictimLabel}.")
			return False, None, None

		if self.victimCarryPointHandle is None:
			print("Collection failed: '/Robot/VictimCarryPoint' was not found in the robot model.")
			return False, None, None

		if not self.victimHandles:
			print("Collection failed: there are no generated victims in the scene.")
			return False, None, None

		robotCollectionHandle = None
		closestLabel = None
		closestHandle = None
		closestDistance = float('inf')
		try:
			# Option 1 makes the temporary collection override individual measurable flags.
			robotCollectionHandle = self.sim.createCollection(1)
			self.sim.addItemToCollection(
				robotCollectionHandle, self.sim.handle_tree, self.robotHandle, 0)

			for label, victimHandle in self.victimHandles.items():
				try:
					distance = self._get_horizontal_robot_to_victim_distance(
						victimHandle, robotCollectionHandle)
				except Exception:
					continue
				if distance is not None and distance < closestDistance:
					closestLabel = label
					closestHandle = victimHandle
					closestDistance = distance
		except Exception as e:
			print(f"Collection failed while measuring victim distance: {e}")
			return False, None, None
		finally:
			if robotCollectionHandle is not None:
				try:
					self.sim.destroyCollection(robotCollectionHandle)
				except Exception:
					pass

		maximumDistance = self.robotParameters.victimCollectionDistance
		if closestHandle is None or closestDistance > maximumDistance:
			print(f"Collection failed: no victim is within {maximumDistance:.3f} m.")
			return False, None, None

		if not self._attach_victim_to_carry_point(closestHandle):
			return False, None, None

		self.carriedVictimHandle = closestHandle
		self.carriedVictimLabel = closestLabel
		try:
			self.victimPositions[closestLabel] = self.sim.getObjectPosition(closestHandle, -1)
		except Exception:
			pass
		print(f"Collected victim {closestLabel} (distance: {closestDistance:.3f} m).")
		return True, closestLabel, closestDistance

	def ReleaseVictim(self):
		"""
		Place the currently carried victim back onto the maze floor.

		The victim is positioned slightly in front of the robot so it is not hidden below
		the chassis. Its original floor orientation is restored and its lowest transformed
		bounding-box corner is placed 1 mm above the floor.

		Returns:
			tuple: (success, victim_label), with victim_label set to None on failure.
		"""
		if not self.HasVictim():
			print("Release failed: the robot is not carrying a victim.")
			return False, None

		victimHandle = self.carriedVictimHandle
		victimLabel = self.carriedVictimLabel
		try:
			robotPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			robotOrientation = self.sim.getObjectOrientation(self.robotHandle, -1)
			robotYaw = robotOrientation[2]
			dropX = robotPosition[0] + VICTIM_RELEASE_FORWARD_OFFSET * math.cos(robotYaw)
			dropY = robotPosition[1] + VICTIM_RELEASE_FORWARD_OFFSET * math.sin(robotYaw)

			victimMinimumZ, _ = self._get_oriented_shape_z_extent(
				victimHandle, self.victimTemplateOrientation)
			dropZ = self.floorTopZ - victimMinimumZ + VICTIM_SURFACE_CLEARANCE

			parentHandle = self.generatedSceneRootHandle
			if parentHandle is None:
				parentHandle = -1
			self.sim.setObjectParent(victimHandle, parentHandle, True)
			self.sim.setObjectOrientation(victimHandle, -1, list(self.victimTemplateOrientation))
			self.sim.setObjectPosition(victimHandle, -1, [dropX, dropY, dropZ])

			self.victimPositions[victimLabel] = [dropX, dropY, dropZ]
			self.carriedVictimHandle = None
			self.carriedVictimLabel = None
			print(f"Released victim {victimLabel} onto the maze floor.")
			return True, victimLabel
		except Exception as e:
			print(f"Error releasing victim {victimLabel}: {e}")
			return False, None

	def UpdateObjectPositions(self):
		"""
		Updates the positions of all objects in the simulation.
		This should be called in every loop to get accurate object detection.
		
		Returns:
			tuple: (robotPose, obstaclePositions) for debugging purposes
		"""
		# Get current object positions from CoppeliaSim
		self.GetObjectPositions()
		
		return self.robotPose, self.obstaclePositions
	########################################
	##### INTERNAL HELPER FUNCTIONS #######
	########################################
	# These functions are used internally by the class
	
	def OpenConnectionToZMQ(self, coppelia_server_ip, port=23000):
		"""Connect to CoppeliaSim using ZMQ Remote API."""
		print('Connecting to CoppeliaSim...')
		try:
			print(f'Connecting to {coppelia_server_ip}:{port}...')
			self.client = RemoteAPIClient(host=coppelia_server_ip, port=port)
			self.sim = self.client.require('sim')
			print('Connected to CoppeliaSim successfully.')
			
			# Test the connection
			simulation_time = self.sim.getSimulationTime()
			print(f'Connection test successful. Simulation time: {simulation_time}')
			
			# Check simulation state
			sim_state = self.sim.getSimulationState()
			if sim_state == self.sim.simulation_stopped:
				print('Simulation is currently stopped.')
			elif sim_state == self.sim.simulation_paused:
				print('Warning: Simulation is paused.')
			elif sim_state == self.sim.simulation_advancing:
				print('Simulation is running.')
			
		except Exception as e:
			print(f'Failed to connect to CoppeliaSim: {e}')
			print('Make sure CoppeliaSim is running with the correct scene loaded.')
			print('\nTroubleshooting steps:')
			print('1. Restart CoppeliaSim')
			print('2. Load your scene file')
			print('3. Check that ZMQ Remote API is enabled')
			sys.exit(-1)

	def _try_get_object(self, path):
		"""Attempt to resolve a scene object path without raising. Returns the handle, or None if not found."""
		try:
			handle = self.sim.getObject(path, {'noError': True})
		except TypeError:
			# Installed API version may not support the {'noError': True} options argument
			try:
				handle = self.sim.getObject(path)
			except Exception:
				return None
		except Exception:
			return None
		if handle is None or handle == -1:
			return None
		return handle

	def _resolve_first_available(self, candidatePaths, label, required=True, verbose=True):
		"""
		Try a list of candidate object paths and return the handle of the first one that resolves.
		Logs which path was used, so differences between scene layouts are easy to diagnose.
		Missing optional objects are recorded in self.missingOptionalObjects for the startup summary.
		"""
		for path in candidatePaths:
			handle = self._try_get_object(path)
			if handle is not None:
				if verbose:
					print(f"Resolved {label} -> '{path}' (handle {handle})")
				return handle
		if required:
			if verbose:
				print(f"Error: could not resolve required object '{label}'. Tried: {candidatePaths}")
		else:
			if verbose:
				print(f"Note: optional object '{label}' not found. Tried: {candidatePaths}")
			self.missingOptionalObjects.append(label)
		return None

	def GetCOPPELIAObjectHandles(self):
		"""
		Resolve object handles needed for the search and rescue maze scene.

		Only the robot, drive motors, floor, table boundary walls, and the maze/post/victim
		templates are required in this phase - startup aborts if any of these are missing.
		Robot sensors (vision sensor, object detector, proximity/distance sensors and rear
		motors) are optional and resolved best-effort. Optional obstacles never abort startup.
		"""
		# --- Essential: robot + drive motors ---
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			print('Failed to get Robot object handle.')
			sys.exit(-1)

		errorCode = self.GetScriptHandle()
		if errorCode != 0:
			print('Failed to get Script handle.')
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3, errorCode4 = self.GetMotorHandles()
		if errorCode1 != 0 or errorCode2 != 0:
			print('Failed to get drive Motor handles.')
			sys.exit(-1)
		elif errorCode3 != 0 or errorCode4 != 0:
			print("Note: rear wheel motors not found (fine for two-wheel differential robots).")

		# --- Essential: search and rescue maze scene objects ---
		self.floorHandle = self._resolve_first_available(['/floor'], 'floor', required=True)
		if self.floorHandle is None:
			sys.exit(-1)

		self.tableWallHandles = []
		for i in range(4):
			handle = self._resolve_first_available([f'/table_wall[{i}]'], f'table_wall[{i}]', required=True)
			if handle is None:
				sys.exit(-1)
			self.tableWallHandles.append(handle)

		self.mazeWallTemplateHandle = self._resolve_first_available(['/maze_wall'], 'maze_wall template', required=True)
		self.baseStationWallTemplateHandle = self._resolve_first_available(
			['/base_station_wall'], 'base_station_wall template', required=True)
		self.victimWallTemplateHandle = self._resolve_first_available(
			['/victim_wall'], 'victim_wall template', required=True)
		self.rubbleVictimWallTemplateHandle = self._resolve_first_available(
			['/rubble_victim_wall'], 'rubble_victim_wall template', required=True)
		self.hazardWallTemplateHandle = self._resolve_first_available(
			['/hazard_wall'], 'hazard_wall template', required=True)
		self.wallPostTemplateHandle = self._resolve_first_available(['/wall_post'], 'wall_post template', required=True)
		self.victimTemplateHandle = self._resolve_first_available(['/victim'], 'victim template', required=True)
		wallTemplateHandles = (
			self.mazeWallTemplateHandle,
			self.baseStationWallTemplateHandle,
			self.victimWallTemplateHandle,
			self.rubbleVictimWallTemplateHandle,
			self.hazardWallTemplateHandle,
		)
		if any(handle is None for handle in wallTemplateHandles) or self.wallPostTemplateHandle is None or self.victimTemplateHandle is None:
			sys.exit(-1)

		# --- Search-and-rescue collection point and optional robot sensors ---
		self.GetVictimCarryPointHandle()
		self.GetCameraHandle()
		self.GetObjectDetectorHandle()
		self.getProximityhandle()
		self.GetDistanceSensorHandles()

		# Optional maze obstacles are best-effort and never abort startup.
		self.GetObstacleHandles()

	############################################
	####### COPPELIA OBJECT HANDLE FUNCTIONS #######
	############################################
	# These functions are called by the GetCOPPELIAObjectHandles function

	# Get COPPELIA Robot Handle
	def GetRobotHandle(self):
		handle = self._try_get_object('/Robot')
		if handle is None:
			print("Error getting robot handle: '/Robot' not found")
			return -1
		self.robotHandle = handle
		return 0

	# Get Script Handle (attached to Robot object)
	def GetScriptHandle(self):
		# The robot handle doubles as the script handle for callScriptFunction calls
		self.scriptHandle = self.robotHandle
		return 0 if self.scriptHandle is not None else -1

	# Get ZMQ Camera Handle (the robot's onboard vision sensor - NOT the external /camera overview camera)
	def GetCameraHandle(self):
		self.cameraHandle = self._resolve_first_available(
			['/Robot/VisionSensor', '/VisionSensor'], 'robot VisionSensor', required=False)
		return 0 if self.cameraHandle is not None else -1

	# Get ZMQ Object Detector Handle
	def GetObjectDetectorHandle(self):
		self.objectDetectorHandle = self._resolve_first_available(
			['/Robot/VisionSensor/ObjectDetector', '/Robot/ObjectDetector'], 'ObjectDetector', required=False)
		return 0 if self.objectDetectorHandle is not None else -1

	def GetVictimCarryPointHandle(self):
		"""Resolve the scene-authored dummy that defines the carried victim pose."""
		self.victimCarryPointHandle = self._resolve_first_available(
			['/Robot/VictimCarryPoint'], 'VictimCarryPoint', required=False)
		return 0 if self.victimCarryPointHandle is not None else -1

			
	# Get COPPELIA Motor Handles
	# Get ZMQ Motor Handles
	def GetMotorHandles(self):
		"""Resolve drive motor handles. Left/right motors are required; rear motors are optional (4-wheel robots only)."""
		errorCode1 = 0
		errorCode2 = 0
		errorCode3 = 0
		errorCode4 = 0

		if self.robotParameters.driveType == 'differential':
			self.leftMotorHandle = self._try_get_object('/LeftMotor')
			if self.leftMotorHandle is None:
				print("Error getting left motor handle")
				errorCode1 = -1

			self.rightMotorHandle = self._try_get_object('/RightMotor')
			if self.rightMotorHandle is None:
				print("Error getting right motor handle")
				errorCode2 = -1

			self.leftRearMotorHandle = self._try_get_object('/LeftRearMotor')
			if self.leftRearMotorHandle is None:
				errorCode3 = -1

			self.rightRearMotorHandle = self._try_get_object('/RightRearMotor')
			if self.rightRearMotorHandle is None:
				errorCode4 = -1

		return errorCode1, errorCode2, errorCode3, errorCode4

	# Get optional maze obstacle handles.
	def GetObstacleHandles(self):
		"""Best-effort resolution of optional obstacle handles."""
		error_codes = [0, 0, 0]
		missing = []
		for index in range(3):
			path = f'/Obstacle_{index}'
			handle = self._try_get_object(path)
			self.obstacleHandles[index] = handle
			if handle is None:
				error_codes[index] = -1
				missing.append(path)
		if missing:
			print(f"Optional obstacles not found: {', '.join(missing)}")
		return tuple(error_codes)

	# Get ZMQ proximity sensor handle.
	def getProximityhandle(self):
		self.proximityHandle = self._resolve_first_available(
			['/Robot/VisionSensor/Proximity_sensor', '/Proximity_sensor'], 'Proximity_sensor', required=False)
		return 0 if self.proximityHandle is not None else -1

	def GetDistanceSensorHandles(self):
		"""Resolve the optional robot-relative left, front and right proximity sensors."""
		for direction, suffix in (('left', 'Left'), ('front', 'Front'), ('right', 'Right')):
			alias = f'DistanceSensor{suffix}'
			self.distanceSensorHandles[direction] = self._resolve_first_available(
				[
					f'/Robot/DistanceSensors/{alias}',
					f'/Robot/{alias}',
					f'/DistanceSensors/{alias}',
					f'/{alias}',
				],
				alias,
				required=False)

		return 0 if all(handle is not None for handle in self.distanceSensorHandles.values()) else -1

	###############################################
	####### ROBOT AND SCENE SETUP FUNCTIONS #######
	###############################################
	# These functions are called within the init function

	# Updates the robot within COPPELIA based on the robot paramters
	def UpdateCOPPELIARobot(self):
		# Set Camera Pose and Orientation (skipped if no VisionSensor was resolved)
		if self.cameraHandle is None:
			print("Note: skipping camera pose/orientation setup - no VisionSensor resolved.")
			return
		self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
		self.SetCameraOrientation(self.robotParameters.cameraOrientation)

	def SetScene(self):
		"""
		Builds the static EGB320 search and rescue maze scene.

		Validates the maze configuration, clears any previously generated maze objects,
		then generates the wall posts, internal/perimeter walls and victims, and (optionally) places
		the robot at its starting cell. Must be called while the simulation is stopped, since
		it moves/scales/copies objects (this is done for us by StartSimulator).
		"""
		print('Preparing search and rescue maze scene...')

		if not self.sceneParameters.autoGenerateMaze:
			print('autoGenerateMaze is False - skipping maze generation.')
			return

		try:
			self.sceneParameters.validate_maze_parameters()
		except ValueError as e:
			print(f"Error: invalid maze configuration: {e}")
			sys.exit(-1)

		if self.sceneParameters.clearGeneratedMaze:
			removedCount = self._clear_generated_maze()
		else:
			removedCount = 0
			self._ensure_generated_scene_root()

		self._get_floor_info()
		self._cache_template_geometry()

		if self.sceneParameters.generateMazeObjects:
			postCount = self._generate_wall_posts()
			wallCount = self._generate_internal_walls()
			victimCount = self._generate_victims()
		else:
			print('generateMazeObjects is False - leaving the table clear for diagnostics.')
			postCount = 0
			wallCount = 0
			victimCount = 0

		self._park_templates_outside_playable_area()

		if self.sceneParameters.robotStartingPosition is not None:
			x, y, theta = self.sceneParameters.robotStartingPosition
			self._set_robot_pose([x, y, theta])
		elif self.sceneParameters.placeRobotAtBase:
			self._place_robot_at_base()

		self._log_table_wall_positions()
		self._print_scene_summary(postCount, wallCount, victimCount, removedCount)

		self._set_obstacle_positions()

	########################################
	##### SEARCH AND RESCUE MAZE GENERATION #####
	########################################
	# Helper methods used by SetScene to build the maze. Students typically don't need to
	# modify these functions.

	def _get_shape_bbox_size(self, handle):
		"""
		Return the (sizeX, sizeY, sizeZ) bounding box dimensions of a shape, in its own
		reference frame. Raises ValueError if the object is not a shape.
		"""
		try:
			objectType = self.sim.getObjectType(handle)
		except Exception as e:
			raise ValueError(f"Could not determine object type for handle {handle}: {e}")

		if objectType != self.sim.object_shape_type:
			raise ValueError(f"Expected object handle {handle} to be a shape, but it is not (type={objectType}).")

		try:
			# sim.getShapeBB returns (sizeXYZ, pose7) - a 3-element size list plus the
			# shape's local reference frame pose; we only need the size here.
			boundingBox, _ = self.sim.getShapeBB(handle)
		except Exception as e:
			raise ValueError(f"Could not read bounding box for shape handle {handle}: {e}")

		return boundingBox[0], boundingBox[1], boundingBox[2]

	@staticmethod
	def _transform_point(matrix, point):
		"""Transform a 3D point with a CoppeliaSim 3x4 transformation matrix."""
		return [
			matrix[0] * point[0] + matrix[1] * point[1] + matrix[2] * point[2] + matrix[3],
			matrix[4] * point[0] + matrix[5] * point[1] + matrix[6] * point[2] + matrix[7],
			matrix[8] * point[0] + matrix[9] * point[1] + matrix[10] * point[2] + matrix[11],
		]

	def _get_shape_bbox_points(self, handle):
		"""Return all eight bounding-box corners in the shape's object reference frame."""
		size, boundingBoxPose = self.sim.getShapeBB(handle)
		boundingBoxMatrix = self.sim.poseToMatrix(boundingBoxPose)
		halfSize = [dimension / 2.0 for dimension in size]
		return [
			self._transform_point(boundingBoxMatrix, [sx * halfSize[0], sy * halfSize[1], sz * halfSize[2]])
			for sx in (-1.0, 1.0)
			for sy in (-1.0, 1.0)
			for sz in (-1.0, 1.0)
		]

	def _get_shape_z_extent_for_matrix(self, handle, shapeMatrix):
		"""Return the minimum/maximum Z of a shape bounding box after a transform."""
		zValues = [
			self._transform_point(shapeMatrix, point)[2]
			for point in self._get_shape_bbox_points(handle)
		]
		return min(zValues), max(zValues)

	def _get_oriented_shape_z_extent(self, handle, orientation):
		"""Return a shape's Z extent about its origin for a requested Euler orientation."""
		orientationMatrix = self.sim.buildMatrix([0.0, 0.0, 0.0], list(orientation))
		return self._get_shape_z_extent_for_matrix(handle, orientationMatrix)

	def _get_floor_info(self):
		"""
		Determine the floor centre (used as the default maze origin) and the floor's top
		surface Z, and compute the maze's world-coordinate bounds. Also verifies the floor
		is aligned with the world X/Y axes, since the grid maths below assumes this.
		"""
		floorPosition = self.sim.getObjectPosition(self.floorHandle, -1)
		floorOrientation = self.sim.getObjectOrientation(self.floorHandle, -1)

		orientationTolerance = 0.01  # radians (~0.57 degrees)
		if abs(floorOrientation[0]) > orientationTolerance or abs(floorOrientation[1]) > orientationTolerance:
			floorErrorMessage = (
				f"Error: /floor is not aligned with the world X/Y plane (roll={floorOrientation[0]:.4f}, "
				f"pitch={floorOrientation[1]:.4f} rad). Maze generation assumes an axis-aligned floor."
			)
			print(floorErrorMessage)
			sys.exit(-1)
		if abs(self.WrapToPi(floorOrientation[2])) > orientationTolerance:
			floorYawWarning = (
				f"Warning: /floor yaw is {floorOrientation[2]:.4f} rad - the maze grid follows the floor's "
				f"local X/Y axes, so this should be a multiple of pi/2 for the grid to look axis-aligned."
			)
			print(floorYawWarning)

		floorSizeX, floorSizeY, floorSizeZ = self._get_shape_bbox_size(self.floorHandle)

		self.floorCenter = (floorPosition[0], floorPosition[1])
		self.floorTopZ = floorPosition[2] + floorSizeZ / 2.0

		if self.sceneParameters.mazeOriginXY is not None:
			originX, originY = self.sceneParameters.mazeOriginXY
		else:
			originX, originY = self.floorCenter

		mazeWidth = self.sceneParameters.mazeColumns * self.sceneParameters.mazeCellSize
		mazeHeight = self.sceneParameters.mazeRows * self.sceneParameters.mazeCellSize

		self.mazeXMinimum = originX - mazeWidth / 2.0
		self.mazeYMaximum = originY + mazeHeight / 2.0

		print(f"Floor centre: ({self.floorCenter[0]:.4f}, {self.floorCenter[1]:.4f}), floor top Z: {self.floorTopZ:.4f} m")
		print(f"Maze origin: ({originX:.4f}, {originY:.4f}), footprint: {mazeWidth:.3f} m x {mazeHeight:.3f} m")

	def _build_rotation_matrix(self, eulerOrientation):
		"""
		Build the 3x3 rotation matrix for a CoppeliaSim Euler orientation (alpha, beta, gamma),
		using CoppeliaSim's Rx(alpha)*Ry(beta)*Rz(gamma) convention. Used to detect which local
		axis of a template ends up vertical and which world direction another axis points at,
		since templates aren't always authored with "up"/"forward" along local Z/X (e.g. a wall
		modelled lying flat then rotated upright with a fixed roll).
		"""
		alpha, beta, gamma = eulerOrientation
		cx, sx = math.cos(alpha), math.sin(alpha)
		cy, sy = math.cos(beta), math.sin(beta)
		cz, sz = math.cos(gamma), math.sin(gamma)

		def matmul3(a, b):
			return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

		Rx = [[1, 0, 0], [0, cx, -sx], [0, sx, cx]]
		Ry = [[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]]
		Rz = [[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]]
		return matmul3(matmul3(Rx, Ry), Rz)

	def _get_wall_template_geometry(self, handle, name):
		"""Return axis/orientation metadata needed to place and face a wall template."""
		size = self._get_shape_bbox_size(handle)
		orientation = self.sim.getObjectOrientation(handle, -1)
		rotation = self._build_rotation_matrix(orientation)

		worldZComponents = [abs(rotation[2][axis]) for axis in range(3)]
		heightAxisIndex = worldZComponents.index(max(worldZComponents))
		horizontalAxes = [axis for axis in range(3) if axis != heightAxisIndex]
		lengthAxisIndex = max(horizontalAxes, key=lambda axis: size[axis])
		thicknessAxisIndex = next(axis for axis in horizontalAxes if axis != lengthAxisIndex)

		geometry = {
			'name': name,
			'handle': handle,
			'size': size,
			'orientation': orientation,
			'heightAxisIndex': heightAxisIndex,
			'lengthAxisIndex': lengthAxisIndex,
			'thicknessAxisIndex': thicknessAxisIndex,
			'worldHeight': size[heightAxisIndex],
			'lengthAxisBaseYaw': math.atan2(rotation[1][lengthAxisIndex], rotation[0][lengthAxisIndex]),
			# The decal textures are applied to the wall's thin local axis. Treat the local
			# positive-thickness face as the marker face and rotate it toward the associated cell.
			'markerFaceBaseYaw': math.atan2(rotation[1][thicknessAxisIndex], rotation[0][thicknessAxisIndex]),
		}
		return geometry

	def _cache_template_geometry(self):
		"""Cache wall, post and victim template geometry before templates are moved."""
		wallTemplates = {
			'maze_wall': self.mazeWallTemplateHandle,
			'base_station_wall': self.baseStationWallTemplateHandle,
			'victim_wall': self.victimWallTemplateHandle,
			'rubble_victim_wall': self.rubbleVictimWallTemplateHandle,
			'hazard_wall': self.hazardWallTemplateHandle,
		}
		self.wallTemplateGeometry = {
			handle: self._get_wall_template_geometry(handle, name)
			for name, handle in wallTemplates.items()
		}

		mazeWallGeometry = self.wallTemplateGeometry[self.mazeWallTemplateHandle]
		self.mazeWallTemplateSize = mazeWallGeometry['size']
		self.mazeWallTemplateOrientation = mazeWallGeometry['orientation']
		self.mazeWallHeightAxisIndex = mazeWallGeometry['heightAxisIndex']
		self.mazeWallLengthAxisIndex = mazeWallGeometry['lengthAxisIndex']
		self.mazeWallWorldHeight = mazeWallGeometry['worldHeight']
		self.mazeWallLengthAxisBaseYaw = mazeWallGeometry['lengthAxisBaseYaw']

		self.wallPostTemplateSize = self._get_shape_bbox_size(self.wallPostTemplateHandle)
		self.wallPostTemplateOrientation = self.sim.getObjectOrientation(self.wallPostTemplateHandle, -1)
		try:
			self.victimTemplateSize = self._get_shape_bbox_size(self.victimTemplateHandle)
		except ValueError as e:
			print(f"Warning: victim template is not a simple shape ({e}); using zero height for placement.")
			self.victimTemplateSize = (0.0, 0.0, 0.0)
		self.victimTemplateOrientation = self.sim.getObjectOrientation(self.victimTemplateHandle, -1)

		axisNames = ['X', 'Y', 'Z']
		for geometry in self.wallTemplateGeometry.values():
			print(
				f"{geometry['name']} template size: {geometry['size']}, "
				f"height axis: local {axisNames[geometry['heightAxisIndex']]}, "
				f"length axis: local {axisNames[geometry['lengthAxisIndex']]}")
		print(f"wall_post template size (x,y,z): {self.wallPostTemplateSize}")
		print(f"victim template size (x,y,z): {self.victimTemplateSize}")

	def _grid_point_to_world(self, column, row):
		"""Convert a maze grid intersection (column, row) into world (x, y) coordinates."""
		cellSize = self.sceneParameters.mazeCellSize
		x = self.mazeXMinimum + column * cellSize
		y = self.mazeYMaximum - row * cellSize
		return x, y

	def _cell_center_to_world(self, column, row):
		"""Convert a maze cell (column, row) into the world (x, y) coordinates of its centre."""
		cellSize = self.sceneParameters.mazeCellSize
		x = self.mazeXMinimum + (column + 0.5) * cellSize
		y = self.mazeYMaximum - (row + 0.5) * cellSize
		return x, y

	def _get_perimeter_wall_segments(self):
		"""Return one wall segment for every cell edge around the complete maze border."""
		columns = self.sceneParameters.mazeColumns
		rows = self.sceneParameters.mazeRows
		segments = []

		# North and south borders.
		for column in range(columns):
			segments.append(((column, 0), (column + 1, 0)))
			segments.append(((column, rows), (column + 1, rows)))

		# West and east borders.
		for row in range(rows):
			segments.append(((0, row), (0, row + 1)))
			segments.append(((columns, row), (columns, row + 1)))

		return segments

	@staticmethod
	def _normalise_grid_segment(startPoint, endPoint):
		"""Return a direction-independent key for a wall segment."""
		return tuple(sorted((tuple(startPoint), tuple(endPoint))))

	def _cell_side_segment(self, cell, side):
		"""Return the grid segment forming one cardinal side of a cell."""
		column, row = cell
		segments = {
			'N': ((column, row), (column + 1, row)),
			'E': ((column + 1, row), (column + 1, row + 1)),
			'S': ((column, row + 1), (column + 1, row + 1)),
			'W': ((column, row), (column, row + 1)),
		}
		return segments[side]

	def _cell_wall_sides(self, cell):
		"""Return which N/E/S/W sides of a cell are closed by a wall or table boundary."""
		column, row = cell
		axisAlignedWalls = {
			self._normalise_grid_segment(startPoint, endPoint)
			for startPoint, endPoint in EXAMPLE_MAZE_SEGMENTS
			if startPoint[0] == endPoint[0] or startPoint[1] == endPoint[1]
		}
		boundarySides = {
			'N': row == 0,
			'E': column == self.sceneParameters.mazeColumns - 1,
			'S': row == self.sceneParameters.mazeRows - 1,
			'W': column == 0,
		}
		return {
			side: boundarySides[side] or self._normalise_grid_segment(*self._cell_side_segment(cell, side)) in axisAlignedWalls
			for side in ('N', 'E', 'S', 'W')
		}

	def _shortest_path_entry_directions(self):
		"""Return the movement direction used to first enter each cell from the base cell."""
		directionSteps = {
			'N': (0, -1),
			'E': (1, 0),
			'S': (0, 1),
			'W': (-1, 0),
		}
		baseCell = tuple(self.sceneParameters.baseCell)
		queue = deque([baseCell])
		visited = {baseCell}
		entryDirections = {}

		while queue:
			cell = queue.popleft()
			wallSides = self._cell_wall_sides(cell)
			for direction, (columnStep, rowStep) in directionSteps.items():
				if wallSides[direction]:
					continue
				neighbour = (cell[0] + columnStep, cell[1] + rowStep)
				if neighbour in visited:
					continue
				visited.add(neighbour)
				entryDirections[neighbour] = direction
				queue.append(neighbour)

		return entryDirections

	def _plan_marker_walls(self):
		"""
		Choose marker-bearing wall segments for the base, victims and empty dead ends.

		Internal marker walls replace the ordinary wall on the same segment. Marker walls on
		the outer maze boundary are added as one-cell liners immediately inside the table wall.
		"""
		oppositeDirection = {'N': 'S', 'E': 'W', 'S': 'N', 'W': 'E'}
		assignments = {}

		def assign(kind, cell, side, templateHandle, alias, label=None):
			wallSides = self._cell_wall_sides(cell)
			if not wallSides[side]:
				raise ValueError(f"Cannot place {kind} marker at cell {cell}: side {side} is open")
			startPoint, endPoint = self._cell_side_segment(cell, side)
			segmentKey = self._normalise_grid_segment(startPoint, endPoint)
			if segmentKey in assignments:
				raise ValueError(
					f"Marker wall conflict on segment {segmentKey}: "
					f"{assignments[segmentKey]['kind']} and {kind}")
			assignments[segmentKey] = {
				'kind': kind,
				'cell': tuple(cell),
				'side': side,
				'templateHandle': templateHandle,
				'alias': alias,
				'label': label,
				'startPoint': startPoint,
				'endPoint': endPoint,
			}

		# The base is a three-sided corner cell in the approved maze. Put its marker on the
		# back wall opposite the only opening so it faces the normal return approach.
		baseCell = tuple(self.sceneParameters.baseCell)
		baseWalls = self._cell_wall_sides(baseCell)
		baseOpenSides = [side for side, closed in baseWalls.items() if not closed]
		if len(baseOpenSides) == 1:
			baseMarkerSide = oppositeDirection[baseOpenSides[0]]
		else:
			baseBoundarySides = [
				side for side in ('S', 'W', 'N', 'E')
				if baseWalls[side] and (
					(side == 'N' and baseCell[1] == 0) or
					(side == 'E' and baseCell[0] == self.sceneParameters.mazeColumns - 1) or
					(side == 'S' and baseCell[1] == self.sceneParameters.mazeRows - 1) or
					(side == 'W' and baseCell[0] == 0))
			]
			if not baseBoundarySides:
				raise ValueError(f"Base cell {baseCell} has no suitable wall for the base marker")
			baseMarkerSide = baseBoundarySides[0]
		assign('base', baseCell, baseMarkerSide, self.baseStationWallTemplateHandle, 'EGB320_GEN_BASE_STATION_WALL')

		# Prefer a wall directly ahead on the shortest route from base. If that side is open
		# (e.g. a victim in a through corridor), use a deterministic existing side wall.
		entryDirections = self._shortest_path_entry_directions()
		victimCells = {tuple(cell) for cell in self.sceneParameters.victimCells.values()}
		for label, configuredCell in self.sceneParameters.victimCells.items():
			cell = tuple(configuredCell)
			wallSides = self._cell_wall_sides(cell)
			approachDirection = entryDirections.get(cell)
			candidateSides = []
			for side in (approachDirection, 'E', 'W', 'S', 'N'):
				if side is not None and side not in candidateSides:
					candidateSides.append(side)
			markerSide = next((side for side in candidateSides if wallSides[side]), None)
			if markerSide is None:
				raise ValueError(f"Victim '{label}' at cell {cell} has no wall for its marker")

			isLevel3 = str(label).upper() == 'L3'
			templateHandle = self.rubbleVictimWallTemplateHandle if isLevel3 else self.victimWallTemplateHandle
			kind = 'rubble_victim' if isLevel3 else 'victim'
			safeLabel = ''.join(character if character.isalnum() else '_' for character in str(label).upper())
			assign(kind, cell, markerSide, templateHandle, f'EGB320_GEN_{kind.upper()}_WALL_{safeLabel}', label=str(label))

		# Every remaining three-sided cell is a hazard dead end. Its marker goes on the back
		# wall opposite the opening, and the base/victim cells are explicitly excluded.
		for row in range(self.sceneParameters.mazeRows):
			for column in range(self.sceneParameters.mazeColumns):
				cell = (column, row)
				if cell == baseCell or cell in victimCells:
					continue
				wallSides = self._cell_wall_sides(cell)
				openSides = [side for side, closed in wallSides.items() if not closed]
				if len(openSides) != 1:
					continue
				hazardSide = oppositeDirection[openSides[0]]
				assign(
					'hazard', cell, hazardSide, self.hazardWallTemplateHandle,
					f'EGB320_GEN_HAZARD_WALL_C{column}_R{row}')

		return assignments

	def _generate_wall_posts(self):
		"""Create one /wall_post copy at every grid intersection: (mazeRows+1) x (mazeColumns+1) posts."""
		postHeight = self.wallPostTemplateSize[2]
		count = 0
		for row in range(self.sceneParameters.mazeRows + 1):
			for column in range(self.sceneParameters.mazeColumns + 1):
				x, y = self._grid_point_to_world(column, row)
				newHandle = self.sim.copyPasteObjects([self.wallPostTemplateHandle], 0)[0]
				zPos = self.floorTopZ + postHeight / 2.0
				self.sim.setObjectPosition(newHandle, -1, [x, y, zPos])
				self.sim.setObjectOrientation(newHandle, -1, list(self.wallPostTemplateOrientation))
				self.sim.setObjectAlias(newHandle, f"EGB320_GEN_POST_R{row}_C{column}")
				self.sim.setObjectParent(newHandle, self.generatedSceneRootHandle, True)
				self.generatedWallPostHandles.append(newHandle)
				count += 1
		return count

	def _copy_and_scale_wall_template(self, templateHandle, geometry, scaleFactor, copyWholeModel=False):
		"""Copy a wall template and scale every copied shape along the wall-length direction."""
		copyOptions = 1 if copyWholeModel else 0
		try:
			newHandle = self.sim.copyPasteObjects([templateHandle], copyOptions)[0]
		except Exception as e:
			if copyWholeModel:
				raise RuntimeError(
					f"Could not copy the complete {geometry['name']} model. Ensure its root has "
					"'Object is model' enabled and the marker is parented beneath it."
				) from e
			raise

		if copyWholeModel:
			# Whole-model copying preserves the marker child. Scale every shape in the copied
			# hierarchy so the wall and decal remain the same width. Determine each shape's
			# local length axis from its world orientation, which also supports a flipped decal.
			shapeHandles = self.sim.getObjectsInTree(newHandle, self.sim.sceneobject_shape, 0)
			if len(shapeHandles) < 2:
				try:
					self.sim.removeModel(newHandle)
				except Exception:
					self.sim.removeObject(newHandle)
				raise RuntimeError(
					f"The copied {geometry['name']} model does not contain a marker child shape."
				)

			rootRotation = self._build_rotation_matrix(geometry['orientation'])
			lengthDirection = [
				rootRotation[row][geometry['lengthAxisIndex']]
				for row in range(3)
			]
		else:
			shapeHandles = [newHandle]
			lengthDirection = None

		for shapeHandle in shapeHandles:
			if lengthDirection is None:
				lengthAxisIndex = geometry['lengthAxisIndex']
			else:
				shapeOrientation = self.sim.getObjectOrientation(shapeHandle, -1)
				shapeRotation = self._build_rotation_matrix(shapeOrientation)
				axisAlignment = [
					abs(sum(shapeRotation[row][axis] * lengthDirection[row] for row in range(3)))
					for axis in range(3)
				]
				lengthAxisIndex = axisAlignment.index(max(axisAlignment))

			scaleXYZ = [1.0, 1.0, 1.0]
			scaleXYZ[lengthAxisIndex] = scaleFactor
			self.sim.scaleObject(shapeHandle, scaleXYZ[0], scaleXYZ[1], scaleXYZ[2], 0)

		return newHandle

	def _create_wall_between_grid_points(self, startPoint, endPoint, index, templateHandle=None, alias=None, markerCell=None):
		"""
		Create a wall-template copy spanning the given grid intersections.
		Scales only the template's long axis to match the segment length, and orients the
		wall so that axis lies along the segment. For marker walls, markerCell identifies
		the associated cell and the decal face is rotated inward toward that cell.
		"""
		if templateHandle is None:
			templateHandle = self.mazeWallTemplateHandle
		geometry = self.wallTemplateGeometry[templateHandle]

		x1, y1 = self._grid_point_to_world(*startPoint)
		x2, y2 = self._grid_point_to_world(*endPoint)

		dx = x2 - x1
		dy = y2 - y1
		length = math.hypot(dx, dy)
		if length < 1e-9:
			raise ValueError(f"Wall segment {index} has zero length: {startPoint} -> {endPoint}")

		segmentYaw = math.atan2(dy, dx)
		midX = (x1 + x2) / 2.0
		midY = (y1 + y2) / 2.0
		zPos = self.floorTopZ + geometry['worldHeight'] / 2.0

		scaleFactor = length / geometry['size'][geometry['lengthAxisIndex']]
		newHandle = self._copy_and_scale_wall_template(
			templateHandle,
			geometry,
			scaleFactor,
			copyWholeModel=markerCell is not None)

		# Compose the extra yaw needed with the template's own natural tilt via a temporary
		# reference dummy, rather than hand-rolling Euler/matrix composition: setting an
		# object's position/orientation "relative to" another handle works regardless of
		# actual parent-child hierarchy, so no reparenting is required.
		deltaYaw = segmentYaw - geometry['lengthAxisBaseYaw']
		if markerCell is not None:
			cellX, cellY = self._cell_center_to_world(*markerCell)
			desiredFaceYaw = math.atan2(cellY - midY, cellX - midX)
			placedFaceYaw = geometry['markerFaceBaseYaw'] + deltaYaw
			if math.cos(placedFaceYaw - desiredFaceYaw) < 0:
				deltaYaw += math.pi

		helperDummy = self.sim.createDummy(0.01)
		self.sim.setObjectPosition(helperDummy, -1, [midX, midY, zPos])
		self.sim.setObjectOrientation(helperDummy, -1, [0, 0, deltaYaw])
		self.sim.setObjectPosition(newHandle, helperDummy, [0, 0, 0])
		self.sim.setObjectOrientation(newHandle, helperDummy, list(geometry['orientation']))
		self.sim.removeObject(helperDummy)

		if alias is None:
			alias = f"EGB320_GEN_WALL_{index:03d}"
		self.sim.setObjectAlias(newHandle, alias)
		self.sim.setObjectParent(newHandle, self.generatedSceneRootHandle, True)

		self.generatedMazeWallHandles.append(newHandle)
		if markerCell is not None:
			self.generatedMarkerWallHandles.append(newHandle)
		return newHandle

	def _generate_internal_walls(self):
		"""Create all internal/perimeter walls, substituting marker walls where required."""
		markerAssignments = self._plan_marker_walls()
		self.markerWallPlacements = []
		wallSegments = list(EXAMPLE_MAZE_SEGMENTS)
		seenSegments = {
			self._normalise_grid_segment(startPoint, endPoint)
			for startPoint, endPoint in wallSegments
		}
		for startPoint, endPoint in self._get_perimeter_wall_segments():
			segmentKey = self._normalise_grid_segment(startPoint, endPoint)
			if segmentKey not in seenSegments:
				wallSegments.append((startPoint, endPoint))
				seenSegments.add(segmentKey)

		count = 0
		for index, (startPoint, endPoint) in enumerate(wallSegments):
			segmentKey = self._normalise_grid_segment(startPoint, endPoint)
			marker = markerAssignments.pop(segmentKey, None)
			if marker is None:
				self._create_wall_between_grid_points(startPoint, endPoint, index)
			else:
				self._create_wall_between_grid_points(
					startPoint,
					endPoint,
					index,
					templateHandle=marker['templateHandle'],
					alias=marker['alias'],
					markerCell=marker['cell'])
				self.markerWallPlacements.append(marker)
			count += 1

		# Defensive fallback for any marker assigned to an unexpected wall segment that is not
		# present in the configured internal or generated perimeter wall lists.
		for marker in markerAssignments.values():
			self._create_wall_between_grid_points(
				marker['startPoint'],
				marker['endPoint'],
				count,
				templateHandle=marker['templateHandle'],
				alias=marker['alias'],
				markerCell=marker['cell'])
			self.markerWallPlacements.append(marker)
			count += 1

		for marker in self.markerWallPlacements:
			print(
				f"Marker wall: {marker['kind']} at cell {marker['cell']} "
				f"on side {marker['side']} ({marker['alias']})")
		return count

	def _generate_victims(self):
		"""Create one /victim copy at each configured victim cell centre."""
		victimMinimumZ, _ = self._get_oriented_shape_z_extent(
			self.victimTemplateHandle, self.victimTemplateOrientation)
		# Offset the object origin so the lowest transformed bounding-box corner is just
		# above the floor. This remains correct when the imported victim's local Z axis is
		# not its world vertical axis or its bounding box is offset from the object frame.
		zPos = self.floorTopZ - victimMinimumZ + VICTIM_SURFACE_CLEARANCE
		count = 0
		for label, cell in self.sceneParameters.victimCells.items():
			column, row = cell
			x, y = self._cell_center_to_world(column, row)
			newHandle = self.sim.copyPasteObjects([self.victimTemplateHandle], 0)[0]
			self.sim.setObjectPosition(newHandle, -1, [x, y, zPos])
			self.sim.setObjectOrientation(newHandle, -1, list(self.victimTemplateOrientation))
			self.sim.setObjectAlias(newHandle, f"EGB320_GEN_VICTIM_{label}")
			self.sim.setObjectParent(newHandle, self.generatedSceneRootHandle, True)
			self.victimHandles[label] = newHandle
			self.victimPositions[label] = [x, y, zPos]
			count += 1
		return count

	def _park_templates_outside_playable_area(self):
		"""Move all source templates outside the playable area for future regeneration."""
		parkingSpots = {
			self.mazeWallTemplateHandle: (-3.0, -3.0),
			self.baseStationWallTemplateHandle: (-3.0, -3.4),
			self.victimWallTemplateHandle: (-3.0, -3.8),
			self.rubbleVictimWallTemplateHandle: (-3.0, -4.2),
			self.hazardWallTemplateHandle: (-3.0, -4.6),
			self.wallPostTemplateHandle: (-3.4, -3.0),
			self.victimTemplateHandle: (-3.4, -3.4),
		}
		for handle, (parkX, parkY) in parkingSpots.items():
			try:
				currentPosition = self.sim.getObjectPosition(handle, -1)
				self.sim.setObjectPosition(handle, -1, [parkX, parkY, currentPosition[2]])
			except Exception as e:
				print(f"Warning: could not park template handle {handle}: {e}")

	def _set_robot_pose(self, pose2D):
		"""Set the robot's 2D pose [x, y, theta], preserving its current Z coordinate."""
		x, y, theta = pose2D
		try:
			currentPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			self.sim.setObjectPosition(self.robotHandle, -1, [x, y, currentPosition[2]])
			self.sim.setObjectOrientation(self.robotHandle, -1, [0, 0, theta])
			try:
				self.sim.resetDynamicObject(self.robotHandle)
			except Exception:
				pass
			# Zero any residual motor target velocity so a stale command from a previous
			# run/session can't start driving the wheels the instant the simulation starts.
			for motorHandle in (self.leftMotorHandle, self.rightMotorHandle, self.leftRearMotorHandle, self.rightRearMotorHandle):
				if motorHandle is not None:
					try:
						self.sim.setJointTargetVelocity(motorHandle, 0.0)
					except Exception:
						pass
			print(f"Robot placed at ({x:.4f}, {y:.4f}) with yaw {theta:.4f} rad")
		except Exception as e:
			print(f"Warning: could not set robot starting pose: {e}")

	def _place_robot_at_base(self):
		"""Place the robot at the centre of sceneParameters.baseCell, facing sceneParameters.baseYaw."""
		column, row = self.sceneParameters.baseCell
		x, y = self._cell_center_to_world(column, row)
		self._set_robot_pose([x, y, self.sceneParameters.baseYaw])

	def _log_table_wall_positions(self):
		"""Log the table_wall boundary positions and the maze grid's world-coordinate bounds."""
		mazeWidth = self.sceneParameters.mazeColumns * self.sceneParameters.mazeCellSize
		mazeHeight = self.sceneParameters.mazeRows * self.sceneParameters.mazeCellSize
		xMax = self.mazeXMinimum + mazeWidth
		yMin = self.mazeYMaximum - mazeHeight

		print(f"Maze grid bounds: X [{self.mazeXMinimum:.4f}, {xMax:.4f}], Y [{yMin:.4f}, {self.mazeYMaximum:.4f}]")

		for index, handle in enumerate(self.tableWallHandles):
			try:
				position = self.sim.getObjectPosition(handle, -1)
				print(f"table_wall[{index}] position (x,y,z): {position[0]:.4f}, {position[1]:.4f}, {position[2]:.4f}")
			except Exception as e:
				print(f"Warning: could not read table_wall[{index}] position: {e}")

	def _ensure_generated_scene_root(self):
		"""Create the EGB320_GENERATED_SCENE dummy that all generated maze objects are parented to, if needed."""
		existingHandle = self._try_get_object('/EGB320_GENERATED_SCENE')
		if existingHandle is not None:
			self.generatedSceneRootHandle = existingHandle
			return
		self.generatedSceneRootHandle = self.sim.createDummy(0.01)
		self.sim.setObjectAlias(self.generatedSceneRootHandle, 'EGB320_GENERATED_SCENE')
		self.sim.setObjectPosition(self.generatedSceneRootHandle, -1, [0, 0, 0])

	def _remove_objects_by_alias_prefix(self, prefix):
		"""Remove any scene object whose alias starts with the given prefix (cleanup safety net)."""
		removed = 0
		try:
			allHandles = self.sim.getObjectsInTree(self.sim.handle_scene, self.sim.handle_all, 0)
		except Exception as e:
			print(f"Warning: could not enumerate scene objects for cleanup: {e}")
			return removed
		for handle in allHandles:
			try:
				alias = self.sim.getObjectAlias(handle)
			except Exception:
				continue
			if alias and alias.startswith(prefix):
				try:
					self.sim.removeObject(handle)
					removed += 1
				except Exception:
					pass
		return removed

	def _clear_generated_maze(self):
		"""
		Remove all previously generated maze objects (posts, walls, victims and their root
		dummy), then create a fresh EGB320_GENERATED_SCENE root ready for the new maze.
		Safe to call repeatedly: across StartSimulator() calls in the same process, sim
		stop/start cycles, and fresh Python processes reconnecting to a scene that still
		has leftover generated objects from a previous run.
		"""
		removedCount = 0

		rootHandle = self._try_get_object('/EGB320_GENERATED_SCENE')
		if rootHandle is not None:
			try:
				childHandles = self.sim.getObjectsInTree(rootHandle, self.sim.handle_all, 0)
			except Exception as e:
				print(f"Warning: could not enumerate previous generated objects: {e}")
				childHandles = []
			generatedHandles = [handle for handle in childHandles if handle != rootHandle]
			if generatedHandles:
				try:
					# Remove the hierarchy in one operation so marker children are removed together
					# with their model-base walls and cannot be orphaned between regenerations.
					self.sim.removeObjects(generatedHandles)
					removedCount += len(generatedHandles)
				except Exception:
					# Compatibility fallback for older CoppeliaSim releases.
					for childHandle in reversed(generatedHandles):
						try:
							self.sim.removeObject(childHandle)
							removedCount += 1
						except Exception:
							pass
			try:
				self.sim.removeObject(rootHandle)
				removedCount += 1
			except Exception as e:
				print(f"Warning: could not remove previous generated root: {e}")

		# Fallback sweep in case objects lost their parent or the root was already removed
		removedCount += self._remove_objects_by_alias_prefix('EGB320_GEN_')

		self._ensure_generated_scene_root()

		self.generatedMazeWallHandles = []
		self.generatedMarkerWallHandles = []
		self.generatedWallPostHandles = []
		self.victimHandles = {}
		self.victimPositions = {}
		self.markerWallPlacements = []
		self.carriedVictimHandle = None
		self.carriedVictimLabel = None

		print(f"Cleanup removed {removedCount} previously generated object(s).")
		return removedCount

	def _print_scene_summary(self, postCount, wallCount, victimCount, removedCount):
		"""Print a concise summary of the generated search and rescue scene."""
		mazeWidth = self.sceneParameters.mazeColumns * self.sceneParameters.mazeCellSize
		mazeHeight = self.sceneParameters.mazeRows * self.sceneParameters.mazeCellSize
		victimCellsText = ", ".join(f"{label}={cell}" for label, cell in self.sceneParameters.victimCells.items())

		print("EGB320 search and rescue scene generated")
		print(f"Grid: {self.sceneParameters.mazeRows} x {self.sceneParameters.mazeColumns} cells")
		print(f"Cell size: {self.sceneParameters.mazeCellSize:.3f} m")
		print(f"Maze footprint: {mazeWidth:.3f} m x {mazeHeight:.3f} m")
		print(f"Posts created: {postCount}")
		print(f"Walls created: {wallCount}")
		markerCounts = {
			kind: sum(1 for marker in self.markerWallPlacements if marker['kind'] == kind)
			for kind in ('base', 'victim', 'rubble_victim', 'hazard')
		}
		print(
			"Marker walls created: "
			f"base={markerCounts['base']}, exposed victims={markerCounts['victim']}, "
			f"trapped victims={markerCounts['rubble_victim']}, hazards={markerCounts['hazard']}")
		print(f"Victims created: {victimCount}")
		print(f"Base cell: {tuple(self.sceneParameters.baseCell)}")
		print(f"Victim cells: {victimCellsText}")
		print(f"Objects removed during cleanup: {removedCount}")

	def _set_obstacle_positions(self):
		"""Place optional obstacles at configured positions, when present."""
		if not any(h is not None for h in self.obstacleHandles):
			return
		obstacleHeight = 0.15
		startingPositions = [
			self.sceneParameters.obstacle0_StartingPosition,
			self.sceneParameters.obstacle1_StartingPosition,
			self.sceneParameters.obstacle2_StartingPosition,
		]
		for index, obstaclePosition in enumerate(startingPositions):
			if self.obstacleHandles[index] is None or obstaclePosition is None or obstaclePosition == -1:
				continue
			coppeliaStartingPosition = [obstaclePosition[0], obstaclePosition[1], obstacleHeight / 2]
			try:
				self.sim.setObjectPosition(self.obstacleHandles[index], -1, coppeliaStartingPosition)
			except Exception as e:
				print(f"Warning: error setting obstacle {index} position: {e}")

	### CAMERA FUNCTIONS ###

	# Sets the camera's pose
	# Inputs:
	#		x - distance between the camera and the center of the robot in the direction of the front of the robot
	#		z - height of the camera relative to the floor in metres
	#		pitch - tilt of the camera in radians
	def SetCameraPose(self, x, z, pitch):
		# assume the students want the camera in the center of the robot (so no y)
		# assume the student only wants to rotate the camera to point towards the ground or sky (so no roll or yaw)

		# update robot parameters
		self.robotParameters.cameraDistanceFromRobotCenter = x
		self.robotParameters.cameraHeightFromFloor = z
		self.robotParameters.cameraTilt = pitch

		# Need to change Z as in COPPELIA the robot frame is in the center of the Cylinder
		# z in COPPELIA robot frame = z - (cylinder height)/2 - wheel diameter
		z = z - 0.075 - 2*self.robotParameters.wheelRadius

		# Need to change the pitch by adding pi/2 (90 degrees) as pitch of 0 points up
		pitch = pitch + math.pi/2.0

		# set camera pose
		try:
			self.sim.setObjectPosition(self.cameraHandle, self.robotHandle, [x, 0, z])
			# Flip the camera horizontally by changing yaw from math.pi/2.0 to -math.pi/2.0
			self.sim.setObjectOrientation(self.cameraHandle, self.robotHandle, [math.pi, pitch, -math.pi/2.0])
		except Exception as e:
			print(f"Error setting camera pose: {e}")

	

	# Set Camera Orientation to either portrait or landscape
	def SetCameraOrientation(self, orientation):
		# get resolution based on orientation and robot parameters
		if orientation == 'portrait':
			x_res = self.robotParameters.cameraResolutionY  # swap X and Y for portrait
			y_res = self.robotParameters.cameraResolutionX
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle * x_res / y_res
		elif orientation == 'landscape':
			x_res = self.robotParameters.cameraResolutionX
			y_res = self.robotParameters.cameraResolutionY
			self.verticalViewAngle = self.robotParameters.cameraPerspectiveAngle * y_res / x_res
			self.horizontalViewAngle = self.robotParameters.cameraPerspectiveAngle
		else:
			print('The camera orientation %s is not known. You must specify either portrait or landscape')
			return


		# update robot parameters
		self.robotParameters.cameraOrientation = orientation

		# set resolution of camera (vision sensor object) - resolution parameters are int32 parameters
		try:
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_x, x_res)
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_y, y_res)
		except Exception as e:
			print(f"Error setting camera resolution: {e}")
		

	def SetCameraResolution(self, x_res, y_res):
		"""
		Set the camera resolution to specific width and height values.
		
		Args:
			x_res (int): Camera width resolution in pixels
			y_res (int): Camera height resolution in pixels
		
		Returns:
			bool: True if successful, False otherwise
		"""
		if self.cameraHandle is None:
			print("Error: Camera not initialized")
			return False
			
		self.robotParameters.cameraResolutionX = x_res
		self.robotParameters.cameraResolutionY = y_res
		
		try:
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_x, x_res)
			self.sim.setObjectInt32Param(self.cameraHandle, self.sim.visionintparam_resolution_y, y_res)
			print(f"Camera resolution set to {x_res}x{y_res}")
			return True
		except Exception as e:
			print(f"Error setting camera resolution: {e}")
			return False
		
	########################################
	##### INTERNAL IMPLEMENTATION #########
	########################################
	# The functions below are used internally by the library
	# Students typically don't need to modify these functions

	# Prints the pose/position of the objects in the scene
	def PrintObjectPositions(self):
		print("\n\n***** OBJECT POSITIONS *****")
		if self.robotPose is not None:
			print("Robot 2D Pose (x,y,theta): %0.4f, %0.4f, %0.4f"%(self.robotPose[0], self.robotPose[1], self.robotPose[2]))

		if self.cameraPose is not None:
			print("Camera 3D Pose (x,y,z,roll,pitch,yaw): %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f"%(self.cameraPose[0], self.cameraPose[1], self.cameraPose[2], self.cameraPose[3], self.cameraPose[4], self.cameraPose[5]))

		for index, obstacle in enumerate(self.obstaclePositions):
			if obstacle is not None:
				print("Obstacle %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, obstacle[0], obstacle[1], obstacle[2]))

	# Gets the pose/position in the global coordinate frame of all the objects in the scene.
	# Stores them in class variables. Variables will be set to none if could not be updated
	def GetObjectPositions(self):
		# Clear cached values so callers can distinguish an update failure.
		self.robotPose = None
		self.cameraPose = None
		self.obstaclePositions = [None, None, None]

		# GET 2D ROBOT POSE
		try:
			robotPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			robotOrientation = self.sim.getObjectOrientation(self.robotHandle, -1)
			self.robotPose = [robotPosition[0], robotPosition[1], robotOrientation[2]]
		except Exception as e:
			print(f"Error getting robot pose: {e}")
			robotOrientation = [0.0, 0.0, 0.0]

		# GET 3D CAMERA POSE
		if self.cameraHandle is not None:
			try:
				cameraPosition = self.sim.getObjectPosition(self.cameraHandle, -1)
				cameraOrientation = self.sim.getObjectOrientation(self.cameraHandle, -1)
				self.cameraPose = [
					cameraPosition[0], cameraPosition[1], cameraPosition[2],
					cameraOrientation[0], cameraOrientation[1], cameraOrientation[2]]
			except Exception as e:
				print(f"Error getting camera pose: {e}")

		# Optional obstacle positions.
		for index, obstacleHandle in enumerate(self.obstacleHandles):
			if obstacleHandle is None:
				continue
			try:
				self.obstaclePositions[index] = self.sim.getObjectPosition(obstacleHandle, -1)
			except Exception as e:
				print(f"Error getting obstacle position {index}: {e}")

	# Checks to see if an Object is within the field of view of the camera
	def GetRBInCameraFOV(self, objectPosition):
		# calculate range and bearing on 2D plane - relative to the camera
		cameraPose2d = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2d, objectPosition)

		# vertical_test_cam_pose = [0,self.cameraPose[2],0]
		# vertical_test_pos = [_range,objectPosition[2]]
		# _vert_range, _vert_bearing = self.GetRangeAndBearingFromPoseAndPoint(vertical_test_cam_pose, vertical_test_pos)
		_valid = abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2 \
		# 	and abs(_vert_bearing) < self.robotParameters.cameraPerspectiveAngle/4

		# angle from camera's axis to the object's position
		# verticalAngle = math.atan2(objectPosition[2]-self.cameraPose[2], _range)

		#OLD code needs removing

		# # check to see if in field of view
		# if abs(_bearing) > (self.horizontalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# if abs(verticalAngle) > (self.verticalViewAngle/2.0):
		# 	# return False to indicate object outside camera's FOV and range and bearing
		# 	return False, _range, _bearing

		# return True to indicate is in FOV and range and bearing
		return _valid, _range, _bearing

	def ObjectInCameraFOV(self,objectPosition):
		_,_bearing = self.GetRBInCameraFOV(objectPosition)
		return np.abs(_bearing) <= self.robotParameters.cameraPerspectiveAngle / 2
			
	
	# Determines if a 2D point is inside the arena, returns true if that is the case
	def PointInsideArena(self, position):
		if position[0] > -1 and position[0] < 1 and position[1] > -1 and position[1] < 1:
			return True

		return False


	# Legacy 2025 boundary-only wall geometry. Retained internally as a possible reference for
	# a future maze-aware replacement, but deliberately not called by GetDetectedWallPoints().
	# Gets the range and bearing to a corner that is within the camera's field of view.
	# Will only return a single corner, as only one corner can be in the field of view with the current setup.
	# returns:
	#	a list containing a [range, bearing] or an empty list if no corner is within the field of view
	def FieldCornerRangeBearing(self, cameraPose):
		rangeAndBearing = []

		# Get range and bearing from camera's pose to each corner
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, 1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [-1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, [1, -1])
		if abs(_bearing) < (self.horizontalViewAngle/2.0):
			rangeAndBearing = [_range, _bearing]

		return rangeAndBearing


	# Gets the range and bearing to where the edge of camera's field of view intersects with the arena walls.
	# returns:
	#	None - if there are no valid wall points (i.e. the robot is right up against a wall and facing it)
	#	A list of [range, bearing] arrays. There will either be 1 or 2 [range, bearing] arrays depending on the situation
	#		will return 1 if the robot is close to a wall but not directly facing it and one edge of the camera's view limit is up against the wall, while the other can see part of the field
	#		will return 2 if the robot can see the wall but is not facing a corner
	def CameraViewLimitsRangeAndBearing(self, cameraPose):
		viewLimitIntersectionPoints = []
		rangeAndBearings = []

		# Get valid camera view limit points along the east wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'east')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the north wall (wall in positive y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'north')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the west wall
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'west')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Get valid camera view limit points along the south wall (wall in negative y-direction)
		p1, p2 = self.CameraViewLimitWallIntersectionPoints(cameraPose, 'south')
		if p1 != None:
			viewLimitIntersectionPoints.append(p1)
		if p2 != None:
			viewLimitIntersectionPoints.append(p2)

		# Calculate range and bearing to the valid view limit wall intersection points and store in a list
		for point in viewLimitIntersectionPoints:
			_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, point)
			rangeAndBearings.append([_range, _bearing])

		# return None if rangeAndBearings list is empty
		if rangeAndBearings == []:
			return None
		else:
			return rangeAndBearings

	
	# Gets the points where the edges of the camera's field of view intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	p1 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	#	p2 - will be [x,y] point if it is a valid wall point (i.e. lies on the arena's walls and is within the field of view) or None if it is not valid
	def CameraViewLimitWallIntersectionPoints(self, cameraPose, wall):
		
		# calculate range to wall along camera's axis using the point where the camera's axis intersects with the specified wall
		x, y = self.CameraViewAxisWallIntersectionPoint(cameraPose, wall)
		centreRange = math.sqrt(math.pow(cameraPose[0]-x, 2) + math.pow(cameraPose[1]-y, 2))


		# determine camera view limit intersection points on wall
		if wall == 'east' or wall == 'west':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi/2.0 - self.horizontalViewAngle/2.0 + cameraPose[2])
		elif wall == 'north' or wall == 'south':
			d1 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(math.pi - self.horizontalViewAngle/2.0 - cameraPose[2])
			d2 = centreRange*math.sin(self.horizontalViewAngle/2.0) / math.sin(cameraPose[2] - self.horizontalViewAngle/2.0)


		# add d1 and d2 (or subtract) to the camera's axis wall intersection point (add/subtract and x/y depends on wall)
		if wall == 'east' or wall == 'west':
			p1 = [x, y+d1]
			p2 = [x, y-d2]
		elif wall == 'north' or wall == 'south':
			p1 = [x-d1, y]
			p2 = [x+d2, y]

		# determine camera view limit intersection point range and bearings relative to camera
		range1, bearing1 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p1)
		range2, bearing2 = self.GetRangeAndBearingFromPoseAndPoint(cameraPose, p2)

		# Check that the two view limit intersection points are valid (i.e. occur on the arena boundary and not outside, that the bearing is within view and the range is greater than a minimum distance)
		# Need to add small percentage to the angle due to the numerical evaluation of COPPELIA this is to ensure that after checking against all walls that 2 points are returned this is where the *1.05 comes from
		# make sure p1 is within bounds and that bearing is valid
		if (p1[0] < -1 or p1[0] > 1 or p1[1] < -1 or p1[1] > 1):
			p1 = None
		elif abs(bearing1) > (self.horizontalViewAngle/2.0)*1.05:
			p1 = None
		elif range1 < self.robotParameters.minWallDetectionDistance:
			p1 = None
		
		# make sure p2 is within bounds
		if (p2[0] < -1 or p2[0] > 1 or p2[1] < -1 or p2[1] > 1):
			p2 = None
		elif abs(bearing2) > (self.horizontalViewAngle/2.0)*1.05:
			p2 = None
		elif range2 < self.robotParameters.minWallDetectionDistance:
			p2 = None

		return p1, p2


	# Gets the point where the camera's view axis (centre of image) intersects with the specified wall.
	# inputs:
	#	cameraPose - pose of the camera [x, y, theta] in the global coordinate frame
	# 	wall - wall want to get the camera view limit points of ('east', 'west', 'north', 'south').
	# returns:
	#	x - the x coordinate where the camera's axis intersects with the specified wall
	#	y - the y coordinate where the camera's axis intersects with the specified wall
	def CameraViewAxisWallIntersectionPoint(self, cameraPose, wall):
		if wall == 'east':
			x = 1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]
		
		elif wall == 'north':
			y = 1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		elif wall == 'west':
			x = -1
			y = (x - cameraPose[0]) * math.tan(cameraPose[2]) + cameraPose[1]

		elif wall == 'south':
			y = -1
			x = (y - cameraPose[1]) / math.tan(cameraPose[2]) + cameraPose[0]

		return x, y
	

	# Wraps input value to be between -pi and pi
	def WrapToPi(self, radians):
		return ((radians + math.pi) % (2* math.pi) - math.pi)

	# Gets the range and bearing given a 2D pose (x,y,theta) and a point(x,y). 
	# The bearing will be relative to the pose's angle
	def GetRangeAndBearingFromPoseAndPoint(self, pose, point):
		_range = math.sqrt(math.pow(pose[0] - point[0], 2) + math.pow(pose[1] - point[1], 2))
		_bearing = self.WrapToPi(math.atan2((point[1]-pose[1]), (point[0]-pose[0])) - pose[2])

		return _range, _bearing

def print_debug_range_bearing(object_type, range_bearing_data):
	"""
	Helper function to print range and bearing information for detected objects.
	Useful for debugging object detection.
	
	Args:
		object_type (str): Name of the object type being displayed
		range_bearing_data: Range and bearing data from GetDetectedObjects()
	"""
	if range_bearing_data is None:
		print(f"{object_type}: No objects detected")
		return
	
	# Handle items array (6-element list, one per item type)
	if object_type == "Items" and isinstance(range_bearing_data, list) and len(range_bearing_data) == 6:
		item_names = ["Bowls", "Mugs", "Bottles", "Soccer Balls", "Rubiks Cubes", "Cereal Boxes"]
		any_items_found = False
		
		for item_type, detections in enumerate(range_bearing_data):
			if detections is not None and len(detections) > 0:
				any_items_found = True
				for i, rb in enumerate(detections):
					if rb is not None and len(rb) >= 2:
						range_m = rb[0]
						bearing_rad = rb[1]
						bearing_deg = math.degrees(bearing_rad)
						print(f"{item_names[item_type]}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
		
		if not any_items_found:
			print(f"{object_type}: No items detected")
		return
	
	# Handle single detection or list of detections
	if isinstance(range_bearing_data, list):
		if len(range_bearing_data) == 0:
			print(f"{object_type}: No detections")
			return
		
		# Check if this is a single [range, bearing] pair
		if len(range_bearing_data) == 2 and isinstance(range_bearing_data[0], (int, float)):
			range_m = range_bearing_data[0]
			bearing_rad = range_bearing_data[1]
			bearing_deg = math.degrees(bearing_rad)
			print(f"{object_type}: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
		else:
			# List of multiple detections
			for i, rb in enumerate(range_bearing_data):
				if rb is not None and isinstance(rb, list) and len(rb) >= 2:
					range_m = rb[0]
					bearing_rad = rb[1]
					bearing_deg = math.degrees(bearing_rad)
					print(f"{object_type}[{i}]: Range = {range_m:.3f}m, Bearing = {bearing_rad:.3f}rad ({bearing_deg:.1f}°)")
				elif rb is None:
					print(f"{object_type}[{i}]: Not detected")
	else:
		print(f"{object_type}: Invalid data format")

# Parameter classes for robot and scene configuration
class RobotParameters(object):
	"""Parameters for configuring the maze robot."""
	def __init__(self):
		# Drive Parameters
		self.driveType = 'differential'  # currently only 'differential' implemented
		self.minimumLinearSpeed = 0.0   # minimum speed in m/s
		self.maximumLinearSpeed = 0.25  # maximum speed in m/s
		self.driveSystemQuality = 1.0   # quality from 0 to 1 (1 = perfect)
		
		# Wheel Parameters (set automatically for differential drive)
		self.wheelBase = 0.15           # distance between wheels in m
		self.wheelRadius = 0.03         # wheel radius in m
		
		# Camera Parameters
		self.cameraOrientation = 'landscape'  # 'landscape' or 'portrait'
		self.cameraDistanceFromRobotCenter = 0.1  # distance from robot center in m
		self.cameraHeightFromFloor = 0.15     # height from floor in m
		self.cameraTilt = 0.0                 # tilt angle in radians
		self.cameraResolutionX = 640          # camera width in pixels
		self.cameraResolutionY = 480          # camera height in pixels
		self.cameraPerspectiveAngle = math.pi/3  # field of view angle in radians
		
		# Detection Parameters
		self.maxObstacleDetectionDistance = 1.5  # max distance to detect obstacles in m
		self.minWallDetectionDistance = 0.02      # min distance for a wall point to be considered valid, in m
		
		# Victim collection parameter from the 2026 assessment rules
		self.victimCollectionDistance = 0.10  # shortest horizontal clearance in metres
		
		# Simulation Parameters
		self.sync = False  # synchronous mode (deprecated with ZMQ Remote API)


class SceneParameters(object):
	"""Parameters for configuring the EGB320 search-and-rescue maze scene (2026)."""
	def __init__(self):
		# --- Search and rescue maze parameters (2026) ---
		self.mazeRows = 7
		self.mazeColumns = 7
		self.mazeCellSize = 0.280  # metres
		self.mazeOriginXY = None   # None = use the centre of /floor as the maze origin
		self.autoGenerateMaze = True
		self.clearGeneratedMaze = True
		# False clears previously generated objects but creates no posts, walls or victims.
		# The templates are still parked off-table and the robot is placed normally.
		self.generateMazeObjects = True
		self.baseCell = (0, 6)
		self.baseYaw = math.pi / 2
		self.victimCells = {
			"L1": (1, 5),
			"L2": (4, 3),
			"L3": (6, 0),
		}
		self.placeRobotAtBase = True

		# Robot starting position [x, y, theta] in metres and radians.
		# If set, this overrides placeRobotAtBase/baseCell.
		self.robotStartingPosition = None

		# Optional obstacle starting positions [x, y] in metres.
		# Set to -1 to use current CoppeliaSim position, None if not wanted in scene
		self.obstacle0_StartingPosition = None
		self.obstacle1_StartingPosition = None
		self.obstacle2_StartingPosition = None

	def validate_maze_parameters(self):
		"""
		Validate maze-related settings before any scene objects are created.
		Raises ValueError with a clear message if a check fails.
		"""
		if self.mazeRows <= 0 or self.mazeColumns <= 0:
			raise ValueError(f"mazeRows and mazeColumns must be positive (got {self.mazeRows} x {self.mazeColumns})")

		if self.mazeCellSize <= 0:
			raise ValueError(f"mazeCellSize must be positive (got {self.mazeCellSize})")

		maxColumnIndex = self.mazeColumns
		maxRowIndex = self.mazeRows

		for (startPoint, endPoint) in EXAMPLE_MAZE_SEGMENTS:
			for point in (startPoint, endPoint):
				column, row = point
				if not (0 <= column <= maxColumnIndex):
					raise ValueError(f"Wall endpoint column {column} out of range [0, {maxColumnIndex}]: {point}")
				if not (0 <= row <= maxRowIndex):
					raise ValueError(f"Wall endpoint row {row} out of range [0, {maxRowIndex}]: {point}")
			if startPoint == endPoint:
				raise ValueError(f"Wall segment has identical start and end point: {startPoint}")

		maxCellColumnIndex = self.mazeColumns - 1
		maxCellRowIndex = self.mazeRows - 1
		seenCells = set()
		for label, cell in self.victimCells.items():
			column, row = cell
			if not (0 <= column <= maxCellColumnIndex):
				raise ValueError(f"Victim '{label}' column {column} out of range [0, {maxCellColumnIndex}]")
			if not (0 <= row <= maxCellRowIndex):
				raise ValueError(f"Victim '{label}' row {row} out of range [0, {maxCellRowIndex}]")
			if cell in seenCells:
				raise ValueError(f"Victim cells must be unique - duplicate cell {cell}")
			seenCells.add(cell)

		if tuple(self.baseCell) in seenCells:
			raise ValueError(f"baseCell {self.baseCell} must not coincide with a victim cell")


