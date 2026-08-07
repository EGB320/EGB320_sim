# Import required Python modules
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import sys
from enum import IntEnum

"""
EGB320 Search and Rescue Robot Library (2026)

This library provides a Python interface for controlling the EGB320 robot in CoppeliaSim.

As of this phase, the library targets the 2026 search and rescue maze challenge. It uses
the CoppeliaSim ZeroMQ Python remote API to deterministically generate a 7x7 cell maze
(walls, corner posts) and place three victim objects. The legacy 2025 warehouse pick and
place functions (picking stations, shelves, items, obstacles, row markers) are retained for
backward compatibility but are optional and are not required for this phase.

MAIN FUNCTIONS:
===========================

Robot Control:
- StartSimulator() / StopSimulator()    : Start/stop the simulation (generates the maze while stopped)
- SetTargetVelocities(x_dot, theta_dot) : Set robot movement velocities
- UpdateObjectPositions()               : Update object positions (call this every loop!)

Object Detection:
- GetDetectedObjects(objects)           : Get range/bearing to all detected objects (legacy warehouse objects)
- GetCameraImage()                      : Get camera image for computer vision
- GetDetectedWallPoints()               : Get range/bearing to visible walls

Legacy Item Collection (2025 warehouse challenge, unused in this phase):
- CollectItem(closest_picking_station)  : Collect items from picking stations
- DropItemInClosestShelfBay()          : Drop items in nearest empty shelf bay
- itemCollected()                       : Check if robot is carrying an item
- DropItem()                           : Drop the currently held item

Configuration:
- SetCameraResolution(x_res, y_res)    : Set camera resolution

EXAMPLE USAGE:
=============
# Initialize robot
robot = COPPELIA_WarehouseRobot(robotParameters, sceneParameters)
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



# This class defines object types in the warehouse simulation
class warehouseObjects(IntEnum):
	
	# Item types
	bowl = 0
	mug = 1
	bottle = 2
	soccer = 3
	rubiks = 4
	cereal = 5

	# Obstacle objects
	obstacle0 = 6
	obstacle1 = 7
	obstacle2 = 8
	
	# Picking station objects
	pickingStation = 22
	pickingStation1 = 19
	pickingStation2 = 20
	pickingStation3 = 21

	# Row marker objects
	row_marker_1 = 10
	row_marker_2 = 11
	row_marker_3 = 12

	# Shelf objects
	shelf_0 = 13
	shelf_1 = 14
	shelf_2 = 15
	shelf_3 = 16
	shelf_4 = 17
	shelf_5 = 18

	# Object groups for detection
	items = 101
	obstacles = 102
	row_markers = 103
	shelves = 104
	PickingStationMarkers = 105


################################
##### WAREHOUSE BOT CLASS #####
################################

class COPPELIA_WarehouseRobot(object):
	"""
	Main class for controlling the warehouse robot in CoppeliaSim.
	This class provides functions for robot navigation, object detection, and item collection.
	"""
	
	####################################
	#### WAREHOUSE BOT INITIALIZATION ###
	####################################

	def __init__(self, robotParameters, sceneParameters, coppelia_server_ip='127.0.0.1', port=23000):
		"""
		Initialize the warehouse robot connection to CoppeliaSim.
		
		Args:
			robotParameters: RobotParameters object with robot configuration
			sceneParameters: SceneParameters object with scene configuration
			coppelia_server_ip: IP address of CoppeliaSim server (default: '127.0.0.1')
			port: Port number for ZMQ Remote API (default: 23000)
		"""
		print(f"Initializing warehouse robot connection...")
		
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
		self.collectorForceSensorHandle = None
		self.leftMotorHandle = None
		self.rightMotorHandle = None
		self.leftRearMotorHandle = None
		self.rightRearMotorHandle = None
		self.v60MotorHandle = None
		self.v180MotorHandle = None
		self.v300MotorHandle = None
		self.itemTemplateHandles = [None] * 6
		self.itemHandles = np.zeros((6,4,3),dtype=np.int16)
		self.obstacleHandles = [None, None, None]
		self.pickingStationHandle = None
		self.pickingStationMarkerHandles = [None, None, None]
		self.pickingStationItemHandles = [None, None, None]
		self.rowMarkerHandles = [None,None,None]
		self.shelfHandles = [None]*6
		self.bayHandles = np.full((6,4,3), None, dtype=object)
		self.proximityHandle = None

		# Search and rescue maze scene object handles (essential for this phase)
		self.floorHandle = None
		self.tableWallHandles = []
		self.mazeWallTemplateHandle = None
		self.wallPostTemplateHandle = None
		self.victimTemplateHandle = None

		# Generated maze object bookkeeping (used for cleanup on regeneration)
		self.generatedSceneRootHandle = None
		self.generatedMazeWallHandles = []
		self.generatedWallPostHandles = []
		self.victimHandles = {}
		self.victimPositions = {}

		# Cached maze geometry (populated by _get_floor_info/_cache_template_geometry)
		self.floorCenter = None
		self.floorTopZ = None
		self.mazeXMinimum = None
		self.mazeYMaximum = None
		self.mazeWallTemplateSize = None
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
		self.itemSize = 0.05      # diameter of item in meters

		# Object position variables
		self.robotPose = None
		self.cameraPose = None
		self.itemPositions = np.full((6,4,3,3),np.nan,dtype=np.float32)
		self.pickingStationPosition = None
		self.obstaclePositions = [None, None, None]
		self.rowMarkerPositions = [None, None, None]

		# Item collection state
		self.itemConnectedToRobot = False
		self.heldItemHandle = None

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
		
		print("Warehouse robot initialization complete!")
	
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
	
	def _process_multiple_object_detection(self, positions, start_index, objectsDetected, max_detection_distance):
		"""Helper function to process detection of multiple objects with sequential indices"""
		results = []
		for index, position in enumerate(positions):
			result = self._process_single_object_detection(position, start_index + index, objectsDetected, max_detection_distance)
			results.append(result)
		return results
	
	def _add_item_to_range_bearing(self, itemRangeBearing, item_type, range_bearing):
		"""Helper function to add item detection to range bearing list"""
		if itemRangeBearing[item_type] is None:
			itemRangeBearing[item_type] = []
		itemRangeBearing[item_type].append(range_bearing)

	def GetDetectedObjects(self, objects=None):
		"""
		Gets the range and bearing to all detected objects in the camera's field of view.
		
		Args:
			objects: List of object types to detect (default: all objects)
			
		Returns:
			tuple: (itemsRB, packingStationRB, obstaclesRB, rowMarkerRB, shelfRB, pickingStationRB)
				- itemsRB: Range and bearing to items [6-element list, one per item type]
				- packingStationRB: Range and bearing to main picking station
				- obstaclesRB: Range and bearing to obstacles  
				- rowMarkerRB: Range and bearing to row markers [3-element list]
				- shelfRB: Range and bearing to shelves [6-element list]
				- pickingStationRB: Range and bearing to individual picking stations [3-element list]
		"""
		# Initialize return variables
		itemRangeBearing = [None]*6
		pickingStationRangeBearing = None
		obstaclesRangeBearing = None
		rowMarkerRangeBearing = [None,None,None]
		shelfRangeBearing = [None]*6
		pickingStationMarkersRangeBearing = [None, None, None]

		# Default to detecting all objects if none specified
		if objects is None:
			objects = [warehouseObjects.items, warehouseObjects.shelves, warehouseObjects.row_markers,
					  warehouseObjects.obstacles, warehouseObjects.pickingStation, warehouseObjects.PickingStationMarkers]

		# Check if camera pose is available
		if self.cameraPose is None:
			return itemRangeBearing, pickingStationRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing, pickingStationMarkersRangeBearing

		# Get object detection data from CoppeliaSim vision sensor
		try:
			result, data, packets = self.sim.handleVisionSensor(self.objectDetectorHandle)
			
			if result == -1 or not packets or len(packets) == 0:
				objectsDetected = []
			else:
				objectsDetected = packets
				
		except Exception as e:
			objectsDetected = []
		if objectsDetected and len(objectsDetected) > 0:
			
			# Check for shelves (indices 13-18 in detection array)
			if warehouseObjects.shelves in objects:
				shelfRB = self.GetShelfRangeBearing()
				for index, rb in enumerate(shelfRB):
					shelf_index = 13 + index
					if self._is_object_detected(objectsDetected, shelf_index):
						if rb and rb[0] < self.robotParameters.maxShelfDetectionDistance:
							shelfRangeBearing[index] = rb

			# Check for items (indices 0-5: bowl, mug, bottle, soccer, rubiks, cereal)
			if warehouseObjects.items in objects:
				for item_type in range(6):
					if self._is_object_detected(objectsDetected, item_type):
						item_names = ["BOWL", "MUG", "BOTTLE", "SOCCER_BALL", "RUBIKS_CUBE", "CEREAL_BOX"]
						item_name = item_names[item_type]
						
						try:
							item_handle = self.sim.getObject(f'/{item_name}')
							item_position = self.sim.getObjectPosition(item_handle, -1)
							
							result = self._process_single_object_detection(
								item_position, item_type, objectsDetected, 
								self.robotParameters.maxItemDetectionDistance)
							
							if result is not None:
								self._add_item_to_range_bearing(itemRangeBearing, item_type, result)
						except Exception:
							pass
				# Check for items at picking stations
				for station_index in range(3):
					item_type = self.sceneParameters.pickingStationContents[station_index]
					
					if item_type != -1 and 0 <= item_type <= 5:
						station_handle = self.pickingStationMarkerHandles[station_index]
						
						if station_handle is not None:
							try:
								station_position = self.sim.getObjectPosition(station_handle, -1)
								item_position = [station_position[0], station_position[1], station_position[2]]
								
								if self._is_object_detected(objectsDetected, item_type) and self.PointInsideArena(item_position):
									_valid, _range, _bearing = self.GetRBInCameraFOV(item_position)
									
									if _range < self.robotParameters.maxItemDetectionDistance and abs(_bearing) < self.robotParameters.cameraPerspectiveAngle/2:
										self._add_item_to_range_bearing(itemRangeBearing, item_type, [_range, _bearing])
							except Exception:
								pass
			# Check for obstacles (indices 6-8)
			if warehouseObjects.obstacles in objects:
				for index, obstaclePosition in enumerate(self.obstaclePositions):
					if obstaclePosition is not None:
						result = self._process_single_object_detection(obstaclePosition, 6 + index, objectsDetected, self.robotParameters.maxObstacleDetectionDistance)
						if result is not None:
							if obstaclesRangeBearing is None:
								obstaclesRangeBearing = []
							obstaclesRangeBearing.append(result)

			# Check for main picking station (index 9)
			if warehouseObjects.pickingStation in objects:
				if self.pickingStationPosition is not None:
					pickingStationRangeBearing = self._process_single_object_detection(
						self.pickingStationPosition, 22, objectsDetected, self.robotParameters.maxPickingStationDetectionDistance)

			# Check for row markers (indices 10-12)
			if warehouseObjects.row_markers in objects:
				rowMarkerRangeBearing = self._process_multiple_object_detection(
					self.rowMarkerPositions, 10, objectsDetected, self.robotParameters.maxRowMarkerDetectionDistance)

			# Check for individual picking stations (indices 19-21)
			if warehouseObjects.PickingStationMarkers in objects:
				for station_index in range(3):
					if self._is_object_detected(objectsDetected, 19 + station_index):
						station_handle = self.pickingStationMarkerHandles[station_index]
						if station_handle is not None:
							try:
								station_position = self.sim.getObjectPosition(station_handle, -1)
								result = self._process_single_object_detection(
									station_position, 19 + station_index, objectsDetected, 
									self.robotParameters.maxPickingStationMarkersDetectionDistance)
								if result is not None:
									pickingStationMarkersRangeBearing[station_index] = result
							except Exception:
								pass

		return itemRangeBearing, pickingStationRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing, pickingStationMarkersRangeBearing


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
		Gets the range and bearing to wall points visible in the camera's field of view.
		
		Returns:
			list: List of [range, bearing] arrays for visible wall points, or None if no walls detected
		"""
		if self.cameraPose is None:
			return None
		
		cameraPose2D = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]

		# Get range and bearing to wall intersection points
		wallPoints = self.CameraViewLimitsRangeAndBearing(cameraPose2D)
		if wallPoints is None:
			return None

		# Check for corners in field of view
		cornerRangeBearing = self.FieldCornerRangeBearing(cameraPose2D)
		if cornerRangeBearing:
			wallPoints.append(cornerRangeBearing)
			
		return wallPoints
		

	def SetTargetVelocities(self, x_dot, theta_dot):
		"""
		Set the target velocities for the robot.
		
		Args:
			x_dot: Forward velocity in m/s
			theta_dot: Rotational velocity in rad/s
		"""
		if self.robotParameters.driveType == 'differential':
			# Robot physical parameters (fixed for the simulation)
			self.robotParameters.wheelBase = 0.15
			self.robotParameters.wheelRadius = 0.03

			# Calculate speed limits
			minWheelSpeed = self.robotParameters.minimumLinearSpeed / self.robotParameters.wheelRadius
			maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

			# Calculate individual wheel speeds
			leftWheelSpeed = (x_dot - 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.leftWheelBias
			rightWheelSpeed = (x_dot + 0.5*theta_dot*self.robotParameters.wheelBase) / self.robotParameters.wheelRadius + self.rightWheelBias

			# Add noise if drive system quality is not perfect
			if self.robotParameters.driveSystemQuality != 1:
				leftWheelSpeed = np.random.normal(leftWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)[0]
				rightWheelSpeed = np.random.normal(rightWheelSpeed, (1-self.robotParameters.driveSystemQuality)*1, 1)[0]

			# Limit wheel speeds
			leftWheelSpeed = min(leftWheelSpeed, maxWheelSpeed)
			rightWheelSpeed = min(rightWheelSpeed, maxWheelSpeed)

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

	def itemCollected(self):
		"""
		Returns True if the robot is currently carrying an item.
		
		Returns:
			bool: True if item is collected, False otherwise
		"""
		return self.itemConnectedToRobot

	def DropItem(self):
		"""
		Drops the currently held item.
		"""
		if self.itemConnectedToRobot:
			try:
				if hasattr(self, 'heldItemHandle') and self.heldItemHandle is not None:
					objectHandle = self.heldItemHandle
					
					if objectHandle != -1:
						try:
							# Verify handle is still valid
							self.sim.getObjectPosition(objectHandle, -1)
							
							# Release the item from the robot
							self.sim.setObjectParent(objectHandle, -1, True)
							
							print(f"Released item with handle: {objectHandle}")
							self.heldItemHandle = None
							self.itemConnectedToRobot = False
							
						except Exception as e:
							print(f"Error releasing item: {e}")
							self.heldItemHandle = None
							self.itemConnectedToRobot = False
					else:
						print("Error: Invalid item handle during release")
						self.heldItemHandle = None
						self.itemConnectedToRobot = False
				else:
					print("No items to release")
					self.itemConnectedToRobot = False
					
			except Exception as e:
				print(f"Error during item release: {e}")
				self.itemConnectedToRobot = False

	def JoinRobotAndItem(self, item_handle):
		"""
		Attaches an item to the robot for collection.
		
		Args:
			item_handle: Handle of the item to collect
			
		Returns:
			bool: True if successful, False otherwise
		"""
		try:
			if item_handle == -1:
				print("Invalid item handle")
				return False
				
			# Verify the handle is valid
			try:
				self.sim.getObjectPosition(item_handle, -1)
			except Exception:
				print(f"Invalid object handle: {item_handle}")
				return False
			
			# Attach item to robot
			self.sim.setObjectPosition(item_handle, self.robotHandle, [0.13, 0, -0.06])
			self.sim.setObjectParent(item_handle, self.robotHandle, True)
			
			# Store the item handle
			self.heldItemHandle = item_handle
			
			print(f"Collected item with handle: {item_handle}")
			return True
			
		except Exception as e:
			print(f"Error collecting item: {e}")
			return False

	def CollectItem(self, closest_picking_station=False):
		"""
		Attempts to collect items from picking stations.
		
		Args:
			closest_picking_station: If True, collects from the closest picking station
			
		Returns:
			tuple: (success, station_number) where:
				- success: True if item was collected
				- station_number: Which station the item was collected from (1-3), or None if failed
		"""
		if closest_picking_station:
			closest_distance = float('inf')
			closest_station = None
			closest_handle = None
			
			# Find the closest item at picking stations
			for station_index in range(3):
				item_handle = self.pickingStationItemHandles[station_index]
				
				if item_handle is not None:
					try:
						item_position = self.sim.getObjectPosition(item_handle, self.collectorForceSensorHandle)
						distance = math.sqrt(sum(pos**2 for pos in item_position))

						if distance < closest_distance:
							closest_distance = distance
							closest_station = station_index
							closest_handle = item_handle
							
					except Exception:
						self.pickingStationItemHandles[station_index] = None
			
			# Attempt to collect the closest item
			if closest_station is not None and closest_distance < self.robotParameters.maxCollectDistance:
				if not self.itemConnectedToRobot:
					if self.JoinRobotAndItem(closest_handle):
						self.itemConnectedToRobot = True
						self.pickingStationItemHandles[closest_station] = None
						print(f"Collected item from picking station {closest_station + 1}! (Distance: {closest_distance:.3f}m)")
						return True, closest_station + 1
					else:
						print(f"Error collecting item from picking station {closest_station + 1}")
						return False, None
				else:
					print("Robot already carrying an item")
					return False, None
			else:
				if closest_station is not None:
					print(f"Closest item is too far away ({closest_distance:.3f}m > {self.robotParameters.maxCollectDistance:.3f}m)")
				else:
					print("No items found at any picking stations")
				return False, None

		# Default behavior: try to collect from any nearby picking station
		for station_index in range(3):
			item_handle = self.pickingStationItemHandles[station_index]
			
			if item_handle is not None:
				try:
					item_position = self.sim.getObjectPosition(item_handle, self.collectorForceSensorHandle)
					distance = math.sqrt(sum(pos**2 for pos in item_position))
					
					if distance < self.robotParameters.maxCollectDistance and not self.itemConnectedToRobot:
						if self.JoinRobotAndItem(item_handle):
							self.itemConnectedToRobot = True
							self.pickingStationItemHandles[station_index] = None
							print(f"Collected item from picking station {station_index + 1}! (Distance: {distance:.3f}m)")
							return True, station_index + 1
						else:
							print(f"Error collecting item from picking station {station_index + 1}")
							return False, None
				except Exception:
					self.pickingStationItemHandles[station_index] = None
		
		return False, None

	def GetItemBayHeight(self, itemPosition):
		"""Helper function to determine which shelf height level an item is at."""
		if itemPosition[2] < 0.1:
			return 0
		elif itemPosition[2] < 0.2:
			return 1
		else:
			return 2

	def DropItemInClosestShelfBay(self, max_drop_distance=0.5):
		"""
		Drops an item in the closest empty shelf bay.
		
		Args:
			max_drop_distance: Maximum distance to consider a shelf bay for dropping (in meters)
			
		Returns:
			tuple: (success, shelf_info) where:
				- success: True if item was dropped successfully
				- shelf_info: Dictionary with shelf information, or None if failed
		"""
		if not self.itemConnectedToRobot:
			print("No item to drop")
			return False, None
			
		if self.robotPose is None:
			print("Robot position unknown")
			return False, None
			
		closest_bay = None
		min_distance = float('inf')
		
		# Check all shelf bays to find the closest empty one
		for shelf in range(6):
			for x in range(4):
				for y in range(3):
					# Check if this bay is empty
					if np.isnan(self.itemPositions[shelf, x, y]).any():
						bay_handle = self.bayHandles[shelf, x, y]
						
						if bay_handle is not None:
							try:
								position = self.sim.getObjectPosition(bay_handle, self.collectorForceSensorHandle)
								distance = math.sqrt(sum(pos**2 for pos in position))

								if distance < min_distance:
									min_distance = distance
									closest_bay = {
										'shelf': shelf,
										'x': x, 
										'y': y,
										'distance': distance,
										'handle': bay_handle
									}
							except Exception:
								continue
		
		if closest_bay is None:
			print(f"No empty shelf bay found within {max_drop_distance}m")
			return False, None
			
		# Drop the item
		try:
			print(f"Dropping item at shelf {closest_bay['shelf']}, bay [{closest_bay['x']},{closest_bay['y']}] (distance: {closest_bay['distance']:.2f}m)")
			self.DropItem()
			print(f"Successfully dropped item in shelf {closest_bay['shelf']}")
			return True, closest_bay
			
		except Exception as e:
			print(f"Error dropping item: {e}")
			return False, None

	def UpdateObjectPositions(self):
		"""
		Updates the positions of all objects in the simulation.
		This should be called in every loop to get accurate object detection.
		
		Returns:
			tuple: (robotPose, itemPositions, obstaclePositions) for debugging purposes
		"""
		# Get current object positions from CoppeliaSim
		self.GetObjectPositions()
		
		# Update item collection state
		self.UpdateItem()

		return self.robotPose, self.itemPositions, self.obstaclePositions
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
		Robot sensors (vision sensor, object detector, proximity sensor, collector force
		sensor, rear motors) are optional and resolved best-effort using candidate paths that
		cover both the new and legacy scene layouts. Legacy 2025 warehouse objects (picking
		stations, obstacles, row markers, shelves, item templates) are resolved best-effort
		only and never abort startup if missing.
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
		self.wallPostTemplateHandle = self._resolve_first_available(['/wall_post'], 'wall_post template', required=True)
		self.victimTemplateHandle = self._resolve_first_available(['/victim'], 'victim template', required=True)
		if self.mazeWallTemplateHandle is None or self.wallPostTemplateHandle is None or self.victimTemplateHandle is None:
			sys.exit(-1)

		# --- Optional: robot sensors (candidate paths cover both the new and legacy scene layouts) ---
		self.GetCollectorForceSensorHandle()
		self.GetCameraHandle()
		self.GetObjectDetectorHandle()
		self.getProximityhandle()

		# --- Legacy (2025 warehouse challenge) objects: best-effort only, never abort startup ---
		# Picking stations, item templates, row markers and shelves/bays no longer exist in the
		# search and rescue maze scene, so they are not resolved here. Obstacles are kept since
		# they may still be used (e.g. for future rubble/hazard objects).
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

	# Get ZMQ CollectorForceSensor Handle
	def GetCollectorForceSensorHandle(self):
		self.collectorForceSensorHandle = self._resolve_first_available(
			['/Robot/CollectorForceSensor'], 'CollectorForceSensor', required=False)
		return 0 if self.collectorForceSensorHandle is not None else -1

			
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

	# Get ZMQ Picking Station Handles (legacy 2025 warehouse challenge; not used in this phase)
	def GetPickingStationHandle(self):
		"""Best-effort resolution of legacy picking station handles. Missing objects are expected in this phase."""
		missing = []

		self.pickingStationHandle = self._try_get_object('/Picking_station')
		if self.pickingStationHandle is None:
			missing.append('/Picking_station')

		for i in range(3):
			path = f'/Picking_station_{i+1}'
			handle = self._try_get_object(path)
			self.pickingStationMarkerHandles[i] = handle
			if handle is None:
				missing.append(path)

		if missing:
			print(f"Legacy picking station objects not found (expected in this phase): {', '.join(missing)}")
		return 0

	# Get ZMQ item Template Handles (legacy 2025 warehouse challenge; not used in this phase)
	def GetItemTemplateHandles(self):
		"""Best-effort resolution of legacy warehouse item templates. Missing objects are expected in this phase."""
		error_codes = []
		missing = []
		for index, name in enumerate(["BOWL","MUG","BOTTLE","SOCCER_BALL","RUBIKS_CUBE","CEREAL_BOX"]):
			handle = self._try_get_object(f'/{name}')
			self.itemTemplateHandles[index] = handle
			if handle is not None:
				error_codes.append(0)
			else:
				error_codes.append(-1)
				missing.append(name)
		if missing:
			print(f"Legacy item templates not found (expected in this phase): {', '.join(missing)}")
		return error_codes

	# Get ZMQ Obstacle Handles (legacy 2025 warehouse challenge; not used in this phase)
	def GetObstacleHandles(self):
		"""Best-effort resolution of legacy obstacle handles. Missing objects are expected in this phase."""
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
			print(f"Legacy obstacles not found (expected in this phase): {', '.join(missing)}")
		return tuple(error_codes)
	
	# Get ZMQ Row marker handles (legacy 2025 warehouse challenge; not used in this phase)
	def GetRowMarkerHandles(self):
		"""Best-effort resolution of legacy row marker handles. Missing objects are expected in this phase."""
		error_codes = [0, 0, 0]
		missing = []
		for i in range(3):
			path = f'/row_marker{i+1}'
			handle = self._try_get_object(path)
			self.rowMarkerHandles[i] = handle
			if handle is None:
				error_codes[i] = -1
				missing.append(path)
		if missing:
			print(f"Legacy row markers not found (expected in this phase): {', '.join(missing)}")
		return tuple(error_codes)


	# Get ZMQ shelf handles (legacy 2025 warehouse challenge; not used in this phase)
	def getShelfHandles(self):
		"""Best-effort resolution of legacy shelf handles. Missing objects are expected in this phase."""
		errorCodes = [0]*6
		missing = []
		for i in range(6):
			path = f'/Shelf{i}'
			handle = self._try_get_object(path)
			self.shelfHandles[i] = handle
			if handle is None:
				errorCodes[i] = -1
				missing.append(path)
		if missing:
			print(f"Legacy shelves not found (expected in this phase): {', '.join(missing)}")
		return tuple(errorCodes)

	# Get ZMQ proximity sensor handle.
	def getProximityhandle(self):
		self.proximityHandle = self._resolve_first_available(
			['/Robot/VisionSensor/Proximity_sensor', '/Proximity_sensor'], 'Proximity_sensor', required=False)
		return 0 if self.proximityHandle is not None else -1

	# Get ZMQ bay handles for each shelf (legacy 2025 warehouse challenge; not used in this phase)
	def getBayHandles(self):
		"""Best-effort resolution of legacy shelf bay handles. Missing objects are expected in this phase."""
		error_codes = []
		foundCount = 0
		totalCount = 0
		for shelf in range(6):  # 6 shelves (0-5)
			for x in range(4):  # 4 x positions (0-3)
				for y in range(3):  # 3 y positions/heights (0-2)
					totalCount += 1
					bay_name = '/Shelf%d/Bay%d%d' % (shelf, x, y)
					handle = self._try_get_object(bay_name)
					self.bayHandles[shelf, x, y] = handle
					if handle is not None:
						foundCount += 1
						error_codes.append(0)
					else:
						error_codes.append(-1)
		print(f"Legacy shelf bays found: {foundCount}/{totalCount} (expected 0 in this phase)")
		return error_codes
	
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
		then generates the wall posts, internal walls and victims, and (optionally) places
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

		postCount = self._generate_wall_posts()
		wallCount = self._generate_internal_walls()
		victimCount = self._generate_victims()

		self._park_templates_outside_playable_area()

		if self.sceneParameters.robotStartingPosition is not None:
			x, y, theta = self.sceneParameters.robotStartingPosition
			self._set_robot_pose([x, y, theta])
		elif self.sceneParameters.placeRobotAtBase:
			self._place_robot_at_base()

		self._log_table_wall_positions()
		self._print_scene_summary(postCount, wallCount, victimCount, removedCount)

		# Legacy 2025 warehouse extra - safe no-op when no legacy obstacles exist in the scene
		self._legacy_set_obstacle_positions()

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

	def _cache_template_geometry(self):
		"""Cache template bounding boxes/orientations before templates are copied from or moved."""
		self.mazeWallTemplateSize = self._get_shape_bbox_size(self.mazeWallTemplateHandle)
		self.mazeWallTemplateOrientation = self.sim.getObjectOrientation(self.mazeWallTemplateHandle, -1)
		self.wallPostTemplateSize = self._get_shape_bbox_size(self.wallPostTemplateHandle)
		self.wallPostTemplateOrientation = self.sim.getObjectOrientation(self.wallPostTemplateHandle, -1)
		try:
			self.victimTemplateSize = self._get_shape_bbox_size(self.victimTemplateHandle)
		except ValueError as e:
			print(f"Warning: victim template is not a simple shape ({e}); using zero height for placement.")
			self.victimTemplateSize = (0.0, 0.0, 0.0)
		self.victimTemplateOrientation = self.sim.getObjectOrientation(self.victimTemplateHandle, -1)

		# The maze_wall template's local X/Y/Z axes don't necessarily line up with "length/height
		# /thickness" - e.g. it may be authored lying flat then rotated upright with a fixed roll.
		# Find which local axis is vertical (under the template's own base orientation) so we scale
		# and reorient the correct axis for each generated wall segment.
		R = self._build_rotation_matrix(self.mazeWallTemplateOrientation)
		worldZComponents = [abs(R[2][0]), abs(R[2][1]), abs(R[2][2])]
		heightAxisIndex = worldZComponents.index(max(worldZComponents))
		remainingAxes = [i for i in range(3) if i != heightAxisIndex]
		if self.mazeWallTemplateSize[remainingAxes[0]] >= self.mazeWallTemplateSize[remainingAxes[1]]:
			lengthAxisIndex = remainingAxes[0]
		else:
			lengthAxisIndex = remainingAxes[1]
		self.mazeWallHeightAxisIndex = heightAxisIndex
		self.mazeWallLengthAxisIndex = lengthAxisIndex
		self.mazeWallWorldHeight = self.mazeWallTemplateSize[heightAxisIndex]
		# World-frame yaw that the template's own length axis already points at, under its base
		# orientation - needed so we rotate it by exactly the right extra amount for each segment.
		self.mazeWallLengthAxisBaseYaw = math.atan2(R[1][lengthAxisIndex], R[0][lengthAxisIndex])

		axisNames = ['X', 'Y', 'Z']
		print(f"maze_wall template size (x,y,z): {self.mazeWallTemplateSize}, orientation: {self.mazeWallTemplateOrientation}")
		print(f"maze_wall detected height axis: local {axisNames[heightAxisIndex]} ({self.mazeWallWorldHeight:.4f} m), "
			  f"length axis: local {axisNames[lengthAxisIndex]} ({self.mazeWallTemplateSize[lengthAxisIndex]:.4f} m)")
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

	def _create_wall_between_grid_points(self, startPoint, endPoint, index):
		"""
		Create a single maze_wall copy spanning the given grid intersections.
		Scales only the template's long axis to match the segment length, and orients the
		wall so that axis lies along the segment - this supports horizontal, vertical and
		diagonal walls with a single code path.
		"""
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
		zPos = self.floorTopZ + self.mazeWallWorldHeight / 2.0

		newHandle = self.sim.copyPasteObjects([self.mazeWallTemplateHandle], 0)[0]

		scaleFactor = length / self.mazeWallTemplateSize[self.mazeWallLengthAxisIndex]
		scaleXYZ = [1.0, 1.0, 1.0]
		scaleXYZ[self.mazeWallLengthAxisIndex] = scaleFactor
		self.sim.scaleObject(newHandle, scaleXYZ[0], scaleXYZ[1], scaleXYZ[2], 0)

		# Compose the extra yaw needed with the template's own natural tilt via a temporary
		# reference dummy, rather than hand-rolling Euler/matrix composition: setting an
		# object's position/orientation "relative to" another handle works regardless of
		# actual parent-child hierarchy, so no reparenting is required.
		deltaYaw = segmentYaw - self.mazeWallLengthAxisBaseYaw
		helperDummy = self.sim.createDummy(0.01)
		self.sim.setObjectPosition(helperDummy, -1, [midX, midY, zPos])
		self.sim.setObjectOrientation(helperDummy, -1, [0, 0, deltaYaw])
		self.sim.setObjectPosition(newHandle, helperDummy, [0, 0, 0])
		self.sim.setObjectOrientation(newHandle, helperDummy, list(self.mazeWallTemplateOrientation))
		self.sim.removeObject(helperDummy)

		self.sim.setObjectAlias(newHandle, f"EGB320_GEN_WALL_{index:03d}")
		self.sim.setObjectParent(newHandle, self.generatedSceneRootHandle, True)

		self.generatedMazeWallHandles.append(newHandle)
		return newHandle

	def _generate_internal_walls(self):
		"""Create one maze_wall copy for every segment in EXAMPLE_MAZE_SEGMENTS (40 walls)."""
		count = 0
		for index, (startPoint, endPoint) in enumerate(EXAMPLE_MAZE_SEGMENTS):
			self._create_wall_between_grid_points(startPoint, endPoint, index)
			count += 1
		return count

	def _generate_victims(self):
		"""Create one /victim copy at each configured victim cell centre."""
		victimHeight = self.victimTemplateSize[2]
		clearance = 0.001  # small gap to avoid initial mesh penetration with the floor
		count = 0
		for label, cell in self.sceneParameters.victimCells.items():
			column, row = cell
			x, y = self._cell_center_to_world(column, row)
			newHandle = self.sim.copyPasteObjects([self.victimTemplateHandle], 0)[0]
			zPos = self.floorTopZ + victimHeight / 2.0 + clearance
			self.sim.setObjectPosition(newHandle, -1, [x, y, zPos])
			self.sim.setObjectOrientation(newHandle, -1, list(self.victimTemplateOrientation))
			self.sim.setObjectAlias(newHandle, f"EGB320_GEN_VICTIM_{label}")
			self.sim.setObjectParent(newHandle, self.generatedSceneRootHandle, True)
			self.victimHandles[label] = newHandle
			self.victimPositions[label] = [x, y, zPos]
			count += 1
		return count

	def _park_templates_outside_playable_area(self):
		"""Move the maze_wall, wall_post and victim templates outside the playable area (kept for future regeneration)."""
		parkingSpots = {
			self.mazeWallTemplateHandle: (-3.0, -3.0),
			self.wallPostTemplateHandle: (-3.0, -3.5),
			self.victimTemplateHandle: (-3.0, -4.0),
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
			for childHandle in childHandles:
				if childHandle == rootHandle:
					continue
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
		self.generatedWallPostHandles = []
		self.victimHandles = {}
		self.victimPositions = {}

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
		print(f"Internal walls created: {wallCount}")
		print(f"Victims created: {victimCount}")
		print(f"Base cell: {tuple(self.sceneParameters.baseCell)}")
		print(f"Victim cells: {victimCellsText}")
		print(f"Objects removed during cleanup: {removedCount}")

	def _legacy_set_obstacle_positions(self):
		"""Legacy 2025 warehouse-challenge obstacle placement. No-op when no legacy obstacles exist in the scene."""
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
				print(f"Warning: error setting legacy obstacle {index} position: {e}")

	def _legacy_set_picking_station_contents(self):
		"""Legacy 2025 warehouse-challenge picking station item placement. No-op when no legacy picking stations exist."""
		if not any(h is not None for h in self.pickingStationMarkerHandles):
			return
		self.SetPickingStationContents()

	def SetPickingStationContents(self):
		"""Place items at picking stations based on sceneParameters.pickingStationContents"""
		print('Placing items at picking stations...')
		
		for station_index in range(3):
			item_type = self.sceneParameters.pickingStationContents[station_index]
			
			if item_type != -1 and 0 <= item_type <= 5:  # Valid item type
				station_handle = self.pickingStationMarkerHandles[station_index]
				
				if station_handle is not None:
					try:
						# Get the position of the picking station
						station_position = self.sim.getObjectPosition(station_handle, -1)
						
						# Place item slightly above the picking station surface
						item_position = [station_position[0], station_position[1], station_position[2]+0.01]
						
						# Copy the item template to this position
						item_names = ["BOWL", "MUG", "BOTTLE", "SOCCER_BALL", "RUBIKS_CUBE", "CEREAL_BOX"]
						template_handle = self.itemTemplateHandles[item_type]
						
						if template_handle is not None:
							# Copy the item from template
							new_item_handle = self.sim.copyPasteObjects([template_handle], 0)[0]
							
							# Store the item handle for collection purposes
							self.pickingStationItemHandles[station_index] = new_item_handle
							
							# Position the item at the picking station
							self.sim.setObjectPosition(new_item_handle, -1, item_position)
							
							print(f'Placed {item_names[item_type]} at picking station {station_index + 1}')
						else:
							print(f'Warning: Template for {item_names[item_type]} not found')
							
					except Exception as e:
						print(f'Error placing item at picking station {station_index + 1}: {e}')
				else:
					print(f'Warning: Picking station {station_index + 1} handle not found')

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
		if self.robotPose != None:
			print("Robot 2D Pose (x,y,theta): %0.4f, %0.4f, %0.4f"%(self.robotPose[0], self.robotPose[1], self.robotPose[2]))

		if self.cameraPose != None:
			print("Camera 3D Pose (x,y,z,roll,pitch,yaw): %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f"%(self.cameraPose[0], self.cameraPose[1], self.cameraPose[2], self.cameraPose[3], self.cameraPose[4], self.cameraPose[5]))
		
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			itemPosition = self.itemPositions[shelf,x,y]
			if np.all(np.isnan(itemPosition)) == False:
				print("item from bay [%d,%d,%d] Position (x,y,z): %0.4f, %0.4f, %0.4f"%(shelf,x,y, itemPosition[0], itemPosition[1], itemPosition[2]))
			
		if self.packingBayPosition != None:
			print("PackingBay Position (x,y,z): %0.4f, %0.4f, %0.4f"%(self.packingBayPosition[0], self.packingBayPosition[1], self.packingBayPosition[2]))
			
		for index, obstacle in enumerate(self.obstaclePositions):
			if obstacle != None:
				print("Obstacle %d Position (x,y,z): %0.4f, %0.4f, %0.4f"%(index, obstacle[0], obstacle[1], obstacle[2]))

	# Gets the pose/position in the global coordinate frame of all the objects in the scene.
	# Stores them in class variables. Variables will be set to none if could not be updated
	def GetObjectPositions(self):
		# Set camera pose and object position to None so can check in an error occurred
		self.robotPose = None
		self.cameraPose = None
		# self.itemPositions = [None]*len(self.itemHandles)
		self.pickingStationPosition = None
		self.obstaclePositions = [None, None, None]

		# GET 2D ROBOT POSE
		try:
			robotPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			robotOrientation = self.sim.getObjectOrientation(self.robotHandle, -1)
			self.robotPose = [robotPosition[0], robotPosition[1], robotPosition[1], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
		except Exception as e:
			print(f"Error getting robot pose: {e}")

		# GET 3D CAMERA POSE
		try:
			cameraPosition = self.sim.getObjectPosition(self.cameraHandle, -1)
			self.cameraPose = [cameraPosition[0], cameraPosition[1], cameraPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]]
		except Exception as e:
			print(f"Error getting camera pose: {e}")
		

		# GET POSITION OF EACH OBJECT
		# for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
		# 	handle = self.itemHandles[shelf,x,y]
		# 	try:
		# 		itemPosition = self.sim.getObjectPosition(handle, -1)
		# 		self.itemPositions[shelf,x,y] = itemPosition
		# 	except Exception as e:
		# 		print(f"Error getting item position for shelf {shelf}, position ({x},{y}): {e}")

		# packingBay position (legacy; no-op when no legacy picking station exists in the scene)
		if self.pickingStationHandle is not None:
			try:
				self.pickingStationPosition = self.sim.getObjectPosition(self.pickingStationHandle, -1)
			except Exception as e:
				print(f"Error getting picking station position: {e}")

		# obstacle positions (legacy; no-op for obstacles that don't exist in the scene)
		obstaclePositions = [None, None, None]
		for index, obs in enumerate(self.obstaclePositions):
			if self.obstacleHandles[index] is None:
				continue
			try:
				obstaclePositions[index] = self.sim.getObjectPosition(self.obstacleHandles[index], -1)
				self.obstaclePositions[index] = obstaclePositions[index]
			except Exception as e:
				print(f"Error getting obstacle position {index}: {e}")

		# row marker positions (legacy; row markers no longer exist in the search and rescue scene)
		rowMarkerPositions = [None,None,None]
		for index, rowMarker in enumerate(self.rowMarkerPositions):
			if self.rowMarkerHandles[index] is None:
				continue
			try:
				rowMarkerPositions[index] = self.sim.getObjectPosition(self.rowMarkerHandles[index], -1)
				self.rowMarkerPositions[index] = rowMarkerPositions[index]
			except Exception as e:
				print(f"Error getting row marker position {index}: {e}")

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


	# Update the item
	def UpdateItem(self):
		for shelf,x,y in [(s,x,y) for s in range(6) for x in range(4) for y in range(3)]:
			itemPosition = self.itemPositions[shelf,x,y]
		
			if np.all(np.isnan(itemPosition)) == False:

				itemDist = self.CollectorToItemDistance(itemPosition)


				if self.itemConnectedToRobot == True:
					# random chance to disconnect
					if np.random.rand() > self.robotParameters.collectorQuality:
						# terminate connection between item and robot to simulate collector
						try:
							self.sim.callScriptFunction('RobotReleaseItem', self.scriptHandle, [], [], [], "")
							self.itemConnectedToRobot = False
						except Exception as e:
							print(f"Error calling RobotReleaseItem script function: {e}")

				elif itemDist != None and itemDist > 0.03:
					self.itemConnectedToRobot = False

	
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

	# Gets the range and bearing to all shelves from the camera position
	def GetShelfRangeBearing(self):
		"""
		Calculate range and bearing to all shelves from the camera position.
		
		Returns:
			list: A list of [range, bearing] pairs for each shelf (6 shelves total).
				  Returns None for shelves that cannot be detected or don't exist.
		"""
		shelfRB = [None] * 6  # Initialize list for 6 shelves
		
		if self.cameraPose is None:
			return shelfRB
			
		cameraPose2D = [self.cameraPose[0], self.cameraPose[1], self.cameraPose[5]]
		
		for shelf_index in range(6):
			shelf_handle = self.shelfHandles[shelf_index]
			
			if shelf_handle is not None:
				try:
					# Get the shelf position
					shelf_position = self.sim.getObjectPosition(shelf_handle, -1)
					
					# Calculate range and bearing from camera to shelf
					_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2D, shelf_position)
					
					# Check if shelf is within detection range and field of view
					if _range < self.robotParameters.maxShelfDetectionDistance:
						# Check if the shelf is within the camera's field of view
						if abs(_bearing) < self.robotParameters.cameraPerspectiveAngle / 2:
							shelfRB[shelf_index] = [_range, _bearing]
				except Exception as e:
					# If we can't get the shelf position, leave it as None
					continue
		
		return shelfRB


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
	"""Parameters for configuring the warehouse robot"""
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
		self.maxItemDetectionDistance = 1.0      # max distance to detect items in m
		self.maxPickingStationDetectionDistance = 2.5  # max distance to detect picking station in m
		self.maxPickingStationMarkersDetectionDistance = 2.5  # max distance to detect picking station markers in m
		self.maxObstacleDetectionDistance = 1.5  # max distance to detect obstacles in m
		self.maxRowMarkerDetectionDistance = 2.5  # max distance to detect row markers in m
		self.maxShelfDetectionDistance = 2.0     # max distance to detect shelves in m
		
		# Collector Parameters
		self.collectorQuality = 1.0      # collector quality from 0 to 1
		self.maxCollectDistance = 0.15   # max distance for collection in m
		
		# Simulation Parameters
		self.sync = False  # synchronous mode (deprecated with ZMQ Remote API)


class SceneParameters(object):
	"""
	Parameters for configuring the EGB320 search and rescue maze scene (2026).

	Legacy 2025 warehouse-challenge fields are retained for backward compatibility but are
	not used by the maze generation introduced in this phase.
	"""
	def __init__(self):
		# --- Search and rescue maze parameters (2026) ---
		self.mazeRows = 7
		self.mazeColumns = 7
		self.mazeCellSize = 0.280  # metres
		self.mazeOriginXY = None   # None = use the centre of /floor as the maze origin
		self.autoGenerateMaze = True
		self.clearGeneratedMaze = True
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

		# --- Legacy warehouse-challenge settings (2025 and earlier; unused in this phase) ---
		# Picking station contents [station]. Set to -1 to leave empty.
		# Index 0 = picking station 1, Index 1 = picking station 2, Index 2 = picking station 3
		self.pickingStationContents = [-1, -1, -1]

		# Bay contents [shelf][x][y]. Set to -1 for empty bays.
		# shelf: 0-5, x: 0-3, y: 0-2 (height levels)
		self.bayContents = np.full((6, 4, 3), -1, dtype=int)

		# Obstacle starting positions [x, y] in metres.
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


