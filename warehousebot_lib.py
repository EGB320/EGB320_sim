# Import required Python modules
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import math
import numpy as np
import sys
from enum import IntEnum

"""
EGB320 Warehouse Robot Library

This library provides a Python interface for controlling a warehouse robot in CoppeliaSim.

MAIN FUNCTIONS:
===========================

Robot Control:
- StartSimulator() / StopSimulator()    : Start/stop the simulation
- SetTargetVelocities(x_dot, theta_dot) : Set robot movement velocities
- UpdateObjectPositions()               : Update object positions (call this every loop!)

Object Detection:
- GetDetectedObjects(objects)           : Get range/bearing to all detected objects  
- GetCameraImage()                      : Get camera image for computer vision
- GetDetectedWallPoints()               : Get range/bearing to visible walls

Item Collection:
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
robot.StartSimulator()

# Main control loop
while True:
    # Move robot
    robot.SetTargetVelocities(0.1, 0.0)  # Move forward at 0.1 m/s
    
    # Get object detections
    objects = robot.GetDetectedObjects([warehouseObjects.items, warehouseObjects.obstacles])
    items, _, obstacles, _, _, _ = objects
    
    # Update positions (important!)
    robot.UpdateObjectPositions()
    
    # Collect items if nearby
    success, station = robot.CollectItem(closest_picking_station=True)
    
    # Drop items at shelves
    if robot.itemCollected():
        robot.DropItemInClosestShelfBay()

For more examples, see EGB320_CoppeliaSim_Example.py
"""






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
	pickingStation = 9
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
		self.packingStationHandle = None
		self.pickingStationHandles = [None, None, None]
		self.pickingStationItemHandles = [None, None, None]
		self.rowMarkerHandles = [None,None,None]
		self.shelfHandles = [None]*6
		self.bayHandles = np.full((6,4,3), None, dtype=object)
		self.proximityHandle = None

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
		self.packingStationPosition = None
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
		"""
		print('Starting CoppeliaSim simulation...')
		
		try:
			if self.robotParameters.sync:
				print('Setting synchronous mode (may cause issues - consider setting sync=False)')
				self.sim.setStepping(True)
			
			self.sim.startSimulation()
			print('CoppeliaSim simulation started successfully.')
		except Exception as e:
			print(f'Error starting simulation: {e}')
			print('Try starting the simulation manually by pressing Play in CoppeliaSim.')
			sys.exit(-1)
		
		print('Setting up scene...')
		self.SetScene()
		
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
		packingStationRangeBearing = None
		obstaclesRangeBearing = None
		rowMarkerRangeBearing = [None,None,None]
		shelfRangeBearing = [None]*6
		pickingStationRangeBearing = [None, None, None]

		# Default to detecting all objects if none specified
		if objects is None:
			objects = [warehouseObjects.items, warehouseObjects.shelves, warehouseObjects.row_markers,
					  warehouseObjects.obstacles, warehouseObjects.pickingStation, warehouseObjects.PickingStationMarkers]

		# Check if camera pose is available
		if self.cameraPose is None:
			return itemRangeBearing, packingStationRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing, pickingStationRangeBearing

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
						station_handle = self.pickingStationHandles[station_index]
						
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
				if self.packingStationPosition is not None:
					packingStationRangeBearing = self._process_single_object_detection(
						self.packingStationPosition, 9, objectsDetected, self.robotParameters.maxPackingBayDetectionDistance)

			# Check for row markers (indices 10-12)
			if warehouseObjects.row_markers in objects:
				rowMarkerRangeBearing = self._process_multiple_object_detection(
					self.rowMarkerPositions, 10, objectsDetected, self.robotParameters.maxRowMarkerDetectionDistance)

			# Check for individual picking stations (indices 19-21)
			if warehouseObjects.PickingStationMarkers in objects:
				for station_index in range(3):
					if self._is_object_detected(objectsDetected, 19 + station_index):
						station_handle = self.pickingStationHandles[station_index]
						if station_handle is not None:
							try:
								station_position = self.sim.getObjectPosition(station_handle, -1)
								result = self._process_single_object_detection(
									station_position, 19 + station_index, objectsDetected, 
									self.robotParameters.maxPackingBayDetectionDistance)
								if result is not None:
									pickingStationRangeBearing[station_index] = result
							except Exception:
								pass

		return itemRangeBearing, packingStationRangeBearing, obstaclesRangeBearing, rowMarkerRangeBearing, shelfRangeBearing, pickingStationRangeBearing


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

	def GetCOPPELIAObjectHandles(self):
		"""Get handles to all objects in the CoppeliaSim scene."""
		# Get essential object handles
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			print('Failed to get Robot object handle.')
			sys.exit(-1)

		errorCode = self.GetScriptHandle()
		if errorCode != 0:
			print('Failed to get Script handle.')
			sys.exit(-1)

		errorCode = self.GetCameraHandle()
		if errorCode != 0:
			print('Failed to get Camera handle.')
			sys.exit(-1)

		errorCode = self.GetObjectDetectorHandle()
		if errorCode != 0:
			print('Failed to get Object Detector handle.')
			sys.exit(-1)

		errorCode = self.GetCollectorForceSensorHandle()
		if errorCode != 0:
			print('Failed to get Collector Force Sensor handle.')
			sys.exit(-1)

		# Get motor handles
		errorCode1, errorCode2, errorCode3, errorCode4 = self.GetMotorHandles()
		if errorCode1 != 0 or errorCode2 != 0:
			print('Failed to get Motor handles.')
			sys.exit(-1)
		elif errorCode3 != 0 or errorCode4 != 0:
			print("Warning: Could not get rear wheel motors (normal for some robot models)")

		# Get scene object handles
		packingStationErrorCode = self.GetPickingStationHandle()
		if packingStationErrorCode != 0:
			print('Failed to get picking station handles.')
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetObstacleHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get obstacle handles.')
			sys.exit(-1)

		errorCode1, errorCode2, errorCode3 = self.GetRowMarkerHandles()
		if errorCode1 != 0 or errorCode2 != 0 or errorCode3 != 0:
			print('Failed to get row marker handles.')
			sys.exit(-1)
		
		errorCodes = self.GetItemTemplateHandles()
		if any([code != 0 for code in errorCodes]):
			print('Failed to get item template handles.')
			sys.exit(-1)
			
		errorCodes = self.getShelfHandles()
		if any([code != 0 for code in errorCodes]):
			print('Failed to get shelf handles.')
			sys.exit(-1)
	
		errorCode = self.getProximityhandle()
		if errorCode != 0:
			print('Failed to get proximity sensor handle.')
			sys.exit(-1)
		
		errorCodes = self.getBayHandles()
		if any([code != 0 for code in errorCodes]):
			print('Failed to get bay handles.')
			sys.exit(-1)
	
	############################################
	####### COPPELIA OBJECT HANDLE FUNCTIONS #######
	############################################
	# These functions are called by the GetCOPPELIAObjectHandles function

	# Get COPPELIA Robot Handle
	def GetRobotHandle(self):
		try:
			self.robotHandle = self.sim.getObject('/Robot')
			return 0
		except Exception as e:
			print(f"Error getting robot handle: {e}")
			return -1

	# Get Script Handle (attached to Robot object)
	def GetScriptHandle(self):
		try:
			# Try common script paths - the script is usually attached as a child script to the Robot
			self.scriptHandle = self.sim.getObject('/Robot')  # Use robot handle for script calls
			return 0
		except Exception as e:
			print(f"Error getting script handle: {e}")
			return -1

	# Get ZMQ Camera Handle
	def GetCameraHandle(self):
		try:
			self.cameraHandle = self.sim.getObject('/VisionSensor')
			return 0
		except Exception as e:
			print(f"Error getting camera handle: {e}")
			return -1

	# Get ZMQ Object Detector Handle
	def GetObjectDetectorHandle(self):
		try:
			self.objectDetectorHandle = self.sim.getObject('/Robot/ObjectDetector')
			return 0
		except Exception as e:
			print(f"Error getting object detector handle: {e}")
			return -1

	# Get ZMQ CollectorForceSensor Handle
	def GetCollectorForceSensorHandle(self):
		try:
			self.collectorForceSensorHandle = self.sim.getObject('/Robot/CollectorForceSensor')
			return 0
		except Exception as e:
			print(f"Error getting collector force sensor handle: {e}")
			return -1

			
	# Get COPPELIA Motor Handles
	# Get ZMQ Motor Handles
	def GetMotorHandles(self):
		errorCode1 = 0
		errorCode2 = 0
		errorCode3 = 0
		errorCode4 = 0

		try:
			if self.robotParameters.driveType == 'differential':
				self.leftMotorHandle = self.sim.getObject('/LeftMotor')
				self.rightMotorHandle = self.sim.getObject('/RightMotor')
				try:
					self.leftRearMotorHandle = self.sim.getObject('/LeftRearMotor')
					self.rightRearMotorHandle = self.sim.getObject('/RightRearMotor')
				except:
					# Some robots may not have rear motors
					self.leftRearMotorHandle = None
					self.rightRearMotorHandle = None
		except Exception as e:
			print(f"Error getting motor handles: {e}")
			errorCode1 = -1
		
		return errorCode1, errorCode2, errorCode3, errorCode4

	# Get ZMQ Picking Station Handles
	def GetPickingStationHandle(self):
		try:
			self.packingStationHandle = self.sim.getObject('/Picking_station')
			
			# Try to get multiple picking station handles
			for i in range(3):
				try:
					station_name = f'/Picking_station_{i+1}'
					self.pickingStationHandles[i] = self.sim.getObject(station_name)
				except Exception:
					# If specific picking station doesn't exist, keep as None
					self.pickingStationHandles[i] = None
			
			return 0
		except Exception as e:
			print(f"Error getting picking station handle: {e}")
			return -1

	# Get ZMQ item Template Handles
	def GetItemTemplateHandles(self):
		error_codes = []
		for index, name in enumerate(["BOWL","MUG","BOTTLE","SOCCER_BALL","RUBIKS_CUBE","CEREAL_BOX"]):
			try:
				handle = self.sim.getObject(f'/{name}')
				error_codes.append(0)
				self.itemTemplateHandles[index] = handle
			except Exception as e:
				print(f"Error getting item template handle for {name}: {e}")
				error_codes.append(-1)
			
		return error_codes

	# Get ZMQ Obstacle Handles
	def GetObstacleHandles(self):
		error_codes = [0, 0, 0]
		try:
			self.obstacleHandles[0] = self.sim.getObject('/Obstacle_0')
		except:
			error_codes[0] = -1
		try:
			self.obstacleHandles[1] = self.sim.getObject('/Obstacle_1')
		except:
			error_codes[1] = -1
		try:
			self.obstacleHandles[2] = self.sim.getObject('/Obstacle_2')
		except:
			error_codes[2] = -1
		return tuple(error_codes)
	
	# Get ZMQ Row marker handles
	def GetRowMarkerHandles(self):
		error_codes = [0, 0, 0]
		try:
			self.rowMarkerHandles[0] = self.sim.getObject('/row_marker1')
		except:
			error_codes[0] = -1
		try:
			self.rowMarkerHandles[1] = self.sim.getObject('/row_marker2')
		except:
			error_codes[1] = -1
		try:
			self.rowMarkerHandles[2] = self.sim.getObject('/row_marker3')
		except:
			error_codes[2] = -1
		return tuple(error_codes)


	# Get ZMQ shelf handles
	def getShelfHandles(self):
		errorCodes = [0]*6
		for i in range(6):
			try:
				self.shelfHandles[i] = self.sim.getObject(f'/Shelf{i}')
			except:
				errorCodes[i] = -1
		return tuple(errorCodes)

	# Get ZMQ proximity sensor handle.
	def getProximityhandle(self):
		try:
			self.proximityHandle = self.sim.getObject('/Proximity_sensor')
			return 0
		except Exception as e:
			print(f"Error getting proximity sensor handle: {e}")
			return -1

	# Get ZMQ bay handles for each shelf
	def getBayHandles(self):
		error_codes = []
		for shelf in range(6):  # 6 shelves (0-5)
			for x in range(4):  # 4 x positions (0-3)
				for y in range(3):  # 3 y positions/heights (0-2)
					try:
						# Bay naming convention: /Shelf{shelf}/Bay{x}{y}
						bay_name = '/Shelf%d/Bay%d%d' % (shelf, x, y)
						self.bayHandles[shelf, x, y] = self.sim.getObject(bay_name)
						error_codes.append(0)
					except Exception as e:
						print("Warning: Could not get handle for %s: %s" % (bay_name, str(e)))
						self.bayHandles[shelf, x, y] = None
						error_codes.append(-1)
		return error_codes
	
	###############################################
	####### ROBOT AND SCENE SETUP FUNCTIONS #######
	###############################################
	# These functions are called within the init function

	# Updates the robot within COPPELIA based on the robot paramters
	def UpdateCOPPELIARobot(self):
		# Set Camera Pose and Orientation
		self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
		self.SetCameraOrientation(self.robotParameters.cameraOrientation)

	# Sets the position of the item, robot and obstacles based on parameters
	def SetScene(self):
		print('Setting up scene objects...')
		
		# Get bay handles for positioning items (ZMQ Remote API doesn't need streaming setup)
		# bayHandles = np.zeros_like(self.sceneParameters.bayContents)
		
		# print('Attempting to get bay handles...')
		# for shelf in range(6):
		# 	for x in range(4):
		# 		for y in range(3):
		# 			bay_path = f"/Shelf{shelf}/Bay{x}{y}"
		# 			try:
		# 				print(f'Getting handle for {bay_path}')
		# 				bayHandles[shelf,x,y] = self.sim.getObject(bay_path)
		# 				print(f'Successfully got handle for {bay_path}')
		# 			except Exception as e:
		# 				print(f"Warning: Could not get bay handle for {bay_path}: {e}")
		# 				print(f"This bay may not exist in the scene - continuing...")
		# 				bayHandles[shelf,x,y] = -1  # Mark as invalid
		
		print('Bay handle retrieval completed.')
		
		# Set obstacle positions
		print('Setting obstacle positions...')
		obstacleHeight = 0.15
		for index, obstaclePosition in enumerate([self.sceneParameters.obstacle0_StartingPosition, self.sceneParameters.obstacle1_StartingPosition, self.sceneParameters.obstacle2_StartingPosition]):
			if obstaclePosition != -1:
				if obstaclePosition != None:
					coppeliaStartingPosition = [obstaclePosition[0], obstaclePosition[1], obstacleHeight/2]
					try:
						print(f'Setting obstacle {index} position to {coppeliaStartingPosition}')
						self.sim.setObjectPosition(self.obstacleHandles[index], -1, coppeliaStartingPosition)
					except Exception as e:
						print(f"Warning: Error setting obstacle {index} position: {e}")
				else:
					try:
						default_position = [2,  -0.3 + (-0.175*index), 0.8125]
						print(f'Setting obstacle {index} to default position {default_position}')
						self.sim.setObjectPosition(self.obstacleHandles[index], -1, default_position)
					except Exception as e:
						print(f"Warning: Error setting default obstacle {index} position: {e}")
		
		# Set picking station contents
		print('Setting picking station contents...')
		self.SetPickingStationContents()
		
		print('Scene setup completed.')
		
		

	def SetPickingStationContents(self):
		"""Place items at picking stations based on sceneParameters.pickingStationContents"""
		print('Placing items at picking stations...')
		
		for station_index in range(3):
			item_type = self.sceneParameters.pickingStationContents[station_index]
			
			if item_type != -1 and 0 <= item_type <= 5:  # Valid item type
				station_handle = self.pickingStationHandles[station_index]
				
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
		self.packingStationPosition = None
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

		# packingBay position
		try:
			packingStationPosition = self.sim.getObjectPosition(self.packingStationHandle, -1)
			self.packingStationPosition = packingStationPosition
		except Exception as e:
			print(f"Error getting picking station position: {e}")

		# obstacle positions
		obstaclePositions = [None, None, None]
		for index, obs in enumerate(self.obstaclePositions):
			try:
				obstaclePositions[index] = self.sim.getObjectPosition(self.obstacleHandles[index], -1)
				self.obstaclePositions[index] = obstaclePositions[index]
			except Exception as e:
				print(f"Error getting obstacle position {index}: {e}")

		# row marker positions
		rowMarkerPositions = [None,None,None]
		for index, rowMarker in enumerate(self.rowMarkerPositions):
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
		self.maxPackingBayDetectionDistance = 2.5  # max distance to detect packing bay in m
		self.maxObstacleDetectionDistance = 1.5  # max distance to detect obstacles in m
		self.maxRowMarkerDetectionDistance = 2.5  # max distance to detect row markers in m
		self.maxShelfDetectionDistance = 2.0     # max distance to detect shelves in m
		
		# Collector Parameters
		self.collectorQuality = 1.0      # collector quality from 0 to 1
		self.maxCollectDistance = 0.15   # max distance for collection in m
		
		# Simulation Parameters
		self.sync = False  # synchronous mode (deprecated with ZMQ Remote API)


class SceneParameters(object):
	"""Parameters for configuring the warehouse scene"""
	def __init__(self):
		# Picking station contents [station]. Set to -1 to leave empty.
		# Index 0 = picking station 1, Index 1 = picking station 2, Index 2 = picking station 3
		self.pickingStationContents = [-1, -1, -1]
		
		# Bay contents [shelf][x][y]. Set to -1 for empty bays.
		# shelf: 0-5, x: 0-3, y: 0-2 (height levels)
		# Not used currently
		self.bayContents = np.full((6, 4, 3), -1, dtype=int)
		
		# Obstacle starting positions [x, y] in metres
		# Set to -1 to use current CoppeliaSim position, None if not wanted in scene
		self.obstacle0_StartingPosition = None
		self.obstacle1_StartingPosition = None  
		self.obstacle2_StartingPosition = None
		
		# Robot starting position [x, y, theta] in metres and radians
		# Set to None to use current CoppeliaSim position
		self.robotStartingPosition = None

