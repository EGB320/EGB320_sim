"""
Python interface for the 2026 EGB320 CoppeliaSim search-and-rescue maze.

Students should start with :class:`MazeBot` and the two example programs. Most code below
the public API section implements scene generation and does not need to be modified.
"""

import math
import random
import time
from enum import IntEnum

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from maze_generation import (
	MazeLayout,
	PRESET_BASE_CELL,
	PRESET_BASE_YAW,
	PRESET_MAZE_SEGMENTS,
	PRESET_VICTIM_CELLS,
	VICTIM_LEVELS,
	generate_random_maze,
	validate_maze_layout,
	validate_victim_count,
)


# Deterministic example maze layout: internal wall segments between grid
# intersections. Each entry is ((startColumn, startRow), (endColumn, endRow)) using the
# grid intersection convention described in SetScene/_grid_point_to_world. Intentionally
# not merged/optimised so each entry corresponds to one generated internal wall copy.
EXAMPLE_MAZE_SEGMENTS = list(PRESET_MAZE_SEGMENTS)

# Small separation used when placing a victim on either the maze floor or robot chassis.
VICTIM_SURFACE_CLEARANCE = 0.001
VICTIM_CARRY_ORIENTATION = (0.0, math.pi / 2.0, 0.0)
VICTIM_RELEASE_FORWARD_OFFSET = 0.15

# Emissive colours used by both the marker materials and robot_object_detector_script.lua.
# They deliberately avoid the green shades used by the three optional obstacle objects.
MARKER_EMISSIVE_COLOURS = {
	'base': (0.0, 0.0, 1.0),
	'victim': (0.0, 1.0, 1.0),
	'rubble_victim': (1.0, 0.0, 1.0),
	'hazard': (1.0, 0.0, 0.0),
}
VICTIM_DETECTOR_COLOUR = (1.0, 1.0, 0.0)
VISUAL_MARKER_COLOUR = (1.0, 1.0, 1.0)
# Detector colour planes and the ObjectDetector sensor are not shown in the editor.
# The robot's Lua script temporarily puts the detector planes on layer 1 only for
# the duration of an explicit detector render, without changing the global layer mask.
VISUAL_MARKER_VISIBILITY_LAYER = 1
DETECTOR_MARKER_HIDDEN_LAYER = 0
DETECTOR_MARKER_FACE_OFFSET = 0.0002

# CoppeliaSim's documented Legacy OpenGL vision-sensor mode is 0. OpenGL3 is an
# optional plugin renderer selected with mode 7. Students choose between these names
# through RobotParameters rather than depending on CoppeliaSim's numeric mode values.
OBJECT_DETECTOR_RENDER_MODES = {
	'legacy': ('Legacy OpenGL', 0),
	'opengl3': ('OpenGL3', 7),
}



# Object types retained for the search-and-rescue challenge.
class MazeObject(IntEnum):
	"""Object categories accepted by :meth:`MazeBot.GetDetectedObjects`."""
	# Obstacle objects
	obstacle0 = 0
	obstacle1 = 1
	obstacle2 = 2

	# Search-and-rescue wall marker types. Values 0-6 match detector packet indices.
	baseStationMarker = 3
	victimMarker = 4
	rubbleVictimMarker = 5
	hazardMarker = 6
	victim = 7
	victimObject = 7

	# Detection groups
	obstacles = 100
	markers = 101
	victims = 102


# Keep the earlier enum name working for existing teaching material.
mazeObjects = MazeObject


################################
##### MAZE BOT CLASS #####
################################

class MazeBot(object):
	"""
	Main class for controlling the search-and-rescue maze robot in CoppeliaSim.
	This class provides robot navigation, sensing, maze generation and victim collection.
	"""
	
	####################################
	#### MAZE BOT INITIALIZATION ###
	####################################

	def __init__(self, robotParameters=None, sceneParameters=None,
			coppelia_server_ip='127.0.0.1', port=23000):
		"""
		Initialize the maze robot connection to CoppeliaSim.
		
		Args:
			robotParameters: Optional RobotParameters configuration. Defaults are used when omitted.
			sceneParameters: Optional SceneParameters configuration. Defaults are used when omitted.
			coppelia_server_ip: IP address of CoppeliaSim server (default: '127.0.0.1')
			port: Port number for ZMQ Remote API (default: 23000)
		"""
		print("Initializing maze robot connection...")
		
		# Store parameters
		self.robotParameters = (
			RobotParameters() if robotParameters is None else robotParameters)
		self.sceneParameters = (
			SceneParameters() if sceneParameters is None else sceneParameters)
		self.port = port

		# Initialize wheel bias for drive system simulation
		self.leftWheelBias = 0
		self.rightWheelBias = 0

		# CoppeliaSim connection variables
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
		self.obstacleHandles = [None, None, None]
		self.distanceSensorHandles = {
			'left': None,
			'front': None,
			'right': None,
		}

		# Search-and-rescue maze scene object handles
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
		self.generatedVisualMarkerHandles = []
		self.generatedDetectorMarkerHandles = []
		self.generatedVictimDetectorHandles = []
		self.victimDetectorHandles = {}
		self.generatedWallPostHandles = []
		self.victimHandles = {}
		self.victimPositions = {}
		self.markerWallPlacements = []
		self.carriedVictimHandle = None
		self.carriedVictimLabel = None
		self.deliveredVictimLabels = set()

		# Cached maze geometry (populated by _get_floor_info/_cache_template_geometry)
		self.floorCenter = None
		self.floorTopZ = None
		self.mazeXMinimum = None
		self.mazeYMaximum = None
		self.wallTemplateGeometry = {}
		self.wallPostTemplateSize = None
		self.wallPostTemplateOrientation = None
		self.victimTemplateSize = None
		self.victimTemplateOrientation = None

		# Optional objects that could not be resolved during handle lookup (diagnostics only)
		self.missingOptionalObjects = []

		# Wheel bias simulation for imperfect drive systems
		if self.robotParameters.driveSystemQuality != 1:
			biasStandardDeviation = (1 - self.robotParameters.driveSystemQuality) * 0.2
			self.leftWheelBias = random.gauss(0.0, biasStandardDeviation)
			self.rightWheelBias = random.gauss(0.0, biasStandardDeviation)

		# Internal object position variables used to calculate sensor detections
		self.cameraPose = None
		self.cameraForwardYaw = None
		self.obstaclePositions = [None, None, None]

		# Local odometry is deliberately derived from wheel encoder counts, never from
		# CoppeliaSim's global robot pose. The last counts form the integration baseline.
		self._odometryPose = [0.0, 0.0, 0.0]
		self._odometryLastEncoderCounts = None

		# Connect to CoppeliaSim
		print("Connecting to CoppeliaSim...")
		self.OpenConnectionToZMQ(coppelia_server_ip, self.port)

		# Get object handles from the simulation
		print("Getting simulation object handles...")
		self.GetCOPPELIAObjectHandles()
		if self.missingOptionalObjects:
			print(f"Optional objects not found: {', '.join(self.missingOptionalObjects)}")

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

		The selected preset or random maze (posts, walls, victims) is generated while
		the simulation is stopped. Existing generated objects are cleared first, so
		starting/stopping/starting again never leaves duplicates in the scene.
		"""
		print('Starting CoppeliaSim simulation...')

		try:
			simState = self.sim.getSimulationState()
			if simState != self.sim.simulation_stopped:
				print('Simulation is currently running - stopping it to (re)generate the scene...')
				self.sim.stopSimulation()
				while self.sim.getSimulationState() != self.sim.simulation_stopped:
					time.sleep(0.05)
				print('Simulation stopped.')

		except Exception as e:
			print(f'Error checking/stopping simulation state: {e}')

		print('Preparing static scene (maze generation) while simulation is stopped...')
		self.SetScene()

		try:
			self.sim.startSimulation()
			print('CoppeliaSim simulation started successfully.')
		except Exception as e:
			raise RuntimeError(
				'Could not start CoppeliaSim. You can also try the Play button '
				'in CoppeliaSim.') from e

		time.sleep(1)
		self.GetObjectPositions()
		# Define the local odometry origin after the robot has been placed and allowed
		# to settle. This samples the encoders but does not alter their public counts.
		self.ResetOdometry()

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

	def _read_object_detector_packet(self):
		"""Return the flat numeric packet produced by the ObjectDetector vision script."""
		if self.objectDetectorHandle is None or self.scriptHandle is None:
			return []
		try:
			# Perform the visibility swap and sensor render atomically inside CoppeliaSim.
			# This avoids changing the editor's global visible-layer mask, which otherwise
			# makes layer 9 and the ObjectDetector view flicker on each API call.
			result, _data, packets = self.sim.callScriptFunction(
				'handleObjectDetector', self.scriptHandle)
			if result == -1 or not packets:
				return []
			return list(packets)
		except Exception as e:
			print(f'Warning: ObjectDetector render failed: {e}')
			return []

	def _process_marker_detection(self, markerPosition, detectionIndex, objectsDetected):
		"""Convert one colour flag and known static marker position to range/bearing."""
		if markerPosition is None or not self._is_object_detected(objectsDetected, detectionIndex):
			return None
		_valid, _range, _bearing = self.GetRBInCameraFOV(markerPosition)
		if _valid and _range < self.robotParameters.maxMarkerDetectionDistance:
			return [_range, _bearing]
		return None

	@staticmethod
	def _marker_selector_map():
		"""Map public marker selectors to internal maze-generation marker kinds."""
		return {
			MazeObject.baseStationMarker: 'base',
			MazeObject.victimMarker: 'victim',
			MazeObject.rubbleVictimMarker: 'rubble_victim',
			MazeObject.hazardMarker: 'hazard',
		}

	def _get_marker_detections_from_packet(self, objectsDetected, requestedMarkerTypes=None):
		"""Return marker detections grouped by marker kind, using one detector packet."""
		markerSelectorMap = self._marker_selector_map()
		requestedMarkerTypes = set(markerSelectorMap) if requestedMarkerTypes is None else set(requestedMarkerTypes)
		markerDetections = {kind: [] for kind in markerSelectorMap.values()}

		# Packet: [obstacle0, obstacle1, obstacle2, base marker, victim marker,
		# rubble-victim marker, hazard marker, victim object].
		if len(objectsDetected) != 8:
			return markerDetections

		for markerSelector, kind in markerSelectorMap.items():
			if markerSelector not in requestedMarkerTypes:
				continue
			candidates = []
			for marker in self.markerWallPlacements:
				if marker['kind'] != kind:
					continue
				result = self._process_marker_detection(
					marker.get('position'), int(markerSelector), objectsDetected)
				if result is not None:
					candidates.append(result)
			# A colour flag identifies the visible type, not a specific wall when several
			# walls share that colour. The nearest in-FOV wall is the only defensible
			# association; farther candidates are normally occluded by it in the maze.
			if candidates:
				markerDetections[kind].append(min(candidates, key=lambda detection: detection[0]))

		return markerDetections

	def GetDetectedObjects(self, objects=None):
		"""
		Gets the range and bearing to requested obstacles, victims or wall markers in the camera FOV.
		
		Args:
			objects: List containing obstacle/victim/marker selectors or group selectors.
				Defaults to all optional obstacles.
			
		Returns:
			list: Requested detections as [range, bearing] pairs. Use GetDetections()
				when marker type labels need to be retained in a mixed query.
		"""
		# Default to detecting all obstacles if none are specified.
		if objects is None:
			objects = [MazeObject.obstacles]

		requestedObjects = set(objects)
		obstacleTypes = (
			MazeObject.obstacle0,
			MazeObject.obstacle1,
			MazeObject.obstacle2,
		)
		detectAllObstacles = MazeObject.obstacles in requestedObjects
		markerSelectorMap = self._marker_selector_map()
		detectAllMarkers = MazeObject.markers in requestedObjects
		detectVictims = (
			MazeObject.victim in requestedObjects or
			MazeObject.victims in requestedObjects)
		requestedMarkerTypes = {
			selector for selector in markerSelectorMap
			if detectAllMarkers or selector in requestedObjects
		}
		if (not detectAllObstacles and
				not any(obstacle in requestedObjects for obstacle in obstacleTypes) and
				not requestedMarkerTypes and
				not detectVictims):
			return []

		# Check if camera and detector data are available.
		if self.cameraPose is None or self.objectDetectorHandle is None:
			return []

		objectsDetected = self._read_object_detector_packet()

		obstaclesRangeBearing = []
		if objectsDetected:
			for index, obstaclePosition in enumerate(self.obstaclePositions):
				if not detectAllObstacles and obstacleTypes[index] not in requestedObjects:
					continue
				if obstaclePosition is not None:
					result = self._process_single_object_detection(
						obstaclePosition,
						index,
						objectsDetected,
						self.robotParameters.maxObstacleDetectionDistance)
					if result is not None:
						obstaclesRangeBearing.append(result)

		markerDetections = self._get_marker_detections_from_packet(
			objectsDetected, requestedMarkerTypes)
		for kind in markerSelectorMap.values():
			obstaclesRangeBearing.extend(markerDetections[kind])

		if detectVictims:
			obstaclesRangeBearing.extend(
				self._get_victim_detections_from_packet(objectsDetected))

		return obstaclesRangeBearing

	def _get_victim_detections_from_packet(self, objectsDetected):
		"""Return the closest visible, uncollected victim reported by the detector."""
		victimDetectionIndex = int(MazeObject.victim)
		if not self._is_object_detected(objectsDetected, victimDetectionIndex):
			return []

		candidates = []
		for label, victimHandle in self.victimHandles.items():
			if (victimHandle == self.carriedVictimHandle or
					label in self.deliveredVictimLabels):
				continue
			try:
				position = self.sim.getObjectPosition(victimHandle, -1)
			except Exception:
				position = self.victimPositions.get(label)
			if position is None:
				continue
			valid, rangeMetres, bearingRadians = self.GetRBInCameraFOV(position)
			if valid and rangeMetres < self.robotParameters.maxVictimDetectionDistance:
				candidates.append([rangeMetres, bearingRadians])

		# The colour packet identifies the class rather than individual instances. Return
		# only the closest geometrically valid victim to avoid reporting occluded victims.
		return [min(candidates, key=lambda detection: detection[0])] if candidates else []

	def GetDetectedVictims(self):
		"""
		Detect the closest visible victim object.

		Returns:
			list: Zero-or-one ``[range, bearing]`` pair. An empty list means that no
				uncollected victim was detected.
		"""
		if self.cameraPose is None or self.objectDetectorHandle is None:
			return []
		return self._get_victim_detections_from_packet(
			self._read_object_detector_packet())

	def GetDetections(self):
		"""
		Detect the four wall-marker types and yellow victim objects with one sensor update.

		Returns:
			dict: Zero-or-one [range, bearing] pair under each of ``base``,
				``victim``, ``rubble_victim``, ``hazard`` and ``victim_object``.
				The ``victim`` key is the cyan victim marker on a wall, whereas
				``victim_object`` is the separate yellow victim lying on the ground.
				A list is empty when that type is not visible.
		"""
		emptyResult = {
			'base': [],
			'victim': [],
			'rubble_victim': [],
			'hazard': [],
			'victim_object': [],
		}
		if self.cameraPose is None or self.objectDetectorHandle is None:
			return emptyResult

		objectsDetected = self._read_object_detector_packet()
		detections = self._get_marker_detections_from_packet(objectsDetected)
		detections['victim_object'] = self._get_victim_detections_from_packet(
			objectsDetected)
		return detections

	def GetDetectedMarkers(self):
		"""Compatibility name for :meth:`GetDetections`."""
		return self.GetDetections()


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
			# Calculate speed limits
			maxWheelSpeed = self.robotParameters.maximumLinearSpeed / self.robotParameters.wheelRadius

			# Convert the requested robot motion into left/right wheel angular speeds.
			# wheelBase is the calibrated effective track width.  For a four-wheel
			# skid-steer chassis this is wider than the geometric left/right spacing
			# because the tyres must also scrub sideways while the robot turns.
			leftWheelSpeed = (
				(x_dot - 0.5 * theta_dot * self.robotParameters.wheelBase)
				/ self.robotParameters.wheelRadius
				+ self.leftWheelBias)
			rightWheelSpeed = (
				(x_dot + 0.5 * theta_dot * self.robotParameters.wheelBase)
				/ self.robotParameters.wheelRadius
				+ self.rightWheelBias)

			# Add noise if drive system quality is not perfect
			if self.robotParameters.driveSystemQuality != 1:
				noiseStandardDeviation = 1 - self.robotParameters.driveSystemQuality
				leftWheelSpeed = random.gauss(leftWheelSpeed, noiseStandardDeviation)
				rightWheelSpeed = random.gauss(rightWheelSpeed, noiseStandardDeviation)

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

	def _read_wheel_encoder_angles(self, scriptFunction='getWheelEncoderData'):
		"""Read one simulator-side encoder sample as time and cumulative wheel angles."""
		if self.scriptHandle is None:
			raise RuntimeError('Wheel encoders are unavailable because the Robot script was not found.')

		try:
			result = self.sim.callScriptFunction(scriptFunction, self.scriptHandle)
		except Exception as e:
			raise RuntimeError(
				'Wheel encoder feedback is unavailable. Ensure the supplied scene contains '
				'the matching Robot Lua script.') from e

		if not isinstance(result, (list, tuple)) or len(result) != 3:
			raise RuntimeError(
				f"Robot script function '{scriptFunction}' returned invalid encoder data: {result!r}")

		sampleTime, leftAngle, rightAngle = result
		if not all(
				isinstance(value, (int, float)) and math.isfinite(value)
				for value in (sampleTime, leftAngle, rightAngle)):
			raise RuntimeError(
				f"Robot script function '{scriptFunction}' returned non-finite encoder data: {result!r}")

		return float(sampleTime), float(leftAngle), float(rightAngle)

	def _encoder_sample_from_angles(self, sampleTime, leftAngle, rightAngle):
		"""Quantize cumulative wheel angles into the configured encoder tick resolution."""
		countsPerRevolution = self.robotParameters.encoderCountsPerRevolution
		if (isinstance(countsPerRevolution, bool) or
				not isinstance(countsPerRevolution, int) or
				countsPerRevolution <= 0):
			raise ValueError('encoderCountsPerRevolution must be a positive integer.')

		countsPerRadian = countsPerRevolution / (2.0 * math.pi)
		return {
			'time': sampleTime,
			'left_ticks': int(round(leftAngle * countsPerRadian)),
			'right_ticks': int(round(rightAngle * countsPerRadian)),
		}

	def GetWheelEncoders(self):
		"""
		Read the cumulative left and right wheel encoder counts.

		Returns:
			dict: ``{'time': seconds, 'left_ticks': int, 'right_ticks': int}``.
			Time is CoppeliaSim simulation time. Counts are signed, cumulative values
			from the last simulator start or :meth:`ResetWheelEncoders` call.

		The Robot Lua script accumulates wheel rotation every simulation step, so full
		wheel revolutions are retained even when this function is called infrequently.
		"""
		return self._encoder_sample_from_angles(
			*self._read_wheel_encoder_angles('getWheelEncoderData'))

	def ResetWheelEncoders(self):
		"""
		Reset both public encoder counters to zero without moving the wheel joints.

		The current odometry pose is preserved. Its count baseline is rebased to the
		new zero values so resetting the encoders cannot create a false odometry jump.
		"""
		sample = self._encoder_sample_from_angles(
			*self._read_wheel_encoder_angles('resetWheelEncoders'))
		self._odometryLastEncoderCounts = (
			sample['left_ticks'], sample['right_ticks'])

	def _update_odometry_from_encoder_sample(self, sample):
		"""Integrate one cumulative encoder sample using differential-drive kinematics."""
		currentCounts = (sample['left_ticks'], sample['right_ticks'])
		if self._odometryLastEncoderCounts is None:
			self._odometryLastEncoderCounts = currentCounts
			return

		leftDeltaTicks = currentCounts[0] - self._odometryLastEncoderCounts[0]
		rightDeltaTicks = currentCounts[1] - self._odometryLastEncoderCounts[1]
		self._odometryLastEncoderCounts = currentCounts

		wheelRadius = self.robotParameters.wheelRadius
		wheelBase = self.robotParameters.wheelBase
		if not math.isfinite(wheelRadius) or wheelRadius <= 0.0:
			raise ValueError('wheelRadius must be a positive finite value.')
		if not math.isfinite(wheelBase) or wheelBase <= 0.0:
			raise ValueError('wheelBase must be a positive finite value.')

		metresPerTick = (
			2.0 * math.pi * wheelRadius /
			self.robotParameters.encoderCountsPerRevolution)
		leftDistance = leftDeltaTicks * metresPerTick
		rightDistance = rightDeltaTicks * metresPerTick
		centreDistance = 0.5 * (leftDistance + rightDistance)
		headingChange = (rightDistance - leftDistance) / wheelBase

		# The midpoint heading is substantially more accurate than the old heading
		# when the robot translates and turns during the same encoder interval.
		midpointHeading = self._odometryPose[2] + 0.5 * headingChange
		self._odometryPose[0] += centreDistance * math.cos(midpointHeading)
		self._odometryPose[1] += centreDistance * math.sin(midpointHeading)
		self._odometryPose[2] = self.WrapToPi(
			self._odometryPose[2] + headingChange)

	def GetOdometry(self):
		"""
		Return the robot pose estimated only from wheel encoder counts.

		Returns:
			dict: ``{'time': seconds, 'x': metres, 'y': metres, 'heading': radians}``.

		The pose is relative to the last simulator start or :meth:`ResetOdometry`
		call. It is not the simulator's global pose and will naturally accumulate
		error when wheels slip or the configured wheel geometry is imperfect.
		"""
		sample = self.GetWheelEncoders()
		self._update_odometry_from_encoder_sample(sample)
		return {
			'time': sample['time'],
			'x': self._odometryPose[0],
			'y': self._odometryPose[1],
			'heading': self._odometryPose[2],
		}

	def ResetOdometry(self):
		"""Set the local odometry pose to ``(0, 0, 0)`` at the current encoder counts."""
		sample = self.GetWheelEncoders()
		self._odometryPose = [0.0, 0.0, 0.0]
		self._odometryLastEncoderCounts = (
			sample['left_ticks'], sample['right_ticks'])

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
				if label in self.deliveredVictimLabels:
					continue
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

	def ReleaseVictim(self, forward_offset=None, lateral_offset=0.0, delivered=False):
		"""
		Place the currently carried victim back onto the maze floor.

		The victim is positioned slightly in front of the robot so it is not hidden below
		the chassis. Its original floor orientation is restored and its lowest transformed
		bounding-box corner is placed 1 mm above the floor.

		Args:
			forward_offset: Optional metres forward from the robot centre. ``None`` uses
				the normal release offset.
			lateral_offset: Metres to the robot's left, useful for separating several
				victims in one base drop zone.
			delivered: If True, permanently exclude this victim from later collection and
				detection and make it invisible to proximity sensors.

		Returns:
			tuple: (success, victim_label), with victim_label set to None on failure.
		"""
		if not self.HasVictim():
			print("Release failed: the robot is not carrying a victim.")
			return False, None
		if forward_offset is None:
			forward_offset = VICTIM_RELEASE_FORWARD_OFFSET
		if (not isinstance(forward_offset, (int, float)) or
				not math.isfinite(forward_offset) or forward_offset < 0.0):
			raise ValueError('forward_offset must be a non-negative finite value')
		if (not isinstance(lateral_offset, (int, float)) or
				not math.isfinite(lateral_offset)):
			raise ValueError('lateral_offset must be a finite value')

		victimHandle = self.carriedVictimHandle
		victimLabel = self.carriedVictimLabel
		try:
			robotPosition = self.sim.getObjectPosition(self.robotHandle, -1)
			robotOrientation = self.sim.getObjectOrientation(self.robotHandle, -1)
			robotYaw = robotOrientation[2]
			dropX = (
				robotPosition[0] + forward_offset * math.cos(robotYaw) -
				lateral_offset * math.sin(robotYaw))
			dropY = (
				robotPosition[1] + forward_offset * math.sin(robotYaw) +
				lateral_offset * math.cos(robotYaw))

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
			if delivered:
				self.deliveredVictimLabels.add(victimLabel)
				# Keep the delivered victim visible in the base drop zone, but prevent it
				# from appearing as a close wall or a new yellow detector target.
				detectorProxyHandle = getattr(
					self, 'victimDetectorHandles', {}).get(victimLabel)
				for shapeHandle in self.sim.getObjectsInTree(
						victimHandle, self.sim.object_shape_type, 0):
					try:
						specialProperties = self.sim.getObjectSpecialProperty(shapeHandle)
						if shapeHandle == detectorProxyHandle:
							specialProperties &= ~self.sim.objectspecialproperty_renderable
						else:
							specialProperties &= ~self.sim.objectspecialproperty_detectable_all
						self.sim.setObjectSpecialProperty(
							shapeHandle, specialProperties)
					except Exception as e:
						print(
							f"Warning: could not disable proximity detection for "
							f"delivered victim {victimLabel}: {e}")
			self.carriedVictimHandle = None
			self.carriedVictimLabel = None
			print(f"Released victim {victimLabel} onto the maze floor.")
			return True, victimLabel
		except Exception as e:
			print(f"Error releasing victim {victimLabel}: {e}")
			return False, None

	def UpdateObjectPositions(self):
		"""
		Refresh the internal object positions used to calculate sensor detections.
		This should be called in every loop to get accurate object detection.

		The robot's global pose and the global object positions are deliberately not
		returned because they are not measurements available to the navigation system.
		"""
		self.GetObjectPositions()
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
			raise ConnectionError(
				'Could not connect to CoppeliaSim. Confirm that the simulator is '
				'running, the 2026 scene is loaded, and the ZeroMQ Remote API is enabled.') from e

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
		templates are required; startup aborts if any of these are missing.
		Robot sensors (vision sensor, object detector, distance sensors and rear
		motors) are optional and resolved best-effort. Optional obstacles never abort startup.
		"""
		# --- Essential: robot + drive motors ---
		errorCode = self.GetRobotHandle()
		if errorCode != 0:
			raise RuntimeError("Required scene object '/Robot' was not found.")

		errorCode = self.GetScriptHandle()
		if errorCode != 0:
			raise RuntimeError("The simulation script attached to '/Robot' was not found.")

		errorCode1, errorCode2, errorCode3, errorCode4 = self.GetMotorHandles()
		if errorCode1 != 0 or errorCode2 != 0:
			raise RuntimeError('Required left/right drive motors were not found.')
		elif errorCode3 != 0 or errorCode4 != 0:
			print("Note: rear wheel motors not found (fine for two-wheel differential robots).")

		# --- Essential: search and rescue maze scene objects ---
		self.floorHandle = self._resolve_first_available(['/floor'], 'floor', required=True)
		if self.floorHandle is None:
			raise RuntimeError("Required scene object '/floor' was not found.")

		self.tableWallHandles = []
		for i in range(4):
			handle = self._resolve_first_available([f'/table_wall[{i}]'], f'table_wall[{i}]', required=True)
			if handle is None:
				raise RuntimeError(f"Required scene object '/table_wall[{i}]' was not found.")
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
			raise RuntimeError('One or more required maze template objects were not found.')

		# Colour only each child marker plane. The white structural wall remains unchanged.
		self._configure_marker_template_planes()

		# --- Search-and-rescue collection point and optional robot sensors ---
		self.GetVictimCarryPointHandle()
		self.GetCameraHandle()
		self.GetObjectDetectorHandle()
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
		"""Resolve the simulation script attached to the Robot model."""
		if self.robotHandle is None:
			self.scriptHandle = None
			return -1
		try:
			self.scriptHandle = self.sim.getScriptAssociatedWithObject(self.robotHandle)
		except Exception:
			# Newer scenes can expose scripts as scene objects beneath their model.
			self.scriptHandle = self._try_get_object('/Robot/Script')
		return 0 if self.scriptHandle is not None and self.scriptHandle != -1 else -1

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

	def _get_object_detector_render_mode(self):
		"""Validate the public renderer name and return its display name and mode value."""
		renderer = self.robotParameters.objectDetectorRenderer
		if not isinstance(renderer, str):
			raise ValueError(
				"objectDetectorRenderer must be either 'legacy' or 'opengl3'.")

		renderer = renderer.strip().lower()
		if renderer not in OBJECT_DETECTOR_RENDER_MODES:
			raise ValueError(
				f"Unknown objectDetectorRenderer {self.robotParameters.objectDetectorRenderer!r}. "
				"Use 'legacy' or 'opengl3'.")

		# Store the normalized spelling so diagnostics and later inspection are stable.
		self.robotParameters.objectDetectorRenderer = renderer
		return OBJECT_DETECTOR_RENDER_MODES[renderer]

	# Updates the robot within COPPELIA based on the robot parameters
	def UpdateCOPPELIARobot(self):
		rendererDisplayName, rendererMode = self._get_object_detector_render_mode()
		self._configure_skid_steer_contact_geometry()

		# Set Camera Pose and Orientation (skipped if no VisionSensor was resolved)
		if self.cameraHandle is None:
			print("Note: skipping camera pose/orientation setup - no VisionSensor resolved.")
		else:
			self.SetCameraPose(self.robotParameters.cameraDistanceFromRobotCenter, self.robotParameters.cameraHeightFromFloor, self.robotParameters.cameraTilt)
			self.SetCameraOrientation(self.robotParameters.cameraOrientation)

		if self.objectDetectorHandle is not None:
			try:
				# Configure the detector independently from the full-resolution camera. The
				# detector needs visible RGB colours, so object-ID colour mode is not valid.
				self.sim.setObjectInt32Param(
					self.objectDetectorHandle,
					self.sim.visionintparam_render_mode,
					rendererMode)
				self.sim.setObjectInt32Param(
					self.objectDetectorHandle,
					self.sim.objintparam_visibility_layer,
					DETECTOR_MARKER_HIDDEN_LAYER)
				print(f"ObjectDetector renderer: {rendererDisplayName}.")
			except Exception as e:
				print(f"Warning: could not configure ObjectDetector render mode: {e}")

	def _configure_skid_steer_contact_geometry(self):
		"""Place four-wheel contact axles for reliable centre-point turns.

		A rigid four-wheel chassis cannot turn like an ideal differential-drive robot
		unless its tyres scrub sideways.  The original axles were far enough apart that
		Bullet often resisted or stalled an in-place turn.  Moving the physical contact
		axles slightly toward the chassis centre keeps four driven/supporting wheels but
		reduces the opposing scrub torque symmetrically.  The calibrated geometry turns
		about the robot centre without the large translation caused by lowering only one
		axle's friction.
		"""
		if self.robotParameters.driveType != 'differential':
			return
		if (getattr(self, 'leftRearMotorHandle', None) is None or
				getattr(self, 'rightRearMotorHandle', None) is None):
			# A two-wheel differential robot already has one central drive axle.
			return

		offset = self.robotParameters.wheelAxleLongitudinalOffset
		if not math.isfinite(offset) or offset < 0.0:
			raise ValueError(
				'wheelAxleLongitudinalOffset must be a non-negative finite value.')

		motorPlacements = (
			(self.leftMotorHandle, offset),
			(self.rightMotorHandle, offset),
			(self.leftRearMotorHandle, -offset),
			(self.rightRearMotorHandle, -offset),
		)
		for motorHandle, longitudinalPosition in motorPlacements:
			position = self.sim.getObjectPosition(motorHandle, self.robotHandle)
			position[0] = longitudinalPosition
			self.sim.setObjectPosition(motorHandle, self.robotHandle, position)
			# Imported mesh poses contained small, different angular errors on each
			# corner. Exact motor axes and collision-wheel poses make clockwise and
			# counter-clockwise contact forces symmetric.
			self.sim.setObjectOrientation(
				motorHandle, self.robotHandle, [-math.pi / 2.0, 0.0, 0.0])

			wheelHandle = self.sim.getObjectChild(motorHandle, 0)
			if wheelHandle != -1 and self.sim.getObjectType(wheelHandle) == self.sim.object_shape_type:
				self.sim.setObjectPosition(wheelHandle, motorHandle, [0.0, 0.0, 0.0])
				self.sim.setObjectOrientation(
					wheelHandle, motorHandle, [0.0, -math.pi / 2.0, 0.0])

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

		# Select the fixed teaching maze or generate a fresh validated random topology
		# before any CoppeliaSim objects are copied. An integer seed reproduces the same
		# random maze; a None seed intentionally chooses a new one on every start.
		self.sceneParameters.prepare_maze_layout(force_random=True)
		self.sceneParameters.validate_maze_parameters()

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
			raise ValueError(floorErrorMessage)
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

	def _find_marker_child(self, wallModelHandle, childAlias, required=True):
		"""Return a specifically named child shape beneath a marker-wall model base."""
		shapeHandles = self.sim.getObjectsInTree(wallModelHandle, self.sim.sceneobject_shape, 0)
		childShapes = [handle for handle in shapeHandles if handle != wallModelHandle]
		matches = []
		for handle in childShapes:
			alias = self.sim.getObjectAlias(handle, 1).replace('\\', '/')
			if alias.rsplit('/', 1)[-1].lower() == childAlias.lower():
				matches.append(handle)

		if len(matches) == 1:
			return matches[0]
		if childAlias == 'marker' and not matches and len(childShapes) == 1:
			return childShapes[0]
		if not required and not matches:
			return None
		raise ValueError(
			f"Marker-wall model handle {wallModelHandle} must contain exactly one "
			f"child shape named '{childAlias}'.")

	def _find_marker_plane(self, wallModelHandle):
		"""Return the textured visual marker child."""
		return self._find_marker_child(wallModelHandle, 'marker')

	def _find_detector_marker_plane(self, wallModelHandle, required=True):
		"""Return the untextured detector-only marker child."""
		return self._find_marker_child(wallModelHandle, 'detector_marker', required=required)

	def _ensure_detector_marker_plane(self, wallModelHandle, visualMarkerHandle):
		"""Create/reuse an untextured plane matching the visual marker geometry and pose."""
		detectorMarkerHandle = self._find_detector_marker_plane(wallModelHandle, required=False)
		if detectorMarkerHandle is None:
			# bit4 strips all textures while retaining the exact source-plane geometry.
			detectorMarkerHandle = self.sim.copyPasteObjects([visualMarkerHandle], 16)[0]
			self.sim.setObjectParent(detectorMarkerHandle, wallModelHandle, True)
			self.sim.setObjectAlias(detectorMarkerHandle, 'detector_marker')

		# Place the colour plane slightly closer to the cell/camera. It covers the
		# textured marker only during the atomic ObjectDetector render.
		self.sim.setObjectPosition(
			detectorMarkerHandle,
			visualMarkerHandle,
			[0.0, 0.0, DETECTOR_MARKER_FACE_OFFSET])
		self.sim.setObjectOrientation(detectorMarkerHandle, visualMarkerHandle, [0.0, 0.0, 0.0])
		self.sim.setShapeTexture(
			detectorMarkerHandle, -1, self.sim.texturemap_plane, 0, [1.0, 1.0])
		self.sim.setObjectInt32Param(
			detectorMarkerHandle,
			self.sim.objintparam_visibility_layer,
			DETECTOR_MARKER_HIDDEN_LAYER)
		self.sim.setObjectSpecialProperty(
			detectorMarkerHandle, self.sim.objectspecialproperty_renderable)
		self.sim.setObjectInt32Param(detectorMarkerHandle, self.sim.shapeintparam_static, 1)
		self.sim.setObjectInt32Param(detectorMarkerHandle, self.sim.shapeintparam_respondable, 0)
		return detectorMarkerHandle

	def _configure_marker_template_planes(self):
		"""Configure white visual markers and hidden detector-only colour planes."""
		templates = {
			'base': self.baseStationWallTemplateHandle,
			'victim': self.victimWallTemplateHandle,
			'rubble_victim': self.rubbleVictimWallTemplateHandle,
			'hazard': self.hazardWallTemplateHandle,
		}
		for kind, wallModelHandle in templates.items():
			markerPlaneHandle = self._find_marker_plane(wallModelHandle)
			detectorMarkerHandle = self._ensure_detector_marker_plane(
				wallModelHandle, markerPlaneHandle)

			# The original textured marker remains white for VisionSensor and students.
			self.sim.setObjectInt32Param(
				markerPlaneHandle,
				self.sim.objintparam_visibility_layer,
				VISUAL_MARKER_VISIBILITY_LAYER)
			self.sim.setShapeColor(
				markerPlaneHandle,
				None,
				self.sim.colorcomponent_ambient_diffuse,
				list(VISUAL_MARKER_COLOUR))
			self.sim.setShapeColor(
				markerPlaneHandle,
				None,
				self.sim.colorcomponent_emission,
				list(VISUAL_MARKER_COLOUR))

			# The untextured plane supplies a solid detector-safe class colour.
			self.sim.setShapeColor(
				detectorMarkerHandle,
				None,
				self.sim.colorcomponent_ambient_diffuse,
				list(MARKER_EMISSIVE_COLOURS[kind]))
			self.sim.setShapeColor(
				detectorMarkerHandle,
				None,
				self.sim.colorcomponent_emission,
				list(MARKER_EMISSIVE_COLOURS[kind]))
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
			for startPoint, endPoint in self.sceneParameters.mazeWallSegments
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
		if len(baseOpenSides) != 1:
			raise ValueError(
				f"Base cell {baseCell} must have exactly one open side "
				f"(found {baseOpenSides})")
		baseMarkerSide = oppositeDirection[baseOpenSides[0]]
		assign('base', baseCell, baseMarkerSide, self.baseStationWallTemplateHandle, 'EGB320_GEN_BASE_STATION_WALL')

		# Victims must occupy three-sided dead ends. Put each marker on the terminal wall
		# opposite the sole opening so an approaching robot sees the marker above the victim.
		victimCells = {tuple(cell) for cell in self.sceneParameters.victimCells.values()}
		for label, configuredCell in self.sceneParameters.victimCells.items():
			cell = tuple(configuredCell)
			wallSides = self._cell_wall_sides(cell)
			openSides = [side for side, closed in wallSides.items() if not closed]
			if len(openSides) != 1:
				raise ValueError(
					f"Victim '{label}' at cell {cell} must be in a dead end "
					f"(found open sides: {openSides})")
			markerSide = oppositeDirection[openSides[0]]

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
		wallSegments = list(self.sceneParameters.mazeWallSegments)
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
				markerWallHandle = self._create_wall_between_grid_points(
					startPoint,
					endPoint,
					index,
					templateHandle=marker['templateHandle'],
					alias=marker['alias'],
					markerCell=marker['cell'])
				markerPlaneHandle = self._find_marker_plane(markerWallHandle)
				detectorMarkerHandle = self._find_detector_marker_plane(markerWallHandle)
				marker['wallHandle'] = markerWallHandle
				marker['markerHandle'] = markerPlaneHandle
				marker['detectorMarkerHandle'] = detectorMarkerHandle
				marker['position'] = self.sim.getObjectPosition(markerPlaneHandle, -1)
				self.generatedVisualMarkerHandles.append(markerPlaneHandle)
				self.generatedDetectorMarkerHandles.append(detectorMarkerHandle)
				self.markerWallPlacements.append(marker)
			count += 1

		# Defensive fallback for any marker assigned to an unexpected wall segment that is not
		# present in the configured internal or generated perimeter wall lists.
		for marker in markerAssignments.values():
			markerWallHandle = self._create_wall_between_grid_points(
				marker['startPoint'],
				marker['endPoint'],
				count,
				templateHandle=marker['templateHandle'],
				alias=marker['alias'],
				markerCell=marker['cell'])
			markerPlaneHandle = self._find_marker_plane(markerWallHandle)
			detectorMarkerHandle = self._find_detector_marker_plane(markerWallHandle)
			marker['wallHandle'] = markerWallHandle
			marker['markerHandle'] = markerPlaneHandle
			marker['detectorMarkerHandle'] = detectorMarkerHandle
			marker['position'] = self.sim.getObjectPosition(markerPlaneHandle, -1)
			self.generatedVisualMarkerHandles.append(markerPlaneHandle)
			self.generatedDetectorMarkerHandles.append(detectorMarkerHandle)
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
			# Victims are targets to collect, not physical obstacles.  In particular, the
			# robot must be able to enter the 0.10 m collection envelope before calling
			# CollectVictim().  Do this on every shape in case the victim template is later
			# changed from a single shape into a model with several child shapes.
			for shapeHandle in self.sim.getObjectsInTree(
					newHandle, self.sim.object_shape_type, 0):
				self.sim.setObjectInt32Param(
					shapeHandle, self.sim.shapeintparam_static, 1)
				self.sim.setObjectInt32Param(
					shapeHandle, self.sim.shapeintparam_respondable, 0)
			self.sim.setObjectPosition(newHandle, -1, [x, y, zPos])
			self.sim.setObjectOrientation(newHandle, -1, list(self.victimTemplateOrientation))
			self.sim.setObjectAlias(newHandle, f"EGB320_GEN_VICTIM_{label}")
			self.sim.setObjectParent(newHandle, self.generatedSceneRootHandle, True)
			detectorProxyHandle = self._create_victim_detector_proxy(newHandle)
			self.victimHandles[label] = newHandle
			self.victimPositions[label] = [x, y, zPos]
			self.generatedVictimDetectorHandles.append(detectorProxyHandle)
			self.victimDetectorHandles[label] = detectorProxyHandle
			count += 1
		return count

	def _create_victim_detector_proxy(self, victimHandle):
		"""Create a simple hidden colour proxy that follows one visual victim shape."""
		boundingBoxSize, boundingBoxPose = self.sim.getShapeBB(victimHandle)
		# Preserve the victim's bounding-box silhouette, but give very thin dimensions
		# enough thickness to remain visible in the detector's low-resolution image.
		proxySize = [max(float(size), 0.025) for size in boundingBoxSize]
		proxyHandle = self.sim.createPrimitiveShape(
			self.sim.primitiveshape_cuboid, proxySize, 0)
		self.sim.setObjectAlias(proxyHandle, 'victim_detector_proxy')
		self.sim.setObjectParent(proxyHandle, victimHandle, False)
		self.sim.setObjectPose(proxyHandle, victimHandle, boundingBoxPose)
		self.sim.setObjectInt32Param(
			proxyHandle,
			self.sim.objintparam_visibility_layer,
			DETECTOR_MARKER_HIDDEN_LAYER)
		self.sim.setObjectSpecialProperty(
			proxyHandle, self.sim.objectspecialproperty_renderable)
		self.sim.setObjectInt32Param(proxyHandle, self.sim.shapeintparam_static, 1)
		self.sim.setObjectInt32Param(proxyHandle, self.sim.shapeintparam_respondable, 0)
		self.sim.setShapeColor(
			proxyHandle,
			None,
			self.sim.colorcomponent_ambient_diffuse,
			list(VICTIM_DETECTOR_COLOUR))
		self.sim.setShapeColor(
			proxyHandle,
			None,
			self.sim.colorcomponent_emission,
			list(VICTIM_DETECTOR_COLOUR))
		return proxyHandle

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
		self.generatedVisualMarkerHandles = []
		self.generatedDetectorMarkerHandles = []
		self.generatedVictimDetectorHandles = []
		self.victimDetectorHandles = {}
		self.generatedWallPostHandles = []
		self.victimHandles = {}
		self.victimPositions = {}
		self.markerWallPlacements = []
		self.carriedVictimHandle = None
		self.carriedVictimLabel = None
		self.deliveredVictimLabels = set()

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
		print(
			f"Victims created: {victimCount} "
			f"(configured: {self.sceneParameters.numberOfVictims})")
		print(f"Maze source: {self.sceneParameters.mazeGenerationMode}")
		if self.sceneParameters.activeMazeSeed is not None:
			print(
				f"Random maze seed: {self.sceneParameters.activeMazeSeed} "
				f"(accepted on attempt {self.sceneParameters.mazeGenerationAttempt})")
		print(f"Base cell: {tuple(self.sceneParameters.baseCell)}")
		print(f"Victim cells: {victimCellsText}")
		print(f"Victim path distances: {self.sceneParameters.mazeVictimDistances}")
		print(f"Hazard dead-end cells: {self.sceneParameters.mazeHazardCells}")
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
		elif orientation == 'landscape':
			x_res = self.robotParameters.cameraResolutionX
			y_res = self.robotParameters.cameraResolutionY
		else:
			print(
				f"Unknown camera orientation '{orientation}'. "
				"Use 'portrait' or 'landscape'.")
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

	# Cache the camera and optional-object world positions used internally to synthesize
	# robot-relative detections. Values are set to None if they cannot be refreshed.
	def GetObjectPositions(self):
		# Clear cached values so callers can distinguish an update failure.
		self.cameraPose = None
		self.cameraForwardYaw = None
		self.obstaclePositions = [None, None, None]

		# GET 3D CAMERA POSE
		if self.cameraHandle is not None:
			try:
				cameraPosition = self.sim.getObjectPosition(self.cameraHandle, -1)
				cameraOrientation = self.sim.getObjectOrientation(self.cameraHandle, -1)
				cameraMatrix = self.sim.getObjectMatrix(self.cameraHandle, -1)
				# CoppeliaSim vision sensors look along local +Z. The mounted sensor's
				# Euler Z angle is not its horizontal viewing direction.
				self.cameraForwardYaw = math.atan2(cameraMatrix[6], cameraMatrix[2])
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

	def GetRBInCameraFOV(self, objectPosition):
		"""Return ``(visible, range, bearing)`` for a world point relative to the camera."""
		cameraYaw = self.cameraForwardYaw if self.cameraForwardYaw is not None else self.cameraPose[5]
		cameraPose2d = [self.cameraPose[0], self.cameraPose[1], cameraYaw]
		_range, _bearing = self.GetRangeAndBearingFromPoseAndPoint(cameraPose2d, objectPosition)
		_valid = abs(_bearing) < self.robotParameters.cameraPerspectiveAngle / 2
		return _valid, _range, _bearing
			
	
	# Determines if a 2D point is inside the arena, returns true if that is the case
	def PointInsideArena(self, position):
		return -1 < position[0] < 1 and -1 < position[1] < 1

	# Wraps input value to be between -pi and pi
	def WrapToPi(self, radians):
		return (radians + math.pi) % (2 * math.pi) - math.pi

	# Gets the range and bearing given a 2D pose (x,y,theta) and a point(x,y). 
	# The bearing will be relative to the pose's angle
	def GetRangeAndBearingFromPoseAndPoint(self, pose, point):
		_range = math.hypot(pose[0] - point[0], pose[1] - point[1])
		_bearing = self.WrapToPi(math.atan2((point[1]-pose[1]), (point[0]-pose[0])) - pose[2])

		return _range, _bearing

# Parameter classes for robot and scene configuration
class RobotParameters(object):
	"""Parameters for configuring the maze robot."""
	def __init__(self):
		# Drive Parameters
		self.driveType = 'differential'  # currently only 'differential' implemented
		self.maximumLinearSpeed = 0.25  # maximum speed in m/s
		self.driveSystemQuality = 1.0   # quality from 0 to 1 (1 = perfect)
		self.encoderCountsPerRevolution = 360  # signed quadrature counts per wheel turn
		
		# Calibrated effective track width for this four-wheel skid-steer robot.  It is
		# intentionally wider than the geometric wheel spacing because tyre scrub makes
		# the chassis turn less than an ideal two-wheel differential-drive model.
		self.wheelBase = 0.145          # effective wheel separation in metres
		self.wheelRadius = 0.0245       # collision-wheel radius in metres
		# Distance from the robot centre to each physical front/rear axle. The original
		# 0.04 m offset created excessive opposing tyre scrub during in-place turns.
		self.wheelAxleLongitudinalOffset = 0.03  # front=+offset, rear=-offset
		
		# Camera Parameters
		self.cameraOrientation = 'landscape'  # 'landscape' or 'portrait'
		self.cameraDistanceFromRobotCenter = 0.1  # distance from robot center in m
		self.cameraHeightFromFloor = 0.15     # height from floor in m
		self.cameraTilt = 0.0                 # tilt angle in radians
		self.cameraResolutionX = 640          # camera width in pixels
		self.cameraResolutionY = 480          # camera height in pixels
		self.cameraPerspectiveAngle = math.pi/3  # field of view angle in radians
		# Low-resolution colour detector renderer. 'legacy' is the stable default;
		# 'opengl3' opts into CoppeliaSim's optional simOpenGL3 plugin renderer.
		self.objectDetectorRenderer = 'legacy'
		
		# Detection Parameters
		self.maxObstacleDetectionDistance = 1.5  # max distance to detect obstacles in m
		self.maxMarkerDetectionDistance = 1.5    # max distance to detect wall markers in m
		self.maxVictimDetectionDistance = 1.5    # max distance to detect victim objects in m
		
		# Victim collection parameter from the 2026 assessment rules
		self.victimCollectionDistance = 0.10  # shortest horizontal clearance in metres
		
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

		# Select 'preset' for the original hand-authored teaching maze or 'random' for a
		# new challenge maze. Set randomMazeSeed to an integer to reproduce a layout;
		# leave it as None to choose a fresh seed whenever StartSimulator() builds a scene.
		self.mazeGenerationMode = 'preset'
		self.randomMazeSeed = None
		self.randomMazeMaximumAttempts = 1000
		self.randomMazeBaseOpeningSide = 'N'
		# Generate the ordered prefix of victim levels: 1 -> L1, 2 -> L1/L2,
		# 3 -> L1/L2/L3. Values outside the inclusive range 1--3 are rejected.
		self.numberOfVictims = 3

		# Public preset fields make switching from random back to the original maze
		# explicit. mazeWallSegments/baseCell/victimCells below always describe the
		# currently active layout and are what the renderer consumes.
		self.presetMazeWallSegments = list(EXAMPLE_MAZE_SEGMENTS)
		self.presetBaseCell = tuple(PRESET_BASE_CELL)
		self.presetBaseYaw = PRESET_BASE_YAW
		self.presetVictimCells = dict(PRESET_VICTIM_CELLS)
		self.mazeWallSegments = list(self.presetMazeWallSegments)
		self.baseCell = tuple(self.presetBaseCell)
		self.baseYaw = self.presetBaseYaw
		# Each victim is in a three-sided dead end. The associated marker is generated on
		# the terminal wall opposite the one open side (see _plan_marker_walls).
		self.victimCells = dict(self.presetVictimCells)
		self.activeMazeSeed = None
		self.mazeGenerationAttempt = 1
		self.mazeVictimDistances = {}
		self.mazeHazardCells = []
		self.activeMazeLayout = None
		self._activeMazeMode = 'preset'
		self.placeRobotAtBase = True

		# Robot starting position [x, y, theta] in metres and radians.
		# If set, this overrides placeRobotAtBase/baseCell.
		self.robotStartingPosition = None

		# Optional obstacle starting positions [x, y] in metres.
		# Set to -1 to use current CoppeliaSim position, None if not wanted in scene
		self.obstacle0_StartingPosition = None
		self.obstacle1_StartingPosition = None
		self.obstacle2_StartingPosition = None

	def _apply_maze_layout(self, layout):
		"""Copy a topology-only :class:`MazeLayout` into the active scene fields."""
		self.mazeWallSegments = [
			(tuple(startPoint), tuple(endPoint))
			for startPoint, endPoint in layout.wall_segments
		]
		self.baseCell = tuple(layout.base_cell)
		self.victimCells = {
			label: tuple(cell) for label, cell in layout.victim_cells.items()
		}
		self.activeMazeSeed = layout.seed
		self.mazeGenerationAttempt = layout.generation_attempt
		self.mazeVictimDistances = {
			label: layout.distances_from_base.get(tuple(cell))
			for label, cell in layout.victim_cells.items()
		}
		self.mazeHazardCells = [tuple(cell) for cell in layout.hazard_cells]
		self.activeMazeLayout = layout
		self._activeMazeMode = layout.mode

	def prepare_maze_layout(self, force_random=False):
		"""Select or generate the active topology before scene objects are created.

		``force_random`` is used by :meth:`MazeBot.SetScene` so each simulator start is a
		new generation step. An integer ``randomMazeSeed`` still gives the same topology
		on every start, which is useful for demonstrations and debugging.
		"""
		mode = str(self.mazeGenerationMode).strip().lower()
		if mode not in ('preset', 'random'):
			raise ValueError(
				"mazeGenerationMode must be 'preset' or 'random' "
				f"(got {self.mazeGenerationMode!r})")
		self.mazeGenerationMode = mode
		victimCount = validate_victim_count(self.numberOfVictims)

		if mode == 'random':
			if force_random or self._activeMazeMode != 'random':
				layout = generate_random_maze(
					rows=self.mazeRows,
					columns=self.mazeColumns,
					base_cell=tuple(self.baseCell),
					base_open_side=self.randomMazeBaseOpeningSide,
					seed=self.randomMazeSeed,
					maximum_attempts=self.randomMazeMaximumAttempts,
					victim_count=victimCount,
				)
				# Face the robot through the base's sole opening. This also keeps the
				# navigation system's initial cardinal direction consistent with the maze.
				self.baseYaw = {
					'N': math.pi / 2.0,
					'E': 0.0,
					'S': -math.pi / 2.0,
					'W': math.pi,
				}[str(self.randomMazeBaseOpeningSide).upper()]
				self._apply_maze_layout(layout)
			return self.activeMazeLayout

		# Restore the named preset only when changing back from random mode. On the first
		# preset build, direct edits to the legacy active fields remain supported.
		if self._activeMazeMode == 'random':
			self.baseYaw = self.presetBaseYaw
			wallSegments = self.presetMazeWallSegments
			baseCell = self.presetBaseCell
		else:
			wallSegments = self.mazeWallSegments
			baseCell = self.baseCell
			# Keep legacy direct edits to victimCells as preset edits. Because this is an
			# update rather than replacement, increasing the count later restores levels
			# that were previously omitted from the active layout.
			self.presetVictimCells.update({
				label: tuple(cell)
				for label, cell in self.victimCells.items()
				if label in VICTIM_LEVELS
			})
		victimCells = {
			label: self.presetVictimCells[label]
			for label in VICTIM_LEVELS[:victimCount]
		}
		layout = MazeLayout(
			rows=self.mazeRows,
			columns=self.mazeColumns,
			wall_segments=list(wallSegments),
			base_cell=tuple(baseCell),
			victim_cells=dict(victimCells),
			mode='preset',
		)
		details = validate_maze_layout(layout)
		layout.distances_from_base = details['distances_from_base']
		layout.hazard_cells = details['hazard_cells']
		self._apply_maze_layout(layout)
		return layout

	def validate_maze_parameters(self):
		"""
		Validate maze-related settings before any scene objects are created.
		Raises ValueError with a clear message if a check fails.
		"""
		if self.mazeRows <= 0 or self.mazeColumns <= 0:
			raise ValueError(f"mazeRows and mazeColumns must be positive (got {self.mazeRows} x {self.mazeColumns})")

		if not math.isfinite(self.mazeCellSize) or self.mazeCellSize <= 0:
			raise ValueError(f"mazeCellSize must be positive (got {self.mazeCellSize})")
		if not math.isfinite(self.baseYaw):
			raise ValueError(f"baseYaw must be finite (got {self.baseYaw})")
		victimCount = validate_victim_count(self.numberOfVictims)
		expectedVictimLabels = set(VICTIM_LEVELS[:victimCount])
		if set(self.victimCells) != expectedVictimLabels:
			raise ValueError(
				f"numberOfVictims={victimCount} requires victimCells labels "
				f"{sorted(expectedVictimLabels)}; call prepare_maze_layout() before validation")

		layout = MazeLayout(
			rows=self.mazeRows,
			columns=self.mazeColumns,
			wall_segments=list(self.mazeWallSegments),
			base_cell=tuple(self.baseCell),
			victim_cells=dict(self.victimCells),
			mode=str(self.mazeGenerationMode).strip().lower(),
			seed=self.activeMazeSeed,
			generation_attempt=self.mazeGenerationAttempt,
		)
		details = validate_maze_layout(layout)
		layout.distances_from_base = details['distances_from_base']
		layout.hazard_cells = details['hazard_cells']
		self.mazeVictimDistances = dict(details['victim_distances'])
		self.mazeHazardCells = list(details['hazard_cells'])
		self.activeMazeLayout = layout
		return details


# Earlier class name retained so existing staff solutions continue to run.
COPPELIA_MazeRobot = MazeBot


