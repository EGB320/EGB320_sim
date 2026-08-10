-- EGB320 Search and Rescue MazeBot control script for CoppeliaSim (2026)
-- Maze generation and robot control are performed from mazebot_lib.py through the
-- ZeroMQ remote API. This simulation script intentionally has no pick-and-place helpers.

sim = require('sim')
simVision = require('simVision')

local objectDetectorHandle = -1
local robotHandle = -1
local visualMarkerHandles = {}
local detectorMarkerHandles = {}
local visualVictimHandles = {}
local victimDetectorHandles = {}

local function refreshGeneratedMarkerHandles()
    visualMarkerHandles = {}
    detectorMarkerHandles = {}
    visualVictimHandles = {}
    victimDetectorHandles = {}

    local ok, handle = pcall(sim.getObject, '/Robot/VisionSensor/ObjectDetector')
    objectDetectorHandle = ok and handle or -1
    ok, handle = pcall(sim.getObject, '/Robot')
    robotHandle = ok and handle or -1

    local shapes = sim.getObjectsInTree(sim.handle_scene, sim.sceneobject_shape, 0)
    for _, shapeHandle in ipairs(shapes) do
        local leafAlias = sim.getObjectAlias(shapeHandle, 0)
        local pathAlias = sim.getObjectAlias(shapeHandle, 1)
        -- A uniquely named first proxy may be reported as /victim_detector_proxy
        -- rather than with its parent path, so classify proxies by their leaf alias.
        if leafAlias == 'victim_detector_proxy' then
            victimDetectorHandles[#victimDetectorHandles + 1] = shapeHandle
        elseif string.find(pathAlias, 'EGB320_GEN_', 1, true) then
            if leafAlias == 'marker' then
                visualMarkerHandles[#visualMarkerHandles + 1] = shapeHandle
            elseif leafAlias == 'detector_marker' then
                detectorMarkerHandles[#detectorMarkerHandles + 1] = shapeHandle
            end
        end
    end

    -- Resolve the visual hierarchy from each proxy's parent instead of matching the
    -- EGB320_GEN_VICTIM_ prefix: victim marker-wall aliases deliberately use the same
    -- prefix and must remain as structural detector occluders.
    for _, proxyHandle in ipairs(victimDetectorHandles) do
        local victimRootHandle = sim.getObjectParent(proxyHandle)
        local victimShapes = sim.getObjectsInTree(
            victimRootHandle, sim.sceneobject_shape, 0)
        for _, victimShapeHandle in ipairs(victimShapes) do
            if victimShapeHandle ~= proxyHandle then
                visualVictimHandles[#visualVictimHandles + 1] = victimShapeHandle
            end
        end
    end
end

local function setVisibilityLayers(handles, layer)
    for _, handle in ipairs(handles) do
        sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, layer)
    end
end

local function isDescendantOf(handle, ancestorHandle)
    local currentHandle = handle
    while currentHandle ~= -1 do
        if currentHandle == ancestorHandle then
            return true
        end
        currentHandle = sim.getObjectParent(currentHandle)
    end
    return false
end

local function showUncollectedVictimDetectors()
    for _, handle in ipairs(victimDetectorHandles) do
        -- A collected victim is parented beneath /Robot/VictimCarryPoint. Keep its
        -- proxy hidden so the robot cannot detect the victim it is already carrying.
        if robotHandle ~= -1 and isDescendantOf(handle, robotHandle) then
            sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 0)
        else
            sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 1)
        end
    end
end

local function detectObjectColours(sensorHandle)
    -- Packet order (also used by mazeObjects in mazebot_lib.py): obstacle 0,
    -- obstacle 1, obstacle 2, base marker, victim marker, rubble-victim marker,
    -- hazard marker, victim object.
    local detections = {0, 0, 0, 0, 0, 0, 0, 0}

    simVision.sensorImgToWorkImg(sensorHandle)
    simVision.workImgToBuffer1(sensorHandle)

    local function detectColor(colorRGB, tolerance, minimumBlobSize)
        simVision.buffer1ToWorkImg(sensorHandle)
        simVision.selectiveColorOnWorkImg(
            sensorHandle, colorRGB, tolerance, true, true, false)
        local _, packedPacket = simVision.blobDetectionOnWorkImg(
            sensorHandle, minimumBlobSize or 0.01, 0.001, false, nil)
        if packedPacket then
            local data = sim.unpackFloatTable(packedPacket, 0, 0, 0)
            return (data[1] or 0) > 0
        end
        return false
    end

    detections[1] = detectColor({0.02, 1.00, 0.00}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[2] = detectColor({0.15, 1.00, 0.15}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[3] = detectColor({0.00, 0.50, 0.29}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[4] = detectColor({0.00, 0.00, 1.00}, {0.12, 0.12, 0.60}) and 1 or 0
    detections[5] = detectColor({0.00, 1.00, 1.00}, {0.12, 0.60, 0.60}) and 1 or 0
    detections[6] = detectColor({1.00, 0.00, 1.00}, {0.60, 0.12, 0.60}) and 1 or 0
    detections[7] = detectColor({1.00, 0.00, 0.00}, {0.60, 0.12, 0.12}) and 1 or 0
    detections[8] = detectColor(
        {1.00, 1.00, 0.00}, {0.60, 0.60, 0.12}, 0.002) and 1 or 0

    -- Leave the sensor image showing the original RGB render rather than the last mask.
    simVision.buffer1ToWorkImg(sensorHandle)
    simVision.workImgToSensorImg(sensorHandle)
    return detections
end

function sysCall_init()
    print('Initializing EGB320 search and rescue maze robot script...')
    refreshGeneratedMarkerHandles()
end

-- Called by mazebot_lib.py. All visibility changes and the detector render occur
-- in one simulator-side call, so the editor never displays an intermediate state.
-- Crucially, this does not modify sim.intparam_visible_layers.
function handleObjectDetector()
    if objectDetectorHandle == -1 then
        refreshGeneratedMarkerHandles()
    end
    if objectDetectorHandle == -1 then
        return -1, {}, {}
    end

    setVisibilityLayers(visualMarkerHandles, 0)
    setVisibilityLayers(visualVictimHandles, 0)
    setVisibilityLayers(detectorMarkerHandles, 1)
    showUncollectedVictimDetectors()

    local ok, result, data = pcall(sim.handleVisionSensor, objectDetectorHandle)

    -- Always restore the student-facing scene, including when detector handling fails.
    setVisibilityLayers(detectorMarkerHandles, 0)
    setVisibilityLayers(victimDetectorHandles, 0)
    setVisibilityLayers(visualMarkerHandles, 1)
    setVisibilityLayers(visualVictimHandles, 1)

    if not ok then
        error(result)
    end

    -- CoppeliaSim suppresses a vision sensor's associated sysCall_vision callback when
    -- handling that sensor from inside another simulation-script callback. Process the
    -- freshly rendered image here so the atomic API always returns a detection packet.
    local detections = detectObjectColours(objectDetectorHandle)
    return result, data, detections
end

function sysCall_actuation()
    -- Wheel target velocities are set directly by the Python API.
end

function sysCall_sensing()
    -- Camera and proximity sensors are read directly by the Python API.
end

function sysCall_cleanup()
    print('Cleaning up maze robot script...')
end

print('EGB320 search and rescue maze robot script loaded successfully')
