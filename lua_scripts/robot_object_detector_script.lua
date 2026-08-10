simVision = require('simVision')
sim = require('sim')

function sysCall_init()
    -- Inherit the parent camera's visible-RGB renderer. Render mode 1 is CoppeliaSim's
    -- object-ID colour mode and makes colour detection impossible.
    local ok, handle = pcall(sim.getObject, '.')
    if ok then
        local parentHandle = sim.getObjectParent(handle)
        local renderMode = sim.getObjectInt32Param(parentHandle, sim.visionintparam_render_mode)
        sim.setObjectInt32Param(handle, sim.visionintparam_render_mode, renderMode)
    end
end

function sysCall_vision(inData)
    -- Packet order (also used by mazeObjects in mazebot_lib.py):
    -- obstacle 0, obstacle 1, obstacle 2, base marker, victim marker,
    -- rubble-victim marker, hazard marker, victim object.
    local detections = {0, 0, 0, 0, 0, 0, 0, 0}

    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToBuffer1(inData.handle)

    local function detectColor(colorRGB, tolerance, minimumBlobSize)
        simVision.buffer1ToWorkImg(inData.handle)
        simVision.selectiveColorOnWorkImg(inData.handle, colorRGB, tolerance, true, true, false)
        local _, packedPacket = simVision.blobDetectionOnWorkImg(
            inData.handle, minimumBlobSize or 0.01, 0.001, false, nil)
        if packedPacket then
            local data = sim.unpackFloatTable(packedPacket, 0, 0, 0)
            return (data[1] or 0) > 0
        end
        return false
    end

    detections[1] = detectColor({0.02, 1.00, 0.00}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[2] = detectColor({0.15, 1.00, 0.15}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[3] = detectColor({0.00, 0.50, 0.29}, {0.05, 0.05, 0.05}) and 1 or 0

    -- Solid, untextured detector_marker planes are exposed only inside the Robot
    -- script's atomic handleObjectDetector call. Values match MARKER_EMISSIVE_COLOURS.
    -- Allow target channels to darken under scene lighting while keeping tight
    -- tolerances on channels that should be zero. This rejects grey/white walls.
    detections[4] = detectColor({0.00, 0.00, 1.00}, {0.12, 0.12, 0.60}) and 1 or 0
    detections[5] = detectColor({0.00, 1.00, 1.00}, {0.12, 0.60, 0.60}) and 1 or 0
    detections[6] = detectColor({1.00, 0.00, 1.00}, {0.60, 0.12, 0.60}) and 1 or 0
    detections[7] = detectColor({1.00, 0.00, 0.00}, {0.60, 0.12, 0.12}) and 1 or 0

    -- Detector-only victim proxies are solid emissive yellow. A smaller blob limit is
    -- used because a victim occupies fewer pixels than a wall marker at the same range.
    detections[8] = detectColor(
        {1.00, 1.00, 0.00}, {0.60, 0.60, 0.12}, 0.002) and 1 or 0

    -- Restore the original image. Adding buffer1 to the last binary mask caused the
    -- binary mask to clip and show false colours.
    simVision.buffer1ToWorkImg(inData.handle)
    simVision.workImgToSensorImg(inData.handle)

    return {
        trigger = false,
        packedPackets = {sim.packFloatTable(detections)},
    }
end
