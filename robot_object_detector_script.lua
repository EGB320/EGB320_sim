simVision = require('simVision')
sim = require('sim')

function sysCall_init()
    local handle = sim.getObject('/VisionSensor/ObjectDetector')
    sim.setObjectInt32Param(handle, sim.visionintparam_render_mode, 1)
end

function sysCall_vision(inData)
    -- Maze-only detector packet: obstacle 0, obstacle 1, obstacle 2.
    local detections = {0, 0, 0}

    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToBuffer1(inData.handle)

    local function detectColor(colorRGB, tolerance)
        simVision.buffer1ToWorkImg(inData.handle)
        simVision.selectiveColorOnWorkImg(inData.handle, colorRGB, tolerance, true, true, false)
        local _, packedPacket = simVision.blobDetectionOnWorkImg(
            inData.handle, 0.01, 0.001, false, {1.0, 0.0, 1.0})
        if packedPacket then
            local data = sim.unpackFloatTable(packedPacket, 0, 0, 0)
            return data[1] > 0
        end
        return false
    end

    detections[1] = detectColor({0.02, 1.00, 0.00}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[2] = detectColor({0.15, 1.00, 0.15}, {0.05, 0.05, 0.05}) and 1 or 0
    detections[3] = detectColor({0.00, 0.50, 0.29}, {0.05, 0.05, 0.05}) and 1 or 0

    simVision.addBuffer1ToWorkImg(inData.handle)
    simVision.workImgToSensorImg(inData.handle)

    return {
        trigger = false,
        packedPackets = {sim.packFloatTable(detections)},
    }
end
