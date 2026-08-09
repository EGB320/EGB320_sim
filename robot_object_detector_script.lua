simVision = require('simVision')
sim = require('sim')



function sysCall_init()
    local handle = sim.getObject('/VisionSensor/ObjectDetector')
    sim.setObjectInt32Param(handle, sim.visionintparam_render_mode, 1)
    -- do some initialization here
end

function sysCall_actuation()
    -- put your actuation code here
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end

function sysCall_vision(inData)
    -- callback function automatically added for backward compatibility
    -- (vision sensor have no filters anymore, but rather a callback function where image processing can be performed)
    
    --returns detection array for: bowl,mug,bottle,soccer ball,rubiks cube,cereal box,obstacle_0,obstacle_1,obstacle_2,packing_bay,row marker1,row marker2,row marker3,shelf0-5,square_marker1-3,picking_station
    unpackedPackets={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
    
    -- Copy sensor image to work image and save to buffer1 for restoration
    simVision.sensorImgToWorkImg(inData.handle)
    simVision.workImgToBuffer1(inData.handle)  -- Save original image to buffer1
    
    ----
    -- Optimized detection: just check if selective color finds any matching pixels
    ----
    
    -- Helper function to check if any pixels match the color selection
    local function detectColor(colorRGB, tolerance, index, name)        
        simVision.buffer1ToWorkImg(inData.handle)  -- Restore original
        simVision.selectiveColorOnWorkImg(inData.handle, colorRGB, tolerance, true, true, false)
        
        -- Try the original blob detection but with minimal parameters for speed
        local trig, packedPacket = simVision.blobDetectionOnWorkImg(inData.handle, 0.01, 0.001, false, {1.0,0.0,1.0})
        
        if packedPacket then
            local data = sim.unpackFloatTable(packedPacket, 0,0,0)
            if data[1] > 0 then  -- If any blobs found
                return true
            end
        end
        return false
    end

    -- All objects using optimized helper function
    unpackedPackets[1] = detectColor({1.00,0.48,0.00}, {0.05,0.05,0.05}, 1, "BOWL") and 1 or 0
    unpackedPackets[2] = detectColor({1.00,0.00,0.60}, {0.05,0.05,0.05}, 2, "MUG") and 1 or 0
    unpackedPackets[3] = detectColor({1.00,0.00,0.00}, {0.05,0.05,0.05}, 3, "BOTTLE") and 1 or 0
    unpackedPackets[4] = detectColor({0.00,0.58,1.00}, {0.05,0.05,0.05}, 4, "BALL") and 1 or 0
    unpackedPackets[5] = detectColor({0.02,0.00,1.00}, {0.05,0.05,0.05}, 5, "RUBIKS_CUBE") and 1 or 0
    unpackedPackets[6] = detectColor({0.62,0.00,1.00}, {0.05,0.05,0.05}, 6, "CEREAL") and 1 or 0
    
    -- Obstacles
    unpackedPackets[7] = detectColor({0.02,1.00,0.00}, {0.05,0.05,0.05}, 7, "OBSTACLE_0") and 1 or 0
    unpackedPackets[8] = detectColor({0.15,1.00,0.15}, {0.05,0.05,0.05}, 8, "OBSTACLE_1") and 1 or 0
    unpackedPackets[9] = detectColor({0.00,0.50,0.29}, {0.05,0.05,0.05}, 9, "OBSTACLE_2") and 1 or 0
    
    -- Packing bay
    unpackedPackets[10] = detectColor({0.17,1.00,0.5}, {0.04,0.50,1.0}, 10, "PACKING_BAY") and 1 or 0
    
    -- Row markers
    unpackedPackets[11] = detectColor({0.15,0.10,0.05}, {0.02,0.02,0.02}, 11, "ROW_MARKER_1") and 1 or 0
    unpackedPackets[12] = detectColor({0.10,0.15,0.05}, {0.02,0.02,0.02}, 12, "ROW_MARKER_2") and 1 or 0
    unpackedPackets[13] = detectColor({0.05,0.10,0.15}, {0.02,0.02,0.02}, 13, "ROW_MARKER_3") and 1 or 0
    
    -- Shelves
    unpackedPackets[14] = detectColor({0.33,0.306,0.27}, {0.02,0.02,0.02}, 14, "SHELF_0") and 1 or 0
    unpackedPackets[15] = detectColor({0.318,0.33,0.27}, {0.02,0.02,0.02}, 15, "SHELF_1") and 1 or 0
    unpackedPackets[16] = detectColor({0.282,0.33,0.27}, {0.02,0.02,0.02}, 16, "SHELF_2") and 1 or 0
    unpackedPackets[17] = detectColor({0.27,0.33,0.294}, {0.02,0.02,0.02}, 17, "SHELF_3") and 1 or 0
    unpackedPackets[18] = detectColor({0.27,0.33,0.33}, {0.02,0.02,0.02}, 18, "SHELF_4") and 1 or 0
    unpackedPackets[19] = detectColor({0.27,0.294,0.33}, {0.02,0.02,0.02}, 19, "SHELF_5") and 1 or 0
    
    -- Square markers
    unpackedPackets[20] = detectColor({0.75,0.25,0.50}, {0.05,0.05,0.05}, 20, "SQUARE_MARKER_1") and 1 or 0
    unpackedPackets[21] = detectColor({0.50,0.75,0.25}, {0.05,0.05,0.05}, 21, "SQUARE_MARKER_2") and 1 or 0
    unpackedPackets[22] = detectColor({0.25,0.50,0.75}, {0.05,0.05,0.05}, 22, "SQUARE_MARKER_3") and 1 or 0
    
    -- Picking station
    unpackedPackets[23] = detectColor({0.90,0.45,0.20}, {0.05,0.05,0.05}, 23, "PICKING_STATION") and 1 or 0
    
    simVision.addBuffer1ToWorkImg(inData.handle)
    simVision.workImgToSensorImg(inData.handle)
    
    --print(unpackedPackets)

    outData={}
    outData.trigger=false -- whether the sensor should trigger
    outData.packedPackets={sim.packFloatTable(unpackedPackets)}
    --outData.packedPackets={sim.packFloatTable(unpackedPackets)} -- filters may append packets (in packed form, use sim.packFloatTable to pack) to this table
    return outData
    
end

-- See the user manual or the available code snippets for additional callback functions and details
