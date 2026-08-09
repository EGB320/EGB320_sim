-- EGB320 Search and Rescue MazeBot control script for CoppeliaSim (2026)
-- Maze generation and robot control are performed from mazebot_lib.py through the
-- ZeroMQ remote API. This simulation script intentionally has no pick-and-place helpers.

sim = require('sim')

function sysCall_init()
    print('Initializing EGB320 search and rescue maze robot script...')
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
