function sysCall_threadmain()

    --[[ Initialization ]]--

    --Start MATLAB api
    simRemoteApi.start(19999)

    --Handle retreival
    jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR10_joint'..i)
    end
    UR10=sim.getObjectHandle('UR10')
    tip=sim.getObjectHandle('UR10_tip')
    
    --[[ Main loop ]]--
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        -- get tip position relative to the robots' base
        tip_pos=sim.getObjectPosition(tip,UR10)
        -- print tip position
        print('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
        print('End-effector position')
        print(tip_pos)
        -- wait
        sim.wait(10)
    end
    
end
