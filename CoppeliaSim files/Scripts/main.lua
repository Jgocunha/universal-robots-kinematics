function sysCall_threadmain()

    --[[ Initialization ]]--

    --Start MATLAB api
    simRemoteApi.start(19999)

    --Handle retreival
    local jointHandles={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jointHandles[i]=sim.getObjectHandle('UR10_joint'..i)
    end
    local UR10=sim.getObjectHandle('UR10')
    local tip=sim.getObjectHandle('UR10_tip')
    
    -- Auxiliary variables for outputting current end-effector position
    sim.setIntegerSignal('showPos',0)
    local IKsol=1;
    
    --[[ Main loop ]]--
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        if(sim.getIntegerSignal('showPos')==1) then
            sim.setIntegerSignal('showPos',0)
            -- get tip position relative to the robots' base
            local tip_pos=sim.getObjectPosition(tip,UR10)
            -- print tip position
            print('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
            print('End-effector position '.. IKsol)
            print(tip_pos)
            IKsol=IKsol+1
        end
    end
    
end
