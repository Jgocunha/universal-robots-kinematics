function __getObjectPosition__(a,b)
    -- compatibility routine, wrong results could be returned in some situations, in CoppeliaSim <4.0.1
    if b==sim.handle_parent then
        b=sim.getObjectParent(a)
    end
    if (b~=-1) and (sim.getObjectType(b)==sim.object_joint_type) and (sim.getInt32Parameter(sim.intparam_program_version)>=40001) then
        a=a+sim.handleflag_reljointbaseframe
    end
    return sim.getObjectPosition(a,b)
end


function TableConcat(t1,t2)
    for i=1,#t2 do
        t1[#t1+1] = t2[i]  --corrected bug. if t1[#t1+i] is used, indices will be skipped
    end
    return t1
end


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
    local tip=sim.getObjectHandle('tip')

    -- Auxiliary variables for outputting current end-effector position
    sim.setIntegerSignal('showPos',0)
    sim.setStringSignal('tip_pose_final', 'NULL')
    local IKsol=0;
    
    --[[ Main loop ]]--
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        if(sim.getIntegerSignal('showPos')==1) then
            sim.setIntegerSignal('showPos',0)
            -- get tip pose relative to the robots' base
            local tip_pos=__getObjectPosition__(tip,UR10)
            local tip_ori_aux=sim.getObjectOrientation(tip,-1)
            local tip_ori={-1,-1,-1}
            tip_ori[1]=tip_ori_aux[1]*180/math.pi
            tip_ori[2]=tip_ori_aux[2]*180/math.pi
            tip_ori[3]=tip_ori_aux[3]*180/math.pi
            -- print tip pose
            IKsol=IKsol+1
            print('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
            print('Tip pose position '.. IKsol)
            print(string.format("{ x = %.3f, y = %.3f, z = %.3f }",tip_pos[1],tip_pos[2],tip_pos[3]))
            print(string.format("{ a = %.3f, b = %.3f, g = %.3f }",tip_ori[1],tip_ori[2],tip_ori[3]))
            -- send tip pose to Matlab
            local packed_tip_pos=sim.packFloatTable(TableConcat(tip_pos,tip_ori))
            sim.setStringSignal('tip_pose_final', packed_tip_pos)
            -- reset value of IKsol, when reaching final solution
            if(IKsol==8) then
                IKsol=0
            end
        end
    end
    
end
