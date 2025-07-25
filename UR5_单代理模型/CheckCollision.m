function CheckCollision(clientID,sim,h,zbest)
    %  判断最优解的有效性
    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
    sim.simxPauseCommunication(clientID, 1);
    for n = 1:6
        sim.simxSetJointTargetPosition(clientID,h(n),0,sim.simx_opmode_streaming);
    end
    sim.simxPauseCommunication(clientID, 0);
    pause(3);
    new_goal  = reshape(zbest,6,3)';
    [r1, ~, ~, ~, ~] = sim.simxCallScriptFunction(clientID, 'UR5', sim.sim_scripttype_childscript, 'ClearTable', [], [], [], '', sim.simx_opmode_blocking);
    if r1 == sim.simx_return_ok
      disp('开始判断');
    end
    for m = 1:3
        sim.simxPauseCommunication(clientID, 1);
        for n = 1:6
            sim.simxSetJointTargetPosition(clientID,h(n),new_goal(m,n),sim.simx_opmode_streaming);
        end
        sim.simxPauseCommunication(clientID, 0);
        pause(3);
    end
    [r2, ~, retFloats, ~, ~] = sim.simxCallScriptFunction(clientID, 'UR5', sim.sim_scripttype_childscript, 'sendTableToMatlab', [], [], [], '', sim.simx_opmode_blocking);
    if r2 == sim.simx_return_ok
        if any(retFloats == 0)
            disp('该最优解无效')
        else
            disp('该最优解有效')
        end
    end
end