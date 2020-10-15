% Make sure to have the server side running in CoppeliaSim: 
% in a child script of a CoppeliaSim scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!


disp('Program started');
% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    % joint handles
    h=[0, 0, 0, 0, 0, 0];
    [r,h(1)]=sim.simxGetObjectHandle(clientID, 'UR10e_joint1', sim.simx_opmode_blocking);
    [r,h(2)]=sim.simxGetObjectHandle(clientID, 'UR10e_joint2', sim.simx_opmode_blocking);
    [r,h(3)]=sim.simxGetObjectHandle(clientID, 'UR10e_joint3', sim.simx_opmode_blocking);
    [r,h(4)]=sim.simxGetObjectHandle(clientID, 'UR10e_joint4', sim.simx_opmode_blocking);
    [r,h(5)]=sim.simxGetObjectHandle(clientID, 'UR10e_joint5', sim.simx_opmode_blocking);
    [r,h(6)]=sim.simxGetObjectHandle(clientID, 'UR10e_joint6', sim.simx_opmode_blocking);
    
    while true
        sim.simxSetJointTargetPosition(clientID, h(1), pi/2, sim.simx_opmode_streaming);
    end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');
