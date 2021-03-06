function runSim(robot, theta1, theta2, theta3, theta4, theta5, theta6)

%% Initialization

disp('Program started');

dof=6; % degrees of freedom UR10
d=zeros(1,dof+1); %distances
a=zeros(1,dof); %distances
theta=zeros(1,dof); %joint angles
jh=zeros(1,dof); %CoppeliaSim joint handles
totalIKsol=8; %number of inverse kinematic solutions
printTimeInterval=0.006; %correct time interval for printing variables in CoppeliaSim

% Do the connection with CoppeliaSim

% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Denavit-Hartenberg parameters (User interface)

% CoppeliaSim link dimensions for the UR10 model (to get better results)

% d(1)=0.828-0.719;
% d(2)=0.09265;
% d(3)=0.121-0.09265;
% d(4)=0.10549-0.121;
% d(5)=0.16389-0.10549;
% d(6)=0.25718-0.16389;
% d(7)=0.35736-0.25718;
% 
% a(2)=1.4401-0.828;
% a(3)=2.0123-1.4401;
% a(4)=2.0696-2.0123;
% a(5)=2.128-2.0696;

d(1)=0.109;
d(2)=0.10122;
d(3)=0.12067-0.10122;
d(4)=0.11406-0.12067;
d(5)=0.17246-0.11406;
d(6)=0.26612-0.17246;
d(7)=0.36474-0.26612;


a(2)=0.7211-0.109;
a(3)=1.2933-0.7211;
a(4)=1.3506-1.2933;
a(5)=1.409-1.3506;


% Target joint angles
theta(1)=theta1;
theta(2)=theta2;
theta(3)=theta3;
theta(4)=theta4;
theta(5)=theta5;
theta(6)=theta6;

%Definition of the modified Denavit-Hartenberg matrix (Do not change!)
DHMatrix = [ 0         0       d(1)   theta(1);    % 1  0T1 
            -90        0       d(2)   theta(2)-90; % 2  1T2 
             0         a(2)    d(3)   theta(3);    % 3  2T3 
             0         a(3)    d(4)   theta(4);    % 4  3T4
             0         a(4)    d(5)   90;          % 4' 4T4' 5
             90        0       0      theta(5);    % 5  4'T5 6
            -90        0       0      -90;         % 5' 5T5' 7
             0         a(5)    d(6)   theta(6);    % 6  5'T6 8
             0         0       d(7)     0;];       % 7  6T7  9

%% Main program

% Determine the number of reference frames using the DHMatrix
numFrames=size(DHMatrix);
numFrames=numFrames(1);

if (clientID>-1)
    disp('Connected to remote API server');
    
    % Retreive joint handles from CoppeliaSim
    for i = 1 : dof
        [~,jh(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
    end
    disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
    %% Compute forward kinematics
    M=fwdKin(DHMatrix);
    R=M{2}{numFrames-1}([1,2,3],[1,2,3]);
    disp('Forward kinematics solution')
    % Print end-effector position
    disp('End effector position in meters:')
    disp(M{2}{numFrames}(:,4).');
    % Print robot's tip position
    disp('Robot´s tip position in meters:')
    fwd_tip_pose = M{2}{numFrames-1}(:,4).';
    disp(fwd_tip_pose(1:3));
    % Print robot's tip and end-effector orientation
    disp('Robot´s tip and end-effector orientation in degrees:')
    disp(RPY(R));
    disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
    %% Compute inverse kinematics
    joints=(invKin8sol(d,a,M{2}{numFrames}(:,:)));
    % Print the joint values for every IK solution
    disp('Inverse kinematics solutions:')
    disp(int32(rad2deg(joints(:,:))));
     
    % Send joint values to CoppeliaSim
     for i = 1: totalIKsol
         disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
         fprintf('Inverse kinematic solution %d\n',i);
         disp('Value in degrees');
         disp(rad2deg(joints(i,:)));
         disp('Value in radians');
         disp(joints(i,:));
         pause(0.5);
         for j = 1 : dof
             % Send joint values for robot model
             sim.simxSetJointTargetPosition(clientID, jh(j), (joints(i,j)), sim.simx_opmode_blocking);
         end
         pause(0.5);
         % Allow Coppelia to display tip pose
         sim.simxSetIntegerSignal(clientID, 'showPos', 1, sim.simx_opmode_blocking);
         pause(printTimeInterval);
         sim.simxSetIntegerSignal(clientID, 'showPos', 0, sim.simx_opmode_blocking);
     end
else
    disp('Failed connecting to remote API server');
end

sim.delete(); % call the destructor!

disp('Program ended');

end

