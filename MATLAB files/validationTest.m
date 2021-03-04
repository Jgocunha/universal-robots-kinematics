%% Initialization

disp('Program started');

dof=6; % degrees of freedom UR10
d=zeros(1,dof+1); %distances
a=zeros(1,dof); %distances
theta=zeros(1,dof); %joint angles
jh=zeros(1,dof); %CoppeliaSim joint handles
totalIKsol=8; %number of inverse kinematic solutions
printTimeInterval=0.006; %correct time interval for printing variables in CoppeliaSim
acc_tip_pose=zeros(1,6); %accumulative tip pose (retrieved from CoppeliaSim)
fwd_tip_pose=zeros(1,6); % tip pose returned from Forward Kinematics
iterations=1000; %number of code iterations

ValFile='.\ValidationOutput\stdev.xlsx'; % compound matrices output file

% Do the connection with CoppeliaSim

% sim=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

%% Denavit-Hartenberg parameters (User interface)

% CoppeliaSim link dimensions for the UR10 model
d(1)=0.828-0.719;
d(2)=0.10122;
d(3)=0.12067-0.10122;
d(4)=0.11406-0.12067;
d(5)=0.17246-0.11406;
d(6)=0.23-0.17246;
d(7)=0.36594-0.22976;

a(2)=1.4401-0.828;
a(3)=2.0123-1.4401;
a(4)=2.069-2.0123;
a(5)=2.128-2.069;

%% Main program

if (clientID>-1)
    disp('Connected to remote API server');
    
    % Retreive joint handles from CoppeliaSim
    for i = 1 : dof
        [r,jh(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
    end
    
    for it = 1 : iterations
        
        % Set random joint values 
        %for j = 1 : dof
            theta(1)=randi([-360 360],1,1);
            theta(2)=randi([-360 360],1,1);
            theta(3)=randi([-360 360],1,1);
            theta(4)=randi([-360 360],1,1);
            theta(5)=randi([-360 360],1,1);
            theta(6)=randi([-360 360],1,1);
        %end
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
        
         % Determine the number of reference frames using the DHMatrix
        numFrames=size(DHMatrix);
        numFrames=numFrames(1);
        disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
        %% Compute forward kinematics
        % Time and execute the function
        f=@() fwdKin(DHMatrix);
        t(1)=timeit(f);
        M=fwdKin(DHMatrix);
        R=M{2}{numFrames-1}([1,2,3],[1,2,3]);
        disp('Forward kinematics solution')
        % Print end-effector position
        disp('End effector position in meters:')
        disp(M{2}{numFrames}(:,4).');
        % Print robot's tip position
        disp('Robot´s tip position in meters:')
        fwd_tip_pos = M{2}{numFrames-1}(:,4).';
        disp(fwd_tip_pos(1:3));
        % Print robot's tip and end-effector orientation
        disp('Robot´s tip and end-effector orientation in degrees:')
        fwd_tip_ori = RPY(R);
        disp(fwd_tip_ori);
        fwd_tip_pose = [fwd_tip_pos(1:3) fwd_tip_ori];
        disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
        %% Compute inverse kinematics
        % Time and execute the function
        f=@() invKin8sol(d,a,M{2}{numFrames}(:,:));
        t(2)=timeit(f);
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
             % Allow Coppelia to display and send tip pose
             sim.simxSetIntegerSignal(clientID, 'showPos', 1, sim.simx_opmode_blocking);
             pause(printTimeInterval);
             % Retrieve correspondig tip pose from CoppeliaSim
             [r, stringValue] = sim.simxGetStringSignal(clientID, 'tip_pose_final',sim.simx_opmode_blocking);
             % Unpack returned string to float format and sum it to the previous one
             acc_tip_pose = acc_tip_pose + sim.simxUnpackFloats(stringValue);
             sim.simxSetIntegerSignal(clientID, 'showPos', 0, sim.simx_opmode_blocking);
         end
         %% Get deviation of tip pose (CoppeliaSim Vs. FwdKin) - Validation
         % Get average tip pose from CoppeliaSim
         avg_tip_pose = acc_tip_pose/totalIKsol;
         acc_tip_pose=0;
         std_dev = avg_tip_pose - fwd_tip_pose;
         excelValues=[std_dev,t]; %append time to std_dev - send to Excel sheet
         writematrix(excelValues,ValFile,'WriteMode','append');
    end
else
    disp('Failed connecting to remote API server');
end

sim.delete(); % call the destructor!

disp('Program ended');