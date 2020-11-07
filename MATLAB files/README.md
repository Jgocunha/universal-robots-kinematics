# MATLAB code

The MATLAB code is divided into three function files: *MDHMatrix.m*; *fwdKin.m*; and *invKin.m*, and a single script file: *main.m*.

## *MDHMatrix.m* Function File

The *MDHMatrix.m* file contains a function capable of computing Modified Denavit-Hartenberg matrices. It receives as input a single line of a Denavit-Hartenberg matrix, *i.e.* a vector of 4 values [$\alpha$; $a$, $d$, $\theta$] that indicate the rotation around the x axis of frame i (in deg.), the translation along the x axis of frame i (in meters), the translation along the z axis of frame i (in meters), and the rotation around the z axis of frame i (in deg.), respectively, and outputs a 4x4 transformation matrix associated to that line of the Denavit-Hartenberg matrix. Summarily, it computes the transformation matrix from one system of coordinates to another.

```MATLAB
%% Modified Denavit-Hartenberg Matrix

% Transformation matrix from a system of coordinates to another.

%% Function: MDHMatrix 
% 
% Computes the transformation matrix from a system of coordinates to
% another.
% 
% In: DHparams - Denavit-Hartenberg parameters matrix
%       |_ alpha - rotation around the x axis of frame i (deg)
%       |_ a     - translation along the x axis of frame i (meters)
%       |_ d     - translation along the z axis of frame i (meters)
%       |_ theta - rotation around the z axis of frame i (deg)
%
% Out: 4x4 transformation matrix from frame i-1 to i
%%

% Use this function to test the program regularly
function T = MDHMatrix(DHparams)
    T = [ cosd(DHparams(4))                                 -sind(DHparams(4))                      0                 DHparams(2)     ;
          sind(DHparams(4))*cosd(DHparams(1))     cosd(DHparams(4))*cosd(DHparams(1))       -sind(DHparams(1))     -sind(DHparams(1))*DHparams(3);
          sind(DHparams(4))*sind(DHparams(1))     cosd(DHparams(4))*sind(DHparams(1))       cosd(DHparams(1))      cosd(DHparams(1))*DHparams(3) ;
                    0                                               0                               0                       1    ];
end

% Use this function to print matrices
% function T = MDHMatrix(DHparams)
%     T = [ cos(DHparams(4))                                 -sin(DHparams(4))                      0                 DHparams(2)     ;
%           sin(DHparams(4))*cos(DHparams(1))     cos(DHparams(4))*cos(DHparams(1))       -sin(DHparams(1))     -sin(DHparams(1))*DHparams(3);
%           sin(DHparams(4))*sin(DHparams(1))     cos(DHparams(4))*sin(DHparams(1))       cos(DHparams(1))      cos(DHparams(1))*DHparams(3) ;
%                     0                                               0                               0                       1    ];
% end
````



## *fwdKin.m* Function File

The *fwdKin.m* file is responsible for determining the general transformation matrix from the Denavit-Hartenberg parameters matrix. It returns a bi-dimensional array of arrays of matrices named *M*, where *M{1}* contains an array with the individual transformation matrices, and *M{2}* an array with the compound transformation matrices. To compute the individual transformation matrices the *MDHMatrix* function is used.

```MATLAB
%% Forward kinematics
%
% 1st - Assign the reference frames along the robot's structure
% 2nd - Define the Denavit-Hartenberg parameters
% 3rd - Determine the individual transformation matrices
% 4th - Determine the general transformation matrix to obtain the Cartesian
% coordinates of the robot's tip in terms of its joint values.
%

%% Function: fwdKin 
% 
% Computes the position and orientation of a robot's tip.
% 
% In: numFrames - number of reference frames (int)
%     DHMatrix - modified DH matrix (matrix)
%
% Out: M - two dimensional array of arrays of matrices
%       |_ M{1} - iT - array of individual transformation matrices
%       |_ M{2} - T0 - array of compound transformation matrices
%%

function M = fwdKin(DHMatrix)
    
    numFrames=size(DHMatrix);
    numFrames=numFrames(1);
    M=cell(1,2);
    %% Determine the individual transformation matrices
    
    %Create a cell array of individual transformations matrices i-1Ti according to
    %the number of reference frames
    iT = cell(1, numFrames);
    for i = 1 : numFrames
        iT{i} = zeros([4 4]);
    end
    
    %Compute the matrices
    for i = 1:numFrames
        iT{i} = MDHMatrix(DHMatrix(i,:));
    end
    
    M{1}=iT;
    
    %% Determine the general transformation matrix
    
    %Create a cell array of compound transformations matrices 0Ti according to
    %the number of reference frames
    T0 = cell(1, numFrames);
    for i = 1 : numFrames-1
        T0{i} = zeros([4 4]);
    end
    
    T0{1}=iT{1};
    
    %Compute the matrix
    for i = 2:numFrames
        T0{i} = T0{i-1}*iT{i};
    end
    
    M{2}=T0;
end
```


## *invKin.m* Function File

The *invKin.m* file computes the eight inverse kinematic solutions for the robot's joint angles using the equations from the geometric values of the robot's dimensions and the end-effector position. As input it receives two arrays with the robots link dimensions and the position and orientation of the end-effector, as output it returns a 8x6 matrix with the eight inverse solutions for the robot's six joints.

```MATLAB
%% Inverse kinematics
%
% Using the ee position and orientation, determine the joint values.
% 
%%

%% Function: invKin 
% 
% Computes the joint values of the UR robots from their mechanical
% dimensions and end-effector position
% 
% In: d - array with robots dimensions
%     a - array with robots dimensions
%     eePosOri - 4x4 matrix with the position and orientation of the ee
%
% Out: joint - 8x6 matrix with the 8 ik solutions for the 6 joints
%%

function joint=invKin8sol(d, a, eePosOri)
    % 8 inv kin solutions, 6 joints
    ikSol=8;
    numJoints=6;
    joint=zeros(ikSol,numJoints);
    
    %% Computing theta1
    
    % 0P5 position of reference frame {5} in relation to {0}
    P=eePosOri*[0 0 -d(6)-d(7) 1].';
    
    psi=atan2(P(2,1),P(1,1));
    
    % There are two possible solutions for theta1, that depend on whether
    % the shoulder joint (joint 2) is left or right
    phi=acos( (d(2)+d(3)+d(4)+d(5)) / sqrt( P(2,1)^2 + P(1,1)^2 ));

    for i = 1:8
        joint(i,1)=(pi/2+psi-phi)-pi;
    end
    for i = 1:4
        joint(i,1)=(pi/2+psi+phi)-pi;
    end
    
    % From the eePosOri it is possible to know the tipPosOri    
    T_67=MDHMatrix([0 0 d(7) 0]);
    %T_06=T_07*T_76
    T_06=eePosOri*inv(T_67);

    for j = 1:ikSol
        %% Computing theta5
        
        % Knowing theta1 it is possible to know 0T1
        T_01=MDHMatrix([0 0 d(1) rad2deg(joint(j,1))]);

        % 1T6 = 1T0 * 0T6
        T_16 = inv(T_01)*T_06;
        %1P6
        P_16=T_16(:,4);
        
        %Wrist up or down 
        if(ismember(j,[1,2,5,6]))
           joint(j,5)=((acos((P_16(2,1)-(d(2)+d(3)+d(4)+d(5)))/d(6))));
        else
            joint(j,5)=(-(acos((P_16(2,1)-(d(2)+d(3)+d(4)+d(5)))/d(6))));
        end
        
        % Fix Error using atan2 / Inputs must be real.
        joint=real(double(rad2deg(joint)));
        joint=deg2rad(joint);

        %% Computing theta 6

         T_61=inv(T_16);
         % y1 seen from frame 6
         Y_16=T_61(:,2);
        
         % If theta 5 is equal to zero give arbitrary value to theta 6
        if(int8(rad2deg(real(joint(j,5)))) == 0 || int8(rad2deg(real(joint(j,5)))) == 2*pi)
            joint(j,6)=deg2rad(0);
        else
            joint(j,6)= (pi/2 + atan2( -Y_16(2,1)/sin(joint(j,5))  , Y_16(1,1)/sin(joint(j,5))));
        end
        
        %% Computing theta 3, 2 and 4
        
        %Get position of frame 4 from frame 1, P_14
    
        %The value of T_01, T_45, and T_56 are known since joints{1,5,6} have
        %been obtained
        
        %T_45 = T_44'*T_4'5
        T_44_=MDHMatrix([0         a(4)    d(5)   90]);
        T_4_5=MDHMatrix([90        0       0      rad2deg(joint(j,5))]);
        T_45 = T_44_*T_4_5;
        
        %T_56 = T_55'*T_5'6
        T_55_=MDHMatrix([-90        0       0      -90]);
        T_5_6=MDHMatrix([0         a(5)    d(6)   rad2deg(joint(j,6))]);
        T_56 = T_55_*T_5_6;
        
        T_46 = T_45*T_56;
        
        T_64 = inv(T_46);
    
        T_14=T_16*T_64;

        %P_14 is the fourth column of T_14
        P_14=T_14(:,4);

        P_14_xz=sqrt( P_14(1)^2 +  P_14(3)^2);
        
        %Elbow up or down
        if(rem(j,2)==0)
            psi= acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) );
            joint(j,3)=( pi - psi);
            % Masking theta3 for CoppeliaSim (invert value for ang>180)
            if(joint(j,3)>pi)
                joint(j,3)=joint(j,3)-pi*2;
            end
            % theta 2
            joint(j,2) =pi/2 - (atan2( P_14(3), +P_14(1)) + asin( (a(3)*sin(psi))/P_14_xz));
            % theta 4
            % Fix Error using atan2 / Inputs must be real.
            joint=real(double(rad2deg(joint)));
            joint=deg2rad(joint);
            joint(j,4) = joint4(d, a, joint(j,2), joint(j,3), T_06, T_01, T_64);
        else
            psi= acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) );
            joint(j,3)=( pi + psi);
            % Masking theta3 for CoppeliaSim (invert value for ang>180)
            if(joint(j,3)>pi)
                joint(j,3)=joint(j,3)-pi*2;
            end
            % theta 2
            joint(j,2) = pi/2 - (atan2( P_14(3), +P_14(1)) + asin( (a(3)*sin(-psi))/P_14_xz));
            % theta 4
            % Fix Error using atan2 / Inputs must be real.
            joint=real(double(rad2deg(joint)));
            joint=deg2rad(joint);
            joint(j,4) = joint4(d, a, joint(j,2), joint(j,3), T_06, T_01, T_64);
        end
    end
    % Fix Error using atan2 / Inputs must be real.
    joint=real(double(rad2deg(joint)));
    joint=deg2rad(joint);
end

%% Computing theta4
function joint4=joint4(d, a, theta2, theta3, T_06, T_01, T_64)

    T_12=MDHMatrix([-90        0       d(2)   rad2deg(theta2)-90]);
    T_23=MDHMatrix([0         a(2)    d(3)   rad2deg(theta3)]);
    T_03=T_01*T_12*T_23;
    
    T_36=inv(T_03)*T_06;
    T_34=T_36*T_64;
    
    x_34=T_34(:,1);
    
    joint4 = (atan2(x_34(2),x_34(1)));
end
```


## *main.m* Script File


The *main.m* script is where the *fwdKin* and *invKin* functions are called. This script can be broken down into three sections:

- **Initialisation**: where variables are initialised and the connection with *CoppeliaSim* is set up;
- **User Interface**: where the user specifies the values of the dimensions of the robot, and the target joint values. It is also here that the Denavit-Hartenberg matrix is coded;
- **Main Program**: section responsible for checking the connection with *CoppeliaSim*, retrieving the joint handles from the robot model in *CoppeliaSim*, computing the forward and inverse kinematics for the target joint values, and sending the output from the inverse kinematic solution to the robot model in *CoppeliaSim*.

This program is intended to test the correct computation of the end-effector position from a set of target joint values, and vice-versa, compared to the physical representation of the robot in *CoppeliaSim*.

```MATLAB
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

% Distances from the mechanical drawing of the UR10
% These values can be changed to accomodate any of the UR series robots
% d(1)=0.128;
% d(2)=0.088;
% d(3)=0.024;
% d(4)=-0.006;
% d(5)=0.058; 
% d(6)=0.092;
% d(7)=0.10;
% 
% a(2)=0.612;
% a(3)=0.572;
% a(4)=0.058;
% a(5)=0.058;

% CoppeliaSim link dimensions for the UR10 model
d(1)=0.109;
d(2)=0.101222;
d(3)=0.01945;
d(4)=-0.006;
d(5)=0.0585; 
d(6)=0.0572+0.03434;%to the tip
d(7)=0.10185;

a(2)=0.612;
a(3)=0.573;
a(4)=0.0567;
a(5)=0.059;

% Target joint angles
% Select here the target joint angle you want the robot to assume
theta(1)=35;
theta(2)=0;
theta(3)=90;
theta(4)=50;
theta(5)=30;
theta(6)=20;

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
        [r,jh(i)]=sim.simxGetObjectHandle(clientID, strcat('UR10_joint', int2str(i)) , sim.simx_opmode_blocking);
    end
    disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
    %% Compute forward kinematics
    M=fwdKin(DHMatrix);
    disp('Forward kinematics solution')
    % Print end-effector position
    disp('End effector position in meters:')
    disp(M{2}{numFrames}(:,4));
    % Print robot's tip position
    disp('Robot´s tip position in meters:')
    disp(M{2}{numFrames-1}(:,4));
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
             sim.simxSetJointTargetPosition(clientID, jh(j), (joints(i,j)), sim.simx_opmode_streaming);
         end
         pause(0.5);
         sim.simxSetIntegerSignal(clientID, 'showPos', 1, sim.simx_opmode_streaming);
         pause(printTimeInterval);
         sim.simxSetIntegerSignal(clientID, 'showPos', 0, sim.simx_opmode_streaming);
     end
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!

disp('Program ended');
```


# *CoppeliaSim* Simulation

## Scene Setup

The scene is composed of an UR10 robot model available in *CoppeliaSim* model browser, with a Unigripper Co/Light Regular model attached to its flange. Additionally, the robot is setup on top of a cylindrical pedestal with 0.3 m diameter and 0.7 m height.


\begin{figure}[H]
    \centering
    \includegraphics[width=1\textwidth]{Images/SceneSetup.png}
    \caption{Scene setup and hierarchy}
    \label{Figure:CoppeliaSimScene}
\end{figure}

## Embedded Scripts

In this scene there is only a threaded child script that is associated to the robot model. This script simply starts the MATLAB API remote connection, retrieves the handles of the robot and its joints, and has a while loop that is executed every five seconds where the robot's tip position is printed in the command window - this is useful to compare with our forward kinematics solution.



```lua
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
    local IKsol=0;
    
    --[[ Main loop ]]--
    while sim.getSimulationState()~=sim.simulation_advancing_abouttostop do
        if(sim.getIntegerSignal('showPos')==1) then
            sim.setIntegerSignal('showPos',0)
            -- get tip position relative to the robots' base
            local tip_pos=sim.getObjectPosition(tip,UR10)
            -- print tip position
            IKsol=IKsol+1
            print('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
            print('End-effector position '.. IKsol)
            print(tip_pos)
            -- reset value of IKsol, when reaching final solution
            if(IKsol==8) then
                IKsol=0
            end
        end
    end
    
end
```

# Launching the Simulation

To execute the simulation launch the *KinematicsTest.ttt* scene and run it. Afterwards, open the *main.m* script, choose the target joint values and run it. You should see the robot move to the eight inverse kinematic solutions and obtain a result similar to the figure below. Comparing the end-effector position calculated in MATLAB we can see it is equal to the value presented in *CoppeliaSim*. Moreover, in the MATLAB command window it is possible to verify that there is one inverse kinematic solution that is equal to the pre-defined target joint values, and that all the other solutions produce the same end-effector position in *CoppeliaSim*. Follow the following link for a [video](https://youtu.be/Z4mpqeP2lV8) of simulation execution.

