%% Initialization

disp('Program started');

dof=6; % degrees of freedom UR10
d=zeros(1,dof+1); %distances
a=zeros(1,dof); %distances
theta=zeros(1,dof); %joint angles
jh=zeros(1,dof); %CoppeliaSim joint handles
totalIKsol=8; %number of inverse kinematic solutions
iterations=1000; %number of code iterations
error=zeros(1,totalIKsol); %error of ik sols

ValFile='.\ValidationOutput\stdev.xlsx'; % error and computation times output file

%% Denavit-Hartenberg parameters (User interface)

% CoppeliaSim link dimensions for the UR10 model
d(1)=0.109;
d(2)=0.10122;
d(3)=0.12067-0.10122;
d(4)=0.11406-0.12067;
d(5)=0.17246-0.11406;
d(6)=0.26612-0.17246;
d(7)=0; % end-effector

a(2)=0.7211-0.109;
a(3)=1.2933-0.7211;
a(4)=1.3506-1.2933;
a(5)=1.409-1.3506;

%% Main program

for it = 1 : iterations

    % Set random joint values 
    for j = 1 : dof
        theta(j)=randi([-360 360],1,1);
    end
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
    % Print robot's tip position
    disp('Robot´s tip position in meters:')
    fwd_tip_pos = M{2}{numFrames-1}(:,4).';
    disp(fwd_tip_pos(1:3));
    % Print robot's tip and end-effector orientation
    disp('Robot´s tip orientation in degrees:')
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
    % Print and validate IK solutions
     for i = 1: totalIKsol
         disp('_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-')
         fprintf('Inverse kinematic solution %d\n',i);
         disp('Value in degrees');
         disp(rad2deg(joints(i,:)));
         disp('Value in radians');
         disp(joints(i,:));
         joints(i,:)=rad2deg(joints(i,:));
         % Run the forward kinematics function for each IK solution
         DHMatrix = [ 0         0       d(1)   joints(i,1);    % 1  0T1 
        -90        0       d(2)   joints(i,2)-90; % 2  1T2 
         0         a(2)    d(3)   joints(i,3);    % 3  2T3 
         0         a(3)    d(4)   joints(i,4);    % 4  3T4
         0         a(4)    d(5)   90;             % 4' 4T4' 5
         90        0       0      joints(i,5);    % 5  4'T5 6
        -90        0       0      -90;            % 5' 5T5' 7
         0         a(5)    d(6)   joints(i,6);    % 6  5'T6 8
         0         0       d(7)     0;];          % 7  6T7  9
         M=fwdKin(DHMatrix);
         % Print robot's tip position
        disp('Robot´s tip position in meters:')
        fwd_tip_pos_iksol = M{2}{numFrames-1}(:,4).';
        disp(fwd_tip_pos_iksol(1:3));
        % Print robot's tip and end-effector orientation
        disp('Robot´s tip orientation in degrees:')
        fwd_tip_ori_iksol = RPY(R);
        disp(fwd_tip_ori_iksol);
        fwd_tip_pose_iksol = [fwd_tip_pos_iksol(1:3) fwd_tip_ori_iksol];
        error=fwd_tip_pose-fwd_tip_pose_iksol;
        %% Send errors and computation times to excel
        excelValues=[error,t]; %append time to error - send to Excel sheet
        writematrix(excelValues,ValFile,'WriteMode','append');
     end
end

disp('Program ended');