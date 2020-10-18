%% Initialization

disp('Program started');

syms j [1 6]; % joints
syms d [1 7]; % link dimensions x
syms a [2 5]; % link dimensions z

dof=6; % degrees of freedom UR10

iMfile='iMfile.txt'; % individual matrices output file
cMfile='cMfile.txt'; % compound matrices output file

iMfile='iMfile.xlsx'; % individual matrices output file
cMfile='cMfile.xlsx'; % compound matrices output file

%Definition of the modified Denavit-Hartenberg matrix (Do not change!)
DHMatrix = [ 0         0       d(1)   j1;    % 1  0T1 
            -90        0       d(2)   j2-90; % 2  1T2 
             0         a(2)    d(3)   j3;    % 3  2T3 
             0         a(3)    d(4)   j4;    % 4  3T4
             0         a(4)    d(5)   90;    % 4' 4T4' 5
             90        0       0      j5;    % 5  4'T5 6
            -90        0       0      -90;   % 5' 5T5' 7
             0         a(5)    d(6)   j6;    % 6  5'T6 8
             0         0       d(7)     0;]; % 7  6T7  9

%% Main program

% Determine the number of reference frames using the DHMatrix
numFrames=size(DHMatrix);
numFrames=numFrames(1);

% Compute forward kinematics
M=fwdKin(DHMatrix)

M{1}{1}

%w=char(M{1}{1})
writematrix(M{1}{1},iMfile)
% Output matrices
disp('Outputing forward kinematics solution to iMfile.txt and cMfile.txt');
%writecell((M{1}),iMfile);
%writecell(sym2cell(M{1}),iMfile);
%writecell(sym2cell(M{2}),cMfile);

disp('Program ended');