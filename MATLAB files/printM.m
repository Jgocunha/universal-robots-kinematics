%% Initialization

disp('Program started');

syms j [1 6]; % joints
syms d [1 7]; % link dimensions x
syms a [1 5]; % link dimensions z

dof=6; % degrees of freedom UR10

iMfile='iMfile.txt'; % individual matrices output file
cMfile='cMfile.txt'; % compound matrices output file

aux_matrix=strings(4,4); %auxiliary matrix of strings to correctly output the matrices

%Definition of the modified Denavit-Hartenberg matrix (Do not change!)
DHMatrix = [ 0         0       d1   j1;    % 1  0T1 
            -90        0       d2   j2-90; % 2  1T2 
             0         a2      d3   j3;    % 3  2T3 
             0         a3      d4   j4;    % 4  3T4
             0         a4      d5   90;    % 4' 4T4' 5
             90        0       0    j5;    % 5  4'T5 6
            -90        0       0    -90;   % 5' 5T5' 7
             0         a5      d6   j6;    % 6  5'T6 8
             0         0       d7   0;];   % 7  6T7  9

%% Main program

% Determine the number of reference frames using the DHMatrix
numFrames=size(DHMatrix);
numFrames=numFrames(1);

% Compute forward kinematics
M=fwdKin(DHMatrix);

% Output matrices
disp('Outputing forward kinematics solution to iMfile.txt and cMfile.txt');

writematrix("Individual transformation matrices",iMfile);
for i = 1 : numFrames
     writematrix(strcat(num2str(i-1),'T',num2str(i)),iMfile,'WriteMode','append');
     for j = 1 : 4
        for k = 1 : 4
            aux_matrix(j,k)=char(M{1}{i}(j,k));
        end
     end
     writematrix(aux_matrix,iMfile,'WriteMode','append');
end

writematrix("Compound transformation matrices",cMfile);
for i = 1 : numFrames
     writematrix(strcat(num2str(0),'T',num2str(i)),cMfile,'WriteMode','append');
     for j = 1 : 4
        for k = 1 : 4
            aux_matrix(j,k)=char(M{2}{i}(j,k));
        end
     end
     writematrix(aux_matrix,cMfile,'WriteMode','append');
end

disp('Program ended');