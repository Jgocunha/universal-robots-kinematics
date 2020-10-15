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