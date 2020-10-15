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
function T = MDHMatrix(DHparams)
    T = [ cosd(DHparams(4))                                 -sind(DHparams(4))                      0                 DHparams(2)     ;
          sind(DHparams(4))*cosd(DHparams(1))     cosd(DHparams(4))*cosd(DHparams(1))       -sind(DHparams(1))     -sind(DHparams(1))*DHparams(3);
          sind(DHparams(4))*sind(DHparams(1))     cosd(DHparams(4))*sind(DHparams(1))       cosd(DHparams(1))      cosd(DHparams(1))*DHparams(3) ;
                    0                                               0                               0                       1    ];
end

% function T = MDHMatrix(DHparams)
%     T = [ cos(DHparams(4))                                 -sin(DHparams(4))                      0                 DHparams(2)     ;
%           sin(DHparams(4))*cos(DHparams(1))     cos(DHparams(4))*cos(DHparams(1))       -sin(DHparams(1))     -sin(DHparams(1))*DHparams(3);
%           sin(DHparams(4))*sin(DHparams(1))     cos(DHparams(4))*sin(DHparams(1))       cos(DHparams(1))      cos(DHparams(1))*DHparams(3) ;
%                     0                                               0                               0                       1    ];
% end