%% Function: RPY
% 
% Computes the Euler Angles RPY 
% 
% In: iTx - an individual transformation matrix
%
% Out: RPY - [alpha, beta, gamma]
%%
function RPY = RPY(R)

    if R(1,3) == 1 || R(1,3) == -1
      %special case
      gamma = 0; %set arbitrarily
      dlta = atan2(R(1,2),R(1,3));
      if R(1,3) == -1
        beta = pi/2;
        alpha = gamma + dlta;
      else
        beta = -pi/2;
        alpha = -gamma + dlta;
      end
    else
      beta = - asin(R(1,3));
      alpha = atan2(R(2,3)/cos(beta), R(3,3)/cos(beta));
      gamma = atan2(R(1,2)/cos(beta), R(1,1)/cos(beta));
    end

    RPY = rad2deg([-alpha -beta -gamma]);
end
