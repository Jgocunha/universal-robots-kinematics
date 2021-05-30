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
      alpha = 0; %set arbitrarily
      dlta = atan2(R(1,2),R(1,3));
      if R(1,3) == -1
        beta = pi/2;
        gamma = alpha + dlta;
      else
        beta = -pi/2;
        gamma = -alpha + dlta;
      end
    else
      beta = - asin(R(1,3));
      gamma = atan2(R(2,3)/cos(beta), R(3,3)/cos(beta));
      alpha = atan2(R(1,2)/cos(beta), R(1,1)/cos(beta));
    end

    RPY = [alpha beta gamma];
end
