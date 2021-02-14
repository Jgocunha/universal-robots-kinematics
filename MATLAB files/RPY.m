%% Function: RPY
% 
% Computes the Euler Angles RPY 
% 
% In: iTx - an individual transformation matrix
%
% Out: RPY - [gamma, beta, alpha]
%%
% function RPY = RPY(R)
%     %R=iTx([1,2,3],[1,2,3]);
%     
%     beta=rad2deg( atan2( -R(3,1), sqrt( R(3,2)^2 + R(3,3)^2 ) ) );
%       
%     if(beta==90)
%         alpha=0;
%         gamma=rad2deg(atan2(R(1,2),R(2,2)));
%     elseif(beta==-90)
%         alpha=0;
%         gamma=rad2deg(-atan2(R(1,2),R(2,2)));
%     else
%         alpha=rad2deg(atan2( R(2,1)/cos(deg2rad(beta)), R(1,1)/cos(deg2rad(beta)) ));
%         gamma=rad2deg(atan2( R(3,2)/cos(deg2rad(beta)), R(3,3)/cos(deg2rad(beta)) ));
%     end
%     
% 
%     RPY=[gamma, beta, alpha];
% end

function RPY = RPY(R)

    if R(1,3) == 1 || R(1,3) == -1
      %special case
      E3 = 0; %set arbitrarily
      dlta = atan2(R(1,2),R(1,3));
      if R(1,3) == -1
        E2 = pi/2;
        E1 = E3 + dlta;
      else
        E2 = -pi/2;
        E1 = -E3 + dlta;
      end
    else
      E2 = - asin(R(1,3));
      E1 = atan2(R(2,3)/cos(E2), R(3,3)/cos(E2));
      E3 = atan2(R(1,2)/cos(E2), R(1,1)/cos(E2));
    end

    RPY = rad2deg([E3 E2 E1]);
end
