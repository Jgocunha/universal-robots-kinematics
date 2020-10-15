
function RPY = RPY(iTx)
    R=iTx([1,2,3],[1,2,3]);
    
    beta=rad2deg( atan2( -R(3,1), sqrt( R(3,2)^2 + R(3,3)^2 ) ) );
      
    if(beta==90)
        alpha=0;
        gamma=rad2deg(atan2(R(1,2),R(2,2)));
    elseif(beta==-90)
        alpha=0;
        gamma=rad2deg(-atan2(R(1,2),R(2,2)));
    else
        alpha=rad2deg(atan2( R(2,1)/cos(deg2rad(beta)), R(1,1)/cos(deg2rad(beta)) ));
        gamma=rad2deg(atan2( R(3,2)/cos(deg2rad(beta)), R(3,3)/cos(deg2rad(beta)) ));
    end
    

    RPY=[alpha, beta, gamma];
    
    %sciciliano
% 
%     v=rad2deg( atan2( -R(3,1), -sqrt( R(3,2)^2 + R(3,3)^2 )));     
%     phi=rad2deg(atan2(-R(2,1),-R(1,1)));
%     psi=rad2deg(atan2(-R(3,2),-R(3,3)));
%     
%     
%     RPY=[phi, v, psi];
end

