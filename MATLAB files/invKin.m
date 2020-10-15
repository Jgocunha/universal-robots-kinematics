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
%     M - two dimensional array of arrays of matrices
%       |_ M{1} - iT - array of individual transformation matrices
%       |  (this matrix is only used when the angles are known!)
%       |_ M{2} - T0 - array of compound transformation matrices
%          (only the 0T7 matrix can be used, 0T6 is direct also)

% Out: joint - 8x6 matrix with the 8 ik solutions for the 6 joints
%%

function joint=invKin(d, a, M, theta)
    % 8 inv kin solutions, 6 joints
    joint=zeros(8,6);
    
    % Individual Transformation Matrices
    itm=M{1};
    % General Transformation Matrices
    gtm=M{2};
    % Tip robots' flange (flange general transformation matrix)
    fgtm=gtm{8}(:,:);
    % Tip end-effector (end-effector general transformation matrix)
    eegtm=gtm{9}(:,:);
    
    %% Computing theta1
    
    % 0P5 position of reference frame {5} in relation to {0}
    P=eegtm*[0 0 -d(6)-d(7) 1].' 
    
    phi=atan2(P(2,1),P(1,1));
    
    % There are two possible solutions for theta1, that depend on whether
    % the shoulder joint (joint 2) is left or right
    
    %Shoulder left
    alpha1=asin((d(2)+d(3)+d(4)+d(5))/(sqrt(((P(2,1))^2+(P(1,1)^2)))));
    %Shoulder right
    alpha2=-asin((d(2)+d(3)+d(4)+d(5))/(sqrt(((P(2,1))^2+(P(1,1)^2)))));
        
%     for s = 1:8
%         joint(s,1)=(pi/2-alpha2-phi);
%     end
%     for s = 1:4
%         joint(s,1)=(pi/2-alpha1-phi);
%     end
    %joint(:,1)=(pi/2-alpha1-phi);
    joint(:,1)=(-alpha1+phi);
    %% Computing theta5
    
    % Knowing theta1 it is possible to know 0T1
    T_01= itm{1};
    
    % 1T6 = 1T0 * 0T6
    T_16 = inv(T_01)*fgtm;
    %1P6
    P_16=T_16(:,4);
    
    %Wrist up or down   
    for s = 1:8 
        if(ismember(s,[1,2,5,6]))
           joint(s,5)=((acos((P_16(2,1)-(d(2)+d(3)+d(4)+d(5)))/d(6))));
        else
            joint(s,5)=(-(acos((P_16(2,1)-(d(2)+d(3)+d(4)+d(5)))/d(6))));
        end
    end

    %% Computing theta 6
    
     T_61=inv(T_16);
     % y1 seen from frame 6
     Y_16=T_61(:,2);
     
     % If theta 5 is equal to zero give arbitrary value to theta 6
    for k = 1:8
         if(real(joint(k,5)) == 0 || real(joint(k,5)) == 2*pi)
             joint(:,6)=0;
        else
         %joint(:,6)= (-pi/2 + atan2( -Y_16(2,1)/sin(joint(5))  , Y_16(1,1)/sin(joint(5))));
         joint(:,6)= (pi/2 + atan2( -Y_16(2,1)/sin(joint(k,5))  , Y_16(1,1)/sin(joint(k,5))));
        end
    end
    
     %% Computing theta 3
    
    %Get position of frame 4 from frame 1, P_14
    
    %The value of T_01, T_45, and T_56 are known since joints{1,5,6} have
    %been obtained
    
    %T_45 = T_44'*T_4'5
    T_45 = itm{5}*itm{6};
    %T_56 = T_55'*T_5'6
    T_56 = itm{7}*itm{8};
    
    T_46 = T_45*T_56;
    
    T_64 = inv(T_46);
    
    T_14=T_16*T_64;
    
    %P_14 is the fourth column of T_14
    P_14=T_14(:,4);
    
    P_14_xz=sqrt( P_14(1)^2 +  P_14(3)^2);
    
    %Elbow up or down
    for s = 1:8
        if(rem(s,2)==0)
            joint(s,3)=( pi - acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) ));
        else
            joint(s,3)=( pi + acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) ));
            %joint(s,3)=-2*pi+( pi + acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) ));%fix
        end
    end
    
    
     %% Computing theta 2
     %OLD
%      for s = 1:8
%         joint(s,2) = ( atan2( P_14(1), P_14(3)) - acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) );
%     end
%     for s = 1:4
%         joint(s,2) = ( atan2( P_14(1), P_14(3)) + acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) );
%     end
    %SOMETHING
%      for s = 1:8
%         joint(s,2) = ( atan2( -P_14(3), P_14(1)) - acos( ( -P_14_xz^2+a(3)^2+a(2)^2 )/( 2*a(2)*a(3) ) ) );
%     end
%     for s = 1:4
%         joint(s,2) = ( atan2( -P_14(3), P_14(1)) + acos( ( -P_14_xz^2+a(3)^2+a(2)^2 )/( 2*a(2)*a(3) ) ) );
%     end
    %NEW
     for s = 1:8
        %joint(s,2) = -( atan2( P_14(3), -P_14(1)) - acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) ) - pi;
        joint(s,2) = -( atan2( P_14(3), -P_14(1)) - acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) - pi/2);
    end
    for s = 1:4
        joint(s,2) = -( atan2( P_14(3), P_14(1)) + acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) - pi/2);
    end  
    
    %atan2( P_14(1), P_14(3)) + a(3)
    
     %joint(:,2) = ( atan2( P_14(1), P_14(3)) - acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) );
    
    %% Computing theta 4
    
    %The value of joints{2,3} are now known so it is possible to compute
    %T_34
    
    T_34=itm{4};
    
    x_34=T_34(:,1);
    joint(:,4) = (atan2(x_34(1),x_34(2)));
end

























%Elbow up or down
        if(rem(j,2)==0)
            psi= acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) );
            joint(j,3)=( pi - psi);
            % theta 2
            %joint(j,2) = -( atan2( P_14(3), -P_14(1)) - acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) - pi/2);
            %asin( a(3)*sin(-psi)/P_14_xz)
            joint(j,2) = atan2( P_14(3), -P_14(1)) + asin( a(3)*sin(-psi)/P_14_xz);
            % theta 4
                %joint(j,4)=joint4(d, a, joint(j,5), tipPosOri, T_01, T_64);
                % Fix Error using atan2 / Inputs must be real.
                    joint=real(double(rad2deg(joint)));
                    joint=deg2rad(joint);
                T_12=MDHMatrix([-90        0       d(2)   rad2deg(joint(j,2))-90]);
                T_23=MDHMatrix([0         a(2)    d(3)   rad2deg(joint(j,3))]);
                T_03=T_01*T_12*T_23;

                T_36=inv(T_03)*tipPosOri;
                T_34=T_36*T_64;

                x_34=T_34(:,1);

                %joint(j,4) = (atan2(x_34(2),x_34(1)));
        else
            psi= acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) );
            joint(j,3)=( pi + psi);
            % theta 2
            joint(j,2) = atan2( P_14(3), -P_14(1)) + asin( a(3)*sin(-psi)/P_14_xz);
            % theta 4
            %joint(j,3)=( pi + acos( ( P_14_xz^2-a(3)^2-a(2)^2 )/( -2*a(2)*a(3) ) ));
            % theta 2
            %joint(j,2) = -( atan2( P_14(3), -P_14(1)) - acos( ( P_14_xz^2-a(3)^2+a(2)^2 )/( 2*a(2)*P_14_xz ) ) - pi/2);
            % theta 4
                %joint(j,4)=joint4(d, a, joint(j,5), tipPosOri, T_01, T_64);
                                % Fix Error using atan2 / Inputs must be real.
                    joint=real(double(rad2deg(joint)));
                    joint=deg2rad(joint);
                T_12=MDHMatrix([-90        0       d(2)   rad2deg(joint(j,2))-90]);
                T_23=MDHMatrix([0         a(2)    d(3)   rad2deg(joint(j,3))]);
                T_03=T_01*T_12*T_23;

                T_36=inv(T_03)*tipPosOri;
                T_34=T_36*T_64;

                x_34=T_34(:,1);
                
                %joint(j,4) = (atan2(x_34(2),x_34(1)));
                
        end
