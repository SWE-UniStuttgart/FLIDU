    
    function [x_I,y_I,z_I]=calc_coordinates(Yaw, Pitch, Roll, x_L, y_L,z_L)
Yaw=deg2rad(Yaw);
Pitch=deg2rad(Pitch);
Roll=deg2rad(Roll);
    
    
    % Yaw is a rotation around z-axis
    T_Yaw 	= [ cos(Yaw)   -sin(Yaw)	0;
        sin(Yaw) 	cos(Yaw)    0;
        0           0           1];
    
    % Pitch is a rotation around y-axis
    T_Pitch = [	cos(Pitch)	0           sin(Pitch);
        0         	1        	0;
        -sin(Pitch)	0        	cos(Pitch)];
    
    % Roll is a rotation around x-axis
    T_Roll  = [	1           0       	0;
        0       	cos(Roll)  -sin(Roll);
        0          	sin(Roll)	cos(Roll)];
    
    
    T       = T_Yaw*T_Pitch*T_Roll;
   
    x_I     = T(1,1)*x_L + T(1,2)*y_L + T(1,3)*z_L;
    y_I     = T(2,1)*x_L + T(2,2)*y_L + T(2,3)*z_L;
    z_I    = T(3,1)*x_L + T(3,2)*y_L + T(3,3)*z_L;
    
    
    end
    