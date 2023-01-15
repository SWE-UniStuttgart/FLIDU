% This function calculates lidar dynamics, based on dynamic parameter
% inputs

%Created
% Moritz Gräfe 12.2022, Suttgart Wind Energy (SWE)


function [Output] = CalcLidarDynamics(Parameter, Config, Random_seed)


%%%%%%%%%%%%%%%%%%%%%%%%%

x_FL= 0;
y_FL= 0;
z_FL= Config.h_lidar;

calc_time = Config.calc_time;
%%%%%%%%%%%%%%%%%%%%%%%%
A_yaw=Parameter.DoF.A_yaw;
%psi_qstatic=Parameter.DoF.psi_static;

A_pitch=Parameter.DoF.A_pitch;
beta_qstatic=Parameter.DoF.beta_static;

A_roll=Parameter.DoF.A_roll;
%gamma_qstatic=Parameter.DoF.gamma_static;

A_heave=Parameter.DoF.A_heave;

H_ref=Config.H_ref;
h_lidar=Config.h_lidar;


Tp_yaw=Parameter.DoF.Tp_yaw;
Tp_pitch=Parameter.DoF.Tp_pitch;
Tp_roll=Parameter.DoF.Tp_roll;
Tp_heave=Parameter.DoF.Tp_heave;

startphase_yaw=Random_seed(1,Parameter.Wind.Seed)*2*pi;
startphase_pitch=Random_seed(2,Parameter.Wind.Seed)*2*pi;
startphase_roll=Random_seed(3,Parameter.Wind.Seed)*2*pi;
startphase_heave=Random_seed(4,Parameter.Wind.Seed)*2*pi;



Fs = 10;              % Sampling frequency
T  = 1/Fs;             % Sampling period
L  = calc_time*Fs;              % Length of signal
t  = (0:L-1)*T;        % Time vector


A=A_yaw;
T_p=Tp_yaw;

Yaw=deg2rad(A*sin(startphase_yaw+(2*pi)/T_p*t));

Yaw_A=deg2rad(A*sin(startphase_yaw+(2*pi)/T_p*t));


A=A_pitch;
T_p=Tp_pitch;

Pitch=deg2rad(A*sin(startphase_pitch+(2*pi)/T_p*t)+beta_qstatic);

Pitch_A=deg2rad(A*sin(startphase_pitch+(2*pi)/T_p*t)+beta_qstatic);

A=A_roll;
T_p=Tp_roll;

Roll=deg2rad(A*sin(startphase_roll+(2*pi)/T_p*t));

Roll_A=deg2rad(A*sin(startphase_roll+(2*pi)/T_p*t));


A=A_heave;
T_p=Tp_heave;

Heave=A*sin(startphase_heave+(2*pi)/T_p*t);

Heave_A=A*sin(startphase_heave+(2*pi)/T_p*t);


%%%%%%%%%%%%%%%

[x_trans,y_trans,z_trans] = arrayfun(@(Pitch, Roll, Yaw, Heave) calc(Pitch, Roll, Yaw, Heave, x_FL, y_FL, z_FL), Pitch, Roll, Yaw, Heave);
[x_trans_A,y_trans_A,z_trans_A] = arrayfun(@(Pitch, Roll, Yaw, Heave) calc(Pitch, Roll, Yaw, Heave, x_FL, y_FL, z_FL), Pitch_A, Roll_A, Yaw_A, Heave_A);
    function [x_trans,y_trans,z_trans]=calc(Pitch, Roll, Yaw, Heave, x_FL, y_FL,z_FL)
        
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
        
        
        x_trans     = T(1,1)*x_FL + T(1,2)*y_FL + T(1,3)*z_FL;
        y_trans     = T(2,1)*x_FL + T(2,2)*y_FL + T(2,3)*z_FL;
        z_trans    = T(3,1)*x_FL + T(3,2)*y_FL + T(3,3)*z_FL;
        
    end

Output.A_x_vel=max(diff(x_trans_A)*Fs);
Output.A_y_vel=max(diff(y_trans_A)*Fs);
Output.A_z_vel=max(diff(z_trans_A+Heave)*Fs);

Output.sim.channels.x_trans_L=((x_trans)-x_FL)';
Output.sim.channels.y_trans_L=((y_trans)-y_FL)';
Output.sim.channels.z_trans_L=((z_trans+ Heave)-z_FL)';

Output.sim.channels.x_vel =diff(x_trans)*Fs;
Output.sim.channels.x_vel=[Output.sim.channels.x_vel(1,1), Output.sim.channels.x_vel]';

Output.sim.channels.y_vel =diff(y_trans)*Fs;
Output.sim.channels.y_vel=[Output.sim.channels.y_vel(1,1), Output.sim.channels.y_vel]';

Output.sim.channels.z_vel =diff(z_trans)*Fs;
Output.sim.channels.z_vel=[Output.sim.channels.z_vel(1,1), Output.sim.channels.z_vel]';

Output.sim.channels.Pitch=rad2deg(Pitch)';
Output.sim.channels.Roll=rad2deg(Roll)';
Output.sim.channels.Yaw=rad2deg(Yaw)';
Output.sim.channels.Heave=Heave;
Output.sim.channels.Time=t';
end
