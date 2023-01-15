% This function calculates lidar measurement bias based on lidar
% dynamics inputs

%Created
% Moritz Gräfe 12.2022, Suttgart Wind Energy (SWE)

function [Error] = Errorcalc(Parameter, Config, Dynamics)
%% Set Parameters and Config

alpha=Parameter.EC.alpha;
phi=deg2rad(Parameter.EC.phi);
vref_setup=Parameter.EC.vref;
% R=Parameter.EC.R;

% set config

theta_lidar=Config.theta_lidar;
phi_lidar=Config.phi_lidar;
num_beam=Config.num_beam;
r=Config.r;
z_L_upper=Config.z_L_upper;
z_L_lower=Config.z_L_lower;

H_ref=Config.H_ref;
h_lidar=Config.h_lidar;

%% Calculate Lidar coordinates for each beam

for i=1:num_beam %loop over beams
    
    if length(theta_lidar==1)
        theta_1 = theta_lidar(1);
    else
        theta_1 = theta_lidar(i);
    end
    
    if length(phi_lidar)==1
        phi_1=phi_lidar(1);
    else
        phi_1=phi_lidar(i);
    end
    
    x_L(i)=cos(theta_1);
    y_L(i)=sin(theta_1)*cos(phi_1);
    z_L(i)=sin(theta_1)*sin(phi_1);
    
    
    %% calc VLOS under combined DoF
    v_ref=vref_setup;
    
    Yaw = Dynamics.sim.channels.Yaw';
    Pitch = Dynamics.sim.channels.Pitch';
    Roll = Dynamics.sim.channels.Roll';
    h_heave= Dynamics.sim.channels.Heave;
    
    x_vel=Dynamics.sim.channels.x_vel';
    y_vel=Dynamics.sim.channels.y_vel';
    z_vel=Dynamics.sim.channels.z_vel';
    
    
    
    [x_I,y_I,z_I] = arrayfun(@(Yaw, Pitch, Roll) calc_coordinates(Yaw, Pitch,  Roll, x_L(i), y_L(i) ,z_L(i)), Yaw, Pitch, Roll);
    
    
    
    v_los_total(i,:)= v_ref*((z_I.*r+h_lidar+h_heave)/H_ref).^alpha.*(sin(phi).*(x_I)+cos(phi).*(y_I))...
        +(x_I.*x_vel+y_I.*y_vel+z_I.*z_vel);
    
    
    
    
end

%%
Error.corr_coeff=corrcoef(v_los_total');


%% Individual Error by each DoF
v_ref=vref_setup;
u_ref=v_ref*sin(phi);
vv_ref=v_ref*cos(phi);
r_rotor=75;

u_upper=u_ref*((z_L_upper+h_lidar)/H_ref)^alpha;
u_lower=u_ref*((z_L_lower+h_lidar)/H_ref)^alpha;

shear_ref = (log(u_upper)-log(u_lower))/(log(z_L_upper+100)-log(z_L_lower+100));

%%%%%%%%%%%%%%%%% rotor heights test
load('rotor_heights.mat')
heights = ans;
v_rotor=u_ref.*((heights+h_lidar-2)/H_ref).^alpha;
v_REWS_ref=mean(mean(v_rotor));

v_rotor=vv_ref.*((heights+h_lidar)/H_ref).^alpha;
vv_ref=mean(mean(v_rotor));

%% Calc total error
u_rec_total=1/4.*(1./x_L(1).*v_los_total(1,:)+ 1./x_L(2).*v_los_total(2,:)+ 1./x_L(3).*v_los_total(3,:) + 1./x_L(4).*v_los_total(4,:));
v_rec_total=1/4.*(1./y_L(1).*v_los_total(1,:)+ 1./y_L(2).*v_los_total(2,:)+ 1./y_L(3).*v_los_total(3,:) + 1./y_L(4).*v_los_total(4,:));
error_v_total=-vv_ref+mean(v_rec_total);

error_u_total=-v_REWS_ref+mean(u_rec_total);
error_u_total_rel=(-u_ref+mean(u_rec_total))/u_ref*100;
error_v_total_rel=(-vv_ref+mean(v_rec_total))/vv_ref*100;
%shear-yaw
u_rec_upper_total= 1/2*(1./x_L(1).*v_los_total(1,:)+1./x_L(2).*v_los_total(2,:));
u_rec_lower_total= 1/2*(1./x_L(3).*v_los_total(3,:)+1./x_L(4).*v_los_total(4,:));

v_rec_upper_total= 1/2*(1./y_L(1).*v_los_total(1,:)+1./y_L(2).*v_los_total(2,:));
v_rec_lower_total= 1/2*(1./y_L(3).*v_los_total(3,:)+1./y_L(4).*v_los_total(4,:));
ws_upper_total=sqrt(v_rec_upper_total.^2+u_rec_upper_total.^2);
ws_lower_total=sqrt(v_rec_lower_total.^2+u_rec_lower_total.^2);

% shear_rec_total=(ws_upper_total-ws_lower_total)/(z_L_upper-z_L_lower);
alpha=(log(ws_upper_total)-log(ws_lower_total))/(log(z_L_upper+100)-log(z_L_lower+100));
shear_rec_total = alpha;
error_shear_total=mean(shear_rec_total)-mean(shear_ref);
error_shear_total_rel=mean(shear_rec_total-shear_ref)/shear_ref*100;

%% Export Results
Error.u_total=error_u_total;
Error.u_rel=error_u_total_rel;

Error.v_total=error_v_total;
Error.v_rel=error_v_total_rel;

Error.shear_total=error_shear_total;
Error.shear_rel=error_shear_total_rel;

end
