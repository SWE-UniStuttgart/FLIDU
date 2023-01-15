% This function calculates lidar measurement uncertainty based on lidar
% dynamics inputs

%Created 
% Moritz Gräfe 12.2022, Suttgart Wind Energy (SWE) 


function [Uncertainty] = Uncertaintycalc(Parameter, Config, Dynamics, Error)

%set Parameters

A_yaw=Parameter.DoF.A_yaw;
psi_qstatic=Parameter.DoF.psi_static;

A_pitch=Parameter.DoF.A_pitch;
beta_qstatic=Parameter.DoF.beta_static;

A_roll=Parameter.DoF.A_roll;
gamma_qstatic=Parameter.DoF.gamma_static;

A_heave=Parameter.DoF.A_heave;

% Calc tranlational velocity + displacement from rotation
if Config.calc_velocities
%     
%     A_xvel=max(Dynamics.sim.channels.x_vel);
%     A_yvel=max(Dynamics.sim.channels.y_vel);
%     A_zvel=max(Dynamics.sim.channels.z_vel);
    
    A_xvel=Dynamics.A_x_vel;
    A_yvel=Dynamics.A_y_vel;
    A_zvel=Dynamics.A_z_vel;
    
    
else
    A_xvel=Parameter.DoF.A_xvel;
    A_yvel=Parameter.DoF.A_yvel;
    A_zvel=Parameter.DoF.A_zvel;
    
    
end
alpha=Parameter.EC.alpha;
phi=deg2rad(Parameter.EC.phi);
vref_setup=Parameter.EC.vref;
% R=Parameter.EC.R;
R=Error.corr_coeff;
% set config

theta_lidar=Config.theta_lidar;
phi_lidar=Config.phi_lidar;
num_beam=Config.num_beam;
r=Config.r;
z_L_upper=Config.z_L_upper;
z_L_lower=Config.z_L_lower;

H_ref=Config.H_ref;
h_lidar=Config.h_lidar;


for i=1:num_beam %loop over beams
    
    if length(theta_lidar)==1
        theta_1 = theta_lidar(1);
    else
        theta_1 = theta_lidar(i);
    end
    
    if length(phi_lidar)==1
        phi_1=phi_lidar(1);
    else
        phi_1=phi_lidar(i);
    end
    
    
    
    A_yawav=A_yaw;
    A_pitchav=A_pitch;
    A_rollav=A_roll;
    A_heaveav=A_heave;
    
    
    A_xvelav=A_xvel;
    A_yvelav=A_yvel;
    A_zvelav=A_zvel;
    
    
    v_ref=vref_setup;
    h_heave=0;
    
    
    
    %% yaw
    % VLOS(Yaw)
    
    psi=deg2rad(A_yaw*sind(1:(1*359))+psi_qstatic);
    
    x_L(i)=cos(theta_1);
    y_L(i)=sin(theta_1)*cos(phi_1);
    z_L(i)=sin(theta_1)*sin(phi_1);
    
    
    x_I=(cos(psi).*x_L(i)-sin(psi).*y_L(i));
    y_I=(sin(psi).*x_L(i)+cos(psi).*y_L(i));
    z_I=z_L(i);
    
    
    v_los_yaw(i,:)= v_ref*((z_I*r+h_lidar+h_heave)/H_ref)^alpha.*(sin(phi).*(x_I)+cos(phi).*(y_I));
    % DVLOS/DYaw
    d_vlos_yaw_temp= -v_ref*((z_L(i)*r+h_lidar+h_heave)/H_ref)^alpha.*(sin(phi).*(x_L(i).*sin(psi)+y_L(i).*cos(psi))+cos(phi)*(y_L(i).*sin(psi)-x_L(i).*cos(psi)));
    d_vlos_yaw(i,:)= sqrt((d_vlos_yaw_temp* deg2rad(1)).^2);
    %Analysis
    
    mean_vlos_yaw(i,:)=mean(v_los_yaw(i,:))*ones(1,length(v_los_yaw));
    % MAE
    
    perf = mae(1-v_los_yaw);
    error_yaw=perf/1*100;
    
    [ d, idx ]  = min( abs(psi-deg2rad(psi_qstatic)));
    steady_dvlos_yaw(i)=d_vlos_yaw(i,idx);
    U_yaw(i,:)=(A_yawav/sqrt(2))^2*mean(steady_dvlos_yaw(i)).^2;
    
    
    %% Pitch
    
    % VLOS(Pitch)
    
    beta=deg2rad(A_pitch*sind(1:359)+beta_qstatic);
    
    x_I= (cos(beta).*x_L(i) +sin(beta).*z_L(i));
    y_I= y_L(i);
    z_I= (-sin(beta)*x_L(i) + cos(beta)*z_L(i));
    
    
    
    x_I_abs(i,:)=x_I*r;
    y_I_abs(i,:)=y_I*r;
    z_I_abs(i,:)=z_I*r;
    
    v_los_pitch(i,:)= v_ref.*((z_I*r+h_lidar+h_heave)/H_ref).^alpha.*(sin(phi).*(x_I)+cos(phi).*(y_I));
    
    
    % DVLOS/DPitch
    
    d_vlos_pitch_temp=1/H_ref* alpha*v_ref*r.*(-x_L(i).*cos(beta)-z_L(i).*sin(beta)).*(sin(phi).*(x_L(i).*cos(beta)+z_L(i).*sin(beta))+y_L(i).*cos(phi))...
        .*((h_lidar+h_heave+(-x_L(i).*sin(beta)+z_L(i).*cos(beta))*r)/(H_ref)).^(alpha-1)...
        +v_ref*sin(phi).*(z_L(i).*cos(beta)-x_L(i).*sin(beta)).*...
        ((h_lidar+h_heave-(x_L(i).*sin(beta)-z_L(i).*cos(beta))*r)/(H_ref)).^alpha;
    d_vlos_pitch(i,:)= sqrt((d_vlos_pitch_temp* deg2rad(1)).^2);
    

    
    [ d, idx ]  = min( abs( beta-deg2rad(beta_qstatic)));
    steady_dvlos_pitch(i)=d_vlos_pitch(i,idx);
    
    U_pitch(i,:)=(A_pitchav/sqrt(2))^2*mean(steady_dvlos_pitch(i)).^2;
    
    
    %% Roll
    
    gamma=deg2rad(A_roll*sind(1:359)+gamma_qstatic);
    
    
    x_I= x_L(i);
    y_I= (cos(gamma)*y_L(i)-sin(gamma)*z_L(i));
    z_I= (sin(gamma)*y_L(i) + cos(gamma)*z_L(i));
    
    
    
    v_los_roll(i,:)= v_ref*((z_I*r+h_lidar+h_heave)/H_ref).^alpha.*(sin(phi).*(x_I)+cos(phi).*(y_I));
    
    % DVLOS/DRoll
    
    d_vlos_roll_temp=1/H_ref* alpha*v_ref*r.* (y_L(i).*cos(gamma)+z_L(i).*sin(gamma)).*(x_L(i)*sin(phi)+cos(phi)*(y_L(i)*cos(gamma)-z_L(i)*sin(gamma)))...
        .*((h_lidar+h_heave+(y_L(i).*sin(gamma)-z_L(i).*cos(gamma))*r)/(H_ref)).^(alpha-1)...
        +v_ref*cos(phi).*(-y_L(i).*sin(gamma)-z_L(i).*cos(gamma)).*...
        ((h_lidar+h_heave+(y_L(i).*sin(gamma)-z_L(i).*cos(gamma))*r)/(H_ref)).^alpha;
    
    d_vlos_roll(i,:)= sqrt((d_vlos_roll_temp* deg2rad(1)).^2);
    

    
    [d, idx]  = min( abs(gamma-deg2rad(gamma_qstatic)));
    steady_dvlos_roll(i)=d_vlos_roll(i,idx);
    
    U_roll(i,:)=(A_rollav/sqrt(2))^2*mean(steady_dvlos_roll(i)).^2;
    
    
    %% heave
    
    h_heave_qstatic=0;
    h_heave=A_heave*sind(1:359);
    
    
    %Vlos_heave
    
    v_los_heave(i,:)= v_ref*((z_L(i)*r+h_lidar+h_heave)/H_ref).^alpha.*(sin(phi).*(x_L(i))+cos(phi).*(y_L(i)));
    
    %Dvlos/Dheave
    
    d_vlos_heave_temp= v_ref*alpha./(z_L(i)*r+h_lidar+h_heave)...
        .*((z_L(i)*r+h_lidar+h_heave)/H_ref).^alpha...
        .*(x_L(i)*sin(phi)+y_L(i)*cos(phi));
    
    d_vlos_heave(i,:)= sqrt((d_vlos_heave_temp).^2);
    %Analysis
    

    
    [d, idx]  = min( abs(h_heave));
    steady_dvlos_heave(i)=d_vlos_heave(i,idx);
    U_heave(i,:)=(A_heaveav/sqrt(2))^2*mean(steady_dvlos_heave(i)).^2;   
    
    %% Translational velocities
    
    v_ref=vref_setup;
    
    % h_heave=-2:0.1:2;%1*sind(1:360);
    xvel=A_xvel*sind(1:359);
    yvel=A_yvel*sind(1:359);
    zvel=A_zvel*sind(1:359);
    
    
    v_los_transx(i,:)= v_ref*((z_L(i)*r+h_lidar)/H_ref).^alpha.*(sin(phi).*(x_L(i))+cos(phi).*(y_L(i)))+ (x_L(i)*xvel);
    d_vlos_transx(i,:)=ones(1,length(v_los_transx))*x_L(i);
    U_vlosx(i,:)=(A_xvelav/sqrt(2))^2*mean(d_vlos_transx(i,:)).^2; % valid because partial derivatice is a constant
    
    v_los_transy(i,:)= v_ref*((z_L(i)*r+h_lidar)/H_ref).^alpha.*(sin(phi).*(x_L(i))+cos(phi).*(y_L(i)))+ (y_L(i)*yvel);
    d_vlos_transy(i,:)=ones(1,length(v_los_transy))*y_L(i);
    U_vlosy(i,:)=(A_yvelav/sqrt(2))^2*mean(d_vlos_transy(i,:)).^2; % valid because partial derivatice is a constant
    
    v_los_transz(i,:)= v_ref*((z_L(i)*r+h_lidar)/H_ref).^alpha.*(sin(phi).*(x_L(i))+cos(phi).*(y_L(i)))+ (z_L(i)*zvel);
    d_vlos_transz(i,:)=ones(1,length(v_los_transz))*z_L(i);
    U_vlosz(i,:)=(A_zvelav/sqrt(2))^2*mean(d_vlos_transz(i,:)).^2; % valid because partial derivatice is a constant
    
    %% Uncertainty calculation

    U_disp_vlos(i)  =sqrt(U_yaw(i,1)+U_pitch(i,1)+U_roll(i,1)+U_heave(i,1)); % in %
    U_trans_vlos(i) =sqrt(U_vlosx(i,1)+U_vlosy(i,1)+U_vlosz(i,1)); % absolut
    U_total_vlos(i) =sqrt(U_yaw(i,1)+U_pitch(i,1)+U_roll(i,1)+U_heave(i,1)+U_vlosx(i,1)+U_vlosy(i,1)+U_vlosz(i,1)); %absolut
    
    Uncertainty.Beam.disp(i)=U_disp_vlos(i);
    Uncertainty.Beam.trans(i)=U_trans_vlos(i);
    Uncertainty.Beam.total(i)=U_total_vlos(i);
    
end

%%
    U_u_total_nc=((1/4*1/x_L(1)*U_total_vlos(1))^2+...
        ((1/4*1/x_L(2))*U_total_vlos(2))^2+...
        ((1/4*1/x_L(3))*U_total_vlos(3))^2+...
        ((1/4*1/x_L(4))*U_total_vlos(4))^2);
    
    sum1=0;
    sum2=0;
    for il=1:3
        for jl=(il+1):4
            U=2*(1/4*1/x_L(il)*1/4*1/x_L(jl)*U_total_vlos(il)*U_total_vlos(jl)*R(il,jl));
            sum1=sum1+U;
        end
        sum2=sum2+sum1;
        sum1=0;
    end
    U_u_total=sqrt(U_u_total_nc+sum2);
    
    % Export Uncertainty
    
    Uncertainty.U_u_total=U_u_total;

    Uncertainty.u_rec_yaw=1/4.*(1./x_L(1).*v_los_yaw(1,:)+ 1./x_L(2).*v_los_yaw(2,:)+ 1./x_L(3).*v_los_yaw(3,:) + 1./x_L(4).*v_los_yaw(4,:));
    Uncertainty.u_rec_pitch=1/4.*(1./x_L(1).*v_los_pitch(1,:)+ 1./x_L(2).*v_los_pitch(2,:)+ 1./x_L(3).*v_los_pitch(3,:) + 1./x_L(4).*v_los_pitch(4,:));
    Uncertainty.u_rec_roll=1/4.*(1./x_L(1).*v_los_roll(1,:)+ 1./x_L(2).*v_los_roll(2,:)+ 1./x_L(3).*v_los_roll(3,:) + 1./x_L(4).*v_los_roll(4,:));
    Uncertainty.u_rec_heave=1/4.*(1./x_L(1).*v_los_heave(1,:)+ 1./x_L(2).*v_los_heave(2,:)+ 1./x_L(3).*v_los_heave(3,:) + 1./x_L(4).*v_los_heave(4,:));
    Uncertainty.v_los_yaw=v_los_yaw;
    Uncertainty.v_los_pitch=v_los_pitch;
    Uncertainty.v_los_roll=v_los_roll;
    Uncertainty.v_los_heave=v_los_heave;
    
    Uncertainty.psi=psi; 
    Uncertainty.beta=beta; 
    Uncertainty.gamma=gamma;
    Uncertainty.h_heave=h_heave;
       
    Uncertainty.u_rec_transx=1/4.*(1./x_L(1).*v_los_transx(1,:)+ 1./x_L(2).*v_los_transx(2,:)+ 1./x_L(3).*v_los_transx(3,:) + 1./x_L(4).*v_los_transx(4,:));
    Uncertainty.u_rec_transy=1/4.*(1./x_L(1).*v_los_transy(1,:)+ 1./x_L(2).*v_los_transy(2,:)+ 1./x_L(3).*v_los_transy(3,:) + 1./x_L(4).*v_los_transy(4,:));
    Uncertainty.u_rec_transz=1/4.*(1./x_L(1).*v_los_transz(1,:)+ 1./x_L(2).*v_los_transz(2,:)+ 1./x_L(3).*v_los_transz(3,:) + 1./x_L(4).*v_los_transz(4,:));
    Uncertainty.v_los_transx=v_los_transx;
    Uncertainty.v_los_transy=v_los_transy;
    Uncertainty.v_los_transz=v_los_transz;
    Uncertainty.xvel=xvel;
    Uncertainty.yvel=yvel;
    Uncertainty.zvel=zvel;
    


