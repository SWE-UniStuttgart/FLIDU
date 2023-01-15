function varargout  = UncertaintyConfig(iProcess,Purpose, varargin)
% MG on 01/12/2021
%----------------------------------------------------------------------------%
% ProcessName and variation arrays:
ProcessNameCell     = {
    %     % Parameterstudy % activate Parameterstudy in FLIDU_v1_0
    'Pitch_alpha' {'A Pitch [°]' [0:5] ; ' Shear Exponent \alpha [-]' [0.0:0.1:0.2]; 'Seed' [1:2]; 'A_yaw' [0];'A_roll' [0]; 'A_heave' [0]};
    'Pitch_static_beta' {'A Pitch [°]' [0:5] ; 'Mean Pitch [°]' [-5:0]; 'Seed' [1:2]; 'A_yaw' [0];'A_roll' [0]; 'A_heave' [0]};
    % % % % %   % Vary T_p Pitch
    'TPitch_alpha' {'T Pitch [s]' [10:5:50] ; 'Shear Exponent [-]' [0.0:0.1:0.2];'Seed' [1:2]; 'A_{\beta} [°]' [2] ; 'A_yaw' [0];'A_roll' [0]; 'A_heave' [0]};
    'TPitch_static_beta' {'T Pitch [s]' [10:5:50]; 'Mean Pitch [°]' [-5:0];'Seed' [1:2]; 'A_{\beta} [°]' [2] ; 'A_yaw' [0];'A_roll' [0]; 'A_heave' [0]};
    
    
    %     % Single DOF influence on LOS % activate vlos_DOF in FLIDU_v1_0
    %         'vlos_DOF' {'A_{\beta} [°]' [5]; '\alpha [°]' [0.0:0.1:0.2]};
    %
    %     % Create LookupTable for Correction % activate LookupTable in FLIDU_v1_0
    %     'LookupTable' {'URef [°]' [4:4:20] ; 'A_{\beta} [°]' [0:1:7] ; '\alpha [°]' [0.0:0.1:0.2]; 'static \beta [°]' [-5:1:0]; 'Seed' [1:5]; 'A_yaw' [0];'A_roll' [0]; 'A_heave' [0]};
    %
    };
%----------------------------------------------------------------------------%

nProcess                = size(ProcessNameCell, 1);
ProcessName             = ProcessNameCell{iProcess, 1};
PreProcessingVariation  = ProcessNameCell{iProcess, 2};
%--------------------------------------------------------------------------
% Directories
ResultsFilesDir      = ['N:\SWE\71_Simulationsserver\Rok\44_Graefe\FLIDU\Analytic\FLIDU\Results\'];

%------------------------------------------------------------------
%--------------------------------------------------------------------------

switch Purpose
    case 'PreProcessing'
        Variation       = varargin{1};
end

switch ProcessName
    
    case 'Pitch_alpha'
        
        switch Purpose
            case 'PreProcessing'
                %set Parametersc
                
                Parameter.DoF.Tp_yaw=30;
                Parameter.DoF.Tp_pitch=30;
                Parameter.DoF.Tp_roll=30;
                Parameter.DoF.Tp_heave=30;
                
                Parameter.DoF.A_yaw=Variation(4);
                Parameter.DoF.psi_static=0;
                
                Parameter.DoF.A_pitch=Variation(1);
                Parameter.DoF.beta_static=0;
                
                Parameter.DoF.A_roll=Variation(5);
                Parameter.DoF.gamma_static=0;
                
                Parameter.DoF.A_heave=Variation(6);
                
                
                Parameter.EC.alpha=Variation(2);
                Parameter.EC.phi=90; % in degrees; 90° perfect alignment
                Parameter.EC.vref=10;
                
                Parameter.Wind.Seed=Variation(3);
                
                % set config
                Config.calc_time= 600;
                Config.calc_velocities=true;
                Config.theta_lidar=deg2rad(19.8);
                Config.phi_lidar=deg2rad([39.6 140.4 -39.6 -140.40]);
                %                 Config.r=211.75;
                Config.r=212.5624;
                Config.num_beam=4;
                
                Config.z_L_upper=45.9;
                Config.z_L_lower=-45.9;
                
                Config.H_ref=100;
                Config.h_lidar=100;
                
        end
    case 'Pitch_static_beta'
        
        switch Purpose
            case 'PreProcessing'
                %set Parameters
                Parameter.DoF.Tp_yaw=30;
                Parameter.DoF.Tp_pitch=30;
                Parameter.DoF.Tp_roll=30;
                Parameter.DoF.Tp_heave=30;
                
                Parameter.DoF.A_yaw=Variation(4);
                Parameter.DoF.psi_static=0;
                
                Parameter.DoF.A_pitch=Variation(1);
                Parameter.DoF.beta_static=Variation(2);
                
                Parameter.DoF.A_roll=Variation(5);
                Parameter.DoF.gamma_static=0;
                
                Parameter.DoF.A_heave=Variation(6);
                
                
                Parameter.EC.alpha=0.10;
                Parameter.EC.phi=90; % in degrees; 90° perfect alignment
                Parameter.EC.vref=10;
                
                Parameter.Wind.Seed=Variation(3);
                
                % set config
                Config.calc_time= 600;
                Config.calc_velocities=true;
                Config.theta_lidar=deg2rad(19.8);
                Config.phi_lidar=deg2rad([39.6 140.4 -39.6 -140.40]);
                %                 Config.r=211.75;
                Config.r=212.5624;
                Config.num_beam=4;
                
                Config.z_L_upper=45.9;
                Config.z_L_lower=-45.9;
                
                Config.H_ref=100;
                Config.h_lidar=100;
                
        end
        %%%%%%%% switch T pitch cases
        
    case 'TPitch_alpha'
        switch Purpose
            case 'PreProcessing'
                %set Parameters
                Parameter.DoF.Tp_yaw=30;
                Parameter.DoF.Tp_pitch=Variation(1);
                Parameter.DoF.Tp_roll=30;
                Parameter.DoF.Tp_heave=30;
                
                Parameter.DoF.A_yaw=Variation(5);
                Parameter.DoF.psi_static=0;
                
                Parameter.DoF.A_pitch=Variation(4);
                Parameter.DoF.beta_static=0;
                
                Parameter.DoF.A_roll=Variation(6);
                Parameter.DoF.gamma_static=0;
                
                Parameter.DoF.A_heave=Variation(7);
                
                
                Parameter.EC.alpha=Variation(2);
                Parameter.EC.phi=90; % in degrees; 90° perfect alignment
                Parameter.EC.vref=10;
                
                Parameter.Wind.Seed=Variation(3);
                
                
                % set config
                Config.calc_time= 600;
                Config.calc_velocities=true;
                Config.theta_lidar=deg2rad(19.8);
                Config.phi_lidar=deg2rad([39.6 140.4 -39.6 -140.40]);
                %                 Config.r=211.75;
                Config.r=212.5624;
                Config.num_beam=4;
                
                Config.z_L_upper=45.9;
                Config.z_L_lower=-45.9;
                
                Config.H_ref=100;
                Config.h_lidar=100;
        end
    case 'TPitch_static_beta'
        
        switch Purpose
            case 'PreProcessing'
                %set Parameters
                Parameter.DoF.Tp_yaw=30;
                Parameter.DoF.Tp_pitch=Variation(1);
                Parameter.DoF.Tp_roll=30;
                Parameter.DoF.Tp_heave=30;
                
                Parameter.DoF.A_yaw=Variation(5);
                Parameter.DoF.psi_static=0;
                
                Parameter.DoF.A_pitch=Variation(4);
                Parameter.DoF.beta_static=Variation(2);
                
                Parameter.DoF.A_roll=Variation(6);
                Parameter.DoF.gamma_static=0;
                
                Parameter.DoF.A_heave=Variation(7);
                
                Parameter.EC.alpha=0.11;
                Parameter.EC.phi=90; % in degrees; 90° perfect alignment
                Parameter.EC.vref=10;
                Parameter.Wind.Seed=Variation(3);
                
                % set config
                Config.calc_time= 600;
                Config.calc_velocities=true;
                Config.theta_lidar=deg2rad(19.8);
                Config.phi_lidar=deg2rad([39.6 140.4 -39.6 -140.40]);
                %                 Config.r=211.75;
                Config.r=212.5624;
                Config.num_beam=4;
                
                Config.z_L_upper=45.9;
                Config.z_L_lower=-45.9;
                
                Config.H_ref=100;
                Config.h_lidar=100;
        end
        
        
        
        %%%%%%%%%%%%%
        
        
    case 'vlos_DOF'
        switch Purpose
            case 'PreProcessing'
                %set Parameters
                Parameter.DoF.Tp_yaw=30;
                Parameter.DoF.Tp_pitch=30;
                Parameter.DoF.Tp_roll=30;
                Parameter.DoF.Tp_heave=30;
                
                
                Parameter.DoF.A_yaw=5;
                Parameter.DoF.psi_static=0;
                
                Parameter.DoF.A_pitch= Variation(1);
                Parameter.DoF.beta_static= 0;
                
                Parameter.DoF.A_roll=5;
                Parameter.DoF.gamma_static=0;
                
                Parameter.DoF.A_heave=5;
                
                Parameter.DoF.A_xvel=2;
                Parameter.DoF.A_yvel=2;
                Parameter.DoF.A_zvel=2;
                
                Parameter.EC.alpha=Variation(2);
                Parameter.EC.phi=90;
                Parameter.EC.vref=10;
                Parameter.Wind.Seed=1;
                
                % set config
                Config.calc_time= 600;
                Config.calc_velocities=true;
                Config.theta_lidar=deg2rad(19.8);
                Config.phi_lidar=deg2rad([39.6 140.4 -39.6 -140.40]);
                %                 Config.r=211.75;
                Config.r=212.5624;
                Config.num_beam=4;
                
                Config.z_L_upper=45.9;
                Config.z_L_lower=-45.9;
                
                Config.H_ref=100;
                Config.h_lidar=100;
                
            case 'LookupTable'
                
                switch Purpose
                    case 'PreProcessing'
                        
                        Parameter.DoF.Tp_yaw=30;
                        Parameter.DoF.Tp_pitch=30;
                        Parameter.DoF.Tp_roll=30;
                        Parameter.DoF.Tp_heave=30;
                        
                        Parameter.DoF.A_yaw=Variation(6);
                        Parameter.DoF.psi_static=0;
                        
                        Parameter.DoF.A_pitch=Variation(2);
                        Parameter.DoF.beta_static=Variation(4);
                        
                        Parameter.DoF.A_roll=Variation(7);
                        Parameter.DoF.gamma_static=0;
                        
                        Parameter.DoF.A_heave=Variation(8);
                        
                        
                        Parameter.EC.alpha=Variation(3);
                        Parameter.EC.phi=90; % in degrees; 90° perfect alignment
                        Parameter.EC.vref=Variation(1);
                        Parameter.Wind.Seed=Variation(5);
                        
                        % set config
                        Config.calc_time= 600;
                        Config.calc_velocities=true;
                        Config.theta_lidar=deg2rad(19.8);
                        Config.phi_lidar=deg2rad([39.6 140.4 -39.6 -140.40]);
                        %                 Config.r=211.75;
                        Config.r=212.5624;
                        Config.num_beam=4;
                        
                        Config.z_L_upper=45.9;
                        Config.z_L_lower=-45.9;
                        
                        Config.H_ref=100;
                        Config.h_lidar=100;
                end
                
        end
        
end% switch process name


%% Outputs
switch Purpose
    case 'GetNumberOfProcesses'
        varargout{1}    = nProcess;
    case 'GetPreProcessingVariation'
        varargout{1}    = PreProcessingVariation;
        varargout{2}    = ProcessName;
%         varargout{3}    = ResultsFilesDir;
    case 'PreProcessing'
        varargout{1}    = ProcessName;
%         varargout{2}    = ResultsFilesDir;
        varargout{3}    = Config;
        varargout{4}    = Parameter;
        
end

end

