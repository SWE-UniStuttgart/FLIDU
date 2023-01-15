%% FLIDU Main Function
% This function reads settings from UncertaintyConfig.m, calls the
% Uncertainty and Bias calculation functions and plots uncertainty and bias
% results

%Created 
% Moritz Gräfe 12.2022, Suttgart Wind Energy (SWE) 


clc
clear all
close all

% set anaysis case
ParameterStudy= true;
vlos_DOF= false;
LookupTable = false;

figures_repo='Figures\';
%% 2. Loop over processes
nProcess    = UncertaintyConfig(1,'GetNumberOfProcesses');
disp([num2str(nProcess) ' Processes will be preprocessed.'])
Random_seed = rand(4,1000);
for iProcess=1:nProcess
    
    %% 2.1 Get parameters for process
    PreProcessingVariation{iProcess}        = UncertaintyConfig(iProcess,'GetPreProcessingVariation');
    %nInitFiles   = 0;
    [nVariation,nPermutation,Permutation]   = PermutationMatrix(PreProcessingVariation{iProcess});
    
    %% 2.2 Loop over permutations
    for iPermutation    = 1:nPermutation
        Variation       = NaN(nVariation,1);
        for iVariation  = 1:nVariation
            Variation(iVariation)   = PreProcessingVariation{iProcess}{iVariation, 2}(Permutation(iPermutation, iVariation));
        end
        
        [ProcessName,ResultsFilesDir,Config, Parameter] = UncertaintyConfig(iProcess,'PreProcessing', Variation);
        Dynamics=CalcLidarDynamics(Parameter,Config, Random_seed);
        %         Call Error calc
        Error= Errorcalc(Parameter, Config,Dynamics);
        %         Call Uncertainty calc
        Uncertainty=Uncertaintycalc(Parameter,Config, Dynamics, Error);
        %save struct for timeseries plotting
        TS_cases{iPermutation}= Uncertainty;
        
        %% Prepare for plotting
        
        if ParameterStudy
            uncertainty_u{iProcess}(PreProcessingVariation{iProcess}{1, 2}    ==Variation(1), PreProcessingVariation{iProcess}{2, 2}==Variation(2),PreProcessingVariation{iProcess}{3, 2}==Variation(3))=Uncertainty.U_u_total;
            
            ErrorTable.V(PreProcessingVariation{iProcess}{1, 2}==Variation(1) ,...
                PreProcessingVariation{iProcess}{2, 2}==Variation(2), ...
                PreProcessingVariation{iProcess}{3, 2}==Variation(3),...
                PreProcessingVariation{iProcess}{4, 2}==Variation(4),...
                PreProcessingVariation{iProcess}{5, 2}==Variation(5)) = Error.u_total;
            ErrorTable.x(PreProcessingVariation{iProcess}{2, 2}==Variation(2))=Variation(2);
            ErrorTable.y(PreProcessingVariation{iProcess}{3, 2}==Variation(3))=Variation(3);
            ErrorTable.z(PreProcessingVariation{iProcess}{4, 2}==Variation(4))=Variation(4);
            error_u_rel{iProcess}(PreProcessingVariation{iProcess}{1, 2}==Variation(1),PreProcessingVariation{iProcess}{2, 2}==Variation(2),PreProcessingVariation{iProcess}{3, 2}==Variation(3))=Error.u_total  ;
            
        end
    end
    %%
    
    if LookupTable
        ErrorTable.V=mean(ErrorTable.V,5);
        save(ErrorTable)
    end
    
    if ParameterStudy
        error_u_rel{iProcess}=mean(error_u_rel{iProcess},3);
        uncertainty_u{iProcess}=mean(uncertainty_u{iProcess},3);
    end
    
end

%% FLIDU Figure 3
if ParameterStudy
    %Uncertainty and bias  in u for vary A Pitch and vary T Pitch
    
    % fiugre Uuncertainty
    title_numeration={'a','b','c','d'};
    figure
    tiledlayout(2,2)
    for iProcess=1:nProcess
        nexttile
        
        [x,y]=meshgrid(PreProcessingVariation{iProcess}{2, 2},PreProcessingVariation{iProcess}{1, 2});
        
        pcolor(x,y,uncertainty_u{iProcess})
        xlabel(PreProcessingVariation{iProcess}{2, 1})
        ylabel(PreProcessingVariation{iProcess}{1, 1})
        title([title_numeration{iProcess}, ')'])
        shading interp
        c=colorbar;
        caxis([0 1.5])
        c.Label.String = 'Uncertainty [m/s]';
        
    end
    readyforprint([5 4],7,'k','w',0.6)
    filename= join([figures_repo,'Uncertainty_u']);
    savefig(filename)
    print('-depsc2','-painters',filename)
    print(filename,'-dpdf')
    
    %%    %Bias u
    figure
    tiledlayout(2,2)
    for iProcess=1:4
        nexttile
        
        [x,y]=meshgrid(PreProcessingVariation{iProcess}{2, 2},PreProcessingVariation{iProcess}{1, 2});
        
        pcolor(x,y,error_u_rel{iProcess})
        xlabel(PreProcessingVariation{iProcess}{2, 1})
        ylabel(PreProcessingVariation{iProcess}{1, 1})
        title([title_numeration{iProcess}, ')'])
        shading interp
        c=colorbar;
        caxis([-0.15 0.1])
        c.Label.String = 'Bias  [m/s]';
        
    end
    readyforprint([5 4],7,'k','w',0.6)
    filename= join([figures_repo,'Bias_u']);
    savefig(filename)
    print('-depsc2','-painters',filename)
    print(filename,'-dpdf')
    
end

%% DOF
if vlos_DOF
    % translational velocities
    figure
    tiledlayout(1,3)
    num_beam=Config.num_beam;
    
    for iPlotting=3:length(TS_cases)
        
        TS=TS_cases{1,iPlotting};
        
        t=1:length(TS.xvel)
        c=turbo(4)
        
        nexttile
        hold on
        for i=1:num_beam
            v2=plot(TS.xvel, TS.v_los_transx(i,:), '-','Color',c(i,:));
            ylabel('LOS [m/s]')
            
        end
        plot(TS.xvel,TS.u_rec_transx,'--b')
        grid on
        hold off
        
        ylim([6 12])
        xlabel('x-Velocity [m/s]')
        title('x-Translation')
        
        
        nexttile
        hold on
        for i=1:num_beam
            v5=plot(TS.yvel, TS.v_los_transy(i,:), '-','Color',c(i,:));
            ylabel('LOS [m/s]')
            
        end
        plot(TS.yvel,TS.u_rec_transy,'--b')
        hold off
        
        grid on
        ylim([6 12])
        xlabel('y-Velocity [m/s]')
        title('y-Translation')
        
        nexttile
        hold on
        for i=1:num_beam
            v(i)=plot(TS.zvel, TS.v_los_transz(i,:), '-','Color',c(i,:));
            ylabel('LOS [m/s]')
            
        end
        v(i+1)=plot(TS.zvel,TS.u_rec_transz,'--b')
        hold off
        
        grid on
        ylim([6 12])
        xlabel('z-Velocity [m/s]')
        title('z-Translation')
    end
    
    Lgnd =  legend(v,'V_{LOS1}', 'V_{LOS2}','V_{LOS3}','V_{LOS4}','u - Windspeed reconststructed', 'Location','southoutside' ,'NumColumns',5)
    Lgnd.Layout.Tile = 'south';
    readyforprint([8 2],6,'k','w',0.4)
    
    filename= join([figures_repo,'V_LOS_trans']);
    savefig(filename)
    print('-depsc2','-painters',filename)
    print(filename,'-dpdf')
    
    % Rotation + heave
    figure
    tiledlayout(3,4)
    num_beam=Config.num_beam;
    
    for iPlotting=1:length(TS_cases)
        
        TS=TS_cases{1,iPlotting};
        t=1:length(TS.psi);
        c = turbo(4);
        
        nexttile
        hold on
        for i=1:num_beam
            p2=plot(rad2deg(TS.psi), TS.v_los_yaw(i,:),'-','Color',c(i,:),'LineWidth',1);
            ylabel('v_{los} [m/s]')
            ylim([7 11])
            grid on
            
        end
        plot(rad2deg(TS.psi),TS.u_rec_yaw, '--b')
        hold off
        
        xlabel('\psi [°]')
        title('Yaw');
        
        %pitch
        nexttile
        hold on
        for i=1:num_beam
            p5=plot(rad2deg(TS.beta), TS.v_los_pitch(i,:), '-', 'Color',c(i,:),'LineWidth',3);
            ylabel('v_{los} [m/s]')
            ylim([7 11])
            grid on
            
        end
        plot(rad2deg(TS.psi),TS.u_rec_pitch, '--b')
        hold off
        
        xlabel('\beta [°]')
        title('Pitch');
        
        % roll
        nexttile
        hold on
        for i=1:num_beam
            p8=plot(rad2deg(TS.gamma), TS.v_los_roll(i,:), '-','Color',c(i,:),'LineWidth',3);
            ylabel('v_{los} [m/s]')
            ylim([7 11])
            grid on
            
        end
        plot(rad2deg(TS.psi),TS.u_rec_roll, '--b')
        hold off
        xlabel('\gamma [°]')
        title('Roll');
        
        %heave
        nexttile
        hold on
        for i=1:num_beam
            p(i)=plot((TS.h_heave), TS.v_los_heave(i,:), '-', 'Color',c(i,:),'LineWidth',3);
            ylabel('v_{los} [m/s]')
            ylim([7 11])
            grid on
            
        end
        p(i+1)= plot(TS.h_heave,TS.u_rec_heave,'--b')
        hold off
        xlabel('heave [m]')
        title('Heave');
    end
    
    Lgnd =  legend(p,'V_{LOS1}', 'V_{LOS2}','V_{LOS3}','V_{LOS4}','u - Windspeed reconststructed', 'Location','southoutside' ,'NumColumns',5)
    Lgnd.Layout.Tile = 'south';
    readyforprint([8 6],6,'k','w',0.4)
    
    filename= join([figures_repo,'V_LOS_rotation']);
    savefig(filename)
    print('-depsc2','-painters',filename)
    print(filename,'-dpdf')
end