classdef nosieControl < matlab.System
    % adjust the measurement (R) and pocess (Q) noise according to the leg state
    properties
        R_preli=[1.0752,1.1534,0.3108,1.1410,1.4025,0.4953,1.0752,1.5684,0.3992,1.3367, ...
            1.1690,0.6264]*10^-4;
        R_h=0.05*10^-7;
        R_v=[2.4541,1.5699,2.3092]*10^-3
        scaleh=1000;

        Q_r=[10,5,1]*10^-8;
        Q_v=[1,0.01,100]*10^-7;
        Q_pi=[1,1,1]*10^-7;
        Q_abar=[1,1,1]*10^-5;
        scale_pi=1000;
    end

    properties (Access=private)

    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [KF_Q,KF_R] = stepImpl(obj,SPLeg)
            scaleh_i=ones(1,4);
            scalepi_i=ones(1,4);
            for i=1:1:4
                if SPLeg(i)>0.5
                    scaleh_i(i)=1;
                    scalepi_i(i)=1;
                else
                    scaleh_i(i)=obj.scaleh;
                    scalepi_i(i)=obj.scale_pi;
                end
            end
            KF_R=[obj.R_preli,scaleh_i(1)*obj.R_h,scaleh_i(2)*obj.R_h,...
                scaleh_i(3)*obj.R_h,scaleh_i(4)*obj.R_h,obj.R_v];
            KF_Q=[obj.Q_r,obj.Q_v,scalepi_i(1)*obj.Q_pi,scalepi_i(2)*obj.Q_pi,scalepi_i(3)*obj.Q_pi,...
                scalepi_i(4)*obj.Q_pi,obj.Q_abar];
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end
