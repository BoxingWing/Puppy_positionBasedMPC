classdef KalmanFilter_equProj < matlab.System
    % equality constraint projection for kalman filter
%     properties
%         Ts=0.005;
%     end
    
    properties (Access=private)
        xhatOld;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            xhatOld=zeros(21,1);
        end
        
        function xWave = stepImpl(obj,xhat,SPLeg)
            
            obj.xhatOld=xhat;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end