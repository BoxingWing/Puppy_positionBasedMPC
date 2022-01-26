classdef VelLowPass< matlab.System
    % force and moment compensation via PD laws
    properties
        filtN=3;
    end
    properties (Access=private)
        drStore=zeros(3,100);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.drStore=zeros(3,100);
        end
        
        function xFB_Filt = stepImpl(obj,xFB)
            obj.drStore(:,1:obj.filtN-1)=obj.drStore(:,2:obj.filtN);
            obj.drStore(:,obj.filtN)=xFB(7:9);
            xFB_Filt=sum(obj.drStore(:,1:obj.filtN),2)/obj.filtN;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

% function vcap=crossCap(v)
% vcap=[0,-v(3),v(2);
%         v(3),0,-v(1);
%        -v(2),v(1),0];
% end