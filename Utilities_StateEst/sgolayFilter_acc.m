classdef sgolayFilter_acc < matlab.System
    % wrap savitzky-golay filter for simulink model
    properties (Access=private)
       accRec=zeros(3,11);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constant
            obj.accRec=zeros(3,21);
        end
        
        function accNew = stepImpl(obj,accOri)
            obj.accRec(:,1:end-1)=obj.accRec(:,2:end);
            obj.accRec(:,end)=accOri;
            accFiltered=sgolayfilt(obj.accRec',2,11);
            accFiltered=accFiltered';
            accNew=accFiltered(:,end);
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end