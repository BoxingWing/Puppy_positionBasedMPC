classdef ButtonHoldviaEdge < matlab.System
    % one channel up-edge detector
    
    properties (Access=private)
        dataOld;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.dataOld=0;
        end
        
        function data = stepImpl(obj,dataEdge)
            % NOTE: do NOT add vpxAdd and vpyAdd during start and stop
            if dataEdge>0.5
                data=obj.dataOld+1;
            else
                data=obj.dataOld;
            end
            if data>1.5
                data=0;
            end
            obj.dataOld=data;
        end
        
        function flag = supportsMultipleInstanceImpl(obj)
        % Support System object in Simulink For Each subsystem
        % Do not enable For Each support if your System object allocates exclusive resources that may
        % conflict with other System objects, such as allocating file handles, memory by address, or hardware resources.
            flag = true;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    
%     methods (Static, Access=protected)
%         function header = getHeaderImpl
%             header=matlab.system.display.Header(mfilename('class'), ...
%                 'title','Cycloid curve generator',...
%                    'Text',['Generate foot-end trajectory with cycloid curve in flight phase. ', ... 
%                'Constant vx is assigned in stance phase.']);
%         end
%     end
end