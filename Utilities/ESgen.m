classdef ESgen < matlab.System
    % generate emergency stop signal
    
    properties (Access=private)
        ESOld;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.ESOld=0;
        end
        
        function ES = stepImpl(obj,trigger)
            if trigger>0.5 || obj.ESOld>0.5
                ES=1;
            else
                ES=0;
            end
            obj.ESOld=ES;
        end
        
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
    end
    

end