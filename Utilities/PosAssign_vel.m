classdef PosAssign_vel < matlab.System
    % constrain the output servo velocity
    properties
        cons_Vel=5; % degree per second
        dt=0.005;
    end
    
    properties (Access=private)
        PosDesOld;
        Ini_count;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.PosDesOld=zeros(12,1);
            obj.Ini_count=0;
        end
        
        function [PosOut, setEN] = stepImpl(obj,PosDes,PosNow,EN)
            setEN=1;
            if obj.Ini_count<=3
                obj.PosDesOld=PosNow;
                obj.Ini_count=obj.Ini_count+1;
                setEN=0;
            end
            deltaPos=PosDes-obj.PosDesOld;
            for i=1:1:12
                if abs(deltaPos(i)/obj.dt)>=abs(obj.cons_Vel)
                    deltaPos(i)=sign(deltaPos(i))*obj.cons_Vel*obj.dt;
                end
            end
            if EN<0.5
                deltaPos=PosDes-obj.PosDesOld;
            end
            PosOut=obj.PosDesOld+deltaPos;
            obj.PosDesOld=PosOut;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        %         %% Define output properties
        function num = getNumOutputsImpl(~)
            num = 2;
        end
        
        
        function varargout = isOutputFixedSizeImpl(~,~)
            varargout{1} = true;
            varargout{2} = true;
        end
        
        %
        function varargout = isOutputComplexImpl(~)
            varargout{1} = false;
            varargout{2} = false;
        end
        %
        function varargout = getOutputSizeImpl(~)
            varargout{1} = [12,1];
            varargout{2} = [1,1];
        end
        %
        function varargout = getOutputDataTypeImpl(~)
            varargout{1} = 'double';
            varargout{2} = 'double';
        end
    end
end