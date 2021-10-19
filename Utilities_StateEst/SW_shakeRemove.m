classdef SW_shakeRemove < matlab.System
    % prevent SW change to zeros until continuous three periods are all zero
    properties (Access=private)
        SWOld=zeros(3,4);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants

        end
        
        function SWnew = stepImpl(obj,SWori)
            SWnew=zeros(4,1);
            for i=1:1:4
                obj.SWOld(2:3,i)=obj.SWOld(1:2,i);
                obj.SWOld(1,i)=SWori(i);
                if sum(obj.SWOld(:,i))<0.5
                    SWnew(i)=0;
                else
                    SWnew(i)=1;
                end
            end
                    
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end