classdef UtoSlow< matlab.System
    % collect and average the U in the fast loop, then give it to the slow loop
%     properties
% 
%     end
    
    properties(Access=private)
        UOld=zeros(12,3);
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            
        end
        
        function USlow = stepImpl(obj,U,disable)
            if disable>0.5
                obj.UOld=U*ones(1,3);
                USlow=U;
            else
                obj.UOld(:,1:end-1)=obj.UOld(:,2:end);
                obj.UOld(:,end)=U;
                USlow=sum(obj.UOld,2)/length(obj.UOld(1,:));
            end

        end
        
        function d1 = getOutputDataTypeImpl(~)
            d1 = 'double';
        end
        
        function s1 = getOutputSizeImpl(~)
            s1=[12,1];
        end
        
        function f1 = isOutputFixedSizeImpl(~)
            f1=true;
        end
        
        function f1=isOutputComplexImpl(~)
            f1=false;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

