classdef UtoSlow< matlab.System
    % collect and average the U in the fast loop, then give it to the slow loop
%     properties
% 
%     end
    
    properties(Access=private)
        UOld=zeros(12,4);
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.UOld=zeros(12,4);
        end
        
        function USlow = stepImpl(obj,U,disable)
            USlow=zeros(12,1);
            if disable>0.5
                obj.UOld=U*ones(1,4);
                USlow=U;
                for i=1:1:12
                    USlow(i)=U(i);
                end
            else
                obj.UOld(:,1:end-1)=obj.UOld(:,2:end);
                obj.UOld(:,end)=U;
                tmp=sum(obj.UOld,2)/length(obj.UOld(1,:));
                for i=1:1:12
                    USlow(i)=tmp(i);
                end

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

