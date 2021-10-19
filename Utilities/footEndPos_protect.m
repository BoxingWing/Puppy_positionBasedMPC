classdef footEndPos_protect< matlab.System
    % offset and protect the footend positions
    
    properties(Access=private)
        pArray_L_Old=zeros(12,0);
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            obj.pArray_L_Old=[0;0;-190;0;0;-190;0;0;-190;0;0;-190]+[0;37;0;0;-37;0;0;37;0;0;-37;0];
        end
        
        function pArray_L = stepImpl(obj,PendAllL,ES)
            pArray_L=reshape(PendAllL,12,1)*1000;%+[0;37;0;0;-37;0;0;37;0;0;-37;0];
            if ES>0.5 
                pArray_L=obj.pArray_L_Old;
            end
            obj.pArray_L_Old=pArray_L;
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
        
        function c1 = isOutputComplexImpl(~)
            c1=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

