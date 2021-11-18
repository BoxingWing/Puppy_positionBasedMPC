classdef ovWeight< matlab.System
    % weight of ov, first for one to num-1, last one for the end
    properties
        pxW=[10,10];
        pyW=[10,10];
        pzW=[180,180];
        rollW=[2,2];
        pitchW=[2,2];
        yawW=[5,5];
        dxW=[10,10];
        dyW=[10,10];
        dzW=[0.1,0.1];
        wxW=[0.1,0.1];
        wyW=[0.1,0.1];
        wzW=[0.1,0.1];
        numP=6;
    end

    properties(Access=private)

    end

    methods(Access = protected)

        function setupImpl(obj)

        end

        function ovWeight = stepImpl(obj)
            ovWeight=zeros(6,13);
            ovWeight(1:6,:)=ones(obj.numP,1)*[obj.pxW(1),obj.pyW(1),obj.pzW(1),obj.rollW(1),obj.pitchW(1),obj.yawW(1), obj.dxW(1),obj.dyW(1),obj.dzW(1),obj.wxW(1),obj.wyW(1),obj.wzW(1),0];
            ovWeight(6,:)=[obj.pxW(2),obj.pyW(2),obj.pzW(2),obj.rollW(2),obj.pitchW(2),obj.yawW(2),obj.dxW(2),obj.dyW(2),obj.dzW(2),obj.wxW(2),obj.wyW(2),obj.wzW(2),0];
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

        function d1 = getOutputDataTypeImpl(~)
            d1 = 'double';
        end

        function s1 = getOutputSizeImpl(~)
            s1=[6,13];
        end

        function f1= isOutputFixedSizeImpl(~)
            f1=true;
        end

        function c1 = isOutputComplexImpl(~)
            c1=false;
        end
    end
end

