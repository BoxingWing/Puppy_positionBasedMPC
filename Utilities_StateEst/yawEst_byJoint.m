classdef yawEst_byJoint < matlab.System
    % get yaw velocity est from the joint position feedback
    properties
        dt=0.005;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [yawKine,updateEN] = stepImpl(obj,pArray_B,SPLeg,SP)
            % pArray_B is the foot-end position in body coordinate
            pArray_B=reshape(pArray_B,3,4)/1000;
            SP=reshape(SP,3,4);
            NCFlag=false;% no contace flag
            index=[0,0];
            count=0;
            yawNew=0;
            updateEN=1;
            for i=1:1:4
                if SPLeg(i)>0.5
                    count=count+1;
                    index(1)=index(2);
                    index(2)=i;
                end
            end
            if count<=2
                NCFlag=true;
            end
            if NCFlag==false
                pA=diag([1,1,0])*pArray_B(:,index(1));
                pB=diag([1,1,0])*pArray_B(:,index(2));
                pAW=diag([1,1,0])*SP(:,index(1));
                pBW=diag([1,1,0])*SP(:,index(2));
                AB=pA-pB;
                ABW=pAW-pBW;
                yawDelta=-atan2(AB(2),AB(1));
                yawBias=atan2(ABW(2),ABW(1));
                yawNew=yawDelta+yawBias;
            else
                yawNew=0;
                updateEN=0;
            end
            yawKine=yawNew;
            
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end