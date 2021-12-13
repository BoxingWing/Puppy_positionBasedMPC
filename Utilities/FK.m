classdef FK < matlab.System
    % Forward kinematics for Puppy quadruped robot in leg coordinates

    properties
        dt=0.005;
    end

    properties (Access=private)
        AB=44.5;
        BC=120;
        CDP=164.84/180*pi;
        DP=139.063;
        OR=37; % roll axis offset
        AngleArrayOld=zeros(12,1);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.AngleArrayOld=zeros(12,1);
        end
        
        
%         function flag = supportsMultipleInstanceImpl(obj)
%             % Support System object in Simulink For Each subsystem
%             % Do not enable For Each support if your System object allocates exclusive resources that may
%             % conflict with other System objects, such as allocating file handles, memory by address, or hardware resources.
%             flag = true;
%         end
        
        function [pArray_L,errFlag,dq] = stepImpl(obj,AngleArray)
            % pArray_L is the foot-end position in leg coordinate
            [pL1,Flag1]=obj.FK_one(AngleArray(1:3),1);
            [pL2,Flag2]=obj.FK_one(AngleArray(4:6),2);
            [pL3,Flag3]=obj.FK_one(AngleArray(7:9),3);
            [pL4,Flag4]=obj.FK_one(AngleArray(10:12),4);
            
            dq=(AngleArray-obj.AngleArrayOld)/obj.dt; 
            dq(2)=-dq(2);dq(4)=-dq(4); % need to check
            dq(8)=-dq(8);dq(10)=dq(10);

            pArray_L = [pL1;pL2;pL3;pL4];
            errFlag=[Flag1;Flag2;Flag3;Flag4];
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [pP,Flag]=FK_one(obj,Angle,LegNum)
            % calculate one leg IK according to different leg index
            % Flag:
            %     0  normal
            %     1  leg index assignment error
            % all error state will triger a three-zeros output
            Flag=0;
            switch LegNum
                case {1,3}
                    alpha=Angle(1);
                    beta=-Angle(2);
                    roll=Angle(3);
                    ORnow=obj.OR;
                case {2,4}
                    alpha=-Angle(1);
                    beta=Angle(2);
                    roll=Angle(3);
                    ORnow=-obj.OR;
                otherwise
                    pP=[0;0;0];
                    Flag=1;
                    return;
            end
            pD=[-obj.BC;0;0];
            pD=Ry(alpha)*pD;
            ADC=pi-abs(alpha-beta);
            pP=pD+Ry(-ADC+alpha+pi-obj.CDP)*[-obj.DP;0;0];
            pP=Rx(roll)*(pP+[0;ORnow;0]);
        end
    end
end
function M=Rx(sita)
% 3D rotation matrix, from world to body
M=[1,0,0;
    0,cos(sita),-sin(sita);
    0,sin(sita),cos(sita)];
end

function M=Ry(sita)
% 3D rotation matrix, from world to body
M=[cos(sita),0,sin(sita);
    0,1,0;
    -sin(sita),0,cos(sita)];
end
