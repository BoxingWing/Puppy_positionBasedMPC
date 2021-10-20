classdef estX_byJoint < matlab.System
    % get CoM velocity est from the joints position feedback
    properties (Access=private)
        yWidth=97/1000; % distantce between left and right roll axis, i.e. distance between left and right leg coordinates
        xWidth=210.8/1000; % distance between fore and hind pitch axis, i.e. distance between fore and hind leg coordinates
        pArrayOld=zeros(3,4);
        piOFF=zeros(3,4); % offset pos vec from leg coordinate to world coordinate
        vCoMRec=zeros(3,21);
        ymOld=zeros(6,1);
        iniCount=0;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            obj.piOFF(:,1)=[obj.xWidth;obj.yWidth;0]/2;
            obj.piOFF(:,2)=[obj.xWidth;-obj.yWidth;0]/2;
            obj.piOFF(:,3)=[-obj.xWidth;obj.yWidth;0]/2;
            obj.piOFF(:,4)=[-obj.xWidth;-obj.yWidth;0]/2;
            obj.iniCount=0;
        end
        
        function [ym,updateEN,KF_R] = stepImpl(obj,pArray_B,RPY,omegaB,SPLeg,SP,dt,surPara)
            % omegaB must be the world coordinate
            % pArray_B is the foot-end position in body coordinate
            updateEN=1;
            %KF_R=0.5*eye(6);
            KF_R=diag([0.214e-6,0.137e-6,0.0357e-6,2.071e-3,5.364e-4,3.997e-4]);
            pArray_B=reshape(pArray_B,3,4)/1000;
            R=Rz(RPY(3))*Ry(RPY(2))*Rx(RPY(1));
            vArray=(pArray_B-obj.pArrayOld)/dt;
            if obj.iniCount<0.5
                vArray=zeros(3,4);
                obj.iniCount=1;
            end
            vArray=R*vArray;
            vBM=zeros(3,4);
            for i=1:1:4
                vBM(:,i)=-vArray(:,i)-cross(omegaB,R*pArray_B(:,i));
            end
            pCoM=[0;0;0];
            vCoM=[0;0;0];
            hPi=[0;0;0;0];
            count=0; % number of supported legs
            for i=1:1:4
                if SPLeg(i)>0.5
                    vCoM=vCoM+vBM(:,i);
                    SP(3,i)=surPara(1)+surPara(2)*SP(1,i)+surPara(3)*SP(2,i);
                    pCoM=pCoM+SP(:,i)+(-R*pArray_B(:,i));
                    count=count+1;
                else
                    hPi(i)=SP(3,i);
                end
            end
            if count>0.5
                vCoM=vCoM/count;
                pCoM=pCoM/count;
            else
                vCoM=obj.ymOld(4:6);
                pCoM=obj.ymOld(1:3);
                %KF_R=10^-2*eye(6);
                updateEN=0;
            end
            obj.vCoMRec(:,1:end-1)=obj.vCoMRec(:,2:end);
            obj.vCoMRec(:,end)=vCoM;
            %vCoMFiltered=sgolayfilt(obj.vCoMRec',3,21);
            %vCoMFiltered=vCoMFiltered';
%             ym=[pCoM;vCoMFiltered(:,end)];
            ym=[pCoM;vCoM];
            obj.pArrayOld=pArray_B;
            obj.ymOld=ym;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
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

function M=Rz(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[cos(sita),-sin(sita),0;
    sin(sita),cos(sita),0;
    0,0,1];
end