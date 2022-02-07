classdef estY_v2 < matlab.System
    % get CoM velocity and relative foot position from joints position feedback
    properties (Access=private)
        yWidth=97/1000; % distantce between left and right roll axis, i.e. distance between left and right leg coordinates
        xWidth=210.8/1000; % distance between fore and hind pitch axis, i.e. distance between fore and hind leg coordinates
        pArrayOld=zeros(3,4);
        piOFF=zeros(3,4); % offset pos vec from leg coordinate to world coordinate
        vCoMRec=zeros(3,5);
        vCoMRecZ=zeros(1,10);
        pCoMRec=zeros(3,5);
        pCoMRecZ=zeros(1,10);
        ymOld=zeros(6,1);
        iniCount=0;
        SPLegOld=zeros(4,1);
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
        
        function [Yext,updateEN,KF_R] = stepImpl(obj,pArray_B,RPY,omegaB,SPLeg,SP,dt,surPara)
            % omegaB must be the world coordinate
            % pArray_B is the foot-end position in body coordinate
            updateEN=1;
            pArray_B=reshape(pArray_B,3,4)/1000;
            R=Rz(RPY(3))*Ry(RPY(2))*Rx(RPY(1));

            % ------------
            % get foot end velocity in the world frame: vBM
            % ------------
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
            vCoM=[0;0;0];
            count=0;
            for i=1:1:4
                if SPLeg(i)>0.5
                    vCoM=vCoM+vBM(:,i);
                    count=count+1;
                end
            end
            if count>0.5
                vCoM=vCoM/count;
            end
            obj.vCoMRec(:,1:end-1)=obj.vCoMRec(:,2:end);
            obj.vCoMRec(:,end)=vCoM;
            vCoMOut=sum(obj.vCoMRec,2)/length(obj.vCoMRec(1,:));

            % ---------------
            % get relative foot end position in the world frame:pRel
            % ---------------
            pRel=zeros(3,4);
            for i=1:1:4
                pRel(:,i)=R*pArray_B(:,i);
            end
            
            hGround=zeros(1,4);
            for i=1:1:4
                hGround(i)=surPara(1)+surPara(2)*SP(1,i)+surPara(3)*SP(2,i);
            end

            %vCoMFiltered=sgolayfilt(obj.vCoMRec',3,21);
            %vCoMFiltered=vCoMFiltered';
            %ym=[pCoM;vCoMFiltered(:,end)];
            Yext=[reshape(pRel,1,12);hGround;vCoM];

            ym=[pCoMOut;vCoMOut];
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