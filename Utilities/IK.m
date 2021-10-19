classdef IK < matlab.System
    % Inverse kinematics for Puppy quadruped robot w.r.t leg coordinates.
    % Angle offset must be considered outside
    properties
        LF_Off=[0;0;0];
        RF_Off=[0;0;0];
        LH_Off=[0;0;0];
        RH_Off=[0;0;0];
    end
    properties (Access=private)
        AB=44.5;
        BC=120;
        CDP=164.84/180*pi;
        DP=139.063;
        OR=37; % roll axis offset
        PxLim=[-70 70]; % defined w.r.t Leg 1
        PyLim=[-20 100]; % defined w.r.t Leg 1
        PzLim=[-140 -230]; % defined w.r.t Leg 1
        LastInput=[0;0;-190;0;0;-190;0;0;-190;0;0;-190];
        %%%AngleOff=[43.5;29.6;1;-40;-27.5;-1;42.3;27.9;-8;-42.3;-22;1]/180*pi;
        %AngleOff=[43.5;29.6;1;-41.9;-29.3;-1;42;27.6;-8;-43.8;-23.4;1]/180*pi;
        AngleOff=zeros(12,1);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        
        %function flag = supportsMultipleInstanceImpl(obj)
        % Support System object in Simulink For Each subsystem
        % Do not enable For Each support if your System object allocates exclusive resources that may
        % conflict with other System objects, such as allocating file handles, memory by address, or hardware resources.
        %    flag = true;
        %end
        
        function [AngleArray,OffsetPArray,pArray_L_Out,errFlag] = stepImpl(obj,pArray_L)
            % Implement algorithm.
            % PArray:=[Px_i,Py_i,Pz_i], 12*1, foot-end position in leg coordinate
            % AngleArray:=[Mi1,Mi2,Mi3]
            OffsetPArray=[obj.LF_Off;obj.RF_Off;obj.LH_Off;obj.RH_Off];
            pArray_L_Out=pArray_L+OffsetPArray;
            [pArray_L_Out,~]=obj.limitCorece(pArray_L_Out);
            
            [Angle1,Flag1]=obj.IK_one(pArray_L_Out(1:3),1);
            [Angle2,Flag2]=obj.IK_one(pArray_L_Out(4:6),2);
            [Angle3,Flag3]=obj.IK_one(pArray_L_Out(7:9),3);
            [Angle4,Flag4]=obj.IK_one(pArray_L_Out(10:12),4);
            
            AngleArray = [Angle1;Angle2;Angle3;Angle4]+obj.AngleOff;
            
            errFlag=[Flag1;Flag2;Flag3;Flag4];
            obj.LastInput=pArray_L_Out;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [pNew,Flag]=limitCorece(obj,pArray)
            Flag=isnan(pArray);
            index=find(Flag==1);
            if ~isempty(index)
                pArray(index)=obj.LastInput(index);
            end
            % pArray must be row vector
            pArrayS=reshape(pArray,3,4);
            pxNew=zeros(1,4);
            pyNew=zeros(1,4);
            pzNew=zeros(1,4);
            for i=1:1:4
                if i==2 || i==4
                    yLim=-obj.PyLim;
                else
                    yLim=obj.PyLim;
                end
                pxNew(i)=median([pArrayS(1,i);obj.PxLim(1);obj.PxLim(2);]);
                pyNew(i)=median([pArrayS(2,i);yLim(1);yLim(2);]);
                pzNew(i)=median([pArrayS(3,i);obj.PzLim(1);obj.PzLim(2);]);
            end
            pNew=reshape([pxNew;pyNew;pzNew],12,1);
        end
        
        
        function [Angle,Flag]=IK_one(obj,p,LegNum)
            % calculate one leg IK according to different leg index
            % Flag:
            %     0  normal
            %     1  roll calculation error
            %     2  pX, pY out of workspace
            %     3  AD collides with DP
            %     4  elbow touches the ground
            %     5  leg index assignment error
            % all error state will triger a three-zeros output
            Flag=0;
            Angle=[0;0;0];
            switch LegNum
                case {1,3}
                    RP=sqrt(p(2)^2+p(3)^2-obj.OR^2);
                    [pRy,pRz,errFlag]=Get1From2(0,0,p(2),p(3),obj.OR,RP,3);
                    roll=acos([1,0]*[pRy;pRz]/obj.OR)*sign(pRz); % angle for 13 angle
                    if errFlag~=0
                        Flag=1;
                        return;
                    end
                    pP=[p(1);-RP];
                    [xout,yout,errFlag]=Get1From2(0,0,pP(1),pP(2),obj.BC,obj.DP,2);
                    if errFlag~=0
                        Flag=2;
                        return;
                    end
                    pD=[xout;yout];
                    pC=[cos(-obj.CDP),-sin(-obj.CDP);sin(-obj.CDP),cos(-obj.CDP)]...
                        *(pP-pD)/obj.DP*obj.AB+pD;
                    pB=pC-pD;
                    alpha=acos([-1,0]*pD/obj.BC)*sign(pD(2)); % angle for 11 angle
                    beta=acos([-1,0]*pB/obj.AB)*sign(-pB(2)); % angle for 12 angle
                    if pP(2)>pD(2)
                        Flag=3;  % AD collides with DP
                        return;
                    elseif  pP(2)>pC(2)
                        Flag=4; % elbow touches the ground
                        return;
                    end
                    Angle=[alpha;beta;roll];
                case {2,4}
                    RP=sqrt(p(2)^2+p(3)^2-obj.OR^2);
                    [pRy,pRz,errFlag]=Get1From2(0,0,p(2),p(3),obj.OR,RP,2);
                    roll=acos([-1,0]*[pRy;pRz]/obj.OR)*sign(-pRz); % angle for 13 servo
                    if errFlag~=0
                        Flag=1;
                        return;
                    end
                    pP=[p(1);-RP];
                    [xout,yout,errFlag]=Get1From2(0,0,pP(1),pP(2),obj.BC,obj.DP,2);
                    if errFlag~=0
                        Flag=2;
                        return;
                    end
                    pD=[xout;yout];
                    pC=[cos(-obj.CDP),-sin(-obj.CDP);sin(-obj.CDP),cos(-obj.CDP)]...
                        *(pP-pD)/obj.DP*obj.AB+pD;
                    pB=pC-pD;
                    alpha=acos([-1,0]*pD/obj.BC)*sign(-pD(2)); % angle for 11 servo
                    beta=acos([-1,0]*pB/obj.AB)*sign(pB(2)); % angle for 12 servo
                    if pP(2)>pD(2)
                        Flag=3;  % AD collides with DP
                        return;
                    elseif  pP(2)>pC(2)
                        Flag=4; % elbow touches the ground
                        return;
                    end
                    Angle=[alpha;beta;roll];
                otherwise
                    Flag=5;
            end
        end
    end
end
function [xout,yout,Flag]=Get1From2(x1,y1,x2,y2,l1,l2,limit)
%   INPUT:
%   limit:
%   0: y bigger
%   1: y smaller
%   2: x smaller
%   3: x bigger
% OUTPUT:
% Flag:   0  normal
%           1  infeasible
%           2  limit intput error
Flag=0;
D=sqrt((x1-x2)^2+(y1-y2)^2);
if ~(l1+l2>D && abs(l1-l2)<D)
    xout=99;
    yout=99;
    Flag=1;
    return;
end
A=(l1+l2)^2-(x1-x2)^2-(y1-y2)^2;
B=-(l1-l2)^2+(x1-x2)^2+(y1-y2)^2;
E=(-l1^2+l2^2)*(y1-y2)+(y1+y2)*((x1-x2)^2-y1*y2)+y1^3+y2^3;
F=2*((x1-x2)^2+(y1-y2)^2);
temp=sqrt(A*B);
yo1=(x1*temp-x2*temp+E)/F;
yo2=(x2*temp-x1*temp+E)/F;
xo1=sqrt(l1^2-(yo1-y1)^2)+x1;
if abs((xo1-x2)^2+(yo1-y2)^2-l2^2)>10^-4
    xo1=-sqrt(l1^2-(yo1-y1)^2)+x1;
end
xo2=sqrt(l1^2-(yo2-y1)^2)+x1;
if abs((xo2-x2)^2+(yo2-y2)^2-l2^2)>10^-4
    xo2=-sqrt(l1^2-(yo2-y1)^2)+x1;
end
if abs(yo1-yo2)<10^-4
    xo2=2*x1-xo1;
end
switch limit
    case 0
        if yo1>yo2
            yout=yo1;   xout=xo1;
        else
            yout=yo2;   xout=xo2;
        end
    case 1
        if yo1<yo2
            yout=yo1;   xout=xo1;
        else
            yout=yo2;   xout=xo2;
        end
    case 2
        if xo1<xo2
            yout=yo1;  xout=xo1;
        else
            yout=yo2;  xout=xo2;
        end
    case 3
        if xo1>xo2
            yout=yo1; xout=xo1;
        else
            yout=yo2; xout=xo2;
        end
    otherwise
        xout=99;yout=99;
        Flag=2;
end
end