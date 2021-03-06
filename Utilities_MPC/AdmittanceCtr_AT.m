classdef AdmittanceCtr_AT < matlab.System
    % Inverse kinematics for Puppy quadruped robot w.r.t leg coordinates. Note that the output is in mm.
    % Angle offset must be considered outside
    properties
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        hIni=0.19;
        m=3;
        ks1=100; % ks1 (Nm/degree)
        ks2=100; % ks2 (Nm/degree)
        ks3=100; % ks3 (Nm/degree)
        bs1=0.1;
        bs2=0.1;
        bs3=0.1;
        dt=0.005;
    end
    properties (Access=private)
        AB=44.5;
        BC=120;
        CDP=164.84/180*pi;
        DP=139.063;
        OR=37; % roll axis offset
        LastInput=[0;0;-190;0;0;-190;0;0;-190;0;0;-190];
        PendAllnorm=zeros(3,4);
        pArray_L_Old=zeros(3,4);
        pArray_L_Now=zeros(3,4);
        interpol_Count=1;
        MPC_Count_Old=1;
        LegStateOld=zeros(4,1);
        deltaP1Old;
        deltaP2Old;
        deltaP3Old;
        deltaP4Old;
        deltaP1OOld;
        deltaP2OOld;
        deltaP3OOld;
        deltaP4OOld;
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            yW=obj.lateral_width;
            xW=obj.sagetial_width;
            %             obj.PendAllnorm=[0.1075,0.1075,-0.1075,-0.1075;
            %                 0.0750,-0.0750,0.0750,-0.0750;
            %                 0,0,0,0];
            PendAlltmp=zeros(12,1);
            PendAlltmp(1:3)=[xW;yW;0]/2;
            PendAlltmp(4:6)=[xW;-yW;0]/2;
            PendAlltmp(7:9)=[-xW;yW;0]/2;
            PendAlltmp(10:12)=[-xW;-yW;0]/2;
            PendAlltmp=PendAlltmp;%+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            obj.PendAllnorm=reshape(PendAlltmp,3,4);
            obj.deltaP1Old=zeros(3,1);
            obj.deltaP2Old=zeros(3,1);
            obj.deltaP3Old=zeros(3,1);
            obj.deltaP4Old=zeros(3,1);
            obj.deltaP1OOld=zeros(3,1);
            obj.deltaP2OOld=zeros(3,1);
            obj.deltaP3OOld=zeros(3,1);
            obj.deltaP4OOld=zeros(3,1);
            obj.interpol_Count=1;
        end
        
        function pL_Adm = stepImpl(obj,fL,pL_bas,Disable)
            % Implement algorithm of admittance ctr, refer to md file for more info.
            % pL_bas:=[Px_i,Py_i,Pz_i], 12*1, foot-end position in leg coordinate, unit:m
            % pL_Adm: [12,1], unit:mm
            pL_bas=reshape(pL_bas,3,4);
            fL=reshape(fL,12,1);
            if Disable>0.5
                pL_bas=[0,0,0,0;
                    obj.roll_Off,-obj.roll_Off,obj.roll_Off,-obj.roll_Off;
                    -obj.hIni,-obj.hIni,-obj.hIni,-obj.hIni];
                obj.pArray_L_Old=pL_bas;
                obj.pArray_L_Now=pL_bas;
                fL=[0;0;1;0;0;1;0;0;1;0;0;1]*9.8*obj.m/4;
            end

            pArray_L_tmp=pL_bas*1000;
%             +[0,0,0,0;
%                     obj.roll_Off,-obj.roll_Off,obj.roll_Off,-obj.roll_Off;
%                     0,0,0,0]*1000;

            [Angle1,Flag1]=obj.IK_one(pArray_L_tmp(:,1),1);
            [Angle2,Flag2]=obj.IK_one(pArray_L_tmp(:,2),2);
            [Angle3,Flag3]=obj.IK_one(pArray_L_tmp(:,3),3);
            [Angle4,Flag4]=obj.IK_one(pArray_L_tmp(:,4),4);
            
            J1=autoGen_Jacobi_1(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle1(1),Angle1(2),Angle1(3));
            J2=autoGen_Jacobi_2(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle2(1),Angle2(2),Angle2(3));
            J3=autoGen_Jacobi_3(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle3(1),Angle3(2),Angle3(3));
            J4=autoGen_Jacobi_4(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle4(1),Angle4(2),Angle4(3));
            
            Md=diag([obj.bs1,obj.bs2,obj.bs3]);
            d_deltaP1=Md*(obj.deltaP1Old-obj.deltaP1OOld)/obj.dt;
            d_deltaP2=Md*(obj.deltaP2Old-obj.deltaP2OOld)/obj.dt;
            d_deltaP3=Md*(obj.deltaP3Old-obj.deltaP3OOld)/obj.dt;
            d_deltaP4=Md*(obj.deltaP4Old-obj.deltaP4OOld)/obj.dt;

            if abs(obj.ks1)<10^-9 || abs(obj.ks2)<10^-9 || abs(obj.ks3)<10^-9
                deltaP1=[0;0;0];
                deltaP2=[0;0;0];
                deltaP3=[0;0;0];
                deltaP4=[0;0;0];
            else
                deltaP1=diag([1/obj.ks1,1/obj.ks2,1/obj.ks3])/1000*(-J1'*[fL(1);fL(2);fL(3)]-d_deltaP1);
                deltaP2=diag([1/obj.ks1,1/obj.ks2,1/obj.ks3])/1000*(-J2'*[fL(4);fL(5);fL(6)]-d_deltaP2);
                deltaP3=diag([1/obj.ks1,1/obj.ks2,1/obj.ks3])/1000*(-J3'*[fL(7);fL(8);fL(9)]-d_deltaP3);
                deltaP4=diag([1/obj.ks1,1/obj.ks2,1/obj.ks3])/1000*(-J4'*[fL(10);fL(11);fL(12)]-d_deltaP4);
            end
            
            Angle1new=Angle1+deltaP1;
            Angle2new=Angle2+deltaP2;
            Angle3new=Angle3+deltaP3;
            Angle4new=Angle4+deltaP4;
            
            P1new=autoGen_fk_1(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle1new(1),Angle1new(2),Angle1new(3));
            P2new=autoGen_fk_2(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle2new(1),Angle2new(2),Angle2new(3));
            P3new=autoGen_fk_3(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle3new(1),Angle3new(2),Angle3new(3));
            P4new=autoGen_fk_4(obj.OR,obj.AB,obj.BC,obj.DP,obj.CDP,Angle4new(1),Angle4new(2),Angle4new(3));
            
            pL_Adm=[P1new;P2new;P3new;P4new];
            
            errFlag=[Flag1;Flag2;Flag3;Flag4];
            obj.LastInput=pL_bas;
            obj.deltaP1OOld=obj.deltaP1Old;
            obj.deltaP2OOld=obj.deltaP2Old;
            obj.deltaP3OOld=obj.deltaP3Old;
            obj.deltaP4OOld=obj.deltaP4Old;
            obj.deltaP1Old=deltaP1;
            obj.deltaP2Old=deltaP2;
            obj.deltaP3Old=deltaP3;
            obj.deltaP3Old=deltaP4;
            %obj.MPC_Count_Old=MPC_Count;
            %obj.LegStateOld=LegState;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
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
                    if p(2)^2+p(3)^2-obj.OR^2<0
                        Flag=1;
                        return;
                    end
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
                    if p(2)^2+p(3)^2-obj.OR^2<0
                        Flag=1;
                        return;
                    end
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
            if abs(LegNum-3)<0.1 || abs(LegNum-4)<0.1
                Angle(3)=-Angle(3);
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

function M=Rx(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
M=[1,0,0;
    0,cos(sita),-sin(sita);
    0,sin(sita),cos(sita)];
end

function M=Ry(sita)
% 3D rotation matrix, vb=M*v:
% rotate a vector in one frame,
% or change the vector 'v' in rotated frame to 'vb' in world frame
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

function pP = autoGen_fk_1(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_FK_1
%    PP = AUTOGEN_FK_1(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:08

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta2);
t5 = cos(theta3);
t6 = sin(theta1);
t7 = sin(theta2);
t8 = sin(theta3);
t11 = 1.0./lAB;
t9 = t5.^2;
t10 = t8.^2;
t12 = t2-1.0;
t13 = t4-1.0;
t14 = t9+t10;
t15 = t13.*t14;
t16 = t15+1.0;
pP = [-lAD.*(t14.*(cos(theta1)-1.0)+1.0)+lDP.*t11.*(-lAB.*t16.*(t12.*t14+1.0)+lAB.*t3.*t7.*t9+lAB.*t3.*t7.*t10);lOA.*t5-lAD.*t6.*t8+lDP.*t11.*(lAB.*t3.*t8.*t16+lAB.*t7.*t8.*(t10.*t12+1.0)+lAB.*t7.*t8.*t9.*t12);lOA.*t8+lAD.*t5.*t6-lDP.*t11.*(lAB.*t3.*t5.*t16+lAB.*t5.*t7.*(t9.*t12+1.0)+lAB.*t5.*t7.*t10.*t12)];
end

function pP = autoGen_fk_2(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_FK_2
%    PP = AUTOGEN_FK_2(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:09

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta2);
t5 = cos(theta3);
t6 = sin(theta1);
t7 = sin(theta2);
t8 = sin(theta3);
t11 = 1.0./lAB;
t9 = t5.^2;
t10 = t8.^2;
t12 = t2-1.0;
t13 = t4-1.0;
t14 = t9+t10;
t15 = t13.*t14;
t16 = t15+1.0;
pP = [-lAD.*(t14.*(cos(theta1)-1.0)+1.0)-lDP.*t11.*(lAB.*t16.*(t12.*t14+1.0)+lAB.*t3.*t7.*t9+lAB.*t3.*t7.*t10);-lOA.*t5+lAD.*t6.*t8-lDP.*t11.*(-lAB.*t3.*t8.*t16+lAB.*t7.*t8.*(t10.*t12+1.0)+lAB.*t7.*t8.*t9.*t12);-lOA.*t8-lAD.*t5.*t6+lDP.*t11.*(-lAB.*t3.*t5.*t16+lAB.*t5.*t7.*(t9.*t12+1.0)+lAB.*t5.*t7.*t10.*t12)];
end

function pP = autoGen_fk_3(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_FK_3
%    PP = AUTOGEN_FK_3(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:09

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta2);
t5 = cos(theta3);
t6 = sin(theta1);
t7 = sin(theta2);
t8 = sin(theta3);
t11 = 1.0./lAB;
t9 = t5.^2;
t10 = t8.^2;
t12 = t2-1.0;
t13 = t4-1.0;
t14 = t9+t10;
t15 = t13.*t14;
t16 = t15+1.0;
pP = [-lAD.*(t14.*(cos(theta1)-1.0)+1.0)+lDP.*t11.*(-lAB.*t16.*(t12.*t14+1.0)+lAB.*t3.*t7.*t9+lAB.*t3.*t7.*t10);lOA.*t5+lAD.*t6.*t8-lDP.*t11.*(lAB.*t3.*t8.*t16+lAB.*t7.*t8.*(t10.*t12+1.0)+lAB.*t7.*t8.*t9.*t12);-lOA.*t8+lAD.*t5.*t6-lDP.*t11.*(lAB.*t3.*t5.*t16+lAB.*t5.*t7.*(t9.*t12+1.0)+lAB.*t5.*t7.*t10.*t12)];
end

function pP = autoGen_fk_4(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_FK_4
%    PP = AUTOGEN_FK_4(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:09

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta2);
t5 = cos(theta3);
t6 = sin(theta1);
t7 = sin(theta2);
t8 = sin(theta3);
t11 = 1.0./lAB;
t9 = t5.^2;
t10 = t8.^2;
t12 = t2-1.0;
t13 = t4-1.0;
t14 = t9+t10;
t15 = t13.*t14;
t16 = t15+1.0;
pP = [-lAD.*(t14.*(cos(theta1)-1.0)+1.0)-lDP.*t11.*(lAB.*t16.*(t12.*t14+1.0)+lAB.*t3.*t7.*t9+lAB.*t3.*t7.*t10);-lOA.*t5-lAD.*t6.*t8+lDP.*t11.*(-lAB.*t3.*t8.*t16+lAB.*t7.*t8.*(t10.*t12+1.0)+lAB.*t7.*t8.*t9.*t12);lOA.*t8-lAD.*t5.*t6+lDP.*t11.*(-lAB.*t3.*t5.*t16+lAB.*t5.*t7.*(t9.*t12+1.0)+lAB.*t5.*t7.*t10.*t12)];
end

function Jacobi = autoGen_Jacobi_1(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_JACOBI_1
%    JACOBI = AUTOGEN_JACOBI_1(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:08

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta1);
t5 = cos(theta2);
t6 = cos(theta3);
t7 = sin(theta1);
t8 = sin(theta2);
t9 = sin(theta3);
t10 = CDP+theta2;
t11 = cos(t10);
Jacobi = reshape([lAD.*t7,-lAD.*t4.*t9,lAD.*t4.*t6,lDP.*sin(t10),lDP.*t9.*t11,-lDP.*t6.*t11,0.0,-lOA.*t9-lAD.*t6.*t7+lDP.*t3.*t5.*t6+lDP.*t2.*t6.*t8,lOA.*t6-lAD.*t7.*t9+lDP.*t3.*t5.*t9+lDP.*t2.*t8.*t9],[3,3]);
end

function Jacobi = autoGen_Jacobi_2(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_JACOBI_2
%    JACOBI = AUTOGEN_JACOBI_2(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:09

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta1);
t5 = cos(theta2);
t6 = cos(theta3);
t7 = sin(theta1);
t8 = sin(theta2);
t9 = sin(theta3);
t10 = -theta2;
t11 = CDP+t10;
t12 = cos(t11);
Jacobi = reshape([lAD.*t7,lAD.*t4.*t9,-lAD.*t4.*t6,-lDP.*sin(t11),-lDP.*t9.*t12,lDP.*t6.*t12,0.0,lOA.*t9+lAD.*t6.*t7+lDP.*t3.*t5.*t6-lDP.*t2.*t6.*t8,-lOA.*t6+lAD.*t7.*t9+lDP.*t3.*t5.*t9-lDP.*t2.*t8.*t9],[3,3]);
end

function Jacobi = autoGen_Jacobi_3(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_JACOBI_3
%    JACOBI = AUTOGEN_JACOBI_3(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:08

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta1);
t5 = cos(theta2);
t6 = cos(theta3);
t7 = sin(theta1);
t8 = sin(theta2);
t9 = sin(theta3);
t10 = CDP+theta2;
t11 = cos(t10);
Jacobi = reshape([lAD.*t7,lAD.*t4.*t9,lAD.*t4.*t6,lDP.*sin(t10),-lDP.*t9.*t11,-lDP.*t6.*t11,0.0,-lOA.*t9+lAD.*t6.*t7-lDP.*t3.*t5.*t6-lDP.*t2.*t6.*t8,-lOA.*t6-lAD.*t7.*t9+lDP.*t3.*t5.*t9+lDP.*t2.*t8.*t9],[3,3]);
end

function Jacobi = autoGen_Jacobi_4(lOA,lAB,lAD,lDP,CDP,theta1,theta2,theta3)
%AUTOGEN_JACOBI_4
%    JACOBI = AUTOGEN_JACOBI_4(LOA,LAB,LAD,LDP,CDP,THETA1,THETA2,THETA3)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    12-Aug-2021 16:19:09

t2 = cos(CDP);
t3 = sin(CDP);
t4 = cos(theta1);
t5 = cos(theta2);
t6 = cos(theta3);
t7 = sin(theta1);
t8 = sin(theta2);
t9 = sin(theta3);
t10 = -theta2;
t11 = CDP+t10;
t12 = cos(t11);
Jacobi = reshape([lAD.*t7,-lAD.*t4.*t9,-lAD.*t4.*t6,-lDP.*sin(t11),lDP.*t9.*t12,lDP.*t6.*t12,0.0,lOA.*t9-lAD.*t6.*t7-lDP.*t3.*t5.*t6+lDP.*t2.*t6.*t8,lOA.*t6+lAD.*t7.*t9+lDP.*t3.*t5.*t9-lDP.*t2.*t8.*t9],[3,3]);
end


