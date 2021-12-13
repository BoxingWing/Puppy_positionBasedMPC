classdef LegSequence_RT_v3< matlab.System
    % adjust supporting legs, foot-hold points, and CoM state for MPC
    % based on CoM velocity
    properties
        r0=[0;0;0.19];
        theta0=[0;0;0];
        dr0=[0;0;0];
        omega0=[0;0;0];
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        m=3.5;
        kx=0;
        ky=0;
        kRz=0;
        T=0.8; % gait duration
        StepH=0.04;
        SampleTime = 0.005; % Sample Time
        OffsetTime = 0; % Offset Time
        TickTime = 0;
        startPhase=0;
        SampleTimeTypeProp (1, 1) {mustBeMember(SampleTimeTypeProp, ...
            ["Discrete","FixedInMinorStep","Controllable",...
            "Inherited","InheritedNotControllable",...
            "InheritedErrorConstant"])} = "Discrete"
    end
    
    properties(Access=private)
        phiOld;
        PendAllnorm;
        pL_LS; % PendAll of last step
        PendAllLocalOld;
        LegStateOld;
        MPC_legStateOld;
        X_mpc_Old;
        X_interPol;
        RateCount;
        pArray_L_Adm_Old;
        interpol_Count;
        pArray_L_Adm_Now;
        MPC_Count_Old;
        LegCorOri;
        pLnorm;
        vNowN;
        desvxFilt_Old;
        desvyFilt_Old;
        deswzFilt_Old;
    end
    
    methods(Access = protected)
        function sts = getSampleTimeImpl(obj)
            switch char(obj.SampleTimeTypeProp)
                case 'Inherited'
                    sts = createSampleTime(obj,'Type','Inherited');
                case 'InheritedNotControllable'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'AlternatePropagation','Controllable');
                case 'InheritedErrorConstant'
                    sts = createSampleTime(obj,'Type','Inherited',...
                        'ErrorOnPropagation','Constant');
                case 'FixedInMinorStep'
                    sts = createSampleTime(obj,'Type','Fixed In Minor Step');
                case 'Discrete'
                    sts = createSampleTime(obj,'Type','Discrete',...
                        'SampleTime',obj.SampleTime, ...
                        'OffsetTime',obj.OffsetTime);
                case 'Controllable'
                    sts = createSampleTime(obj,'Type','Controllable',...
                        'TickTime',obj.TickTime);
            end
        end
        
        function setupImpl(obj)
            obj.phiOld=0;
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
            PendAlltmp=PendAlltmp+[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off;
            obj.LegCorOri=reshape(PendAlltmp-[0;1;0;0;-1;0;0;1;0;0;-1;0]*obj.roll_Off,3,4);
            obj.PendAllnorm=reshape(PendAlltmp,3,4); % norminal foot position in the world coordinate
            obj.pLnorm=[0;0;-obj.r0(3)]*[1,1,1,1]+reshape([0;1;0;0;-1;0;0;1;0;0;-1;0],3,4)*obj.roll_Off; % norminal foot position in the leg coordinate
            obj.PendAllLocalOld=[0;0;-obj.r0(3)]*[1,1,1,1]+reshape([0;1;0;0;-1;0;0;1;0;0;-1;0],3,4)*obj.roll_Off;
            obj.pL_LS=obj.PendAllLocalOld;
            
            obj.LegStateOld=[1;0;0;1];
            obj.MPC_legStateOld=[1;1;1;1];
            obj.X_mpc_Old=zeros(12,1);
            obj.RateCount=0;
            obj.X_interPol=[obj.r0;zeros(9,1)];
            obj.interpol_Count=0;
            obj.pArray_L_Adm_Old=[0;obj.roll_Off;-obj.r0(3); ...
                0;-obj.roll_Off;-obj.r0(3); ...
                0;obj.roll_Off;-obj.r0(3); ...
               0;-obj.roll_Off;-obj.r0(3);];
            obj.pArray_L_Adm_Now=[0;obj.roll_Off;-obj.r0(3); ...
                0;-obj.roll_Off;-obj.r0(3); ...
                0;obj.roll_Off;-obj.r0(3); ...
               0;-obj.roll_Off;-obj.r0(3);];
            obj.MPC_Count_Old=0;
            obj.vNowN=zeros(3,2);
            obj.desvxFilt_Old=0;
            obj.desvyFilt_Old=0;
            obj.deswzFilt_Old=0;
        end
        
        function [LegState,PendAllLocal] = stepImpl(obj,phi,pArray_L_Adm,X_FB,MPC_Count,touchInd,ref,surP,surVN,zSur,OscStop,LegStateMPC)
            % X_FB: system states from the estimator
            % X_mpc: predicted next step's systems states from the MPC controller
            % touchInd: indicator of wether a swing leg touches the ground
            % T is the moving period
            LegState=[1;0;0;1];
            s=phi/pi;
            if phi>pi
                LegState=[0;1;1;0];
                s=s-1;
            end
            if OscStop>0.5
                LegState=[1;1;1;1];
            end
            desvX=ref(7);
            desvY=ref(8);
            desRoll=ref(4);
            desPit=ref(5);
            desYaw=ref(6);
            xFiltFactor=0.1;
            yFiltFactor=0.2;
            wzFiltFactor=0.03;
            
            desvxFilt=desvX*xFiltFactor+obj.desvxFilt_Old*(1-xFiltFactor);
            desvyFilt=desvY*yFiltFactor+obj.desvyFilt_Old*(1-yFiltFactor);
            %deswzFilt=
            
            obj.desvxFilt_Old=desvxFilt;
            obj.desvyFilt_Old=desvyFilt;
            
            vDes=[desvxFilt;desvyFilt;0];
            vDesL=Rx(desRoll)'*Ry(desPit)'*Rz(desYaw)'*vDes; % Yet to ADD Rx and Ry !!!!!!!!!!!!!!!!!
            
            vNow=[X_FB(7:8);X_FB(9)];
            yawNow=X_FB(6);
            Rrpy=Rz(yawNow)*Ry(X_FB(5))*Rx(X_FB(4));
            vNowL=Rrpy'*vNow; % Yet to ADD Rx and Ry !!!!!!!!!!!!!!!!!
            
            %%% next step foot-placement in the leg coordinate
            vLDZ=[0.02,0.02,0.02]; % dead zone for vNowL
            for i=1:1:3
                if abs(vNowL(i))>=abs(vLDZ(i))
                    vNowL(i)=vNowL(i)-sign(vNowL(i))*abs(vLDZ(i));
                else
                    vNowL(i)=0;
                end
            end
            %v=vNow+[-obj.kx*(vDesL(1)-vNowL(1)); ...
            %    -obj.ky*(vDesL(2)-vNowL(2));0];
            obj.vNowN(:,1:end-1)=obj.vNowN(:,2:end);
            obj.vNowN(:,end)=vNowL;
            
            vNowFilt=sum(obj.vNowN,2)/length(obj.vNowN(1,:));
            
            v=vNowFilt+[-obj.kx*(vDesL(1)-vNowFilt(1)); ...
                -obj.ky*(vDesL(2)-vNowFilt(2));0];
            
            % cross leg compensation
            CrossComY=[0.03,-0.03,-0.03,0.03]*vDesL(1);
            CrossCom=[zeros(1,4);CrossComY;zeros(1,4)];
            
            desAllL=v*obj.T/4*[1,1,1,1]+obj.pLnorm+CrossCom;
            
            % terrain height compensation
            pCoM=[X_FB(1);X_FB(2);X_FB(3)];
            surP_L=Rrpy'*(surP-pCoM);
            surVN_L=Rrpy'*surVN;
            desAllL_z=zeros(4,1);
            for i=1:1:4
                desAllL_z(i)=(surP_L-obj.LegCorOri(:,i))'*surVN_L;
                delta=-obj.r0(3)-desAllL_z(i);
                desAllL_z(i)=desAllL_z(i)+delta*0.8;
            end
            
            %%% next step foot-end position planning in the leg coordinate

            pL_sw=zeros(3,4);
            for i=1:1:4
                if LegState(i)<0.5
                    s=s*1.2; % to accelerate the swing trajectory tracing
                    if s>=1
                        s=1;
                    end
                    sta=obj.pL_LS(:,i);
                    des=desAllL(:,i);
                    %des(3)=sta(3);
                    %des(3)=-0.19;
                    des(3)=desAllL_z(i);
                    ax=[sta(1),sta(1),des(1),des(1)];
                    ay=[sta(2),sta(2),des(2),des(2)];
                    az1=[sta(3),sta(3),(sta(3)+des(3))/2+obj.StepH,(sta(3)+des(3))/2+obj.StepH];
                    az2=[(sta(3)+des(3))/2+obj.StepH,(sta(3)+des(3))/2+obj.StepH,des(3),des(3)];
                    px=BezierCurve(ax,s);
                    py=BezierCurve(ay,s);
                    pz=0;
                    if s<=0.5
                        pz=BezierCurve(az1,2*s);
                    else
                        pz=BezierCurve(az2,2*(s-0.5));
                    end
                    pL_sw(:,i)=[px;py;pz];
%                     if touchInd(i)>0.5
%                         PendAll(:,i)=obj.PendAllOld(:,i);
%                         PendAll(3,i)=zSur(i);
%                     end
                end
            end
            
            %%% generate the foot-end position cmd via interploating the MPC's predicted states
            % As foot-end positions are planned as above, this part only
            % interploate the pCoM to derive the foot-end positions in leg
            % coordinates
            theta=X_FB(4:6);
            Pc=reshape(X_FB(1:3),3,1);
            R=Rz(theta(3))*Ry(theta(2))*Rx(theta(1));
            interpolNUM=5;
            
            if sum(abs(obj.MPC_Count_Old-MPC_Count))<0.1
                if obj.interpol_Count>=interpolNUM
                    obj.interpol_Count=interpolNUM;
                end
                PendAllLocal_st_tmp=(interpolNUM-obj.interpol_Count)/interpolNUM*obj.pArray_L_Adm_Old+obj.interpol_Count/interpolNUM*obj.pArray_L_Adm_Now;
                obj.interpol_Count=obj.interpol_Count+1;
            else
                obj.pArray_L_Adm_Old=obj.pArray_L_Adm_Now;
                for i=1:1:4
                    if obj.MPC_legStateOld(i)<0.5 && LegStateMPC(i)>0.5
                        obj.pArray_L_Adm_Old((3*i-2):3*i)=obj.PendAllLocalOld(:,i);
                    end
                end
                obj.pArray_L_Adm_Now=pArray_L_Adm/1000;
                PendAllLocal_st_tmp=obj.pArray_L_Adm_Old;
                obj.interpol_Count=1;
            end
            
            PendAllLocal=zeros(3,4);
            PendAllLocal_st=reshape(PendAllLocal_st_tmp,3,4);
            PendAllLocal_sw=pL_sw;
            for i=1:1:4
                if LegState(i)<0.5
                    PendAllLocal(:,i)=PendAllLocal_sw(:,i);
                else
                    if LegStateMPC(i)>0.5
                        PendAllLocal(:,i)=PendAllLocal_st(:,i);
                    else
                        PendAllLocal(:,i)=obj.PendAllLocalOld(:,i);
                    end
                end
            end
            
            % update of pL_LS
            for i=1:1:4
                if LegState(i)>0.5
                    obj.pL_LS(:,i)=PendAllLocal(:,i);
                end
            end
                        
            %%% data update
            obj.phiOld=phi;
            obj.LegStateOld=LegState;
            obj.MPC_legStateOld=LegStateMPC;
            obj.PendAllLocalOld=PendAllLocal;
            obj.MPC_Count_Old=MPC_Count;

        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end
        
        function [s1,s2] = getOutputSizeImpl(~)
            s1 = [4,1];
            s2=[3,4];
        end
        
        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
        end
        
        function [cpl1,cpl2] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end

function R=RodFormula(omega)
% Rodrigues' formula
theta=norm(omega);
if theta>1e-11
    a=omega/theta;
else
    a=omega;
end
aCap=[0,-a(3),a(2);
    a(3),0,-a(1);
    -a(2),a(1),0];
R=eye(3)+aCap*sin(theta)+aCap*aCap*(1-cos(theta));
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

function B=BezierCurve(a,s)
% a is the control point vector, s is the time vector
B=0;
M=length(a);
for ii=1:1:M
    B=B+factorial(M-1)/factorial(ii-1)/factorial(M-ii)*s^(ii-1)*(1-s)^(M-ii)*a(ii);
end
end

function [Ad,Bd] = DS_gen(Ts,m,in3,in4,in5,in6,in7,in8)
%DS_GEN
%    [AD,BD] = DS_GEN(TS,M,theta,Iinv,r1,r2,r3,r4)

%    This function was generated by the Symbolic Math Toolbox version 8.6.

Iinv1_1 = in4(1);
Iinv1_2 = in4(4);
Iinv1_3 = in4(7);
Iinv2_1 = in4(2);
Iinv2_2 = in4(5);
Iinv2_3 = in4(8);
Iinv3_1 = in4(3);
Iinv3_2 = in4(6);
Iinv3_3 = in4(9);
r11 = in5(1,:);
r12 = in5(2,:);
r13 = in5(3,:);
r21 = in6(1,:);
r22 = in6(2,:);
r23 = in6(3,:);
r31 = in7(1,:);
r32 = in7(2,:);
r33 = in7(3,:);
r41 = in8(1,:);
r42 = in8(2,:);
r43 = in8(3,:);
theta2 = in3(2,:);
theta3 = in3(3,:);
t2 = cos(theta2);
t3 = cos(theta3);
t4 = sin(theta2);
t5 = sin(theta3);
t6 = Iinv1_1.*r12;
t7 = Iinv1_2.*r11;
t8 = Iinv1_1.*r13;
t9 = Iinv1_3.*r11;
t10 = Iinv1_2.*r13;
t11 = Iinv1_3.*r12;
t12 = Iinv1_1.*r22;
t13 = Iinv1_2.*r21;
t14 = Iinv2_1.*r12;
t15 = Iinv2_2.*r11;
t16 = Iinv1_1.*r23;
t17 = Iinv1_3.*r21;
t18 = Iinv2_1.*r13;
t19 = Iinv2_3.*r11;
t20 = Iinv1_2.*r23;
t21 = Iinv1_3.*r22;
t22 = Iinv2_2.*r13;
t23 = Iinv2_3.*r12;
t24 = Iinv1_1.*r32;
t25 = Iinv1_2.*r31;
t26 = Iinv2_1.*r22;
t27 = Iinv2_2.*r21;
t28 = Iinv3_1.*r12;
t29 = Iinv3_2.*r11;
t30 = Iinv1_1.*r33;
t31 = Iinv1_3.*r31;
t32 = Iinv2_1.*r23;
t33 = Iinv2_3.*r21;
t34 = Iinv3_1.*r13;
t35 = Iinv3_3.*r11;
t36 = Iinv1_2.*r33;
t37 = Iinv1_3.*r32;
t38 = Iinv2_2.*r23;
t39 = Iinv2_3.*r22;
t40 = Iinv3_2.*r13;
t41 = Iinv3_3.*r12;
t42 = Iinv1_1.*r42;
t43 = Iinv1_2.*r41;
t44 = Iinv2_1.*r32;
t45 = Iinv2_2.*r31;
t46 = Iinv3_1.*r22;
t47 = Iinv3_2.*r21;
t48 = Iinv1_1.*r43;
t49 = Iinv1_3.*r41;
t50 = Iinv2_1.*r33;
t51 = Iinv2_3.*r31;
t52 = Iinv3_1.*r23;
t53 = Iinv3_3.*r21;
t54 = Iinv1_2.*r43;
t55 = Iinv1_3.*r42;
t56 = Iinv2_2.*r33;
t57 = Iinv2_3.*r32;
t58 = Iinv3_2.*r23;
t59 = Iinv3_3.*r22;
t60 = Iinv2_1.*r42;
t61 = Iinv2_2.*r41;
t62 = Iinv3_1.*r32;
t63 = Iinv3_2.*r31;
t64 = Iinv2_1.*r43;
t65 = Iinv2_3.*r41;
t66 = Iinv3_1.*r33;
t67 = Iinv3_3.*r31;
t68 = Iinv2_2.*r43;
t69 = Iinv2_3.*r42;
t70 = Iinv3_2.*r33;
t71 = Iinv3_3.*r32;
t72 = Iinv3_1.*r42;
t73 = Iinv3_2.*r41;
t74 = Iinv3_1.*r43;
t75 = Iinv3_3.*r41;
t76 = Iinv3_2.*r43;
t77 = Iinv3_3.*r42;
t78 = Ts.^2;
t81 = 1.0./m;
t79 = t3.^2;
t80 = t5.^2;
t82 = -t7;
t83 = -t9;
t84 = -t11;
t85 = -t13;
t86 = -t15;
t87 = -t17;
t88 = -t19;
t89 = -t21;
t90 = -t23;
t91 = -t25;
t92 = -t27;
t93 = -t29;
t94 = -t31;
t95 = -t33;
t96 = -t35;
t97 = -t37;
t98 = -t39;
t99 = -t41;
t100 = -t43;
t101 = -t45;
t102 = -t47;
t103 = -t49;
t104 = -t51;
t105 = -t53;
t106 = -t55;
t107 = -t57;
t108 = -t59;
t109 = -t61;
t110 = -t63;
t111 = -t65;
t112 = -t67;
t113 = -t69;
t114 = -t71;
t115 = -t73;
t116 = -t75;
t117 = -t77;
t118 = 1.0./t2;
t119 = Ts.*t81;
t159 = (t78.*t81)./2.0;
t120 = t2.*t79;
t121 = t2.*t80;
t122 = t6+t82;
t123 = t8+t83;
t124 = t10+t84;
t125 = t12+t85;
t126 = t14+t86;
t127 = t16+t87;
t128 = t18+t88;
t129 = t20+t89;
t130 = t22+t90;
t131 = t24+t91;
t132 = t26+t92;
t133 = t28+t93;
t134 = t30+t94;
t135 = t32+t95;
t136 = t34+t96;
t137 = t36+t97;
t138 = t38+t98;
t139 = t40+t99;
t140 = t42+t100;
t141 = t44+t101;
t142 = t46+t102;
t143 = t48+t103;
t144 = t50+t104;
t145 = t52+t105;
t146 = t54+t106;
t147 = t56+t107;
t148 = t58+t108;
t149 = t60+t109;
t150 = t62+t110;
t151 = t64+t111;
t152 = t66+t112;
t153 = t68+t113;
t154 = t70+t114;
t155 = t72+t115;
t156 = t74+t116;
t157 = t76+t117;
t158 = t79+t80;
t160 = 1.0./t158;
t161 = t120+t121;
t162 = 1.0./t161;
Ad = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts.*t3.*t162,-Ts.*t5.*t160,Ts.*t3.*t4.*t162,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts.*t5.*t162,Ts.*t3.*t160,Ts.*t4.*t5.*t162,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t78.*(-1.0./2.0),0.0,0.0,0.0,0.0,0.0,-Ts,0.0,0.0,0.0,1.0],[13,13]);
if nargout > 1
    Bd = reshape([t159,0.0,0.0,(t3.*t78.*t118.*t124)./2.0+(t5.*t78.*t118.*t130)./2.0,-t5.*t78.*(t10./2.0-t11./2.0)+t3.*t78.*(t22./2.0-t23./2.0),(t78.*t139)./2.0+(t78.*t118.*(t3.*t4.*t10+t4.*t5.*t22+t3.*t4.*t84+t4.*t5.*t90))./2.0,t119,0.0,0.0,Ts.*t124,Ts.*t130,Ts.*t139,0.0,0.0,t159,0.0,t3.*t78.*t118.*t123.*(-1.0./2.0)-(t5.*t78.*t118.*t128)./2.0,t5.*t78.*(t8./2.0-t9./2.0)-t3.*t78.*(t18./2.0-t19./2.0),t78.*t136.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t8+t4.*t5.*t18+t3.*t4.*t83+t4.*t5.*t88))./2.0,0.0,t119,0.0,-Ts.*t123,-Ts.*t128,-Ts.*t136,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t122)./2.0+(t5.*t78.*t118.*t126)./2.0,-t5.*t78.*(t6./2.0-t7./2.0)+t3.*t78.*(t14./2.0-t15./2.0),(t78.*t133)./2.0+(t78.*t118.*(t3.*t4.*t6+t4.*t5.*t14+t3.*t4.*t82+t4.*t5.*t86))./2.0,0.0,0.0,t119,Ts.*t122,Ts.*t126,Ts.*t133,0.0,t159,0.0,0.0,(t3.*t78.*t118.*t129)./2.0+(t5.*t78.*t118.*t138)./2.0,-t5.*t78.*(t20./2.0-t21./2.0)+t3.*t78.*(t38./2.0-t39./2.0),(t78.*t148)./2.0+(t78.*t118.*(t3.*t4.*t20+t4.*t5.*t38+t3.*t4.*t89+t4.*t5.*t98))./2.0,t119,0.0,0.0,Ts.*t129,Ts.*t138,Ts.*t148,0.0,0.0,t159,0.0,t3.*t78.*t118.*t127.*(-1.0./2.0)-(t5.*t78.*t118.*t135)./2.0,t5.*t78.*(t16./2.0-t17./2.0)-t3.*t78.*(t32./2.0-t33./2.0),t78.*t145.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t16+t4.*t5.*t32+t3.*t4.*t87+t4.*t5.*t95))./2.0,0.0,t119,0.0,-Ts.*t127,-Ts.*t135,-Ts.*t145,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t125)./2.0+(t5.*t78.*t118.*t132)./2.0,-t5.*t78.*(t12./2.0-t13./2.0)+t3.*t78.*(t26./2.0-t27./2.0),(t78.*t142)./2.0+(t78.*t118.*(t3.*t4.*t12+t4.*t5.*t26+t3.*t4.*t85+t4.*t5.*t92))./2.0,0.0,0.0,t119,Ts.*t125,Ts.*t132,Ts.*t142,0.0,t159,0.0,0.0,(t3.*t78.*t118.*t137)./2.0+(t5.*t78.*t118.*t147)./2.0,-t5.*t78.*(t36./2.0-t37./2.0)+t3.*t78.*(t56./2.0-t57./2.0),(t78.*t154)./2.0+(t78.*t118.*(t3.*t4.*t36+t4.*t5.*t56+t3.*t4.*t97+t4.*t5.*t107))./2.0,t119,0.0,0.0,Ts.*t137,Ts.*t147,Ts.*t154,0.0,0.0,t159,0.0,t3.*t78.*t118.*t134.*(-1.0./2.0)-(t5.*t78.*t118.*t144)./2.0,t5.*t78.*(t30./2.0-t31./2.0)-t3.*t78.*(t50./2.0-t51./2.0),t78.*t152.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t30+t4.*t5.*t50+t3.*t4.*t94+t4.*t5.*t104))./2.0,0.0,t119,0.0,-Ts.*t134,-Ts.*t144,-Ts.*t152,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t131)./2.0+(t5.*t78.*t118.*t141)./2.0,-t5.*t78.*(t24./2.0-t25./2.0)+t3.*t78.*(t44./2.0-t45./2.0),(t78.*t150)./2.0+(t78.*t118.*(t3.*t4.*t24+t4.*t5.*t44+t3.*t4.*t91+t4.*t5.*t101))./2.0,0.0,0.0,t119,Ts.*t131,Ts.*t141,Ts.*t150,0.0,t159,0.0,0.0,(t3.*t78.*t118.*t146)./2.0+(t5.*t78.*t118.*t153)./2.0,-t5.*t78.*(t54./2.0-t55./2.0)+t3.*t78.*(t68./2.0-t69./2.0),(t78.*t157)./2.0+(t78.*t118.*(t3.*t4.*t54+t4.*t5.*t68+t3.*t4.*t106+t4.*t5.*t113))./2.0,t119,0.0,0.0,Ts.*t146,Ts.*t153,Ts.*t157,0.0,0.0,t159,0.0,t3.*t78.*t118.*t143.*(-1.0./2.0)-(t5.*t78.*t118.*t151)./2.0,t5.*t78.*(t48./2.0-t49./2.0)-t3.*t78.*(t64./2.0-t65./2.0),t78.*t156.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t48+t4.*t5.*t64+t3.*t4.*t103+t4.*t5.*t111))./2.0,0.0,t119,0.0,-Ts.*t143,-Ts.*t151,-Ts.*t156,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t140)./2.0+(t5.*t78.*t118.*t149)./2.0,-t5.*t78.*(t42./2.0-t43./2.0)+t3.*t78.*(t60./2.0-t61./2.0),(t78.*t155)./2.0+(t78.*t118.*(t3.*t4.*t42+t4.*t5.*t60+t3.*t4.*t100+t4.*t5.*t109))./2.0,0.0,0.0,t119,Ts.*t140,Ts.*t149,Ts.*t155,0.0],[13,12]);
end
end







