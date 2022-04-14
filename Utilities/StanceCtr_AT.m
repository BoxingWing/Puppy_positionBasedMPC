classdef StanceCtr_AT< matlab.System
    % stance leg control module
    properties
        tSW=0.3;
        tSample=0.005;
        m=3;
        Inorm=eye(3);
        r0=0.19;
        lateral_width=0.097;
        sagetial_width=0.2108;
        roll_Off=0.037;
        kp=zeros(1,6);
        kd=zeros(1,6);
        ki=zeros(1,6);
    end

    properties(Access=private)
        isLFnext=true;
        isRFnext=false;
        swCount_LF=0;
        swCount_RF=0;
        swN=1;
        yawOld=zeros(1,5);
        LegStateOld=[1;1;1;1];
        vCoM_sw=[1;1;1]; % CoM velocity at the beginning of the last swing phase
        ki_err_Old=zeros(6,1);
    end

    methods(Access = protected)

        function setupImpl(obj)
            obj.swN=floor(obj.tSW/obj.tSample);
            obj.isLFnext=true;
            obj.isRFnext=false;
            obj.swCount_LF=0;
            obj.swCount_RF=0;
            obj.swN=1;
            obj.LegStateOld=[1;1;1;1];
            obj.vCoM_sw=[1;1;1];
        end

        function [pL_st,fL,pL_Delta] = stepImpl(obj,pL_old,pB_old,pW_m,xFB,xRef,LegState,Disable)
            % pL_old: [3,4] baseline pL in the last step
            % pB_old: [3,4] baseline pB in the last step
            % pW_m : measured pW
            % fL: foot end contact forces in the leg coordinate
            % xDes: [vx,vy,wz], in the local body frame

            % norminal foot end positions
            pL_old=reshape(pL_old,3,4);
            pB_old=reshape(pB_old,3,4);

            if Disable>0.5
                pL_st=reshape([0;obj.roll_Off;-obj.r0;0;-obj.roll_Off;-obj.r0; ...
                    0;obj.roll_Off;-obj.r0;0;-obj.roll_Off;-obj.r0;],3,4);
            else
                
%                 if xRef(7)<10^-3 && xRef(8)<10^-3 % to clear the foot position when Vdef=0
%                     
%                 else
%                     linAct=[xRef(7);xRef(8);0]*[1,1,1,1]*obj.tSample;
%                 end
                linAct=[-xRef(7);-xRef(8);0]*[1,1,1,1]*obj.tSample;
                pL_st=pL_old+linAct+...
                    [cross(pB_old(:,1),[0;0;-xRef(12)]),cross(pB_old(:,2),[0;0;-xRef(12)]), ...
                    cross(pB_old(:,3),[0;0;-xRef(12)]),cross(pB_old(:,4),[0;0;-xRef(12)])]*obj.tSample;
            end

            % contact forces control
            pC=xFB(1:3);
            sita=xFB(4:6);

            obj.yawOld(1:end-1)=obj.yawOld(2:end);
            obj.yawOld(end)=xFB(6);
            yawFilt=sum(obj.yawOld)/length(obj.yawOld);
            Rz=[cos(yawFilt),-sin(yawFilt),0;sin(yawFilt),cos(yawFilt),0;0,0,1];
            %Rz=eye(3);

            T=[cos(sita(2))*cos(sita(3)),-sin(sita(3)),0;
                cos(sita(2))*sin(sita(3)),cos(sita(3)),0;
                -sin(sita(2)),0,1];
            %dsita=T\[xFB(10);xFB(11);xFB(12)];
            errSita=([xRef(4);xRef(5);xRef(6)]-[xFB(4);xFB(5);xFB(6)]);
            errdSita=T\([xRef(10);xRef(11);xRef(12)]-[xFB(10);xFB(11);xFB(12)]);
            errpCoM=Rz'*([xRef(1);xRef(2);xRef(3)]-[xFB(1);xFB(2);xFB(3)]);
            errvCoM=Rz'*([xRef(7);xRef(8);xRef(9)]-[xFB(7);xFB(8);xFB(9)]);

            Kp=diag(obj.kp);
            Kd=diag(obj.kd);
            Ki=diag(obj.ki);
            %             obj.ki_err_Old=obj.ki_err_Old+Ki*reshape(xRef(1:6)-xFB(1:6),6,1);
            %             err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            obj.ki_err_Old=obj.ki_err_Old+Ki*[errpCoM;errSita];
            %err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            if LegState(1)+obj.LegStateOld(1)>0.8 && LegState(1)+obj.LegStateOld(1)<1.2
                obj.ki_err_Old=obj.ki_err_Old*0;
            end
            if LegState(2)+obj.LegStateOld(2)>0.8 && LegState(2)+obj.LegStateOld(2)<1.2
                obj.ki_err_Old=obj.ki_err_Old*0;
            end
            err=Kp*[errpCoM;errSita]+Kd*[errvCoM;errdSita]+obj.ki_err_Old;
            
            pFloat=[Rz'*(pW_m(:,1)-pC),Rz'*(pW_m(:,2)-pC), ...
                Rz'*(pW_m(:,3)-pC),Rz'*(pW_m(:,4)-pC)]; % foot-end positions in the float-base coordinate
            
            % jaocibian matrix for both position and orientation
            M=[LegState(1).*eye(3),LegState(2).*eye(3),LegState(3).*eye(3),LegState(4).*eye(3);...
                LegState(1).*crossCap(pFloat(:,1)),LegState(2).*crossCap(pFloat(:,2)),LegState(3).*crossCap(pFloat(:,3)),LegState(4).*crossCap(pFloat(:,4));];
            % jacobian matrix for only orientation
            M=[LegState(1).*crossCap(pFloat(:,1)),LegState(2).*crossCap(pFloat(:,2)),LegState(3).*crossCap(pFloat(:,3)),LegState(4).*crossCap(pFloat(:,4))];
            err=err(4:6);

            Minv=pinv(M,10^-7);
            if Disable>0.5
                U=zeros(12,1);
                obj.ki_err_Old=obj.ki_err_Old*0;
                obj.yawOld=obj.yawOld*0;
            else
                U=Minv*err;
            end
            fL2W=zeros(12,1);
            for i=1:1:4
                fL2W(3*i-2:3*i)=Rz*U(3*i-2:3*i);
            end
            
            RzV2=[cos(sita(3)),-sin(sita(3)),0;sin(sita(3)),cos(sita(3)),0;0,0,1];
            Iinv=RzV2*diag([1/obj.Inorm(1,1),1/obj.Inorm(2,2),1/obj.Inorm(3,3)])*RzV2';
            [Ad,Bd] = DS_gen(obj.tSample,obj.m,sita,Iinv,pFloat(:,1),pFloat(:,2),pFloat(:,3),pFloat(:,4));
            %X1=Ad*[xFB(1:12);9.8]+Bd*fL2W;
            X1=Ad*[xFB(1:12);0]+Bd*fL2W;

            pC1=X1(1:3);
            sita1=X1(4:6);
%             T1=[cos(sita1(2))*cos(sita1(3)),-sin(sita1(3)),0;
%                 cos(sita1(2))*sin(sita1(3)),cos(sita1(3)),0;
%                 -sin(sita1(2)),0,1];
%             dsita=T1\[X1(10);X1(11);X1(12)];
            %sita1=sita1+dsita*obj.tSample;
            %pC1=pC1+X1(7:9)*obj.tSample;
            Rx1=[1,0,0;0,cos(sita1(1)),-sin(sita1(1));0,sin(sita1(1)),cos(sita1(1))];
            Ry1=[cos(sita1(2)),0,sin(sita1(2));0,1,0;-sin(sita1(2)),0,cos(sita1(2))];
            Rz1=[cos(sita1(3)),-sin(sita1(3)),0;sin(sita1(3)),cos(sita1(3)),0;0,0,1];
            R1=Rz1*Ry1*Rx1;
            pB_New=[R1'*(pW_m(:,1)-pC1),R1'*(pW_m(:,2)-pC1), ...
                R1'*(pW_m(:,3)-pC1),R1'*(pW_m(:,4)-pC1)];
            pL_New = pB2L(pB_New);
            
            sita0=xFB(4:6);
            pC0=xFB(1:3);
            Rx0=[1,0,0;0,cos(sita0(1)),-sin(sita0(1));0,sin(sita0(1)),cos(sita0(1))];
            Ry0=[cos(sita0(2)),0,sin(sita0(2));0,1,0;-sin(sita0(2)),0,cos(sita0(2))];
            Rz0=[cos(sita0(3)),-sin(sita0(3)),0;sin(sita0(3)),cos(sita0(3)),0;0,0,1];
            R0=Rz0*Ry0*Rx0;
            pB0=[R0'*(pW_m(:,1)-pC0),R0'*(pW_m(:,2)-pC0), ...
                R0'*(pW_m(:,3)-pC0),R0'*(pW_m(:,4)-pC0)];
            pL0 = pB2L(pB0);

            pL_Delta=pL_New-reshape(pL0,12,1);
            pL_Delta=reshape(pL_Delta,3,4);
            
            for i=1:1:4
                if LegState(i)<0.5
                    pL_Delta(:,i)=zeros(3,1);
                end
                if pL_Delta(3,i)<-8
                    pL_Delta(3,i)=-8;
                end
            end
            
            fL=U;

            obj.LegStateOld=LegState;

            pL_st=reshape(pL_st,12,1);
            pL_Delta=reshape(pL_Delta,12,1);

            fL=reshape(fL,12,1);
        end
        
        function [d1,d2,d3] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
            d3 = 'double';
        end

        function [s1,s2,s3] = getOutputSizeImpl(~)
            s1 = [12,1];
            s2 = [12,1];
            s3 = [12,1];
        end

        function [f1,f2,f3] = isOutputFixedSizeImpl(~)
            f1 = true;
            f2 = true;
            f3 = true;
        end

        function [cpl1,cpl2,cpl3] = isOutputComplexImpl(~)
            cpl1 = false;
            cpl2 = false;
            cpl3 = false;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end

function vcap=crossCap(v)
vcap=[0,-v(3),v(2);
        v(3),0,-v(1);
       -v(2),v(1),0];
end

function [Ad,Bd] = DS_gen(Ts,m,in3,in4,in5,in6,in7,in8)
%DS_GEN
%    [AD,BD] = DS_GEN(TS,M,IN3,IN4,IN5,IN6,IN7,IN8)

%    This function was generated by the Symbolic Math Toolbox version 8.7.
%    29-Sep-2021 16:12:07

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
theta3 = in3(3,:);
t2 = cos(theta3);
t3 = sin(theta3);
t4 = Iinv3_1.*r12;
t5 = Iinv3_2.*r11;
t6 = Iinv3_1.*r13;
t7 = Iinv3_3.*r11;
t8 = Iinv3_2.*r13;
t9 = Iinv3_3.*r12;
t10 = Iinv3_1.*r22;
t11 = Iinv3_2.*r21;
t12 = Iinv3_1.*r23;
t13 = Iinv3_3.*r21;
t14 = Iinv3_2.*r23;
t15 = Iinv3_3.*r22;
t16 = Iinv3_1.*r32;
t17 = Iinv3_2.*r31;
t18 = Iinv3_1.*r33;
t19 = Iinv3_3.*r31;
t20 = Iinv3_2.*r33;
t21 = Iinv3_3.*r32;
t22 = Iinv3_1.*r42;
t23 = Iinv3_2.*r41;
t24 = Iinv3_1.*r43;
t25 = Iinv3_3.*r41;
t26 = Iinv3_2.*r43;
t27 = Iinv3_3.*r42;
t28 = Ts.^2;
t31 = 1.0./m;
t44 = (Iinv1_1.*r12)./2.0;
t45 = (Iinv1_2.*r11)./2.0;
t46 = (Iinv1_1.*r13)./2.0;
t47 = (Iinv1_3.*r11)./2.0;
t48 = (Iinv1_2.*r13)./2.0;
t49 = (Iinv1_3.*r12)./2.0;
t50 = (Iinv1_1.*r22)./2.0;
t51 = (Iinv1_2.*r21)./2.0;
t52 = (Iinv2_1.*r12)./2.0;
t53 = (Iinv2_2.*r11)./2.0;
t54 = (Iinv1_1.*r23)./2.0;
t55 = (Iinv1_3.*r21)./2.0;
t56 = (Iinv2_1.*r13)./2.0;
t57 = (Iinv2_3.*r11)./2.0;
t58 = (Iinv1_2.*r23)./2.0;
t59 = (Iinv1_3.*r22)./2.0;
t60 = (Iinv2_2.*r13)./2.0;
t61 = (Iinv2_3.*r12)./2.0;
t62 = (Iinv1_1.*r32)./2.0;
t63 = (Iinv1_2.*r31)./2.0;
t64 = (Iinv2_1.*r22)./2.0;
t65 = (Iinv2_2.*r21)./2.0;
t66 = (Iinv1_1.*r33)./2.0;
t67 = (Iinv1_3.*r31)./2.0;
t68 = (Iinv2_1.*r23)./2.0;
t69 = (Iinv2_3.*r21)./2.0;
t70 = (Iinv1_2.*r33)./2.0;
t71 = (Iinv1_3.*r32)./2.0;
t72 = (Iinv2_2.*r23)./2.0;
t73 = (Iinv2_3.*r22)./2.0;
t74 = (Iinv1_1.*r42)./2.0;
t75 = (Iinv1_2.*r41)./2.0;
t76 = (Iinv2_1.*r32)./2.0;
t77 = (Iinv2_2.*r31)./2.0;
t78 = (Iinv1_1.*r43)./2.0;
t79 = (Iinv1_3.*r41)./2.0;
t80 = (Iinv2_1.*r33)./2.0;
t81 = (Iinv2_3.*r31)./2.0;
t82 = (Iinv1_2.*r43)./2.0;
t83 = (Iinv1_3.*r42)./2.0;
t84 = (Iinv2_2.*r33)./2.0;
t85 = (Iinv2_3.*r32)./2.0;
t86 = (Iinv2_1.*r42)./2.0;
t87 = (Iinv2_2.*r41)./2.0;
t88 = (Iinv2_1.*r43)./2.0;
t89 = (Iinv2_3.*r41)./2.0;
t90 = (Iinv2_2.*r43)./2.0;
t91 = (Iinv2_3.*r42)./2.0;
t29 = t2.^2;
t30 = t3.^2;
t32 = -t5;
t33 = -t7;
t34 = -t9;
t35 = -t11;
t36 = -t13;
t37 = -t15;
t38 = -t17;
t39 = -t19;
t40 = -t21;
t41 = -t23;
t42 = -t25;
t43 = -t27;
t92 = Ts.*t31;
t93 = -t45;
t94 = -t47;
t95 = -t49;
t96 = -t51;
t97 = -t53;
t98 = -t55;
t99 = -t57;
t100 = -t59;
t101 = -t61;
t102 = -t63;
t103 = -t65;
t104 = -t67;
t105 = -t69;
t106 = -t71;
t107 = -t73;
t108 = -t75;
t109 = -t77;
t110 = -t79;
t111 = -t81;
t112 = -t83;
t113 = -t85;
t114 = -t87;
t115 = -t89;
t116 = -t91;
t130 = (t28.*t31)./2.0;
t117 = t4+t32;
t118 = t6+t33;
t119 = t8+t34;
t120 = t10+t35;
t121 = t12+t36;
t122 = t14+t37;
t123 = t16+t38;
t124 = t18+t39;
t125 = t20+t40;
t126 = t22+t41;
t127 = t24+t42;
t128 = t26+t43;
t129 = t29+t30;
t132 = t44+t93;
t133 = t46+t94;
t134 = t48+t95;
t135 = t50+t96;
t136 = t52+t97;
t137 = t54+t98;
t138 = t56+t99;
t139 = t58+t100;
t140 = t60+t101;
t141 = t62+t102;
t142 = t64+t103;
t143 = t66+t104;
t144 = t68+t105;
t145 = t70+t106;
t146 = t72+t107;
t147 = t74+t108;
t148 = t76+t109;
t149 = t78+t110;
t150 = t80+t111;
t151 = t82+t112;
t152 = t84+t113;
t153 = t86+t114;
t154 = t88+t115;
t155 = t90+t116;
t131 = 1.0./t129;
t156 = Ts.*t2.*t131;
t157 = Ts.*t3.*t131;
Ad = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t156,-t157,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,t157,t156,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t28.*(-1.0./2.0),0.0,0.0,0.0,0.0,0.0,-Ts,0.0,0.0,0.0,1.0],[13,13]);
if nargout > 1
    mt1 = [t130,0.0,0.0,t2.*t28.*t134+t3.*t28.*t140,-t3.*t28.*t134+t2.*t28.*t140,(t28.*t119)./2.0,t92,0.0,0.0,Ts.*(Iinv1_2.*r13-Iinv1_3.*r12),Ts.*(Iinv2_2.*r13-Iinv2_3.*r12),Ts.*t119,0.0,0.0,t130,0.0,-t2.*t28.*t133-t3.*t28.*t138,t3.*t28.*t133-t2.*t28.*t138,t28.*t118.*(-1.0./2.0),0.0,t92,0.0,-Ts.*(Iinv1_1.*r13-Iinv1_3.*r11),-Ts.*(Iinv2_1.*r13-Iinv2_3.*r11),-Ts.*t118,0.0,0.0,0.0,t130,t2.*t28.*t132+t3.*t28.*t136,-t3.*t28.*t132+t2.*t28.*t136,(t28.*t117)./2.0,0.0,0.0,t92,Ts.*(Iinv1_1.*r12-Iinv1_2.*r11),Ts.*(Iinv2_1.*r12-Iinv2_2.*r11),Ts.*t117,0.0,t130,0.0,0.0,t2.*t28.*t139+t3.*t28.*t146];
    mt2 = [-t3.*t28.*t139+t2.*t28.*t146,(t28.*t122)./2.0,t92,0.0,0.0,Ts.*(Iinv1_2.*r23-Iinv1_3.*r22),Ts.*(Iinv2_2.*r23-Iinv2_3.*r22),Ts.*t122,0.0,0.0,t130,0.0,-t2.*t28.*t137-t3.*t28.*t144,t3.*t28.*t137-t2.*t28.*t144,t28.*t121.*(-1.0./2.0),0.0,t92,0.0,-Ts.*(Iinv1_1.*r23-Iinv1_3.*r21),-Ts.*(Iinv2_1.*r23-Iinv2_3.*r21),-Ts.*t121,0.0,0.0,0.0,t130,t2.*t28.*t135+t3.*t28.*t142,-t3.*t28.*t135+t2.*t28.*t142,(t28.*t120)./2.0,0.0,0.0,t92,Ts.*(Iinv1_1.*r22-Iinv1_2.*r21),Ts.*(Iinv2_1.*r22-Iinv2_2.*r21),Ts.*t120,0.0,t130,0.0,0.0,t2.*t28.*t145+t3.*t28.*t152,-t3.*t28.*t145+t2.*t28.*t152];
    mt3 = [(t28.*t125)./2.0,t92,0.0,0.0,Ts.*(Iinv1_2.*r33-Iinv1_3.*r32),Ts.*(Iinv2_2.*r33-Iinv2_3.*r32),Ts.*t125,0.0,0.0,t130,0.0,-t2.*t28.*t143-t3.*t28.*t150,t3.*t28.*t143-t2.*t28.*t150,t28.*t124.*(-1.0./2.0),0.0,t92,0.0,-Ts.*(Iinv1_1.*r33-Iinv1_3.*r31),-Ts.*(Iinv2_1.*r33-Iinv2_3.*r31),-Ts.*t124,0.0,0.0,0.0,t130,t2.*t28.*t141+t3.*t28.*t148,-t3.*t28.*t141+t2.*t28.*t148,(t28.*t123)./2.0,0.0,0.0,t92,Ts.*(Iinv1_1.*r32-Iinv1_2.*r31),Ts.*(Iinv2_1.*r32-Iinv2_2.*r31),Ts.*t123,0.0,t130,0.0,0.0,t2.*t28.*t151+t3.*t28.*t155,-t3.*t28.*t151+t2.*t28.*t155,(t28.*t128)./2.0,t92,0.0,0.0];
    mt4 = [Ts.*(Iinv1_2.*r43-Iinv1_3.*r42),Ts.*(Iinv2_2.*r43-Iinv2_3.*r42),Ts.*t128,0.0,0.0,t130,0.0,-t2.*t28.*t149-t3.*t28.*t154,t3.*t28.*t149-t2.*t28.*t154,t28.*t127.*(-1.0./2.0),0.0,t92,0.0,-Ts.*(Iinv1_1.*r43-Iinv1_3.*r41),-Ts.*(Iinv2_1.*r43-Iinv2_3.*r41),-Ts.*t127,0.0,0.0,0.0,t130,t2.*t28.*t147+t3.*t28.*t153,-t3.*t28.*t147+t2.*t28.*t153,(t28.*t126)./2.0,0.0,0.0,t92,Ts.*(Iinv1_1.*r42-Iinv1_2.*r41),Ts.*(Iinv2_1.*r42-Iinv2_2.*r41),Ts.*t126,0.0];
    Bd = reshape([mt1,mt2,mt3,mt4],13,12);
end
end

function pArray_L = pB2L(pArray_B)
% transform foot-end pos from leg coordinate to body coordinate
    yWidth=97/1000; % distantce between left and right roll axis, i.e. distance between left and right leg coordinates
    xWidth=210.8/1000; % distance between fore and hind pitch axis, i.e. distance between fore and hind leg coordinates
    piOFF=zeros(12,1);
    piOFF(1:3)=[xWidth;yWidth;0]/2;
    piOFF(4:6)=[xWidth;-yWidth;0]/2;
    piOFF(7:9)=[-xWidth;yWidth;0]/2;
    piOFF(10:12)=[-xWidth;-yWidth;0]/2;
    pArray_L=reshape(pArray_B,12,1)-piOFF;
end