classdef centroidForce_decouple < matlab.System
    % force and moment compensation via PD laws
    properties
        kp_a=zeros(1,6);
        kd_a=zeros(1,6);
        ki_a=zeros(1,6);
        kp_ua=zeros(1,6);
        kd_ua=zeros(1,6);
        ki_ua=zeros(1,6);
        kp_all=zeros(1,6);
        kd_all=zeros(1,6);
        ki_all=zeros(1,6);
        kp_sitaNsNtYaw=zeros(1,3);
        kd_sitaNsNtYaw=zeros(1,3);
        ki_sitaNsNtYaw=zeros(1,3);
        Inorm=eye(3);
        Ts=0.005;
    end
    properties (Access=private)
        UOld=zeros(12,1);
        ki_errA_Old=zeros(3,1);
        ki_errUA_Old=zeros(3,1);
        ki_errAll_Old=zeros(6,1);
        ki_errSita_Old=zeros(3,1);
        sitaStore=zeros(3,400);
        yawOld=zeros(1,3);
        LegStateOld=zeros(4,1);
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end

        function [U,errSita,errSita_de] = stepImpl(obj,LegState,pW,touchInd,xRef,xFB,surVN,Disable)
            pC=xFB(1:3);
            sita=xFB(4:6);

            obj.yawOld(1:end-1)=obj.yawOld(2:end);
            obj.yawOld(end)=xFB(6);
            yawFilt=sum(obj.yawOld)/length(obj.yawOld);
            Rz=[cos(yawFilt),-sin(yawFilt),0;sin(yawFilt),cos(yawFilt),0;0,0,1];

            if LegState(1)>0.5 && LegState(2)<0.5
                ns=(pW(:,1)-pW(:,4))/norm(pW(:,1)-pW(:,4));
            elseif LegState(1)<0.5 && LegState(2)>0.5
                ns=(pW(:,2)-pW(:,3))/norm(pW(:,2)-pW(:,3));
            else
                ns=[0;0;0];
            end
            nt=cross(surVN,ns);
            %ns=Rz'*ns;
            %nt=Rz'*nt;
            nyaw=surVN;

            % get state err in body frame

            T=[cos(sita(2))*cos(sita(3)),-sin(sita(3)),0;
                cos(sita(2))*sin(sita(3)),cos(sita(3)),0;
                -sin(sita(2)),0,1];
            %dsita=T\[xFB(10);xFB(11);xFB(12)];

            errSita=([xRef(4);xRef(5);xRef(6)]-[xFB(4);xFB(5);xFB(6)]);
            errdSita=T\([xRef(10);xRef(11);xRef(12)]-[xFB(10);xFB(11);xFB(12)]);
            errpCoM=([xRef(1);xRef(2);xRef(3)]-[xFB(1);xFB(2);xFB(3)]);
            errvCoM=([xRef(7);xRef(8);xRef(9)]-[xFB(7);xFB(8);xFB(9)]);

            % state err projcetion
            errpCoM_A=errpCoM'*ns*ns;
            errvCoM_A=errvCoM'*ns*ns;
            errpCoM_UA=errpCoM'*nt*nt;
            errvCoM_UA=errvCoM'*nt*nt;

            Rbody=eul2R(xFB(4),xFB(5),xFB(6));
            Rd=eul2R(xRef(4),xRef(5),xRef(6));

            wbody=logR(Rbody);
            wd=logR(Rd);

            wbody_s_sita=wbody'*ns;
            wbody_t_sita=wbody'*nt;
            wbody_yaw_sita=wbody'*nyaw;

            wd_s_sita=wd'*ns;
            wd_t_sita=wd'*nt;
            wd_yaw_sita=wd'*nyaw;

            errSita_de=([wd_t_sita;wd_s_sita;wd_yaw_sita]-[wbody_t_sita;wbody_s_sita;wbody_yaw_sita]);
            errdSita_lo=(xRef(10:12)-xFB(10:12));
            errdSita_de=[errdSita_lo'*nt;errdSita_lo'*ns;errdSita_lo'*nyaw];  

            obj.ki_errA_Old(1:3)=obj.ki_errA_Old(1:3)+diag(obj.ki_a(1:3))*errpCoM_A;
            obj.ki_errUA_Old(1:3)=obj.ki_errUA_Old(1:3)+diag(obj.ki_ua(1:3))*errpCoM_UA;
            obj.ki_errAll_Old(1:3)=obj.ki_errAll_Old(1:3)+diag(obj.ki_all(1:3))*errpCoM;
            obj.ki_errAll_Old(4:6)=obj.ki_errAll_Old(4:6)+diag(obj.ki_all(4:6))*errSita;
            obj.ki_errSita_Old(1:3)=obj.ki_errSita_Old(1:3)+diag(obj.ki_sitaNsNtYaw)*errSita_de;
            if LegState(1)+obj.LegStateOld(1)>0.5 && LegState(1)+obj.LegStateOld(1)<1.5 || ...
                    LegState(2)+obj.LegStateOld(2)>0.5 && LegState(2)+obj.LegStateOld(2)<1.5
                obj.ki_errA_Old=obj.ki_errA_Old*0;
                obj.ki_errUA_Old=obj.ki_errUA_Old*0;
                obj.ki_errAll_Old=obj.ki_errAll_Old*0;
                obj.ki_errSita_Old=obj.ki_errSita_Old*0;
            end
            ErrpCoM_A=diag(obj.kp_a(1:3))*errpCoM_A+diag(obj.kd_a(1:3))*errvCoM_A+obj.ki_errA_Old(1:3);
            ErrpCoM_UA=diag(obj.kp_ua(1:3))*errpCoM_UA+diag(obj.kd_ua(1:3))*errvCoM_UA+obj.ki_errUA_Old(1:3);
            
            ErrSita_de=diag(obj.kp_sitaNsNtYaw)*errSita_de+diag(obj.kd_sitaNsNtYaw)*errdSita_de;
            R_Err_de=expR(ErrSita_de(1)*nt+ErrSita_de(2)*ns+ErrSita_de(3)*nyaw);
            tmp=rotm2eul(R_Err_de);
            ErrSita=[tmp(3);tmp(2);tmp(1)];
            
            ErrpCoM=ErrpCoM_A+ErrpCoM_UA;

            ErrpCoM_All=diag(obj.kp_all(1:3))*errpCoM+diag(obj.kd_all(1:3))*errvCoM+obj.ki_errAll_Old(1:3);
            ErrSita_All=diag(obj.kp_all(4:6))*errSita+diag(obj.kd_all(4:6))*errdSita+obj.ki_errAll_Old(4:6);

            Inow=Rz*obj.Inorm*Rz';
            Iinv=inv(Inow);
            [~,B]=DS_gen(obj.Ts,3,sita,Iinv,(pW(:,1)-pC),(pW(:,2)-pC),(pW(:,3)-pC),(pW(:,4)-pC));
            B14=zeros(6,6);
            B23=zeros(6,6);
            B14(1:3,1:6)=B(1:3,[1:3,10:12]);
            B14(4:6,1:6)=B(4:6,[1:3,10:12]);
            B23(1:3,1:6)=B(1:3,4:9);
            B23(4:6,1:6)=B(4:6,4:9);

            U14=zeros(6,1);
            U23=zeros(6,1);
            U=zeros(12,1);
            if LegState(1)>0.5 && LegState(2)<0.5
                U14=pinv(B14)*[ErrpCoM;ErrSita];
                U=[U14(1:3);U23;U14(4:6)];
            elseif LegState(1)<0.5 && LegState(2)>0.5
                U23=pinv(B23)*[ErrpCoM;ErrSita];
                U=[U14(1:3);U23;U14(4:6)];
            else
                U=pinv(B(1:6,1:12))*[ErrpCoM_All;ErrSita_All];
            end

            if Disable>0.5
                U=zeros(12,1);
                obj.ki_errA_Old=obj.ki_errA_Old*0;
                obj.ki_errUA_Old=obj.ki_errUA_Old*0;
                obj.ki_errAll_Old=obj.ki_errAll_Old*0;
                obj.sitaStore=obj.sitaStore*0;
                obj.yawOld=obj.yawOld*0;
                obj.ki_errSita_Old=obj.ki_errSita_Old*0;
            end

            sitaAva=zeros(3,1);%mean(obj.sitaStore,2)/pi*180;
            sitaStd=zeros(3,1);%std(obj.sitaStore,0,2)/pi*180;
            obj.UOld=U;
            obj.LegStateOld=LegState;
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end

    end
end

%% subfunction
%% subfunction
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

function vcap=crossCap(v)
vcap=[0,-v(3),v(2);
    v(3),0,-v(1);
    -v(2),v(1),0];
end

function w=logR(R)
if norm(R*R-eye(3),1)<10^-5 % max absolute column sum
    w=[0;0;0];
    return;
end
sita=acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
w=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]*sita/2/sin(sita);
end

function Rn=expR(w)
if norm(w)<10^-5
    Rn=eye(3);
    return;
end
a=w/norm(w);
sita=norm(w);
acap=[0,-a(3),a(2);
    a(3),0,-a(1);
    -a(2),a(1),0];
Rn=eye(3)+acap*sin(sita)+acap*acap*(1-cos(sita));
end

function R=eul2R(roll,pit,yaw)
Rz=[cos(yaw),-sin(yaw),0;sin(yaw),cos(yaw),0;0,0,1];
Ry=[cos(pit),0,sin(pit);0,1,0;-sin(pit),0,cos(pit)];
Rx=[1,0,0;0,cos(roll),-sin(roll);0,sin(roll),cos(roll)];
R=Rz*Ry*Rx;
end



