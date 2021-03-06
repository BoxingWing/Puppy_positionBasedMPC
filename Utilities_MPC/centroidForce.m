classdef centroidForce < matlab.System
    % force and moment compensation via PD laws
    properties
        kp=zeros(1,6);
        kd=zeros(1,6);
        ki=zeros(1,6);
        Inorm=eye(3);
        Ts=0.005;
    end
    properties (Access=private)
        UOld=zeros(12,1);
        ki_err_Old=zeros(6,1);
        sitaStore=zeros(3,400);
        yawOld=zeros(1,10);
        LegStateOld=ones(4,1);
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [U,S14,S23] = stepImpl(obj,LegState,pW,touchInd,xRef,xFB,Disable)
            pC=xFB(1:3);
            sita=xFB(4:6);

            obj.yawOld(1:end-1)=obj.yawOld(2:end);
            obj.yawOld(end)=xFB(6);
            yawFilt=sum(obj.yawOld)/length(obj.yawOld);
            Rz=[cos(sita(3)),-sin(sita(3)),0;
                sin(sita(3)),cos(sita(3)),0;
                0,0,1];
            %Inow=Rz*obj.Inorm*Rz';
            Iinv=Rz*diag([1/obj.Inorm(1,1),1/obj.Inorm(2,2),1/obj.Inorm(3,3)])*Rz';
            [Ad,Bd]=DS_gen(obj.Ts,3,sita,Iinv,pW(:,1)-pC,pW(:,2)-pC,pW(:,3)-pC,pW(:,4)-pC);
            [A,B]=SS_gen(3,sita,Iinv,pW(:,1)-pC,pW(:,2)-pC,pW(:,3)-pC,pW(:,4)-pC);
            xPre=Ad*[xFB;9.8];
            %Rz=eye(3);
            
            T=[cos(sita(2))*cos(sita(3)),-sin(sita(3)),0;
                cos(sita(2))*sin(sita(3)),cos(sita(3)),0;
                -sin(sita(2)),0,1];
            %dsita=T\[xFB(10);xFB(11);xFB(12)];
            errSita=([xRef(4);xRef(5);xRef(6)]-[xPre(4);xPre(5);xPre(6)]);
            errdSita=([xRef(10);xRef(11);xRef(12)]-[xPre(10);xPre(11);xPre(12)]);
%             errpCoM=Rz'*([xRef(1);xRef(2);xRef(3)]-[xFB(1);xFB(2);xFB(3)]);
%             errvCoM=Rz'*([xRef(7);xRef(8);xRef(9)]-[xFB(7);xFB(8);xFB(9)]);
            errpCoM=([xRef(1);xRef(2);xRef(3)]-[xPre(1);xPre(2);xPre(3)]);
            errvCoM=([xRef(7);xRef(8);xRef(9)]-[xPre(7);xPre(8);xPre(9)]);
            
            Kp=diag(obj.kp);
            Kd=diag(obj.kd);
            Ki=diag(obj.ki);
            obj.ki_err_Old=obj.ki_err_Old+Ki*[errpCoM;errSita];
            %err=Kp*reshape(xRef(1:6)-xFB(1:6),6,1)+Kd*reshape(xRef(7:12)-xFB(7:12),6,1)+obj.ki_err_Old;
            if LegState(1)+obj.LegStateOld(1)>0.8 && LegState(1)+obj.LegStateOld(1)<1.2
                obj.ki_err_Old=obj.ki_err_Old*0;
            end
            if LegState(2)+obj.LegStateOld(2)>0.8 && LegState(2)+obj.LegStateOld(2)<1.2
                obj.ki_err_Old=obj.ki_err_Old*0;
            end

            errP1=Kp*[errpCoM;errSita]+obj.ki_err_Old;
            errP2=Kd*[errvCoM;errdSita];
            
            %M=[LegState(1).*eye(3),LegState(2).*eye(3),LegState(3).*eye(3),LegState(4).*eye(3);...
            %    LegState(1).*crossCap(Rz'*(pW(:,1)-pC)),LegState(2).*crossCap(Rz'*(pW(:,2)-pC)),LegState(3).*crossCap(Rz'*(pW(:,3)-pC)),LegState(4).*crossCap(Rz'*(pW(:,4)-pC));];
            
            if sum(LegState)<0.5
                Bnew=zeros(12,3);
            else
                Bnew=zeros(12,3*sum(LegState));
            end
            B14=zeros(12,6);
            B23=zeros(12,6);
            B14(:,1:3)=Bd(1:12,1:3);
            B14(:,4:6)=Bd(1:12,10:12);
            B23(:,1:3)=Bd(1:12,4:6);
            B23(:,4:6)=Bd(1:12,7:9);

            S14=zeros(6,1);
            S23=zeros(6,1);
            thred=10^-6;

            if Disable>0.5
                U=zeros(12,1);
                obj.ki_err_Old=zeros(6,1);
                obj.sitaStore=zeros(3,400);
                obj.yawOld=zeros(1,10);
            else
                if LegState(1)>0.5
                    U14=pinv(B14,  thred)*([errP1;errP2]);
                    S14=svd(B14);
                else
                    U14=zeros(6,1);
                end
                if LegState(2)>0.5
                    U23=pinv(B23,thred)*([errP1;errP2]);
                    S23=svd(B23);
                else
                    U23=zeros(6,1);
                end
                U=[U14(1:3);U23;U14(4:6)];

                %obj.sitaStore(:,1:end-1)=obj.sitaStore(:,2:end);
                %obj.sitaStore(:,end)=xFB(4:6);
            end

%             for i=1:1:4
%                 U(3*i-2:3*i)=Rz*U(3*i-2:3*i);
%             end
            
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
function [Ad,Bd] = DS_gen(Ts,m,in3,in4,in5,in6,in7,in8)
%DS_gen
%    [Ad,Bd] = DS_gen(Ts,M,IN3,IN4,IN5,IN6,IN7,IN8)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    19-Jan-2022 20:48:46

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

function [A,B] = SS_gen(m,in2,in3,in4,in5,in6,in7)
%SS_gen
%    [A,B] = SS_gen(M,IN2,IN3,IN4,IN5,IN6,IN7)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    20-Jan-2022 10:51:51

Iinv1_1 = in3(1);
Iinv1_2 = in3(4);
Iinv1_3 = in3(7);
Iinv2_1 = in3(2);
Iinv2_2 = in3(5);
Iinv2_3 = in3(8);
Iinv3_1 = in3(3);
Iinv3_2 = in3(6);
Iinv3_3 = in3(9);
r11 = in4(1,:);
r12 = in4(2,:);
r13 = in4(3,:);
r21 = in5(1,:);
r22 = in5(2,:);
r23 = in5(3,:);
r31 = in6(1,:);
r32 = in6(2,:);
r33 = in6(3,:);
r41 = in7(1,:);
r42 = in7(2,:);
r43 = in7(3,:);
theta3 = in2(3,:);
t2 = cos(theta3);
t3 = sin(theta3);
t6 = 1.0./m;
t4 = t2.^2;
t5 = t3.^2;
t7 = t4+t5;
t8 = 1.0./t7;
t9 = t2.*t8;
t10 = t3.*t8;
A = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t9,-t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t10,t9,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-1.0,0.0,0.0,0.0,0.0],[13,13]);
if nargout > 1
    mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,0.0,Iinv1_2.*r13-Iinv1_3.*r12,Iinv2_2.*r13-Iinv2_3.*r12,Iinv3_2.*r13-Iinv3_3.*r12,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,-Iinv1_1.*r13+Iinv1_3.*r11,-Iinv2_1.*r13+Iinv2_3.*r11,-Iinv3_1.*r13+Iinv3_3.*r11,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,Iinv1_1.*r12-Iinv1_2.*r11,Iinv2_1.*r12-Iinv2_2.*r11,Iinv3_1.*r12-Iinv3_2.*r11,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,0.0,Iinv1_2.*r23-Iinv1_3.*r22,Iinv2_2.*r23-Iinv2_3.*r22,Iinv3_2.*r23-Iinv3_3.*r22,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,-Iinv1_1.*r23+Iinv1_3.*r21,-Iinv2_1.*r23+Iinv2_3.*r21,-Iinv3_1.*r23+Iinv3_3.*r21,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6];
    mt2 = [Iinv1_1.*r22-Iinv1_2.*r21,Iinv2_1.*r22-Iinv2_2.*r21,Iinv3_1.*r22-Iinv3_2.*r21,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,0.0,Iinv1_2.*r33-Iinv1_3.*r32,Iinv2_2.*r33-Iinv2_3.*r32,Iinv3_2.*r33-Iinv3_3.*r32,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,-Iinv1_1.*r33+Iinv1_3.*r31,-Iinv2_1.*r33+Iinv2_3.*r31,-Iinv3_1.*r33+Iinv3_3.*r31,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,Iinv1_1.*r32-Iinv1_2.*r31,Iinv2_1.*r32-Iinv2_2.*r31,Iinv3_1.*r32-Iinv3_2.*r31,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,0.0,Iinv1_2.*r43-Iinv1_3.*r42,Iinv2_2.*r43-Iinv2_3.*r42,Iinv3_2.*r43-Iinv3_3.*r42,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,0.0,-Iinv1_1.*r43+Iinv1_3.*r41];
    mt3 = [-Iinv2_1.*r43+Iinv2_3.*r41,-Iinv3_1.*r43+Iinv3_3.*r41,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,Iinv1_1.*r42-Iinv1_2.*r41,Iinv2_1.*r42-Iinv2_2.*r41,Iinv3_1.*r42-Iinv3_2.*r41,0.0];
    B = reshape([mt1,mt2,mt3],13,12);
end
end

function vcap=crossCap(v)
vcap=[0,-v(3),v(2);
        v(3),0,-v(1);
       -v(2),v(1),0];
end


