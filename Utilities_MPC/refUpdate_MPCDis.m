classdef refUpdate_MPCDis < matlab.System
    % update
    properties
        Ts=0.005;
        m=3;
        Inorm=eye(3);
    end
    
    properties (Access=private)
        
    end
    
    methods(Access = protected)
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
        end
        
        function [refOVnew,mvNew] = stepImpl(obj,refSeqOri,xFB,pW)
            % note Q, R must be matrixes
            % Q for process noise
            % R for measurement noise
            PendAll=reshape(pW,3,4);
            Pc=xFB(1:3);
            theta=xFB(4:6);
            estDis=xFB(14:19);
            Rz=[cos(theta(3)),-sin(theta(3)),0;
                sin(theta(3)),cos(theta(3)),0;
                0,0,1];
            Inow=Rz*obj.Inorm*Rz';
            Iinv=inv(Inow);
            [A,B]=DS_gen(obj.Ts,obj.m,theta,Iinv,PendAll(:,1)-Pc,PendAll(:,2)-Pc,PendAll(:,3)-Pc,PendAll(:,4)-Pc);
            C=diag(ones(13,1));
            
            %             Anew=[A,[eye(6);zeros(7,6)];zeros(6,13),eye(6)];
            %             Bnew=[B;zeros(6,12)];
            %             Cnew=[C,zeros(13,6)];
            
            Bdis=zeros(13,6);
            Cdis=[eye(6);zeros(7,6)];
            
            MA=[(eye(13)-A),-B;C,zeros(13,12)];
            
            [n,~]=size(refSeqOri);
            refOVnew=zeros(n,13);
            refMVnew=zeros(n,12);
            for i=1:1:n
                ref=MA\[Bdis*estDis;(refSeqOri(i,:)'-Cdis*estDis)];
                refOVnew(i,:)=ref(1:13);
                refMVnew(i,:)=ref(14:25);
            end
            mvNew=refMVnew(end,:);
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end
        
        function [s1,s2] = getOutputSizeImpl(~)
            s1=[6,13];
            s2=[1,12];
        end
        
        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1=true;
            f2=true;
        end
        
        function [c1,c2] = isOutputComplexImpl(~)
            c1=false;
            c2=false;
        end
        
    end
end
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