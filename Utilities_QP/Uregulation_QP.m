classdef Uregulation_QP< matlab.System
    % generate online state space model
    % x=[r,theta,dr,omega,g];
    % u=[f1,f2,f3,f4];
    % y=[r,theta,dr,omega,g]
    properties
        miu=0.5;
        maxFz=50;
    end
    
    properties(Access=private)
        
    end
    
    methods(Access = protected)
        
        function setupImpl(obj)
            
        end
        
        function [Unew,exitflag] = stepImpl(obj,U1,U2,surVN,surVH,surVL,LegState)
            % surVN: norm vector of the surface
            % surVH: heading vector along the surface
            % surVL: lateral vector along the surface
            Uadd=U1+U2;
            
            % friction cone
            A1=zeros(8,12);
            b1=zeros(8,1);
            for i=1:1:4
                A1(i,3*i-2:3*i)=-obj.miu*surVN+surVH;
                A1(i+4,3*i-2:3*i)=-obj.miu*surVN-surVH;
            end
            A2=zeros(8,12);
            b2=zeros(8,1);
            for i=1:1:4
                A2(i,3*i-2:3*i)=-obj.miu*surVN+surVL;
                A2(i+4,3*i-2:3*i)=-obj.miu*surVN-surVL;
            end
            % support force greater than zero, gait define
            A3=zeros(8,12);
            b3=zeros(8,1);
            for i=1:1:4
                A3(i,3*i-2:3*i)=surVN;
                A3(i+4,3*i-2:3*i)=-surVN;
                b3(i)=obj.maxFz;
                b3(i+4)=0;
            end
            for i=1:1:4
                if LegState(i)<0.5
                    b3(i)=0;
                    b3(i+4)=0;
                end
            end
            
            A=[A1;A2;A3];
            b=[b1;b2;b3];
            
            bdelta=b-A*Uadd;
            
            H=diag(ones(12,1));
            f=zeros(12,1);
            
            opts = optimoptions('quadprog','Algorithm','active-set','MaxIterations',4);
            [Udelta,~,exitflag] = quadprog(H,f,A,bdelta,[],[],[],[],zeros(12,1),opts);
            
            Unew=Uadd+Udelta;
        end
        
        function [d1,d2] = getOutputDataTypeImpl(~)
            d1 = 'double';
            d2 = 'double';
        end
        
        function [s1,s2] = getOutputSizeImpl(~)
            s1=[12,1];
            s2=[1,1];
        end
        
        function [f1,f2] = isOutputFixedSizeImpl(~)
            f1=true;
            f2=true;
        end
        
        function [c1,c2] = isOutputComplexImpl(~)
            c1=false;
            c2=false;
        end
        
        function resetImpl(obj)
            % Initialize / reset discrete-state properties
        end
        
    end
end
% %% subfunction
% function [Ad,Bd] = DS_gen(Ts,m,in3,in4,in5,in6,in7,in8)
% %DS_GEN
% %    [AD,BD] = DS_GEN(TS,M,theta,Iinv,r1,r2,r3,r4)
%
% %    This function was generated by the Symbolic Math Toolbox version 8.6.
%
% Iinv1_1 = in4(1);
% Iinv1_2 = in4(4);
% Iinv1_3 = in4(7);
% Iinv2_1 = in4(2);
% Iinv2_2 = in4(5);
% Iinv2_3 = in4(8);
% Iinv3_1 = in4(3);
% Iinv3_2 = in4(6);
% Iinv3_3 = in4(9);
% r11 = in5(1,:);
% r12 = in5(2,:);
% r13 = in5(3,:);
% r21 = in6(1,:);
% r22 = in6(2,:);
% r23 = in6(3,:);
% r31 = in7(1,:);
% r32 = in7(2,:);
% r33 = in7(3,:);
% r41 = in8(1,:);
% r42 = in8(2,:);
% r43 = in8(3,:);
% theta2 = in3(2,:);
% theta3 = in3(3,:);
% t2 = cos(theta2);
% t3 = cos(theta3);
% t4 = sin(theta2);
% t5 = sin(theta3);
% t6 = Iinv1_1.*r12;
% t7 = Iinv1_2.*r11;
% t8 = Iinv1_1.*r13;
% t9 = Iinv1_3.*r11;
% t10 = Iinv1_2.*r13;
% t11 = Iinv1_3.*r12;
% t12 = Iinv1_1.*r22;
% t13 = Iinv1_2.*r21;
% t14 = Iinv2_1.*r12;
% t15 = Iinv2_2.*r11;
% t16 = Iinv1_1.*r23;
% t17 = Iinv1_3.*r21;
% t18 = Iinv2_1.*r13;
% t19 = Iinv2_3.*r11;
% t20 = Iinv1_2.*r23;
% t21 = Iinv1_3.*r22;
% t22 = Iinv2_2.*r13;
% t23 = Iinv2_3.*r12;
% t24 = Iinv1_1.*r32;
% t25 = Iinv1_2.*r31;
% t26 = Iinv2_1.*r22;
% t27 = Iinv2_2.*r21;
% t28 = Iinv3_1.*r12;
% t29 = Iinv3_2.*r11;
% t30 = Iinv1_1.*r33;
% t31 = Iinv1_3.*r31;
% t32 = Iinv2_1.*r23;
% t33 = Iinv2_3.*r21;
% t34 = Iinv3_1.*r13;
% t35 = Iinv3_3.*r11;
% t36 = Iinv1_2.*r33;
% t37 = Iinv1_3.*r32;
% t38 = Iinv2_2.*r23;
% t39 = Iinv2_3.*r22;
% t40 = Iinv3_2.*r13;
% t41 = Iinv3_3.*r12;
% t42 = Iinv1_1.*r42;
% t43 = Iinv1_2.*r41;
% t44 = Iinv2_1.*r32;
% t45 = Iinv2_2.*r31;
% t46 = Iinv3_1.*r22;
% t47 = Iinv3_2.*r21;
% t48 = Iinv1_1.*r43;
% t49 = Iinv1_3.*r41;
% t50 = Iinv2_1.*r33;
% t51 = Iinv2_3.*r31;
% t52 = Iinv3_1.*r23;
% t53 = Iinv3_3.*r21;
% t54 = Iinv1_2.*r43;
% t55 = Iinv1_3.*r42;
% t56 = Iinv2_2.*r33;
% t57 = Iinv2_3.*r32;
% t58 = Iinv3_2.*r23;
% t59 = Iinv3_3.*r22;
% t60 = Iinv2_1.*r42;
% t61 = Iinv2_2.*r41;
% t62 = Iinv3_1.*r32;
% t63 = Iinv3_2.*r31;
% t64 = Iinv2_1.*r43;
% t65 = Iinv2_3.*r41;
% t66 = Iinv3_1.*r33;
% t67 = Iinv3_3.*r31;
% t68 = Iinv2_2.*r43;
% t69 = Iinv2_3.*r42;
% t70 = Iinv3_2.*r33;
% t71 = Iinv3_3.*r32;
% t72 = Iinv3_1.*r42;
% t73 = Iinv3_2.*r41;
% t74 = Iinv3_1.*r43;
% t75 = Iinv3_3.*r41;
% t76 = Iinv3_2.*r43;
% t77 = Iinv3_3.*r42;
% t78 = Ts.^2;
% t81 = 1.0./m;
% t79 = t3.^2;
% t80 = t5.^2;
% t82 = -t7;
% t83 = -t9;
% t84 = -t11;
% t85 = -t13;
% t86 = -t15;
% t87 = -t17;
% t88 = -t19;
% t89 = -t21;
% t90 = -t23;
% t91 = -t25;
% t92 = -t27;
% t93 = -t29;
% t94 = -t31;
% t95 = -t33;
% t96 = -t35;
% t97 = -t37;
% t98 = -t39;
% t99 = -t41;
% t100 = -t43;
% t101 = -t45;
% t102 = -t47;
% t103 = -t49;
% t104 = -t51;
% t105 = -t53;
% t106 = -t55;
% t107 = -t57;
% t108 = -t59;
% t109 = -t61;
% t110 = -t63;
% t111 = -t65;
% t112 = -t67;
% t113 = -t69;
% t114 = -t71;
% t115 = -t73;
% t116 = -t75;
% t117 = -t77;
% t118 = 1.0./t2;
% t119 = Ts.*t81;
% t159 = (t78.*t81)./2.0;
% t120 = t2.*t79;
% t121 = t2.*t80;
% t122 = t6+t82;
% t123 = t8+t83;
% t124 = t10+t84;
% t125 = t12+t85;
% t126 = t14+t86;
% t127 = t16+t87;
% t128 = t18+t88;
% t129 = t20+t89;
% t130 = t22+t90;
% t131 = t24+t91;
% t132 = t26+t92;
% t133 = t28+t93;
% t134 = t30+t94;
% t135 = t32+t95;
% t136 = t34+t96;
% t137 = t36+t97;
% t138 = t38+t98;
% t139 = t40+t99;
% t140 = t42+t100;
% t141 = t44+t101;
% t142 = t46+t102;
% t143 = t48+t103;
% t144 = t50+t104;
% t145 = t52+t105;
% t146 = t54+t106;
% t147 = t56+t107;
% t148 = t58+t108;
% t149 = t60+t109;
% t150 = t62+t110;
% t151 = t64+t111;
% t152 = t66+t112;
% t153 = t68+t113;
% t154 = t70+t114;
% t155 = t72+t115;
% t156 = t74+t116;
% t157 = t76+t117;
% t158 = t79+t80;
% t160 = 1.0./t158;
% t161 = t120+t121;
% t162 = 1.0./t161;
% Ad = reshape([1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts.*t3.*t162,-Ts.*t5.*t160,Ts.*t3.*t4.*t162,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts.*t5.*t162,Ts.*t3.*t160,Ts.*t4.*t5.*t162,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,Ts,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t78.*(-1.0./2.0),0.0,0.0,0.0,0.0,0.0,-Ts,0.0,0.0,0.0,1.0],[13,13]);
% if nargout > 1
%     Bd = reshape([t159,0.0,0.0,(t3.*t78.*t118.*t124)./2.0+(t5.*t78.*t118.*t130)./2.0,-t5.*t78.*(t10./2.0-t11./2.0)+t3.*t78.*(t22./2.0-t23./2.0),(t78.*t139)./2.0+(t78.*t118.*(t3.*t4.*t10+t4.*t5.*t22+t3.*t4.*t84+t4.*t5.*t90))./2.0,t119,0.0,0.0,Ts.*t124,Ts.*t130,Ts.*t139,0.0,0.0,t159,0.0,t3.*t78.*t118.*t123.*(-1.0./2.0)-(t5.*t78.*t118.*t128)./2.0,t5.*t78.*(t8./2.0-t9./2.0)-t3.*t78.*(t18./2.0-t19./2.0),t78.*t136.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t8+t4.*t5.*t18+t3.*t4.*t83+t4.*t5.*t88))./2.0,0.0,t119,0.0,-Ts.*t123,-Ts.*t128,-Ts.*t136,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t122)./2.0+(t5.*t78.*t118.*t126)./2.0,-t5.*t78.*(t6./2.0-t7./2.0)+t3.*t78.*(t14./2.0-t15./2.0),(t78.*t133)./2.0+(t78.*t118.*(t3.*t4.*t6+t4.*t5.*t14+t3.*t4.*t82+t4.*t5.*t86))./2.0,0.0,0.0,t119,Ts.*t122,Ts.*t126,Ts.*t133,0.0,t159,0.0,0.0,(t3.*t78.*t118.*t129)./2.0+(t5.*t78.*t118.*t138)./2.0,-t5.*t78.*(t20./2.0-t21./2.0)+t3.*t78.*(t38./2.0-t39./2.0),(t78.*t148)./2.0+(t78.*t118.*(t3.*t4.*t20+t4.*t5.*t38+t3.*t4.*t89+t4.*t5.*t98))./2.0,t119,0.0,0.0,Ts.*t129,Ts.*t138,Ts.*t148,0.0,0.0,t159,0.0,t3.*t78.*t118.*t127.*(-1.0./2.0)-(t5.*t78.*t118.*t135)./2.0,t5.*t78.*(t16./2.0-t17./2.0)-t3.*t78.*(t32./2.0-t33./2.0),t78.*t145.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t16+t4.*t5.*t32+t3.*t4.*t87+t4.*t5.*t95))./2.0,0.0,t119,0.0,-Ts.*t127,-Ts.*t135,-Ts.*t145,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t125)./2.0+(t5.*t78.*t118.*t132)./2.0,-t5.*t78.*(t12./2.0-t13./2.0)+t3.*t78.*(t26./2.0-t27./2.0),(t78.*t142)./2.0+(t78.*t118.*(t3.*t4.*t12+t4.*t5.*t26+t3.*t4.*t85+t4.*t5.*t92))./2.0,0.0,0.0,t119,Ts.*t125,Ts.*t132,Ts.*t142,0.0,t159,0.0,0.0,(t3.*t78.*t118.*t137)./2.0+(t5.*t78.*t118.*t147)./2.0,-t5.*t78.*(t36./2.0-t37./2.0)+t3.*t78.*(t56./2.0-t57./2.0),(t78.*t154)./2.0+(t78.*t118.*(t3.*t4.*t36+t4.*t5.*t56+t3.*t4.*t97+t4.*t5.*t107))./2.0,t119,0.0,0.0,Ts.*t137,Ts.*t147,Ts.*t154,0.0,0.0,t159,0.0,t3.*t78.*t118.*t134.*(-1.0./2.0)-(t5.*t78.*t118.*t144)./2.0,t5.*t78.*(t30./2.0-t31./2.0)-t3.*t78.*(t50./2.0-t51./2.0),t78.*t152.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t30+t4.*t5.*t50+t3.*t4.*t94+t4.*t5.*t104))./2.0,0.0,t119,0.0,-Ts.*t134,-Ts.*t144,-Ts.*t152,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t131)./2.0+(t5.*t78.*t118.*t141)./2.0,-t5.*t78.*(t24./2.0-t25./2.0)+t3.*t78.*(t44./2.0-t45./2.0),(t78.*t150)./2.0+(t78.*t118.*(t3.*t4.*t24+t4.*t5.*t44+t3.*t4.*t91+t4.*t5.*t101))./2.0,0.0,0.0,t119,Ts.*t131,Ts.*t141,Ts.*t150,0.0,t159,0.0,0.0,(t3.*t78.*t118.*t146)./2.0+(t5.*t78.*t118.*t153)./2.0,-t5.*t78.*(t54./2.0-t55./2.0)+t3.*t78.*(t68./2.0-t69./2.0),(t78.*t157)./2.0+(t78.*t118.*(t3.*t4.*t54+t4.*t5.*t68+t3.*t4.*t106+t4.*t5.*t113))./2.0,t119,0.0,0.0,Ts.*t146,Ts.*t153,Ts.*t157,0.0,0.0,t159,0.0,t3.*t78.*t118.*t143.*(-1.0./2.0)-(t5.*t78.*t118.*t151)./2.0,t5.*t78.*(t48./2.0-t49./2.0)-t3.*t78.*(t64./2.0-t65./2.0),t78.*t156.*(-1.0./2.0)-(t78.*t118.*(t3.*t4.*t48+t4.*t5.*t64+t3.*t4.*t103+t4.*t5.*t111))./2.0,0.0,t119,0.0,-Ts.*t143,-Ts.*t151,-Ts.*t156,0.0,0.0,0.0,t159,(t3.*t78.*t118.*t140)./2.0+(t5.*t78.*t118.*t149)./2.0,-t5.*t78.*(t42./2.0-t43./2.0)+t3.*t78.*(t60./2.0-t61./2.0),(t78.*t155)./2.0+(t78.*t118.*(t3.*t4.*t42+t4.*t5.*t60+t3.*t4.*t100+t4.*t5.*t109))./2.0,0.0,0.0,t119,Ts.*t140,Ts.*t149,Ts.*t155,0.0],[13,12]);
% end
% end
% function vcap=crossCap(v)
% vcap=[0,-v(3),v(2);
%         v(3),0,-v(1);
%        -v(2),v(1),0];
% end