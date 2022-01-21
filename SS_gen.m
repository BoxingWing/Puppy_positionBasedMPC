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
