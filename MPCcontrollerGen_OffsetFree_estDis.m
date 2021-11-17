%%% mpc model with measuared disturbances
% 6 disturbances
%%% REMEMBER to change the modelgen block in the simulink file

clear variables;
close all;
% X=[r,theta,dr,omega,g], g=9.8;
% U=[f1,f2,f3,f4];
Pc=[0;0;0.19];
m=3;
BxM=0.296; % x length of the trunk
ByM=0.117;
BzM=0.069;

Ixx=1/12*m*(ByM^2+BzM^2);
Iyy=1/12*m*(BxM^2+BzM^2);
Izz=1/12*m*(ByM^2+BxM^2);
I=diag([ Ixx, Iyy, Izz ]);
theta=ones(3,1)*0;
roll_Off=0.037;
L_Pend=zeros(3,4);
L_Pend(:,1)=[0;0;-190]/1000+[0;1;0]*roll_Off;
L_Pend(:,2)=[0;0;-190]/1000+[0;-1;0]*roll_Off;
L_Pend(:,3)=[0;0;-190]/1000+[0;1;0]*roll_Off;
L_Pend(:,4)=[0;0;-190]/1000+[0;-1;0]*roll_Off;
SPLeg=ones(4,1);
Ts=0.030; % smaple time for MPC, 0.025 for raspberry 4b to run adaptive mpc
Ts_DynSim=0.005; % sample time for central dynamics
T_gait=0.6;

LxM=0.2108; % distance between the fore and hind pitch axis
LyM=0.097; % distance between the left and right roll axis
peOff=zeros(3,4);
peOff(:,1)=[LxM;LyM;0]/2; % W_pe=L_pe+pC+pe_Off
peOff(:,2)=[LxM;-LyM;0]/2;
peOff(:,3)=[-LxM;LyM;0]/2;
peOff(:,4)=[-LxM;-LyM;0]/2;
PendAll=zeros(3,2);
PendAll(:,1)=Pc+peOff(:,1)+L_Pend(:,1);
PendAll(:,2)=Pc+peOff(:,2)+L_Pend(:,2);
PendAll(:,3)=Pc+peOff(:,3)+L_Pend(:,3);
PendAll(:,4)=Pc+peOff(:,4)+L_Pend(:,4);

Rx=[1,0,0;
    0,cos(theta(1)),-sin(theta(1));
    0,sin(theta(1)),cos(theta(1))];
Ry=[cos(theta(2)),0,sin(theta(2));
    0,1,0;
    -sin(theta(2)),0,cos(theta(2))];
Rz=[cos(theta(3)),-sin(theta(3)),0;
    sin(theta(3)),cos(theta(3)),0;
    0,0,1];
R=Rz*Ry*Rx;
Inow=R*I*R';
Iinv=inv(Inow);
[A,B]=DS_gen(Ts,m,theta,Iinv,PendAll(:,1)-Pc,PendAll(:,2)-Pc,PendAll(:,3)-Pc,PendAll(:,4)-Pc);
C=diag(ones(13,1));
D=zeros(13,12);

% augumented input disturbance model
Bd=[0*eye(6);eye(6);zeros(1,6)];
Cd=zeros(13,6);

A2=A;
B2=[B,Bd];
C2=C;
D2=[D,Cd*0];
plantDA=ss(A2,B2,C2,D2,Ts);
plantDA.InputGroup.MV=1:12;
plantDA.InputGroup.MD=13:18;
plantDA.OutputGroup.MO=1:13;

norminal.X=[Pc;theta;[0;0;0];[0;0;0];9.8];
norminal.U=[0;0;1;0;0;1;0;0;1;0;0;1;zeros(6,1)]*m*9.8/4;
norminal.Y=plantDA.C*norminal.X+plantDA.D*norminal.U;
norminal.DX=plantDA.A*norminal.X+plantDA.B*norminal.U-norminal.X;
%% create MPC controller
% X=[r,theta,dr,omega,g], g=9.8;
% Y=[theta,dr,omega,g];
% U=[f1,f2,f3,f4];

surPa=[0;0;0]; % surface para a: z=a1+a2*x+a3*y, MUST be a unit vector
surVN=[0;0;1]; % surface norm vector, MUST be a unit vector
surV1=[1;0;0]; % surface vector v1, MUST be a unit vector
surV2=[0;1;0]; % surface vector v2, MUST be a unit vector
headV=[1;0;0]; % heading direction of the robot, MUST be a unit vector


model=struct('Plant',plantDA,'Nominal',norminal);

numP=6; % prediction horizon: 6 step for 15 ms, 7 step for 20 ms, 8 step for 25 ms.
numM=2; % control horizon
%%% weights
%%% W.OutputVariables=ones(numP,1)*[[2,10,50],[0.25,0.5,10],[0.2,0.2,0.1],[0,0,0.3],0]; %2 10 50 0.25 0.5 10

W.OutputVariables=ones(numP,1)*[[20,20,180],[2,2,5],[10,10,0.1],[0.1,0.1,0.1],0];
%W.OutputVariables=ones(numP,1)*[[10,10,60],[15,20,2],[0.1,0.1,0.1],[0.2,0.2,0.01],0];

%W.OutputVariables(1,:)=W.OutputVariables(1,:)*1;
%W.OutputVariables(end,:)=[[20,20,180],[5,5,5],[10,10,0.1],[0.1,0.1,0.1],0];
%W.ManipulatedVariables=ones(numP,1)*[[0.01,0.01,0.01],[0.01,0.01,0.01],[0.01,0.01,0.01],[0.01,0.01,0.01]];
%W.ManipulatedVariables(end,:)=W.ManipulatedVariables(end,:)*0;
%W.ManipulatedVariablesRate=ones(numP,1)*[[0.1,0.1,0.1],[0.1,0.1,0.1],[0.1,0.1,0.1],[0.1,0.1,0.1]];

%%% manipulated variables
MV(12)=struct('Min',-inf,'Max',inf);

MV(3).Min=-20;
MV(3).Max=70;
MV(6).Min=-20;
MV(6).Max=70;
MV(9).Min=-20;
MV(9).Max=70;
MV(12).Min=-20;
MV(12).Max=70;

% MV(1).ScaleFactor=5; % should be the range of the corresponding variable
% MV(2).ScaleFactor=5;
% MV(3).ScaleFactor=20;
% MV(4).ScaleFactor=5;
% MV(5).ScaleFactor=5;
% MV(6).ScaleFactor=20;
% MV(7).ScaleFactor=5;
% MV(8).ScaleFactor=5;
% MV(9).ScaleFactor=20;
% MV(10).ScaleFactor=5;
% MV(11).ScaleFactor=5;
% MV(12).ScaleFactor=20;

%%% output variables
OV(13)=struct('Min',-inf,'Max',inf);
OV(3).Min=0.16;
OV(3).Max=5;

% OV(1).ScaleFactor=10;
% OV(2).ScaleFactor=10;
% OV(3).ScaleFactor=0.4;
% OV(4).ScaleFactor=10/180*pi;
% OV(5).ScaleFactor=10/180*pi;
% OV(6).ScaleFactor=10/180*pi;
% OV(7).ScaleFactor=1;
% OV(8).ScaleFactor=1;
% OV(9).ScaleFactor=0.2;
% OV(10).ScaleFactor=5;
% OV(11).ScaleFactor=2;
% OV(12).ScaleFactor=1;


% OV(4).Min=-10/180*pi;  % roll
% OV(4).Max=10/180*pi;
% OV(5).Min=-10/180*pi; % pitch
% OV(5).Max=10/180*pi;
% OV(7).Min=-0.5; % vx
% OV(7).Max=0.5;
% OV(8).Min=-0.2; % vy
% OV(8).Max=0.2;
% OV(9).Min=-0.5; % vz
% OV(9).Max=0.5;
% OV(10).Min=-3/180*pi; % droll
% OV(10).Max=3/180*pi;
% OV(11).Min=-3/180*pi; % dpitch
% OV(11).Max=3/180*pi;

mpcPuppy=mpc(model,Ts,numP,numM,W,MV,OV);
%setCustomSolver(mpcPuppy,'quadprog')
mpcPuppy.Optimizer.ActiveSetOptions.MaxIterations=25;
% mpcPuppy.Optimizer.CustomSolver=true;
% mpcPuppy.Optimizer.CustomSolverCodeGen=true;
mpcPuppy.Optimizer.UseSuboptimalSolution=true;
modnor=tf(1,1);
modnor2=tf(1,[1,0]);
outdist=diag(ones(13,1))*modnor;
outdist(13,13)=0;
magOutdist=[0,0,0,0,0,0,0.1,0.1,0.1,0.5,0.5,0.5,0];
%outdist=zeros(13,13);
outdist(13,13)=modnor*0;
outdist(1,1)=modnor*0.1;
outdist(2,2)=modnor*0.1;
outdist(3,3)=modnor*0.1;
outdist(4,4)=modnor*0.1;
outdist(5,5)=modnor*0.1;
outdist(6,6)=modnor*0.1;
outdist(7,7)=modnor*0.1;
outdist(8,8)=modnor*0.1;
outdist(9,9)=modnor*0.1;
outdist(10,10)=modnor*0.1;
outdist(11,11)=modnor*0.1;
outdist(12,12)=modnor*0.1;
%setoutdist(mpcPuppy,'model',outdist);
setoutdist(mpcPuppy,'model',tf(zeros(13,1))); % remove output disturbance model
setEstimator(mpcPuppy,'custom');
% mpcPuppy.DV(1).ScaleFactor=2;
% mpcPuppy.DV(2).ScaleFactor=2;
% mpcPuppy.DV(3).ScaleFactor=2;
% mpcPuppy.DV(4).ScaleFactor=100;
% mpcPuppy.DV(5).ScaleFactor=100;
% mpcPuppy.DV(6).ScaleFactor=100;
xmpc=mpcstate(mpcPuppy);

%%% add constraints
% friction pyramid and payload balance
miu=0.4;
deltaFz=50; % w.r.t body coordinate
deltaFx=50; % w.r.t body coordinate
maxFz=70; % w.r.t. body coordinate

% friction cone----------------------
Eb2=zeros(8,12);
Gb2=zeros(8,1);
Vb2=zeros(8,1);
for i=1:1:4
    Eb2(i,3*i-2:3*i)=-miu*surVN+surV1;
    Eb2(i+4,3*i-2:3*i)=-miu*surVN-surV1;
end
Eb3=zeros(8,12);
Gb3=zeros(8,1);
Vb3=zeros(8,1);
for i=1:1:4
    Eb3(i,3*i-2:3*i)=-miu*surVN+surV2;
    Eb3(i+4,3*i-2:3*i)=-miu*surVN-surV2;
end
% payload balance--------------
surVhead=headV-headV'*surVN*surVN;
surVhead=surVhead/norm(surVhead);
Eb4=zeros(4,12);
Gb4=zeros(4,1);
Vb4=ones(4,1);
Eb4(1,1:3)=surVhead;
Eb4(1,10:12)=-surVhead;
Eb4(2,1:3)=-surVhead;
Eb4(2,10:12)=surVhead;
Gb4(1)=deltaFx;
Gb4(2)=deltaFx;
Eb4(3,4:6)=surVhead;
Eb4(3,7:9)=-surVhead;
Eb4(4,4:6)=-surVhead;
Eb4(4,7:9)=surVhead;
Gb4(3)=deltaFx;
Gb4(4)=deltaFx;

Eb5=zeros(4,12);
Gb5=zeros(4,1);
Vb5=ones(4,1);
Eb5(1,1:3)=surVN;
Eb5(1,10:12)=-surVN;
Eb5(2,1:3)=-surVN;
Eb5(2,10:12)=surVN;
Gb5(1)=deltaFz;
Gb5(2)=deltaFz;
Eb5(3,4:6)=surVN;
Eb5(3,7:9)=-surVN;
Eb5(4,4:6)=-surVN;
Eb5(4,7:9)=surVN;
Gb5(3)=deltaFz;
Gb5(4)=deltaFz;

% support force greater than zero, gait define----------
Eb1=zeros(8,12);
Gb1=zeros(8,1);
Vb1=zeros(8,1);
for i=1:1:4
    Eb1(i,3*i-2:3*i)=surVN;
    Eb1(i+4,3*i-2:3*i)=-surVN;
    Gb1(i)=maxFz;
    Gb1(i+4)=0;
end
for i=1:1:4
    if SPLeg(i)<0.5
        Gb1(i)=0;
        Gb1(i+4)=0;
    end
end

Enew=[Eb2;Eb3;Eb4;Eb5;Eb1];
Gnew=[Gb2;Gb3;Gb4;Gb5;Gb1];
Vnew=[Vb2;Vb3;Vb4;Vb5;Vb1];

% Enew=[Eb2;Eb3;Eb1];
% Gnew=[Gb2;Gb3;Gb1];
% Vnew=[Vb2;Vb3;Vb1];

% E=[1,0,-miu,0,0,0,0,0,0,0,0,0,0;  % friction cone
%     -1,0,-miu,0,0,0,0,0,0,0,0,0,0;
%     0,1,-miu,0,0,0,0,0,0,0,0,0,0;
%     0,-1,-miu,0,0,0,0,0,0,0,0,0,0;
%     0,0,0,1,0,-miu,0,0,0,0,0,0,0;
%     0,0,0,-1,0,-miu,0,0,0,0,0,0,0;
%     0,0,0,0,1,-miu,0,0,0,0,0,0,0;
%     0,0,0,0,-1,-miu,0,0,0,0,0,0,0;
%     zeros(1,6),1,0,-miu,0,0,0,0;
%     zeros(1,6),-1,0,-miu,0,0,0,0;
%     zeros(1,6),0,1,-miu,0,0,0,0;
%     zeros(1,6),0,-1,-miu,0,0,0,0;
%     zeros(1,6),0,0,0,1,0,-miu,0;
%     zeros(1,6),0,0,0,-1,0,-miu,0;
%     zeros(1,6),0,0,0,0,1,-miu,0;
%     zeros(1,6),0,0,0,0,-1,-miu,0;
%     0,0,1,0,0,0,0,0,0,0,0,-1,0; % payload balance
%     0,0,-1,0,0,0,0,0,0,0,0,1,0;
%     0,0,0,0,0,1,0,0,-1,0,0,0,0;
%     0,0,0,0,0,-1,0,0,1,0,0,0,0;
%     1,0,0,0,0,0,0,0,0,0,-1,0,0;
%     -1,0,0,0,0,0,0,0,0,0,1,0,0;
%     0,0,0,1,0,0,-1,0,0,0,0,0,0;
%     0,0,0,-1,0,0,1,0,0,0,0,0,0;
%     ];
% F=[];
% G=[zeros(16,1);ones(4,1)*deltaFz;ones(4,1)*deltaFx];
%V=[ones(16,1)*0;ones(8,1)];
Snew=zeros(32,6);
setconstraint(mpcPuppy,Enew,[],Gnew,Vnew,Snew);
%review(mpcPuppy);


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

















