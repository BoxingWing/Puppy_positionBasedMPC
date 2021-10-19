%%% mpc model with only output disturbance model
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
Ts=0.025; % smaple time for MPC, 0.025 for raspberry 4b to run adaptive mpc
Ts_DynSim=0.005; % sample time for central dynamics
T_gait=0.7;

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

T=[cos(theta(2))*cos(theta(3)),-sin(theta(3)),0;
    cos(theta(2))*sin(theta(3)),cos(theta(3)),0;
    -sin(theta(2)),0,1];

A=zeros(13,13);
A(1:3,7:9)=diag([1,1,1]);
A(4:6,10:12)=inv(T); % Rz' !!!!!!!!!! check here
A(9,13)=-1;


B=zeros(13,12);
B(7:9,1:3)=diag([1,1,1])/m;
B(7:9,4:6)=diag([1,1,1])/m;
B(7:9,7:9)=diag([1,1,1])/m;
B(7:9,10:12)=diag([1,1,1])/m;
B(10:12,1:3)=Inow\crossCap(PendAll(:,1)-Pc);
B(10:12,4:6)=Inow\crossCap(PendAll(:,2)-Pc);
B(10:12,7:9)=Inow\crossCap(PendAll(:,3)-Pc);
B(10:12,10:12)=Inow\crossCap(PendAll(:,4)-Pc);

C=diag(ones(13,1));

D=zeros(13,12);

plant=ss(A,B,C,D);
plantD=c2d(plant,Ts);
plantD2=c2d(plant,Ts_DynSim);

norminal.X=[Pc;theta;[0;0;0];[0;0;0];9.8];
norminal.U=[0;0;1;0;0;1;0;0;1;0;0;1]*m*9.8/4;
norminal.Y=plantD.C*norminal.X+plantD.D*norminal.U;
norminal.DX=plantD.A*norminal.X+plantD.B*norminal.U-norminal.X;
%% create MPC controller
% X=[r,theta,dr,omega,g], g=9.8;
% Y=[theta,dr,omega,g];
% U=[f1,f2,f3,f4];

surPa=[0;0;0]; % surface para a: z=a1+a2*x+a3*y, MUST be a unit vector
surVN=[0;0;1]; % surface norm vector, MUST be a unit vector
surV1=[1;0;0]; % surface vector v1, MUST be a unit vector
surV2=[0;1;0]; % surface vector v2, MUST be a unit vector
headV=[1;0;0]; % heading direction of the robot, MUST be a unit vector


model=struct('Plant',plantD,'Nominal',norminal);

numP=9; % prediction horizon
numM=2; % control horizon
%%% weights
%%% W.OutputVariables=ones(numP,1)*[[2,10,50],[0.25,0.5,10],[0.2,0.2,0.1],[0,0,0.3],0]; %2 10 50 0.25 0.5 10

W.OutputVariables=ones(numP,1)*[[2,2,10],[8,8,1],[1,1,1],[1,1,1],0];
%W.OutputVariables=ones(numP,1)*[[10,10,60],[15,20,2],[0.1,0.1,0.1],[0.2,0.2,0.01],0];

%W.OutputVariables(1,:)=W.OutputVariables(1,:)*1;
%W.OutputVariables(end,:)=[[10,10,60],[15,20,2],[0.1,0.1,0.1],[0.2,0.2,0.01],0];
%W.ManipulatedVariables=ones(numP,1)*[[0.05,0.05,0.01],[0.05,0.05,0.01],[0.05,0.05,0.01],[0.05,0.05,0.01]];
%W.ManipulatedVariables(end,:)=W.ManipulatedVariables(end,:)*0;


%%% manipulated variables
MV(12)=struct('Min',-inf,'Max',inf);

MV(3).Min=-50;
MV(3).Max=50;
MV(6).Min=-50;
MV(6).Max=50;
MV(9).Min=-50;
MV(9).Max=50;
MV(12).Min=-50;
MV(12).Max=50;

MV(1).ScaleFactor=5; % should be the range of the corresponding variable
MV(2).ScaleFactor=5;
MV(3).ScaleFactor=20;
MV(4).ScaleFactor=5;
MV(5).ScaleFactor=5;
MV(6).ScaleFactor=20;
MV(7).ScaleFactor=5;
MV(8).ScaleFactor=5;
MV(9).ScaleFactor=20;
MV(10).ScaleFactor=5;
MV(11).ScaleFactor=5;
MV(12).ScaleFactor=20;

%%% output variables
OV(13)=struct('Min',-inf,'Max',inf);
OV(3).Min=0.16;
OV(3).Max=5;

OV(1).ScaleFactor=10;
OV(2).ScaleFactor=10;
OV(3).ScaleFactor=0.4;
OV(4).ScaleFactor=10/180*pi;
OV(5).ScaleFactor=10/180*pi;
OV(6).ScaleFactor=10/180*pi;
OV(7).ScaleFactor=1;
OV(8).ScaleFactor=1;
OV(9).ScaleFactor=0.2;
OV(10).ScaleFactor=5;
OV(11).ScaleFactor=2;
OV(12).ScaleFactor=1;


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
%mpcPuppy.Optimizer.Algorithm='interior-point';
mpcPuppy.Optimizer.ActiveSetOptions.MaxIterations=30;
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
outdist(7,7)=modnor*0;
outdist(8,8)=modnor*0;
outdist(9,9)=modnor*0;
outdist(10,10)=modnor*0;
outdist(11,11)=modnor*0;
outdist(12,12)=modnor*0;
%setoutdist(mpcPuppy,'model',outdist);
setoutdist(mpcPuppy,'model',tf(zeros(13,1))); % remove output disturbance model
setEstimator(mpcPuppy,'custom');
xmpc=mpcstate(mpcPuppy);

%%% add constraints
% friction pyramid and payload balance
miu=0.6;
deltaFz=10; % w.r.t body coordinate
deltaFx=10; % w.r.t body coordinate
maxFz=60; % w.r.t. body coordinate

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
    Gb1(i+4)=5;
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

setconstraint(mpcPuppy,Enew,[],Gnew,Vnew);
%review(mpcPuppy);






















