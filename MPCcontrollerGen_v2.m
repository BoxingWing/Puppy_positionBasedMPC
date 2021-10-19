%% derive the plant model, only two supporting legs are calculated
% optimizer of 

clear variables;
close all;
% X=[r,theta,dr,omega,g], g=9.8;
% U=[f1,f2,f3,f4];
Pc=[0;0;0.19];
m=3;
I=diag([ 0.0078, 0.0275, 0.0328 ]);
theta=ones(3,1)*0;
roll_Off=0.037;
L_Pend=zeros(3,4);
L_Pend(:,1)=[0;0;-190]/1000+[0;1;0]*roll_Off;
L_Pend(:,2)=[0;0;-190]/1000+[0;-1;0]*roll_Off;
L_Pend(:,3)=[0;0;-190]/1000+[0;1;0]*roll_Off;
L_Pend(:,4)=[0;0;-190]/1000+[0;-1;0]*roll_Off;
SPLeg=ones(4,1);
Ts=0.025; % smaple time for MPC, 0.04 for raspberry 4b to run adaptive mpc
Ts_DynSim=0.005; % sample time for central dynamics
T_gait=0.65;

LxM=0.2108;
LyM=0.097; % 0.171
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

%plantD = setmpcsignals(plantD,'UD',[1 2 3 4 5 6]);
model=struct('Plant',plantD,'Nominal',norminal);

numP=8; % prediction horizon
numM=2; % control horizon
%%% weights
%%% W.OutputVariables=ones(numP,1)*[[2,10,50],[0.25,0.5,10],[0.2,0.2,0.1],[0,0,0.3],0]; %2 10 50 0.25 0.5 10

W.OutputVariables=ones(numP,1)*[[10,10,60],[15,20,2],[0.1,0.1,0.1],[0.01,0.2,0.01],0];
W.OutputVariables(1,:)=W.OutputVariables(1,:)*1;
W.OutputVariables(end,:)=[[10,10,60],[15,20,2],[0.1,0.1,0.1],[0.01,0.2,0.01],0];
%W.ManipulatedVariables=ones(numP,1)*[[0.05,0.05,0.05],[0.05,0.05,0.05],[0.05,0.05,0.05],[0.05,0.05,0.05]]/10;
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

%%% output variables
OV(13)=struct('Min',-inf,'Max',inf);
OV(1).ScaleFactor=1;%0.10;
OV(2).ScaleFactor=1;%0.10;

OV(3).Min=0.16;
OV(3).Max=5;
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
old_status = mpcverbosity('off');
mpcPuppy=mpc(model,Ts,numP,numM,W,MV,OV);
%mpcPuppy.Optimizer.Algorithm='interior-point';
mpcPuppy.Optimizer.ActiveSetOptions.MaxIterations=20;
mpcPuppy.Optimizer.UseSuboptimalSolution=true;
modnor=tf(1,1);
outdist=diag(ones(13,1))*modnor;
outdist(13,13)=0;
setoutdist(mpcPuppy,'model',outdist);
%setoutdist(mpcPuppy,'model',tf(zeros(13,1))); % remove output disturbance model
%setindist(mpcPuppy,'model',outdist);
%setEstimator(mpcPuppy,'custom');
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
% %% simulink closed-loop test
%  mdl='MPC_test_PC_now_v2';
% simout=sim(mdl);
% %% plot results
% %%%% plot states
% 
% tYY=simout.YY.time;
% tUU=simout.UU.time;
% r=simout.YY.signals.values(:,1:3);
% theta=simout.YY.signals.values(:,4:6);
% dr=simout.YY.signals.values(:,7:9);
% omega=simout.YY.signals.values(:,10:12);
% mvRec=reshape(simout.UU.signals.values,12,[]);
% U1=simout.UU.signals.values(:,1:3);
% U2=simout.UU.signals.values(:,4:6);
% U3=simout.UU.signals.values(:,7:9);
% U4=simout.UU.signals.values(:,10:12);
% tRef=simout.refP.time;
% rRef=simout.refP.signals.values(1,1:3,:);
% rRef=reshape(rRef,3,[]);
% thetaRef=simout.refP.signals.values(1,4:6,:);
% thetaRef=reshape(thetaRef,3,[]);
% drRef=simout.refP.signals.values(1,7:9,:);
% drRef=reshape(drRef,3,[]);
% omegaRef=simout.refP.signals.values(1,10:12,:);
% omegaRef=reshape(omegaRef,3,[]);
% tPend=simout.PendAll.time;
% Pend1=simout.PendAll.signals.values(:,1,:);
% Pend1=reshape(Pend1,3,[]);
% Pend2=simout.PendAll.signals.values(:,2,:);
% Pend2=reshape(Pend2,3,[]);
% Pend3=simout.PendAll.signals.values(:,3,:);
% Pend3=reshape(Pend3,3,[]);
% Pend4=simout.PendAll.signals.values(:,4,:);
% Pend4=reshape(Pend4,3,[]);
% 
% figure();
% subplot(4,2,1);
% plot(tYY,r);
% legend('x','y','z');
% subplot(4,2,2);
% plot(tYY,dr);
% legend('dx','dy','dz');
% subplot(4,2,3);
% plot(tYY,theta/pi*180); ylabel('degree')
% legend('\theta_x','\theta_y','\theta_z');
% subplot(4,2,4);
% plot(tYY,omega);
% legend('\omega_x','\omega_y','\omega_z');
% subplot(4,2,5);
% plot(tUU,U1);
% legend('f1x','f1y','f1z');
% subplot(4,2,6);
% plot(tUU,U2);
% legend('f2x','f2y','f2z');
% subplot(4,2,7);
% plot(tUU,U3);
% legend('f3x','f3y','f3z');
% subplot(4,2,8);
% plot(tUU,U4);
% legend('f4x','f4y','f4z');
% %%%% plot local foot-end position
% tPend=simout.PendAllLocal.time;
% PendAllLocal=simout.PendAllLocal.signals.values;
% L1=PendAllLocal(:,1,:);
% L1=reshape(L1,3,[]);
% L2=PendAllLocal(:,2,:);
% L2=reshape(L2,3,[]);
% L3=PendAllLocal(:,3,:);
% L3=reshape(L3,3,[]);
% L4=PendAllLocal(:,4,:);
% L4=reshape(L4,3,[]);
% 
% figure();
% subplot(2,2,1)
% plot3(L1(1,:),L1(2,:),L1(3,:)); axis equal;grid on;
% subplot(2,2,2)
% plot(tPend,L1(1,:),tPend,L1(2,:),tPend,L1(3,:));
% legend('L1x','L1y','L1z');
% subplot(2,2,3)
% plot(L2(1,:),L2(3,:));
% subplot(2,2,4)
% plot(tPend,L2(1,:),tPend,L2(2,:),tPend,L2(3,:));
% legend('L2x','L2y','L2z');
% figure();
% l1=plot3(r(:,1),r(:,2),r(:,3));
% hold on;
% l2=plot3(rRef(1,:),rRef(2,:),rRef(3,:));
% dN=20;
% scaleF=0.3;
% for i=1:dN:length(rRef(1,:))
% quiver3(rRef(1,i),rRef(2,i),rRef(3,i), ...
%     scaleF*drRef(1,i),scaleF*drRef(2,i),scaleF*drRef(3,i),'g');
% end
% plot3(Pend1(1,:),Pend1(2,:),Pend1(3,:));
% plot3(Pend4(1,:),Pend4(2,:),Pend4(3,:));
% plot3(Pend2(1,:),Pend2(2,:),Pend2(3,:));
% plot3(Pend3(1,:),Pend3(2,:),Pend3(3,:));
% [xM,yM]=meshgrid(0.5:0.01:3,-0.4:0.01:0.4);
% sura=simout.sura.signals.values(1,:);
% zM=sura(1)+sura(2).*xM+sura(3).*yM;
% mesh(xM,yM,zM,'edgecolor','interp');
% legend([l1 l2],'Real','Desired');
% axis equal;
% grid on;



%%%%-------------------------------------------------------
%%% compare mpc xsequence with the real one
% figure();
% xseq=simout.xseq.signals.values;
% dt=Ts;
% tseg=(0:1:numP)*dt;
% %
% subplot(2,2,1);
% l1=plot(tYY,r(:,1),'color','#0072BD');
% hold on;
% l2=plot(tYY,r(:,2),'color','#D95319');
% l3=plot(tYY,r(:,3),'color','#77AC30');
% temp=size(xseq);
% for i=1:1:temp(3)
%     tsegNow=tseg+i*dt;
%     rNow=reshape(xseq(:,1:3,i),[],3);
%     plot(tsegNow,rNow(:,1),'linestyle','--','color','#0072BD');
%     plot(tsegNow,rNow(:,2),'linestyle','--','color','#D95319');
%     plot(tsegNow,rNow(:,3),'linestyle','--','color','#77AC30');
% end
% legend([l1 l2 l3],{'x','y','z'});
% %
% subplot(2,2,3);
% s1=plot(tYY,theta(:,1),'color','#0072BD');
% hold on;
% s2=plot(tYY,theta(:,2),'color','#D95319');
% s3=plot(tYY,theta(:,3),'color','#77AC30');
% temp=size(xseq);
% for i=1:1:temp(3)
%     tsegNow=tseg+i*dt;
%     dataNow=reshape(xseq(:,4:6,i),[],3);
%     plot(tsegNow,dataNow(:,1),'linestyle','--','color','#0072BD');
%     plot(tsegNow,dataNow(:,2),'linestyle','--','color','#D95319');
%     plot(tsegNow,dataNow(:,3),'linestyle','--','color','#77AC30');
% end
% legend([s1 s2 s3],{'\theta_x','\theta_y','\theta_z'});
% %
% subplot(2,2,2);
% s1=plot(tYY,dr(:,1),'color','#0072BD');
% hold on;
% s2=plot(tYY,dr(:,2),'color','#D95319');
% s3=plot(tYY,dr(:,3),'color','#77AC30');
% temp=size(xseq);
% for i=1:1:temp(3)
%     tsegNow=tseg+i*dt;
%     dataNow=reshape(xseq(:,7:9,i),[],3);
%     plot(tsegNow,dataNow(:,1),'linestyle','--','color','#0072BD');
%     plot(tsegNow,dataNow(:,2),'linestyle','--','color','#D95319');
%     plot(tsegNow,dataNow(:,3),'linestyle','--','color','#77AC30');
% end
% legend([s1 s2 s3],{'dx','dy','dz'});
% %
% subplot(2,2,4);
% s1=plot(tYY,omega(:,1),'color','#0072BD');
% hold on;
% s2=plot(tYY,omega(:,2),'color','#D95319');
% s3=plot(tYY,omega(:,3),'color','#77AC30');
% temp=size(xseq);
% for i=1:1:temp(3)
%     tsegNow=tseg+i*dt;
%     dataNow=reshape(xseq(:,10:12,i),[],3);
%     plot(tsegNow,dataNow(:,1),'linestyle','--','color','#0072BD');
%     plot(tsegNow,dataNow(:,2),'linestyle','--','color','#D95319');
%     plot(tsegNow,dataNow(:,3),'linestyle','--','color','#77AC30');
% end
% legend([s1 s2 s3],{'\omega_x','\omega_y','\omega_z'});

%% open-loop test
% N=floor(0.4/Ts);
% YY=zeros(13,N);
% UU=zeros(6,N);
% 
% xsys=norminal.X;
% r=[0;0;0;0;0;0;0.1;0;0;zeros(3,1);0];
% for i=1:1:N
%     ysys=plantD.C*xsys;
%     xmpc.Plant=xsys;
%     u=mpcmove(mpcPuppy,xmpc,[],r);
%     YY(:,i)=ysys;
%     UU(:,i)=u;
%     xsys=plantD.A*xsys+plantD.B*u;
% end
% figure();
% subplot(3,2,1)
% tLine=(1:1:N)*Ts;
% plot(tLine,YY(1:3,:));
% legend('x','y','z');
% subplot(3,2,2)
% plot(tLine,YY(7:9,:));
% legend('vx','vy','vz');
% subplot(3,2,3)
% plot(tLine,YY(4:6,:));
% legend('\theta_x','\theta_y','\theta_z')
% subplot(3,2,4)
% plot(tLine,YY(10:12,:));
% legend('\omega_x','\omega_y','\omega_z')
% subplot(3,2,5)
% plot(tLine,UU(1:3,:));
% legend('f1_x','f1_y','f1_z')
% subplot(3,2,6)
% plot(tLine,UU(4:6,:));
% legend('f2_x','f2_y','f2_z')






















