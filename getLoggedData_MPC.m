%% fetch data from raspberry pi
if exist('mypi','var')
    getFile(mypi,'multicore_dataRec*.mat');
else
    mypi=raspberrypi;
    getFile(mypi,'multicore_dataRec*.mat');
end

%% data plot
close all;
load multicore_dataRec1.mat;
load multicore_dataRec2.mat;
load multicore_dataRec3.mat;
load multicore_dataRec4.mat;
load multicore_dataRec5.mat;
load multicore_dataRec6.mat;
load multicore_dataRec7.mat;
load multicore_dataRec8.mat;
load multicore_dataRec9.mat;
load multicore_dataRec10.mat;
load multicore_dataRec11.mat;
load multicore_dataRec12.mat;

t_slow=rt_x_FB_slow(1,:);
x_FB_slow=rt_x_FB_slow(2:14,:); % pCoM, RPY, vCoM, Omega, g
mvOut_slow=rt_mvOut_slow(2:13,:);
t_fast=rt_mvOut_fast(1,:);
mvOut_fast=rt_mvOut_fast(2:13,:);
x_FB_fast=rt_x_FB_fast(2:13,:);
PendAllLocal=rt_PendAllLocal(2:13,:);

pCoM_slow=x_FB_slow(1:3,:);
RPY_slow=x_FB_slow(4:6,:);
vCoM_slow=x_FB_slow(7:9,:);
Omega_slow=x_FB_slow(10:12,:);
Xmpc_slow=rt_X_mpc_slow(2:end,:);
refSeqOut_slow=rt_refSeqOut_slow(2:end,:);

pCoM_fast=x_FB_fast(1:3,:);
RPY_fast=x_FB_fast(4:6,:);
vCoM_fast=x_FB_fast(7:9,:);
Omega_fast=x_FB_fast(10:12,:);

estOff=rt_EstOff_OscEN_ES(2,:);
OscEN=rt_EstOff_OscEN_ES(3,:);
ES=rt_EstOff_OscEN_ES(4,:);
Xmpc=rt_Xmpc_refP_fast(2:14,:);
refP=rt_Xmpc_refP_fast(15:27,:);

SPLeg=rt_SPLeg_SP_touchInd_LegState(2:5,:); % predefined leg state
SP=rt_SPLeg_SP_touchInd_LegState(6:17,:); % support leg position in the world coordinate
touchInd=rt_SPLeg_SP_touchInd_LegState(18:21,:); % tacktile switch feedback
LegState=rt_SPLeg_SP_touchInd_LegState(22:25,:); % norminal leg state

surVN=rt_surVN_surV1_surV2_surP_headG(2:4,:);
surV1=rt_surVN_surV1_surV2_surP_headG(5:7,:);
surV2=rt_surVN_surV1_surV2_surP_headG(8:10,:);
surP=rt_surVN_surV1_surV2_surP_headG(11:13,:);
headG=rt_surVN_surV1_surV2_surP_headG(14:16,:);

PendAll_fast=rt_PendAll_fast(2:end,:);

% cmd and EN signal plot
figure();
plot(t_fast,estOff,t_fast,OscEN,t_fast,ES);
legend('estOff','OscEN','ES');

tmp=find(OscEN>0.5);
Ns=tmp(1);
tmp=find(ES>0.5);
Ne=tmp(1);
startT=t_fast(Ns)-0.5;
endT=t_fast(Ne);

tmp=find(t_fast>=startT);
startN_f=tmp(1);
tmp=find(t_fast>=endT);
endN_f=tmp(1);
tmp=find(t_slow>=startT);
startN_s=tmp(1);
tmp=find(t_slow>=endT);
endN_s=tmp(1);

% leg state plot
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),SPLeg(1,startN_f:endN_f));
hold on;
plot(t_fast(startN_f:endN_f),LegState(1,startN_f:endN_f))
legend('leg1_touchReal')
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),SPLeg(2,startN_f:endN_f));
hold on;
plot(t_fast(startN_f:endN_f),LegState(2,startN_f:endN_f),'--')
legend('leg2_touchReal')
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),SPLeg(3,startN_f:endN_f));
hold on;
plot(t_fast(startN_f:endN_f),LegState(3,startN_f:endN_f),'--')
legend('leg3_toutchReal')
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),SPLeg(4,startN_f:endN_f));
hold on;
plot(t_fast(startN_f:endN_f),LegState(4,startN_f:endN_f),'--')
legend('leg4_touchReal');

% pCoM plot
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),pCoM_fast(:,startN_f:endN_f));
legend('pCoM\_fast\_x','pCoM\_fast\_y','pCoM\_fast\_z');
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),vCoM_fast(:,startN_f:endN_f));
legend('vCoM\_fast\_x','vCoM\_fast\_y','vCoM\_fast\_z');
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),RPY_fast(:,startN_f:endN_f));
legend('RPY\_fast\_x','RPY\_fast\_y','RPY\_fast\_z');
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),Omega_fast(:,startN_f:endN_f));
legend('\omega\_fast\_x','\omega\_fast\_y','\omega\_fast\_z');

% foot-end plot in leg coordiantes
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),PendAllLocal(1:3,startN_f:endN_f));
ylabel('Leg1');legend('px','py','pz');
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),PendAllLocal(4:6,startN_f:endN_f));
ylabel('Leg2');legend('px','py','pz');
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),PendAllLocal(7:9,startN_f:endN_f));
ylabel('Leg3');legend('px','py','pz');
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),PendAllLocal(10:12,startN_f:endN_f));
ylabel('Leg4');legend('px','py','pz');

% foot-end in world coordinates
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),SP(1:3,startN_f:endN_f));
ylabel('Leg1');legend('px_w','py_w','pz_w');
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),SP(4:6,startN_f:endN_f));
ylabel('Leg2');legend('px_w','py_w','pz_w');
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),SP(7:9,startN_f:endN_f));
ylabel('Leg3');legend('px_w','py_w','pz_w');
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),SP(10:12,startN_f:endN_f));
ylabel('Leg4');legend('px_w','py_w','pz_w');

% PendAll
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),PendAll_fast(1:3,startN_f:endN_f));
ylabel('Leg1');legend('px_endAll','py_endAll','pz_endAll');
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),PendAll_fast(4:6,startN_f:endN_f));
ylabel('Leg2');legend('px_endAll','py_endAll','pz_endAll');
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),PendAll_fast(7:9,startN_f:endN_f));
ylabel('Leg3');legend('px_endAll','py_endAll','pz_endAll');
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),PendAll_fast(10:12,startN_f:endN_f));
ylabel('Leg4');legend('px_endAll','py_endAll','pz_endAll');

% plot planned foot-end forces
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),mvOut_fast(1:3,startN_f:endN_f));
ylabel('Leg1');legend('fx','fy','fz')
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),mvOut_fast(4:6,startN_f:endN_f));
ylabel('Leg2');legend('fx','fy','fz')
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),mvOut_fast(7:9,startN_f:endN_f));
ylabel('Leg3');legend('fx','fy','fz')
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),mvOut_fast(10:12,startN_f:endN_f));
ylabel('Leg4');legend('fx','fy','fz')

% % % plot planned mvOut
% % figure();
% % subplot(2,2,1)
% % plot(t_slow(startN_s:endN_s),mvOut_slow(1:3,startN_s:endN_s));
% % ylabel('Leg1');legend('fx','fy','fz')
% % subplot(2,2,2)
% % plot(t_slow(startN_s:endN_s),mvOut_slow(4:6,startN_s:endN_s));
% % ylabel('Leg2');legend('fx','fy','fz')
% % subplot(2,2,3)
% % plot(t_slow(startN_s:endN_s),mvOut_slow(7:9,startN_s:endN_s));
% % ylabel('Leg3');legend('fx','fy','fz')
% % subplot(2,2,4)
% % plot(t_slow(startN_s:endN_s),mvOut_slow(10:12,startN_s:endN_s));
% % ylabel('Leg4');legend('fx','fy','fz')

% plot mpc predicted states
figure();
subplot(2,2,1)
plot(t_fast(startN_f:endN_f),Xmpc(1:3,startN_f:endN_f));
legend('pre\_px','pre\_py','pre\_pz')
subplot(2,2,2)
plot(t_fast(startN_f:endN_f),Xmpc(4:6,startN_f:endN_f));
legend('pre\_roll','pre\_pitch','pre\_yaw')
subplot(2,2,3)
plot(t_fast(startN_f:endN_f),Xmpc(7:9,startN_f:endN_f));
legend('pre\_vx','pre\_vy','pre\_vz')
subplot(2,2,4)
plot(t_fast(startN_f:endN_f),Xmpc(10:12,startN_f:endN_f));
legend('pre\_omegax','pre\_omegay','pre\_omegaz')












