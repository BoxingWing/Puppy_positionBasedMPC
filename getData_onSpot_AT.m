%% fetch data from raspberry pi
mypi=raspberrypi;
getFile(mypi,'multicore_dataRec*.mat');
getFile(mypi,'~/MATLAB_ws/R2021b/raspi_multicore_MPCtest_AT.log'); % get the log file

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
load multicore_dataRec13.mat;

%%% recorded variables
t_slow=rt_x_FB_slow(1,:);
pCoM_slow=rt_x_FB_slow(2:4,:);
RPY_slow=rt_x_FB_slow(5:7,:);
vCoM_slow=rt_x_FB_slow(8:10,:);
OmegaW_slow=rt_x_FB_slow(11:13,:);
mvOut_slow=rt_mvOut_qpStatus_slow(2:13,:);
qpStatus_slow=rt_mvOut_qpStatus_slow(end,:);
refSeq2_slow=rt_refSeq2_refP_slow(2:14,:);
refP_slow=rt_refSeq2_refP_slow(15:end,:);
pLbas=rt_pLbas_fLst_fLRes_pLAdm_pLDelta(2:13,:);
fLst=rt_pLbas_fLst_fLRes_pLAdm_pLDelta(14:25,:);
fLRes=rt_pLbas_fLst_fLRes_pLAdm_pLDelta(26:37,:);
pLAdm=rt_pLbas_fLst_fLRes_pLAdm_pLDelta(38:49,:);
pLDelta=rt_pLbas_fLst_fLRes_pLAdm_pLDelta(50:61,:);
touchInd=rt_estSPLeg_estSP_LegState(2:5,:);
LegState=rt_estSPLeg_estSP_LegState(end-3:end,:);
pArrayLFK=rt_pLm(2:end,:);

%refOVnew=rt_refOVbew_refMVnew_slow(2:14,:);
%refMVnew=rt_refOVbew_refMVnew_slow(15:end,:);

t_fast=rt_mvOut_fast(1,:);
mvOut_fast=rt_mvOut_fast(2:end,:);
pCoM_fast=rt_x_FB_fast(2:4,:);
RPY_fast=rt_x_FB_fast(5:7,:);
vCoM_fast=rt_x_FB_fast(8:10,:);
OmegaW_fast=rt_x_FB_fast(11:13,:);
vxPercent_fast=rt_vxPercent_vyPercent_wzPercent(2,:);
vyPercent_fast=rt_vxPercent_vyPercent_wzPercent(3,:);
wzPercent_fast=rt_vxPercent_vyPercent_wzPercent(4,:);
% PendAllLocal_fast=rt_PendAllLocal(2:end,:);
spMPC_fast=rt_spMPC_fast(2:end,:);

estSPLeg_fast=rt_estSPLeg_estSP_LegState(2:5,:);
estSP_fast=rt_estSPLeg_estSP_LegState(6:17,:);
estXbar=rt_estXbar_slow(2:end,:);
estDis=estXbar(14:19,:);

EstOff=rt_EstOff_OscEN_mpcSTOP_ES(2,:);
OscEN=rt_EstOff_OscEN_mpcSTOP_ES(3,:);
mpcSTOP=rt_EstOff_OscEN_mpcSTOP_ES(4,:);
ES=rt_EstOff_OscEN_mpcSTOP_ES(5,:);
pCoM_mpcPre_slow=rt_xMPCOut_slow(2:4,:);
RPY_mpcPre_slow=rt_xMPCOut_slow(5:7,:);
vCoM_mpcPre_slow=rt_xMPCOut_slow(8:10,:);
omegaW_mpcPre_slow=rt_xMPCOut_slow(11:13,:);

pWRec=estSP_fast;
LegStateRec=LegState;
xRefRec=refP_slow; % t_slow
xFBRec=rt_x_FB_fast(2:13,:);
DisableRec=mpcSTOP;

% sim_pW=timeseries(pWRec,t_fast);
% sim_LegState=timeseries(LegStateRec,t_fast);
% sim_xRef=timeseries(xRefRec,t_slow);
% sim_xFB=timeseries(xFBRec,t_fast);
% sim_disable=timeseries(DisableRec,t_fast);
% sim_tEnd=t_fast(end);
% save('CentroidTestData','sim_pW','sim_LegState','sim_xRef','sim_xFB','sim_disable','sim_tEnd');

% qpXnew=rt_qpXnew_qpUnew_qpExitFlag_fast(2:14,:);
% qpUnew=rt_qpXnew_qpUnew_qpExitFlag_fast(15:26,:);
% qpExitFlag=rt_qpXnew_qpUnew_qpExitFlag_fast(27,:);

startT=0;
endT=10;
startNs=1;
endNs=2;
startNf=1;
endNf=2;

tmp=find(mpcSTOP<=0);
if isempty(tmp)
    tmp=1;   
end
startNf=tmp(1);
tmp=find(t_slow>=t_fast(tmp(1)));
startNs=tmp(1);
tmp=find(ES>=1);
if isempty(tmp)
    tmp=length(t_fast);
end
endT=t_fast(tmp(1));
endNf=tmp(1);
tmp=find(t_slow>=endT);
if isempty(tmp)
    tmp=length(t_slow);
end
endNs=tmp(1);

%%% check the multirate delay
% figure();
% stairs(t_slow,mvOut_slow(1,:));
% hold on;
% stairs(t_fast,mvOut_fast(1,:));
% legend('mvOut1\_slow','mvOut1\_fast');

%%% check the fLst and the pLDelta, fLst is the output of the balance control module
figure()
subplot(2,4,1)
plot(t_fast(startNf:endNf),fLst(1,startNf:endNf), ...
    t_fast(startNf:endNf),fLst(2,startNf:endNf), ...
    t_fast(startNf:endNf),fLst(3,startNf:endNf));
ylabel('virtualF_Leg1');legend('fx','fy','fz')
subplot(2,4,2)
plot(t_fast(startNf:endNf),pLDelta(1,startNf:endNf), ...
    t_fast(startNf:endNf),pLDelta(2,startNf:endNf), ...
    t_fast(startNf:endNf),pLDelta(3,startNf:endNf));
ylabel('virtualF_Leg1');legend('pLdeX','pLdeY','pLdeZ')

subplot(2,4,3)
plot(t_fast(startNf:endNf),fLst(4,startNf:endNf),...
    t_fast(startNf:endNf),fLst(5,startNf:endNf),...
    t_fast(startNf:endNf),fLst(6,startNf:endNf));
ylabel('virtualF_Leg2');legend('fx','fy','fz')
subplot(2,4,4)
plot(t_fast(startNf:endNf),pLDelta(4,startNf:endNf),...
    t_fast(startNf:endNf),pLDelta(5,startNf:endNf),...
    t_fast(startNf:endNf),pLDelta(6,startNf:endNf));
ylabel('virtualF_Leg2');legend('pLdeX','pLdeY','pLdeZ');

subplot(2,4,5)
plot(t_fast(startNf:endNf),fLst(7,startNf:endNf), ...
    t_fast(startNf:endNf),fLst(8,startNf:endNf), ...
    t_fast(startNf:endNf),fLst(9,startNf:endNf));
ylabel('virtualF_Leg3');legend('fx','fy','fz')
subplot(2,4,6)
plot(t_fast(startNf:endNf),pLDelta(7,startNf:endNf), ...
    t_fast(startNf:endNf),pLDelta(8,startNf:endNf), ...
    t_fast(startNf:endNf),pLDelta(9,startNf:endNf));
ylabel('virtualF_Leg3');legend('pLdeX','pLdeY','pLdeZ');

subplot(2,4,7)
plot(t_fast(startNf:endNf),fLst(10,startNf:endNf), ...
    t_fast(startNf:endNf),fLst(11,startNf:endNf),...
    t_fast(startNf:endNf),fLst(12,startNf:endNf));
ylabel('virtualF_Leg4');legend('fx','fy','fz')
subplot(2,4,8)
plot(t_fast(startNf:endNf),pLDelta(10,startNf:endNf), ...
    t_fast(startNf:endNf),pLDelta(11,startNf:endNf),...
    t_fast(startNf:endNf),pLDelta(12,startNf:endNf));
ylabel('virtualF_Leg4');legend('pLdeX','pLdeY','pLdeZ')

%%% chekc the mv
figure()
subplot(2,2,1)
plot(t_fast(startNf:endNf),mvOut_fast(1,startNf:endNf), ...
    t_fast(startNf:endNf),mvOut_fast(2,startNf:endNf),t_fast(startNf:endNf),mvOut_fast(3,startNf:endNf));
ylabel('Leg1');legend('fx','fy','fz')
subplot(2,2,2)
plot(t_fast(startNf:endNf),mvOut_fast(4,startNf:endNf),...
    t_fast(startNf:endNf),mvOut_fast(5,startNf:endNf),...
    t_fast(startNf:endNf),mvOut_fast(6,startNf:endNf));
ylabel('Leg2');legend('fx','fy','fz')
subplot(2,2,3)
plot(t_fast(startNf:endNf),mvOut_fast(7,startNf:endNf), ...
    t_fast(startNf:endNf),mvOut_fast(8,startNf:endNf), ...
    t_fast(startNf:endNf),mvOut_fast(9,startNf:endNf));
ylabel('Leg3');legend('fx','fy','fz')
subplot(2,2,4)
plot(t_fast(startNf:endNf),mvOut_fast(10,startNf:endNf), ...
    t_fast(startNf:endNf),mvOut_fast(11,startNf:endNf),...
    t_fast(startNf:endNf),mvOut_fast(12,startNf:endNf));
ylabel('Leg4');legend('fx','fy','fz')

%%% check payload balance in z direction
figure();
subplot(2,1,1)
plot(t_fast(startNf:endNf),fLRes(3,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),fLRes(12,startNf:endNf));
legend('leg1z','leg4z');ylabel('payload in leg frame(N)')
subplot(2,1,2)
plot(t_fast(startNf:endNf),fLRes(6,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),fLRes(9,startNf:endNf));
legend('leg2z','leg3z');ylabel('payload in leg frame(N)')

%%% check the trunk state
figure();
subplot(2,2,1)
plot(t_fast(startNf:endNf),pCoM_fast(:,startNf:endNf));ylabel('pCoM');legend('x','y','z');
subplot(2,2,2)
plot(t_fast(startNf:endNf),RPY_fast(:,startNf:endNf)/pi*180);ylabel('RPY');legend('x','y','z');
subplot(2,2,3)
plot(t_fast(startNf:endNf),vCoM_fast(:,startNf:endNf));ylabel('vCoM');legend('x','y','z');
subplot(2,2,4)
plot(t_fast(startNf:endNf),OmegaW_fast(:,startNf:endNf));ylabel('omegaW');legend('x','y','z');

%%% check the control cmd and qpStatus
figure();
subplot(2,1,1)
plot(t_fast,EstOff,t_fast,OscEN,t_fast,mpcSTOP,t_fast,ES);
legend('EstOff','OscEN','mpcSTOP','ES');
ylabel('Control Cmd');
subplot(2,1,2)
stairs(t_slow,qpStatus_slow);
ylabel('qpStatus');

%%% check orientation control
figure();
subplot(3,1,1)
plot(t_slow(startNs:endNs),RPY_slow(1,startNs:endNs), ...
    t_slow(startNs:endNs),RPY_mpcPre_slow(1,startNs:endNs))
legend('real','predicted')
ylabel('roll');
subplot(3,1,2)
plot(t_slow(startNs:endNs),RPY_slow(2,startNs:endNs), ...
    t_slow(startNs:endNs),RPY_mpcPre_slow(2,startNs:endNs))
legend('real','predicted')
ylabel('pitch');
subplot(3,1,3)
plot(t_slow(startNs:endNs),RPY_slow(3,startNs:endNs), ...
    t_slow(startNs:endNs),RPY_mpcPre_slow(3,startNs:endNs))
legend('real','predicted')
ylabel('yaw');

%%% check the CoM control
figure();
subplot(3,1,1)
plot(t_slow(startNs:endNs),pCoM_slow(1,startNs:endNs), ...
    t_slow(startNs:endNs),pCoM_mpcPre_slow(1,startNs:endNs))
legend('real','predicted')
ylabel('pCoMx');
subplot(3,1,2)
plot(t_slow(startNs:endNs),pCoM_slow(2,startNs:endNs), ...
    t_slow(startNs:endNs),pCoM_mpcPre_slow(2,startNs:endNs))
legend('real','predicted')
ylabel('pCoMy');
subplot(3,1,3)
plot(t_slow(startNs:endNs),pCoM_slow(3,startNs:endNs), ...
    t_slow(startNs:endNs),pCoM_mpcPre_slow(3,startNs:endNs))
legend('real','predicted')
ylabel('pCoMz');

%%% check foot position interpolation
% figure();
% subplot(3,1,1)
% stairs(t_fast(startNf:endNf),pArrayLAdm(1,startNf:endNf))
% hold on;
% stairs(t_fast(startNf:endNf),pArrayL(1,startNf:endNf)*1000);
% legend('MPC_cmd','interpolated');
% ylabel('Leg1x');
% subplot(3,1,2)
% stairs(t_fast(startNf:endNf),pArrayLAdm(2,startNf:endNf));hold on;
% stairs(t_fast(startNf:endNf),pArrayL(2,startNf:endNf)*1000);
% legend('MPC_cmd','interpolated');ylabel('Leg1y');
% subplot(3,1,3)
% stairs(t_fast(startNf:endNf),pArrayLAdm(3,startNf:endNf));
% hold on;
% stairs(t_fast(startNf:endNf),pArrayL(3,startNf:endNf)*1000);
% legend('MPC_cmd','interpolated');ylabel('Leg1z');

%%% check the ref cmd
figure();
subplot(2,2,1)
plot(t_slow(startNs:endNs),refSeq2_slow(4:6,startNs:endNs)/pi*180);
ylabel('refSeq2');legend('SitaX','SitaY','SitaZ');
subplot(2,2,2)
plot(t_slow(startNs:endNs),refSeq2_slow(1:3,startNs:endNs));
ylabel('refSeq2');legend('pCoMX','pCoMY','pCoMZ');
subplot(2,2,3)
plot(t_slow(startNs:endNs),refP_slow(4:6,startNs:endNs));
ylabel('refP');legend('SitaX','SitaY','SitaZ');
subplot(2,2,4)
plot(t_slow(startNs:endNs),refP_slow(1:3,startNs:endNs));
ylabel('refP');legend('pCoMX','pCoMY','pCoMZ');

%%% check xFB vs xRef
figure();
subplot(2,3,1)
plot(t_slow(startNs:endNs),refP_slow(1,startNs:endNs));
hold on;
plot(t_fast(startNf:endNf),pCoM_fast(1,startNf:endNf));
ylabel('pCoMx vx Ref');
subplot(2,3,2)
plot(t_slow(startNs:endNs),refP_slow(2,startNs:endNs));
hold on;
plot(t_fast(startNf:endNf),pCoM_fast(2,startNf:endNf));
ylabel('pCoMy vx Ref');
subplot(2,3,3)
plot(t_slow(startNs:endNs),refP_slow(3,startNs:endNs));
hold on;
plot(t_fast(startNf:endNf),pCoM_fast(3,startNf:endNf));
ylabel('pCoMz vx Ref');
subplot(2,3,4)
plot(t_slow(startNs:endNs),refSeq2_slow(4,startNs:endNs));
hold on;
plot(t_fast(startNf:endNf),RPY_fast(1,startNf:endNf));
ylabel('roll Ref');
subplot(2,3,5)
plot(t_slow(startNs:endNs),refSeq2_slow(5,startNs:endNs));
hold on;
plot(t_fast(startNf:endNf),RPY_fast(2,startNf:endNf));
ylabel('pitch Ref');
subplot(2,3,6)
plot(t_slow(startNs:endNs),refSeq2_slow(6,startNs:endNs));
hold on;
plot(t_fast(startNf:endNf),RPY_fast(3,startNf:endNf));
ylabel('yaw Ref');



%%% check commanded foot position
figure();
subplot(2,2,1)
yyaxis left;
plot(t_fast(startNf:endNf),pLAdm(1,startNf:endNf), ...
    t_fast(startNf:endNf),pLAdm(2,startNf:endNf),t_fast(startNf:endNf),pLAdm(3,startNf:endNf));
ylabel('leg1\_pL');
yyaxis right
plot(t_fast(startNf:endNf),LegState(1,startNf:endNf),...
    t_fast(startNf:endNf),touchInd(1,startNf:endNf));
ylim([-0.1 1.1]);
legend('x','y','z','LegState','touchInd');

subplot(2,2,2)
yyaxis left
plot(t_fast(startNf:endNf),pLAdm(4,startNf:endNf),...
    t_fast(startNf:endNf),pLAdm(5,startNf:endNf),t_fast(startNf:endNf),pLAdm(6,startNf:endNf));
ylabel('leg2\_pL');
yyaxis right
plot(t_fast(startNf:endNf),LegState(2,startNf:endNf),t_fast(startNf:endNf),touchInd(2,startNf:endNf));
ylim([-0.1 1.1]);
legend('x','y','z','LegState','touchInd');

subplot(2,2,3)
yyaxis left
plot(t_fast(startNf:endNf),pLAdm(7,startNf:endNf),t_fast(startNf:endNf), ...
    pLAdm(8,startNf:endNf),t_fast(startNf:endNf),pLAdm(9,startNf:endNf));
ylabel('leg3\_pL');
yyaxis right
plot(t_fast(startNf:endNf),LegState(3,startNf:endNf),t_fast(startNf:endNf),touchInd(3,startNf:endNf));
ylim([-0.1 1.1]);
legend('x','y','z','LegState','touchInd');

subplot(2,2,4)
yyaxis left
plot(t_fast(startNf:endNf),pLAdm(10,startNf:endNf),t_fast(startNf:endNf),pLAdm(11,startNf:endNf), ...
    t_fast(startNf:endNf),pLAdm(12,startNf:endNf));
yyaxis right
plot(t_fast(startNf:endNf),LegState(4,startNf:endNf),t_fast(startNf:endNf),touchInd(4,startNf:endNf));
ylim([-0.1 1.1]);
ylabel('leg4\_pL');
legend('x','y','z','LegState','touchInd');

%%% check the swing trajectory tracking
% figure();
% subplot(2,2,1)
% plot(t_fast(startNf:endNf),pArrayLAdm(3,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(3,startNf:endNf), ...
%     t_fast(startNf:endNf),pArrayLFK(3,startNf:endNf)/1000);
% legend('Adm','pL','pLFK');
% ylabel('Leg1');
% subplot(2,2,2)
% plot(t_fast(startNf:endNf),pArrayLAdm(6,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(6,startNf:endNf), ...
%     t_fast(startNf:endNf),pArrayLFK(6,startNf:endNf)/1000);
% legend('Adm','pL','pLFK');
% ylabel('Leg2');
% subplot(2,2,3)
% plot(t_fast(startNf:endNf),pArrayLAdm(9,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(9,startNf:endNf), ...
%     t_fast(startNf:endNf),pArrayLFK(9,startNf:endNf)/1000);
% legend('Adm','pL','pLFK');
% ylabel('Leg3');
% subplot(2,2,4)
% plot(t_fast(startNf:endNf),pArrayLAdm(12,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(12,startNf:endNf), ...
%     t_fast(startNf:endNf),pArrayLFK(12,startNf:endNf)/1000);
% legend('Adm','pL','pLFK');
% ylabel('Leg4');

%%% check joystick input command
% figure();
% plot(t_fast(startNf:endNf),vxPercent_fast(startNf:endNf),...
%     t_fast(startNf:endNf),vyPercent_fast(startNf:endNf),...
%     t_fast(startNf:endNf),wzPercent_fast(startNf:endNf));
% legend('vxPer','vyPer','wzPer');

%%% check qp output
% figure();
% subplot(4,3,1)
% plot(t_fast(startNf:endNf),qpUnew(1,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(1,startNf:endNf));
% legend('qpLeg1x','mpcLeg1x')
% subplot(4,3,2)
% plot(t_fast(startNf:endNf),qpUnew(2,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(2,startNf:endNf));
% legend('qpLeg1y','mpcLeg1y')
% subplot(4,3,3)
% plot(t_fast(startNf:endNf),qpUnew(3,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(3,startNf:endNf));
% legend('qpLeg1z','mpcLeg1z')
% subplot(4,3,4)
% plot(t_fast(startNf:endNf),qpUnew(4,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(4,startNf:endNf));
% legend('qpLeg2x','mpcLeg2x')
% subplot(4,3,5)
% plot(t_fast(startNf:endNf),qpUnew(5,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(5,startNf:endNf));
% legend('qpLeg2y','mpcLeg2y')
% subplot(4,3,6)
% plot(t_fast(startNf:endNf),qpUnew(6,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(6,startNf:endNf));
% legend('qpLeg2z','mpcLeg2z')
% subplot(4,3,7)
% plot(t_fast(startNf:endNf),qpUnew(7,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(7,startNf:endNf));
% legend('qpLeg3x','mpcLeg3x')
% subplot(4,3,8)
% plot(t_fast(startNf:endNf),qpUnew(8,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(8,startNf:endNf));
% legend('qpLeg3y','mpcLeg3y')
% subplot(4,3,9)
% plot(t_fast(startNf:endNf),qpUnew(9,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(9,startNf:endNf));
% legend('qpLeg3z','mpcLeg3z')
% subplot(4,3,10)
% plot(t_fast(startNf:endNf),qpUnew(10,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(10,startNf:endNf));
% legend('qpLeg4x','mpcLeg4x')
% subplot(4,3,11)
% plot(t_fast(startNf:endNf),qpUnew(11,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(11,startNf:endNf));
% legend('qpLeg4y','mpcLeg4y')
% subplot(4,3,12)
% plot(t_fast(startNf:endNf),qpUnew(12,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),mvOut_fast(12,startNf:endNf));
% legend('qpLeg4z','mpcLeg4z')

%%% check the estDis
figure();
subplot(2,3,1)
plot(t_slow(startNs:endNs),estDis(1,startNs:endNs));
ylabel('estDis\_pCoMx')
subplot(2,3,2)
plot(t_slow(startNs:endNs),estDis(2,startNs:endNs));
ylabel('estDis\_pCoMy')
subplot(2,3,3)
plot(t_slow(startNs:endNs),estDis(3,startNs:endNs));
ylabel('estDis\_pCoMz')
subplot(2,3,4)
plot(t_slow(startNs:endNs),estDis(4,startNs:endNs));
ylabel('estDis\_RPYx')
subplot(2,3,5)
plot(t_slow(startNs:endNs),estDis(5,startNs:endNs));
ylabel('estDis\_RPYy')
subplot(2,3,6)
plot(t_slow(startNs:endNs),estDis(6,startNs:endNs));
ylabel('estDis\_RPYz')

%%% check estL and pST
% figure();
% subplot(2,3,1);
% plot(t_fast(startNf:endNf),estL(1,startNf:endNf)/m/0.19);
% hold on;
% plot(t_fast(startNf:endNf),vCoM_fast(2,startNf:endNf));
% legend('estL\_x/m/H','vY');
% subplot(2,3,2);
% plot(t_fast(startNf:endNf),estL(2,startNf:endNf));
% hold on;
% plot(t_fast(startNf:endNf),vCoM_fast(1,startNf:endNf));
% legend('estL\_y','vX');
% subplot(2,3,3);
% plot(t_fast(startNf:endNf),estL(3,startNf:endNf));
% ylabel('estL\_z');
% subplot(2,3,4);
% plot(t_fast(startNf:endNf),pST(1,startNf:endNf));
% ylabel('pST\_x');
% subplot(2,3,5);
% plot(t_fast(startNf:endNf),pST(2,startNf:endNf));
% ylabel('pST\_y');
% subplot(2,3,6);
% plot(t_fast(startNf:endNf),pST(3,startNf:endNf));
% ylabel('pST\_z');

%%% plot foot end in the world frame
% figure();
% p1=plot(estSP_fast(1,startNf:endNf-200),estSP_fast(2,startNf:endNf-200),'color',[0 0.4470 0.7410]);
% axis equal; grid on;
% hold on;
% p2=plot(estSP_fast(4,startNf:endNf-200),estSP_fast(5,startNf:endNf-200),'color',[0.8500 0.3250 0.0980]);
% p3=plot(estSP_fast(7,startNf:endNf-200),estSP_fast(8,startNf:endNf-200),'color',[0.9290 0.6940 0.1250]);
% p4=plot(estSP_fast(10,startNf:endNf-200),estSP_fast(11,startNf:endNf-200),'color',[0.4940 0.1840 0.5560]);
% plot(estSP_fast(1,startNf),estSP_fast(2,startNf),'o','color',[0 0.4470 0.7410],'markersize',10);
% plot(estSP_fast(4,startNf),estSP_fast(5,startNf),'o','color',[0.8500 0.3250 0.0980],'markersize',10);
% plot(estSP_fast(7,startNf),estSP_fast(8,startNf),'o','color',[0.9290 0.6940 0.1250],'markersize',10);
% plot(estSP_fast(10,startNf),estSP_fast(11,startNf),'o','color',[0.4940 0.1840 0.5560],'markersize',10);
% 
% 
% legend([p1 p2 p3 p4],{'1','2','3','4'});













