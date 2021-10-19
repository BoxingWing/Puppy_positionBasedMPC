%% fetch data from raspberry pi
mypi=raspberrypi;
getFile(mypi,'multicore_dataRec*.mat');
getFile(mypi,'~/MATLAB_ws/R2021a/raspberrypi_multicore_MPCtest_OnSpot_OffsetFree_UD.log'); % get the log file

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
load multicore_dataRec14.mat;
load multicore_dataRec15.mat;
load multicore_dataRec16.mat;

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
pArrayLAdm=rt_pArrayLAdm(2:end,:);
pArrayL=rt_PendAllLocal(2:end,:);
touchInd=rt_touchInd_LegState(2:5,:);
LegState=rt_touchInd_LegState(6:end,:);
pArrayLFK=rt_pLFKfast(2:end,:);

t_fast=rt_mvOut_fast(1,:);
mvOut_fast=rt_mvOut_fast(2:end,:);
pCoM_fast=rt_x_FB_fast(2:4,:);
RPY_fast=rt_x_FB_fast(5:7,:);
vCoM_fast=rt_x_FB_fast(8:10,:);
OmegaW_fast=rt_x_FB_fast(11:13,:);
vxPercent_fast=rt_vxPercent_vyPercent_wzPercent(2,:);
vyPercent_fast=rt_vxPercent_vyPercent_wzPercent(3,:);
wzPercent_fast=rt_vxPercent_vyPercent_wzPercent(4,:);
PendAllLocal_fast=rt_PendAllLocal(2:end,:);
spMPC_fast=rt_spMPC_fast(2:end,:);
pArrayLnor=rt_pLnor_fast(2:end,:); % parrayL direct from mpc out

EstOff=rt_EstOff_OscEN_mpcSTOP_ES(2,:);
OscEN=rt_EstOff_OscEN_mpcSTOP_ES(3,:);
mpcSTOP=rt_EstOff_OscEN_mpcSTOP_ES(4,:);
ES=rt_EstOff_OscEN_mpcSTOP_ES(5,:);
pCoM_mpcPre_slow=rt_xMPCOut_slow(2:4,:);
RPY_mpcPre_slow=rt_xMPCOut_slow(5:7,:);
vCoM_mpcPre_slow=rt_xMPCOut_slow(8:10,:);
omegaW_mpcPre_slow=rt_xMPCOut_slow(11:13,:);

qpXnew=rt_qpXnew_qpUnew_qpExitFlag_fast(2:14,:);
qpUnew=rt_qpXnew_qpUnew_qpExitFlag_fast(15:26,:);
qpExitFlag=rt_qpXnew_qpUnew_qpExitFlag_fast(27,:);

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

%%% check the mv
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
% figure();
% subplot(2,2,1)
% plot(t_slow(startNs:endNs),refSeq2_slow(4:6,startNs:endNs)/pi*180);
% ylabel('refSeq2');legend('SitaX','SitaY','SitaZ');
% subplot(2,2,2)
% plot(t_slow(startNs:endNs),refSeq2_slow(1:3,startNs:endNs));
% ylabel('refSeq2');legend('pCoMX','pCoMY','pCoMZ');
% subplot(2,2,3)
% plot(t_slow(startNs:endNs),refP_slow(4:6,startNs:endNs));
% ylabel('refP');legend('SitaX','SitaY','SitaZ');
% subplot(2,2,4)
% plot(t_slow(startNs:endNs),refP_slow(1:3,startNs:endNs));
% ylabel('refP');legend('pCoMX','pCoMY','pCoMZ');

%%% check foot position
figure();
subplot(2,2,1)
yyaxis left;
plot(t_fast(startNf:endNf),pArrayL(1,startNf:endNf), ...
    t_fast(startNf:endNf),pArrayL(2,startNf:endNf),t_fast(startNf:endNf),pArrayL(3,startNf:endNf));
ylabel('leg1\_pL');
yyaxis right
plot(t_fast(startNf:endNf),LegState(1,startNf:endNf),...
    t_fast(startNf:endNf),touchInd(1,startNf:endNf));
ylim([-0.1 1.1]);
legend('x','y','z','LegState','touchInd');

subplot(2,2,2)
yyaxis left
plot(t_fast(startNf:endNf),pArrayL(4,startNf:endNf),...
    t_fast(startNf:endNf),pArrayL(5,startNf:endNf),t_fast(startNf:endNf),pArrayL(6,startNf:endNf));
ylabel('leg2\_pL');
yyaxis right
plot(t_fast(startNf:endNf),LegState(2,startNf:endNf),t_fast(startNf:endNf),touchInd(2,startNf:endNf));
ylim([-0.1 1.1]);
legend('x','y','z','LegState','touchInd');

subplot(2,2,3)
yyaxis left
plot(t_fast(startNf:endNf),pArrayL(7,startNf:endNf),t_fast(startNf:endNf), ...
    pArrayL(8,startNf:endNf),t_fast(startNf:endNf),pArrayL(9,startNf:endNf));
ylabel('leg3\_pL');
yyaxis right
plot(t_fast(startNf:endNf),LegState(3,startNf:endNf),t_fast(startNf:endNf),touchInd(3,startNf:endNf));
ylim([-0.1 1.1]);
legend('x','y','z','LegState','touchInd');

subplot(2,2,4)
yyaxis left
plot(t_fast(startNf:endNf),pArrayL(10,startNf:endNf),t_fast(startNf:endNf),pArrayL(11,startNf:endNf), ...
    t_fast(startNf:endNf),pArrayL(12,startNf:endNf));
yyaxis right
plot(t_fast(startNf:endNf),LegState(4,startNf:endNf),t_fast(startNf:endNf),touchInd(4,startNf:endNf));
ylim([-0.1 1.1]);
ylabel('leg4\_pL');
legend('x','y','z','LegState','touchInd');

%%% check the swing trajectory tracking
figure();
subplot(2,2,1)
plot(t_fast(startNf:endNf),pArrayLAdm(3,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(3,startNf:endNf), ...
    t_fast(startNf:endNf),pArrayLFK(3,startNf:endNf)/1000);
legend('Adm','pL','pLFK');
ylabel('Leg1');
subplot(2,2,2)
plot(t_fast(startNf:endNf),pArrayLAdm(6,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(6,startNf:endNf), ...
    t_fast(startNf:endNf),pArrayLFK(6,startNf:endNf)/1000);
legend('Adm','pL','pLFK');
ylabel('Leg2');
subplot(2,2,3)
plot(t_fast(startNf:endNf),pArrayLAdm(9,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(9,startNf:endNf), ...
    t_fast(startNf:endNf),pArrayLFK(9,startNf:endNf)/1000);
legend('Adm','pL','pLFK');
ylabel('Leg3');
subplot(2,2,4)
plot(t_fast(startNf:endNf),pArrayLAdm(12,startNf:endNf)/1000,t_fast(startNf:endNf),pArrayL(12,startNf:endNf), ...
    t_fast(startNf:endNf),pArrayLFK(12,startNf:endNf)/1000);
legend('Adm','pL','pLFK');
ylabel('Leg4');

%%% check joystick input command
% figure();
% plot(t_fast(startNf:endNf),vxPercent_fast(startNf:endNf),...
%     t_fast(startNf:endNf),vyPercent_fast(startNf:endNf),...
%     t_fast(startNf:endNf),wzPercent_fast(startNf:endNf));
% legend('vxPer','vyPer','wzPer');

%%% check qp output
figure();
subplot(4,3,1)
plot(t_fast(startNf:endNf),qpUnew(1,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(1,startNf:endNf));
legend('qpLeg1x','mpcLeg1x')
subplot(4,3,2)
plot(t_fast(startNf:endNf),qpUnew(2,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(2,startNf:endNf));
legend('qpLeg1y','mpcLeg1y')
subplot(4,3,3)
plot(t_fast(startNf:endNf),qpUnew(3,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(3,startNf:endNf));
legend('qpLeg1z','mpcLeg1z')
subplot(4,3,4)
plot(t_fast(startNf:endNf),qpUnew(4,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(4,startNf:endNf));
legend('qpLeg2x','mpcLeg2x')
subplot(4,3,5)
plot(t_fast(startNf:endNf),qpUnew(5,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(5,startNf:endNf));
legend('qpLeg2y','mpcLeg2y')
subplot(4,3,6)
plot(t_fast(startNf:endNf),qpUnew(6,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(6,startNf:endNf));
legend('qpLeg2z','mpcLeg2z')
subplot(4,3,7)
plot(t_fast(startNf:endNf),qpUnew(7,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(7,startNf:endNf));
legend('qpLeg3x','mpcLeg3x')
subplot(4,3,8)
plot(t_fast(startNf:endNf),qpUnew(8,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(8,startNf:endNf));
legend('qpLeg3y','mpcLeg3y')
subplot(4,3,9)
plot(t_fast(startNf:endNf),qpUnew(9,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(9,startNf:endNf));
legend('qpLeg3z','mpcLeg3z')
subplot(4,3,10)
plot(t_fast(startNf:endNf),qpUnew(10,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(10,startNf:endNf));
legend('qpLeg4x','mpcLeg4x')
subplot(4,3,11)
plot(t_fast(startNf:endNf),qpUnew(11,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(11,startNf:endNf));
legend('qpLeg4y','mpcLeg4y')
subplot(4,3,12)
plot(t_fast(startNf:endNf),qpUnew(12,startNf:endNf));
hold on;
plot(t_fast(startNf:endNf),mvOut_fast(12,startNf:endNf));
legend('qpLeg4z','mpcLeg4z')




% figure();
% subplot(2,1,1)
% plot(t_slow,rec_xFB_slow(:,1),t_fast,rec_xFB_fast(:,1),'.');
% legend('px\_slow','py\_fast');
% grid on;
% ylabel('From fast to slow');
% subplot(2,1,2)
% plot(t_slow,rec_mvOut_slow(:,1),t_fast,rec_mvOut_fast(:,1),'.');
% legend('slow','fast');
% grid on;
% ylabel('From slow to fast');
% 
% figure();
% plot(t_fast,rec_xFB_fast(:,4:6)/pi*180);
% legend('roll','pitch','yaw')
% 
% figure();
% subplot(2,2,1)
% plot(rec_pendAllLocal(1,:),rec_pendAllLocal(3,:));
% axis equal; grid on;legend('leg1');
% subplot(2,2,2)
% plot3(rec_pendAllLocal(4,:),rec_pendAllLocal(5,:),rec_pendAllLocal(6,:));
% axis equal; grid on;legend('leg2');
% subplot(2,2,3)
% plot3(rec_pendAllLocal(7,:),rec_pendAllLocal(8,:),rec_pendAllLocal(9,:));
% axis equal; grid on;legend('leg3');
% subplot(2,2,4)
% plot3(rec_pendAllLocal(10,:),rec_pendAllLocal(11,:),rec_pendAllLocal(12,:));
% axis equal; grid on;legend('leg4');

% figure();
% plot(t_slow,rec_xFB_slow(:,3),t_fast,rec_xFB_fast(:,3));
% legend('slow','fast')










