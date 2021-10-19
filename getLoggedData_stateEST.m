%% fetch data from raspberry pi
mypi=raspberrypi;
getFile(mypi,'multicore_dataRecEst*.mat');

%% data plot
close all;
load multicore_dataRecEstA.mat;
load multicore_dataRecEstB.mat;
load multicore_dataRecEstC.mat;


time=rt_acc_omega_RPYOri(1,:);
acc=rt_acc_omega_RPYOri(2:4,:);
omega=rt_acc_omega_RPYOri(5:7,:);
RPYOri=rt_acc_omega_RPYOri(8:10,:);

pArrayB=rt_pArrayB_EN_SW(2:13,:);
EN=rt_pArrayB_EN_SW(14,:);
SW=rt_pArrayB_EN_SW(15:18,:);

uMPC=rt_uMPC(2:end,:);

tEnd=time(end);

%%% transform to timeseries format
sim_acc=timeseries(acc,time);
sim_omega=timeseries(omega,time);
sim_RPY=timeseries(RPYOri,time);
sim_pArrayB=timeseries(pArrayB,time);
sim_EN=timeseries(EN,time);
sim_SW=timeseries(SW,time);
sim_uMPC=timeseries(uMPC,time);

FileName=datestr(now,30);
save(FileName,'sim_acc','sim_omega','sim_RPY','sim_pArrayB',...
    'sim_EN','sim_SW','sim_uMPC','tEnd');


% startT=11;
% endT=84;
% tmp=find(time>=startT);
% startN=tmp(1);
% tmp=find(time>=endT);
% endN=tmp(1);
% %%% EN plot
% figure();
% plot(time,cmd_offEN,time,cmd_step, ...
%     time,cmd_forward,time,cmd_left,time,cmd_right);
% legend('OffEN','StepEN','forwardEN','LeftEN','RightEN')
% %%% tacktile switch plot
% figure();
% subplot(2,1,1)
% plot(time(startN:endN),SPLeg(1,startN:endN),time(startN:endN),SPLeg(4,startN:endN));
% legend('SP1','SP4');
% subplot(2,1,2)
% plot(time(startN:endN),SPLeg(2,startN:endN),time(startN:endN),SPLeg(3,startN:endN));
% legend('SP2','SP3');
% 
% %%% pCoM plot
% figure()
% plot(time(startN:endN),pCoM(1,startN:endN),time(startN:endN),pCoM(2,startN:endN),time(startN:endN),pCoM(3,startN:endN));
% legend('pCoMx','pCoMy','pCoMz');












