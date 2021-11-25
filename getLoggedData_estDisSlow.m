%% load slow loop estDis data and reformat
close all;
load multicore_dataRec_estDis.mat;


time=rt_estU_estxFB_estpW_estReset_slow(1,:);
estU=rt_estU_estxFB_estpW_estReset_slow(2:13,:);
estxFB=rt_estU_estxFB_estpW_estReset_slow(14:26,:);
estpW=rt_estU_estxFB_estpW_estReset_slow(27:38,:);
reset=rt_estU_estxFB_estpW_estReset_slow(39,:);

nanCount=0;
for i=2:1:length(time)
    if sum(isnan(estU(:,i)))>0.5
        estU(:,i)=estU(:,i-1);
        nanCount=nanCount+1;
    end
end

tEnd=time(end);

%%% transform to timeseries format
sim_estU=timeseries(estU,time);
sim_estFB=timeseries(estxFB,time);
sim_pW=timeseries(estpW,time);
sim_reset=timeseries(reset,time);

FileName=[datestr(now,30),'_estSlow'];
save(FileName,'sim_estU','sim_estFB','sim_pW','sim_reset','tEnd');


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












