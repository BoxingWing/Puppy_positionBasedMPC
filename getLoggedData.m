%% fetch data from raspberry pi
mypi=raspberrypi;
getFile(mypi,'raspberrypi_multicore_MPCtest*.mat');
Raspberrypi_MAT_stitcher();
%% data plot
close all;
fileIdx=14;

fileName=sprintf('raspberrypi_multicore_MPCtest_%g__stitched.mat',fileIdx);
dataNow=load(fileName);
rec_xFB_fast=reshape(dataNow.rt_xFB_fast.signals.values,12,[])';
rec_mvOut_fast=reshape(dataNow.rt_mvOut_fast.signals.values,12,[])';
t_fast=dataNow.rt_xFB_fast.time;

rec_xFB_slow=reshape(dataNow.rt_xFB_slow.signals.values,13,[])';
rec_mvOut_slow=dataNow.rt_mvOut_slow.signals.values;
t_slow=dataNow.rt_xFB_slow.time;

figure();
subplot(2,1,1)
plot(t_slow,rec_xFB_slow(:,1),t_fast,rec_xFB_fast(:,1),'.');
legend('slow','fast');
grid on;
ylabel('From fast to slow');
subplot(2,1,2)
plot(t_slow,rec_mvOut_slow(:,1),t_fast,rec_mvOut_fast(:,1),'.');
legend('slow','fast');
grid on;
ylabel('From slow to fast');

figure();
plot(t_fast,rec_xFB_fast(:,4:6));
legend('roll','pitch','yaw')