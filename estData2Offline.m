% reformat recorded rec est inputdata to timeseries format
close all;
clear variables;
load multicore_dataRecEstA.mat;
load multicore_dataRecEstB.mat;
t=rt_acc_omega_RPYOri(1,:);
acc=rt_acc_omega_RPYOri(2:4,:);
omega=rt_acc_omega_RPYOri(5:7,:);
RPYOri=rt_acc_omega_RPYOri(8:10,:);
pArrayB=rt_pArrayB_EN_SW(2:13,:);
EN=rt_pArrayB_EN_SW(14,:);
SW=rt_pArrayB_EN_SW(15:18,:);
tEnd=t(end);

sim_acc=timeseries(acc,t);
sim_omega=timeseries(omega,t);
sim_RPY=timeseries(RPYOri,t);
sim_pArrayB=timeseries(pArrayB,t);
sim_EN=timeseries(EN,t);
sim_SW=timeseries(SW,t);