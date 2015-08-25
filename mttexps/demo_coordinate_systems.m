%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiments of different motion models
%%% CV, CA, Singer, Current, CT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

load measurement.mat
NumMC = 10;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);
 
%%
%%% 
%%% 运动模型
%%% Singer model
T = 0.02;                                                               % 时间采样间隔
Alpha = 0.15;
Amax = 5*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_z = b1model.singer(Alpha,T,Amax,P0,Pmax);
hd_mtnmodel_si = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_lcsf = mttfilter.lcsf(hd_mtnmodel_si,hd_msmodel);
hd_cmkf = mttfilter.cmkf(hd_mtnmodel_si,hd_msmodel);
%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat(xtrue(:,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2,100^2),[1,1,NumMC]);
[Xhat_LCSF,Phat_LCSF] = hd_lcsf.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波
[Xhat_CMKF,Phat_CMKF] = hd_cmkf.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position
IndexPosRMSE_LCSF = rmse(Xhat_LCSF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CMKF = rmse(Xhat_CMKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelRMSE_LCSF = rmse(Xhat_LCSF(2:3:9,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CMKF = rmse(Xhat_CMKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccRMSE_LCSF = rmse(Xhat_LCSF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_CMKF = rmse(Xhat_CMKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexPosGAE_LCSF = gae(Xhat_LCSF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CMKF = gae(Xhat_CMKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelGAE_LCSF = gae(Xhat_LCSF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CMKF = gae(Xhat_CMKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccGAE_LCSF = gae(Xhat_LCSF(3:3:9,:,:)-repmat(AccTarget(:,3:end),[1,1,NumMC]));
IndexAccGAE_CMKF = gae(Xhat_CMKF(3:3:9,:,:)-repmat(AccTarget(:,3:end),[1,1,NumMC]));

%%
save demo_coordinate_systems.mat

%%
%%% filted trajectory
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_LCSF(1,:,kk),Xhat_LCSF(4,:,kk),Xhat_LCSF(7,:,kk),'-r');
plot3(Xhat_CMKF(1,:,kk),Xhat_CMKF(4,:,kk),Xhat_CMKF(7,:,kk),'-b');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','视线系','惯性系','观测')
% view(3)

%%
%%% RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexPosRMSE_LCSF,'-r');
plot(time(3:end),IndexPosRMSE_CMKF,'--b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('视线系','惯性系')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_LCSF,'-r');
plot(time(3:end),IndexVelRMSE_CMKF,'--b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('视线系','惯性系')

figure(13)
hold on
plot(time(3:end),IndexAccRMSE_LCSF,'-r');
plot(time(3:end),IndexAccRMSE_CMKF,'--b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('视线系','惯性系')

%%
%%% error differences
figure(21)
hold on
plot(time(3:end),IndexPosRMSE_LCSF-IndexPosRMSE_CMKF,'-r');
% plot(time(3:end),IndexPosRMSE_CMKF,'--b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
% legend('视线系','惯性系')

figure(22)
hold on
plot(time(3:end),IndexVelRMSE_LCSF-IndexVelRMSE_CMKF,'-r');
% plot(time(3:end),IndexVelRMSE_CMKF,'--b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
% legend('视线系','惯性系')

figure(23)
hold on
plot(time(3:end),IndexAccRMSE_LCSF-IndexAccRMSE_CMKF,'-r');
% plot(time(3:end),IndexAccRMSE_CMKF,'--b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
% legend('视线系','惯性系')

%%
%%% GAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_LCSF,'-r');
plot(time(3:end),IndexPosGAE_CMKF,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('视线系','惯性系')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_LCSF,'-r');
plot(time(3:end),IndexVelGAE_CMKF,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,50])
legend('视线系','惯性系')

figure(33)
hold on
plot(time(3:end),IndexAccGAE_LCSF,'-r');
plot(time(3:end),IndexAccGAE_CMKF,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,50])
legend('视线系','惯性系')


