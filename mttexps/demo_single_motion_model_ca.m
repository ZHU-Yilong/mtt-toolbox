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

% load ('missiletrajectory.mat','PosMissile','VelMissile','AccMissile','xtrue');

%%
% %%% number of time step
% NumStep = size(PosMissile,2);
% time = (1:1:NumStep)*T;
% %%% number of Monte Carlo
% NumMC = 100;
% %%% measurement precision parameters
% SigmaRan = 15;
% SigmaAzi = deg2rad(0.3);
% SigmaEle = deg2rad(0.3);
% %%% target state
% PosTarget = xtrue(1:3:7,1:NumStep);
% VelTarget = xtrue(2:3:8,1:NumStep);
% AccTarget = xtrue(3:3:9,1:NumStep);
% %%% relative position
% PosRelative = PosTarget-PosMissile;
% [AziRelative,EleRelative,RanRelative] = cart2sph(PosRelative(1,:),PosRelative(2,:),PosRelative(3,:));
% %%% line of sight (LOS) or antenna heading angle
% LosAzimuth = AziRelative;
% LosElevation = EleRelative;
% %%% measurement true value
% MeasureTrue = [RanRelative;zeros(1,NumStep);zeros(1,NumStep)];
% %%% radar measurement vector
% RadarMeasure = repmat(MeasureTrue,[1,1,NumMC]) + ...
%     [randn(1,NumStep,NumMC)*SigmaRan;
%      randn(1,NumStep,NumMC)*SigmaAzi;
%      randn(1,NumStep,NumMC)*SigmaEle];
% %%% pseudo measurement vector
% Measure1 = RadarMeasure+repmat([zeros(1,NumStep);LosAzimuth;LosElevation],[1,1,NumMC]);
% DimMeasure = 3;
% PseudoMeasure = zeros(DimMeasure,NumStep,NumMC);
% [Measure2(1,:,:),Measure2(2,:,:),Measure2(3,:,:)] = sph2cart(Measure1(2,:,:),Measure1(3,:,:),Measure1(1,:,:));
% Measure3 = Measure2+repmat(PosMissile,[1,1,NumMC]);
% [PseudoMeasure(2,:,:),PseudoMeasure(3,:,:),PseudoMeasure(1,:,:)] = ...
%     cart2sph(Measure3(1,:,:),Measure3(2,:,:),Measure3(3,:,:));

load ('measurement.mat')
NumMC = 10;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

%%
%%% 
%%% 运动模型
%%% CV model 1
Swx = 1e2;                                                              % x轴过程噪声功率谱密度
Swy = 1e2;                                                              % y轴过程噪声功率谱密度
Swz = 10;
T = 0.02;                                                               % 时间采样间隔
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.cv(Swz,T);
hd_mtnmodel_cv = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% CA model
Swx = 1e2;
Swy = 1e2;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_ca1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% 
Swx = 1e3;
Swy = 1e3;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_ca2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% 
Swx = 1e4;
Swy = 1e4;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_ca3 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_cv = mttfilter.cmkf(hd_mtnmodel_cv,hd_msmodel);
hd_ca1 = mttfilter.cmkf(hd_mtnmodel_ca1,hd_msmodel);
hd_ca2 = mttfilter.cmkf(hd_mtnmodel_ca2,hd_msmodel);
hd_ca3 = mttfilter.cmkf(hd_mtnmodel_ca3,hd_msmodel);
%%% 滤波过程
flag = 1;   % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
[Xhat_CV,Phat_CV] = hd_cv.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
%%%
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
[Xhat_CA1,Phat_CA1] = hd_ca1.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波
[Xhat_CA2,Phat_CA2] = hd_ca2.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
[Xhat_CA3,Phat_CA3] = hd_ca3.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position
IndexPosRMSE_CV = rmse(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));

IndexPosRMSE_CA1 = rmse(Xhat_CA1(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CA2 = rmse(Xhat_CA2(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CA3 = rmse(Xhat_CA3(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelRMSE_CV = rmse(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CA1 = rmse(Xhat_CA1(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CA2 = rmse(Xhat_CA2(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CA3 = rmse(Xhat_CA3(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccRMSE_CA1 = rmse(Xhat_CA1(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_CA2 = rmse(Xhat_CA2(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_CA3 = rmse(Xhat_CA3(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexPosGAE_CV = gae(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CA1 = gae(Xhat_CA1(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CA2 = gae(Xhat_CA2(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CA3 = gae(Xhat_CA3(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelGAE_CV = gae(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CA1 = gae(Xhat_CA1(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CA2 = gae(Xhat_CA2(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CA3 = gae(Xhat_CA3(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccGAE_CA1 = gae(Xhat_CA1(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_CA2 = gae(Xhat_CA2(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_CA3 = gae(Xhat_CA3(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
save demo_single_motion_model_ca.mat

%%
%%% filted trajectory
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_CV(1,:,kk),Xhat_CV(3,:,kk),Xhat_CV(5,:,kk),'-r');
plot3(Xhat_CA1(1,:,kk),Xhat_CA1(4,:,kk),Xhat_CA1(7,:,kk),'-b');
plot3(Xhat_CA2(1,:,kk),Xhat_CA2(4,:,kk),Xhat_CA2(7,:,kk),'-c');
plot3(Xhat_CA3(1,:,kk),Xhat_CA3(4,:,kk),Xhat_CA3(7,:,kk),'-m');
scatter3(MeasureX,MeasureY,MeasureZ);
hold off
axis equal, box on, grid on
legend('真实','CV模型','CA1模型','CA2模型','CA3模型','观测')
% view(3)

%%
%%% RMSE
figure(11)
hold on
plot(IndexPosRMSE_CV,'-r');
plot(IndexPosRMSE_CA1,'-b');
plot(IndexPosRMSE_CA2,'-c');
plot(IndexPosRMSE_CA3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,40])
legend('CV模型','CA1模型','CA2模型','CA3模型')

figure(12)
hold on
plot(IndexVelRMSE_CV,'-r');
plot(IndexVelRMSE_CA1,'-b');
plot(IndexVelRMSE_CA2,'-c');
plot(IndexVelRMSE_CA3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CV模型','CA1模型','CA2模型','CA3模型')

figure(13)
hold on
plot(IndexAccRMSE_CA1,'-b');
plot(IndexAccRMSE_CA2,'-c');
plot(IndexAccRMSE_CA3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,50])
legend('CA1模型','CA2模型','CA3模型')

%%
%%% GAE
figure(31)
hold on
plot(IndexPosGAE_CV,'-r');
plot(IndexPosGAE_CA1,'-b');
plot(IndexPosGAE_CA2,'-c');
plot(IndexPosGAE_CA3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,200])
legend('CV模型','CA1模型','CA2模型','CA3模型')

figure(32)
hold on
plot(IndexVelGAE_CV,'-r');
plot(IndexVelGAE_CA1,'-b');
plot(IndexVelGAE_CA2,'-c');
plot(IndexVelGAE_CA3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CV模型','CA1模型','CA2模型','CA3模型')

figure(33)
hold on
plot(IndexAccGAE_CA1,'-b');
plot(IndexAccGAE_CA2,'-c');
plot(IndexAccGAE_CA3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CA1模型','CA2模型','CA3模型')
