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

load ('measurement.mat')

%%
%%% 
%%% 运动模型
%%% CV model 1
Swx = 1e4;                                                              % x轴过程噪声功率谱密度
Swy = 1e4;                                                              % y轴过程噪声功率谱密度
Swz = 10;
T = 0.02;                                                               % 时间采样间隔
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.cv(Swz,T);
hd_mtnmodel_cv = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% CA model
Swx = 1e3;
Swy = 1e3;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_ca = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% Singer model 1
Alpha = 0.15;
Amax = 3*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_si1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% Singer model 2
Alpha = 0.15;
Amax = 5*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_si2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% Singer model 3
Alpha = 0.15;
Amax = 7*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_si3 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_cv = mttfilter.cmkf(hd_mtnmodel_cv,hd_msmodel);
hd_ca = mttfilter.cmkf(hd_mtnmodel_ca,hd_msmodel);
hd_si1 = mttfilter.cmkf(hd_mtnmodel_si1,hd_msmodel);
hd_si2 = mttfilter.cmkf(hd_mtnmodel_si2,hd_msmodel);
hd_si3 = mttfilter.cmkf(hd_mtnmodel_si3,hd_msmodel);
%%% 滤波过程
flag = 1;   % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
[Xhat_CV,Phat_CV] = hd_cv.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
%%%
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
Xhat_CA = hd_ca.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波
Xhat_SI1 = hd_si1.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
Xhat_SI2 = hd_si2.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
Xhat_SI3 = hd_si3.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position
IndexPosRMSE_CV = rmse(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CA = rmse(Xhat_CA(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_SI1 = rmse(Xhat_SI1(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_SI2 = rmse(Xhat_SI2(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_SI3 = rmse(Xhat_SI3(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelRMSE_CV = rmse(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CA = rmse(Xhat_CA(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI1 = rmse(Xhat_SI1(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI2 = rmse(Xhat_SI2(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI3 = rmse(Xhat_SI3(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccRMSE_CA = rmse(Xhat_CA(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_SI1 = rmse(Xhat_SI1(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_SI2 = rmse(Xhat_SI2(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_SI3 = rmse(Xhat_SI3(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexPosGAE_CV = gae(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CA = gae(Xhat_CA(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_SI1 = gae(Xhat_SI1(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_SI2 = gae(Xhat_SI2(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_SI3 = gae(Xhat_SI3(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelGAE_CV = gae(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CA = gae(Xhat_CA(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI1 = gae(Xhat_SI1(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI2 = gae(Xhat_SI2(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI3 = gae(Xhat_SI3(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccGAE_CA = gae(Xhat_CA(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_SI1 = gae(Xhat_SI1(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_SI2 = gae(Xhat_SI2(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_SI3 = gae(Xhat_SI3(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
save demo_single_motion_model_singer_amax.mat

%%
%%% filted trajectory
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_CV(1,:,kk),Xhat_CV(3,:,kk),Xhat_CV(5,:,kk),'-r');
plot3(Xhat_CA(1,:,kk),Xhat_CA(4,:,kk),Xhat_CA(7,:,kk),'-y');
plot3(Xhat_SI1(1,:,kk),Xhat_SI1(4,:,kk),Xhat_SI1(7,:,kk),'-b');
plot3(Xhat_SI2(1,:,kk),Xhat_SI2(4,:,kk),Xhat_SI2(7,:,kk),'-c');
plot3(Xhat_SI3(1,:,kk),Xhat_SI3(4,:,kk),Xhat_SI3(7,:,kk),'-m');
scatter3(MeasureX,MeasureY,MeasureZ);
hold off
axis equal, box on, grid on
legend('真实','CV模型','CA模型','SI1模型','SI2模型','SI3模型','观测')
% view(3)

%%
%%% RMSE
figure(11)
hold on
plot(IndexPosRMSE_CV,'-r');
plot(IndexPosRMSE_CA,'-y');
plot(IndexPosRMSE_SI1,'-b');
plot(IndexPosRMSE_SI2,'-c');
plot(IndexPosRMSE_SI3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,20])
legend('CV模型','CA模型','SI1模型','SI2模型','SI3模型')

figure(12)
hold on
plot(IndexVelRMSE_CV,'-r');
plot(IndexVelRMSE_CA,'-y');
plot(IndexVelRMSE_SI1,'-b');
plot(IndexVelRMSE_SI2,'-c');
plot(IndexVelRMSE_SI3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,40])
legend('CV模型','CA模型','SI1模型','SI2模型','SI3模型')

figure(13)
hold on
plot(IndexAccRMSE_CA,'-y');
plot(IndexAccRMSE_SI1,'-b');
plot(IndexAccRMSE_SI2,'-c');
plot(IndexAccRMSE_SI3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,50])
legend('CA模型','SI1模型','SI2模型','SI3模型')

%%
%%% GAE
figure(31)
hold on
plot(IndexPosGAE_CV,'-r');
plot(IndexPosGAE_CA,'-y');
plot(IndexPosGAE_SI1,'-b');
plot(IndexPosGAE_SI2,'-c');
plot(IndexPosGAE_SI3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,20])
legend('CV模型','CA模型','SI1模型','SI2模型','SI3模型')

figure(32)
hold on
plot(IndexVelGAE_CV,'-r');
plot(IndexVelGAE_CA,'-y');
plot(IndexVelGAE_SI1,'-b');
plot(IndexVelGAE_SI2,'-c');
plot(IndexVelGAE_SI3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,40])
legend('CV模型','CA模型','SI1模型','SI2模型','SI3模型')

figure(33)
hold on
plot(IndexAccGAE_CA,'-y');
plot(IndexAccGAE_SI1,'-b');
plot(IndexAccGAE_SI2,'-c');
plot(IndexAccGAE_SI3,'-m');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,50])
legend('CA模型','SI1模型','SI2模型','SI3模型')
