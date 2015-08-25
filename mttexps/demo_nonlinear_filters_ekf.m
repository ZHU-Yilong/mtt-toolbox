%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiments of different nonlinear filters
%%% EKF, CMKF
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

load ('missiletrajectory.mat','PosMissile','VelMissile','AccMissile','xtrue','T');

%%
%%% number of time step
NumStep = size(PosMissile,2);
time = (1:1:NumStep)*T;
%%% number of Monte Carlo
NumMC = 100;
%%% measurement precision parameters
SigmaRan = 100;
SigmaAzi = deg2rad(1);
SigmaEle = deg2rad(1);
%%% target state
PosTarget = xtrue(1:3:7,1:NumStep);
VelTarget = xtrue(2:3:8,1:NumStep);
AccTarget = xtrue(3:3:9,1:NumStep);
%%% relative position
PosRelative = PosTarget-PosMissile;
[AziRelative,EleRelative,RanRelative] = cart2sph(PosRelative(1,:),PosRelative(2,:),PosRelative(3,:));
%%% line of sight (LOS) or antenna heading angle
LosAzimuth = AziRelative;
LosElevation = EleRelative;
%%% measurement true value
MeasureTrue = [RanRelative;zeros(1,NumStep);zeros(1,NumStep)];
%%% radar measurement vector
RadarMeasure = repmat(MeasureTrue,[1,1,NumMC]) + ...
    [randn(1,NumStep,NumMC)*SigmaRan;
     randn(1,NumStep,NumMC)*SigmaAzi;
     randn(1,NumStep,NumMC)*SigmaEle];
%%% pseudo measurement vector
Measure1 = RadarMeasure+repmat([zeros(1,NumStep);LosAzimuth;LosElevation],[1,1,NumMC]);
DimMeasure = 3;
PseudoMeasure = zeros(DimMeasure,NumStep,NumMC);
[Measure2(1,:,:),Measure2(2,:,:),Measure2(3,:,:)] = sph2cart(Measure1(2,:,:),Measure1(3,:,:),Measure1(1,:,:));
Measure3 = Measure2+repmat(PosMissile,[1,1,NumMC]);
[PseudoMeasure(2,:,:),PseudoMeasure(3,:,:),PseudoMeasure(1,:,:)] = ...
    cart2sph(Measure3(1,:,:),Measure3(2,:,:),Measure3(3,:,:));

save measurement_high_noise.mat
load measurement_high_noise.mat
NumMC = 10;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

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
%%% Singer model
Alpha = 0.15;
Amax = 5*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_si = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_cv_cmkf = mttfilter.cmkf(hd_mtnmodel_cv,hd_msmodel);
hd_cv_ekf = mttfilter.ekf(hd_mtnmodel_cv,hd_msmodel);

%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
Xhat_CV_CMKF = hd_cv_cmkf.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
Xhat_CV_EKF = hd_cv_ekf.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);


%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position 
IndexPosRMSE_CV_CMKF = rmse(Xhat_CV_CMKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CV_EKF = rmse(Xhat_CV_EKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity 
IndexVelRMSE_CV_CMKF = rmse(Xhat_CV_CMKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CV_EKF = rmse(Xhat_CV_EKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));


%%
%%% index No.3 GAE / LGAE
%%% position 
IndexPosGAE_CV_CMKF = gae(Xhat_CV_CMKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CV_EKF = gae(Xhat_CV_EKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity 
IndexVelGAE_CV_CMKF = gae(Xhat_CV_CMKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CV_EKF = gae(Xhat_CV_EKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));


%%
save demo_nonlinear_filters_ekf.mat

%%
%%% filted trajectory
figure(01)
kk = 2;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'--k');
plot3(Xhat_CV_CMKF(1,:,kk),Xhat_CV_CMKF(3,:,kk),Xhat_CV_CMKF(5,:,kk),'-r');
plot3(Xhat_CV_EKF(1,:,kk),Xhat_CV_EKF(3,:,kk),Xhat_CV_EKF(5,:,kk),'-b');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','CMKF','EKF','观测')
% view(3)

%%
%%% RMSE
figure(11)
hold on
plot(IndexPosRMSE_CV_CMKF,'-r');
plot(IndexPosRMSE_CV_EKF,'-b');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,200])
legend('CMKF','EKF')

figure(12)
hold on
plot(IndexVelRMSE_CV_CMKF,'-r');
plot(IndexVelRMSE_CV_EKF,'-b');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CMKF','EKF')

%%
%%% GAE
figure(31)
hold on
plot(IndexPosGAE_CV_CMKF,'-r');
plot(IndexPosGAE_CV_EKF,'-b');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,200])
legend('CMKF','EKF')

figure(32)
hold on
plot(IndexVelGAE_CV_CMKF,'-r');
plot(IndexVelGAE_CV_EKF,'-b');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CMKF','EKF')


