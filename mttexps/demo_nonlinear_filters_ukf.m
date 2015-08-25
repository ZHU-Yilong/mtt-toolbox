%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiments of different nonlinear filters
%%% EKF, SOEKF, IEKF, UKF, PF, CMKF
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
NumMC = 2;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

%%
%%% 
%%% 运动模型
%%% CV model
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
hd_cv_ukf = mttfilter.ukf(hd_mtnmodel_cv,hd_msmodel);

hd_si_cmkf = mttfilter.cmkf(hd_mtnmodel_si,hd_msmodel);
hd_si_ukf = mttfilter.ukf(hd_mtnmodel_si,hd_msmodel);

%%% 滤波过程
flag = 1;                                                               % 进度条标示
% Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
% Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
%%% CV model
% Xhat_CV_CMKF = hd_cv_cmkf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_CV_EKF = hd_cv_ekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_CV_SOEKF = hd_cv_soekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_CV_IEKF = hd_cv_iekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_CV_UKF = hd_cv_ukf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_CV_PF = hd_cv_pf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
%%% 
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
%%% Singer model
Xhat_SI_CMKF = hd_si_cmkf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波
% Xhat_SI_EKF = hd_si_ekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_SI_SOEKF = hd_si_soekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_SI_IEKF = hd_si_iekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
Xhat_SI_UKF = hd_si_ukf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_SI_PF = hd_si_pf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position CV model
% IndexPosRMSE_CV_CMKF = rmse(Xhat_CV_CMKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_CV_EKF = rmse(Xhat_CV_EKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_CV_SOEKF = rmse(Xhat_CV_SOEKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_CV_IEKF = rmse(Xhat_CV_IEKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_CV_UKF = rmse(Xhat_CV_UKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_CV_PF = rmse(Xhat_CV_PF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% position Singer model
IndexPosRMSE_SI_CMKF = rmse(Xhat_SI_CMKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_EKF = rmse(Xhat_SI_EKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_SOEKF = rmse(Xhat_SI_SOKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_IEKF = rmse(Xhat_SI_IEKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_SI_UKF = rmse(Xhat_SI_UKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_PF = rmse(Xhat_SI_PF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity CV model
% IndexVelRMSE_CV_CMKF = rmse(Xhat_CV_CMKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_CV_EKF = rmse(Xhat_CV_EKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_CV_SOEKF = rmse(Xhat_CV_SOEKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_CV_IEKF = rmse(Xhat_CV_IEKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_CV_UKF = rmse(Xhat_CV_UKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_CV_PF = rmse(Xhat_CV_PF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% velocity Singer model
IndexVelRMSE_SI_CMKF = rmse(Xhat_SI_CMKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_EKF = rmse(Xhat_SI_EKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_SOEKF = rmse(Xhat_SI_SOEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_IEKF = rmse(Xhat_SI_IEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI_UKF = rmse(Xhat_SI_UKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_PF = rmse(Xhat_SI_PF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccRMSE_SI_CMKF = rmse(Xhat_SI_CMKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_EKF = rmse(Xhat_SI_EKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_SOEKF = rmse(Xhat_SI_SOEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_IEKF = rmse(Xhat_SI_IEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_SI_UKF = rmse(Xhat_SI_UKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_PF = rmse(Xhat_SI_PF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position CV model
% IndexPosGAE_CV_CMKF = gae(Xhat_CV_CMKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_CV_EKF = gae(Xhat_CV_EKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_CV_SOEKF = gae(Xhat_CV_SOEKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_CV_IEKF = gae(Xhat_CV_IEKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_CV_UKF = gae(Xhat_CV_UKF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_CV_PF = gae(Xhat_CV_PF(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% position Singer model
IndexPosGAE_SI_CMKF = gae(Xhat_SI_CMKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_EKF = gae(Xhat_SI_EKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_SOEKF = gae(Xhat_SI_SOKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_IEKF = gae(Xhat_SI_IEKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_SI_UKF = gae(Xhat_SI_UKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_PF = gae(Xhat_SI_PF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity CV model
% IndexVelGAE_CV_CMKF = gae(Xhat_CV_CMKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_CV_EKF = gae(Xhat_CV_EKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_CV_SOEKF = gae(Xhat_CV_SOEKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_CV_IEKF = gae(Xhat_CV_IEKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_CV_UKF = gae(Xhat_CV_UKF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_CV_PF = gae(Xhat_CV_PF(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% velocity Singer model
IndexVelGAE_SI_CMKF = gae(Xhat_SI_CMKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_EKF = gae(Xhat_SI_EKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_SOEKF = gae(Xhat_SI_SOEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_IEKF = gae(Xhat_SI_IEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI_UKF = gae(Xhat_SI_UKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_PF = gae(Xhat_SI_PF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccGAE_SI_CMKF = gae(Xhat_SI_CMKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_EKF = gae(Xhat_SI_EKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_SOEKF = gae(Xhat_SI_SOEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_IEKF = gae(Xhat_SI_IEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_SI_UKF = gae(Xhat_SI_UKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_PF = gae(Xhat_SI_PF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
save demo_nonlinear_filters_siukf.mat

%%
%%% filted trajectory
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
% figure(01)
% hold on
% plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'--k');
% plot3(Xhat_CV_CMKF(1,:,kk),Xhat_CV_CMKF(3,:,kk),Xhat_CV_CMKF(5,:,kk),'-r');
% plot3(Xhat_CV_EKF(1,:,kk),Xhat_CV_EKF(3,:,kk),Xhat_CV_EKF(5,:,kk),'-b');
% plot3(Xhat_CV_SOEKF(1,:,kk),Xhat_CV_SOEKF(3,:,kk),Xhat_CV_SOEKF(5,:,kk),'-c');
% plot3(Xhat_CV_IEKF(1,:,kk),Xhat_CV_IEKF(3,:,kk),Xhat_CV_IEKF(5,:,kk),'-m');
% plot3(Xhat_CV_UKF(1,:,kk),Xhat_CV_UKF(3,:,kk),Xhat_CV_UKF(5,:,kk),'-y');
% plot3(Xhat_CV_PF(1,:,kk),Xhat_CV_PF(3,:,kk),Xhat_CV_PF(5,:,kk),'-m');
% scatter3(MeasureX,MeasureY,MeasureZ);
% hold off
% axis equal, box on, grid on
% legend('真实','CMKF','EKF','SOEKF','IEKF','UKF','PF','观测')
% view(3)

figure(02)
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'--k');
plot3(Xhat_SI_CMKF(1,:,kk),Xhat_SI_CMKF(4,:,kk),Xhat_SI_CMKF(7,:,kk),'-r');
% plot3(Xhat_SI_EKF(1,:,kk),Xhat_SI_EKF(3,:,kk),Xhat_SI_EKF(5,:,kk),'-b');
% plot3(Xhat_SI_SOEKF(1,:,kk),Xhat_SI_SOEKF(3,:,kk),Xhat_SI_SOEKF(5,:,kk),'-c');
% plot3(Xhat_SI_IEKF(1,:,kk),Xhat_SI_IEKF(3,:,kk),Xhat_SI_IEKF(5,:,kk),'-m');
plot3(Xhat_SI_UKF(1,:,kk),Xhat_SI_UKF(4,:,kk),Xhat_SI_UKF(7,:,kk),'-y');
% plot3(Xhat_SI_PF(1,:,kk),Xhat_SI_PF(3,:,kk),Xhat_SI_PF(5,:,kk),'-m');
scatter3(MeasureX,MeasureY,MeasureZ);
hold off
axis equal, box on, grid on
% legend('真实','CMKF','EKF','SOEKF','IEKF','UKF','PF','观测')
legend('真实','CMKF','UKF','观测')
% view(3)

%%
%%% RMSE
% figure(11)
% hold on
% plot(IndexPosRMSE_CV_CMKF,'-r');
% plot(IndexPosRMSE_CV_EKF,'-b');
% plot(IndexPosRMSE_CV_SOKF,'-c');
% plot(IndexPosRMSE_CV_IEKF,'-m');
% plot(IndexPosRMSE_CV_UKF,'-y');
% plot(IndexPosRMSE_CV_PF,'-y');
% hold off
% grid on, box on
% xlim([0,NumStep])
% ylim([0,200])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')

% figure(12)
% hold on
% plot(IndexVelRMSE_CV_CMKF,'-r');
% plot(IndexVelRMSE_CV_EKF,'-b');
% plot(IndexVelRMSE_CV_SOEKF,'-c');
% plot(IndexVelRMSE_CV_IEKF,'-m');
% plot(IndexVelRMSE_CV_UKF,'-y');
% plot(IndexVelRMSE_CV_PF,'-y');
% hold off
% grid on, box on
% xlim([0,NumStep])
% ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')

figure(13)
hold on
plot(IndexPosRMSE_SI_CMKF,'-r');
% plot(IndexPosRMSE_SI_EKF,'-b');
% plot(IndexPosRMSE_SI_SOKF,'-c');
% plot(IndexPosRMSE_SI_IEKF,'-m');
plot(IndexPosRMSE_SI_UKF,'-y');
% plot(IndexPosRMSE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,200])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','UKF')
% 
figure(14)
hold on
plot(IndexVelRMSE_SI_CMKF,'-r');
% plot(IndexVelRMSE_SI_EKF,'-b');
% plot(IndexVelRMSE_SI_SOEKF,'-c');
% plot(IndexVelRMSE_SI_IEKF,'-m');
plot(IndexVelRMSE_SI_UKF,'-y');
% plot(IndexVelRMSE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','UKF')
% 
figure(15)
hold on
plot(IndexAccRMSE_SI_CMKF,'-r');
% plot(IndexAccRMSE_SI_EKF,'-b');
% plot(IndexAccRMSE_SI_SOKF,'-c');
% plot(IndexAccRMSE_SI_IEKF,'-m');
plot(IndexAccRMSE_SI_UKF,'-y');
% plot(IndexAccRMSE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','UKF')

%%
%%% GAE
% figure(31)
% hold on
% plot(IndexPosGAE_CV_CMKF,'-r');
% % plot(IndexPosGAE_CV_EKF,'-b');
% % plot(IndexPosGAE_CV_SOKF,'-c');
% % plot(IndexPosGAE_CV_IEKF,'-m');
% plot(IndexPosGAE_CV_UKF,'-y');
% % plot(IndexPosGAE_CV_PF,'-y');
% hold off
% grid on, box on
% xlim([0,NumStep])
% % ylim([0,200])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')

% figure(32)
% hold on
% plot(IndexVelGAE_CV_CMKF,'-r');
% % plot(IndexVelGAE_CV_EKF,'-b');
% % plot(IndexVelGAE_CV_SOEKF,'-c');
% % plot(IndexVelGAE_CV_IEKF,'-m');
% plot(IndexVelGAE_CV_UKF,'-y');
% % plot(IndexVelGAE_CV_PF,'-y');
% hold off
% grid on, box on
% xlim([0,NumStep])
% % ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')

figure(33)
hold on
plot(IndexPosGAE_SI_CMKF,'-r');
% plot(IndexPosGAE_SI_EKF,'-b');
% plot(IndexPosGAE_SI_SOKF,'-c');
% plot(IndexPosGAE_SI_IEKF,'-m');
plot(IndexPosGAE_SI_UKF,'-y');
% plot(IndexPosGAE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,NumStep])
ylim([0,200])
legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
% 
figure(34)
hold on
plot(IndexVelGAE_SI_CMKF,'-r');
% plot(IndexVelGAE_SI_EKF,'-b');
% plot(IndexVelGAE_SI_SOEKF,'-c');
% plot(IndexVelGAE_SI_IEKF,'-m');
plot(IndexVelGAE_SI_UKF,'-y');
% plot(IndexVelGAE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
% 
figure(35)
hold on
plot(IndexAccGAE_SI_CMKF,'-r');
% plot(IndexAccGAE_SI_EKF,'-b');
% plot(IndexAccGAE_SI_SOKF,'-c');
% plot(IndexAccGAE_SI_IEKF,'-m');
plot(IndexAccGAE_SI_UKF,'-y');
% plot(IndexAccGAE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,NumStep])
% ylim([0,100])
legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
