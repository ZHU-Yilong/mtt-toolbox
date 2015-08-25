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

load measurement.mat
NumMC = 10;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

%%
%%% 
%%% 运动模型
%%% CV model
Swz = 10;
T = 0.02;                                                               % 时间采样间隔
hd_mtnmodel_z = b1model.cv(Swz,T);
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
hd_si_cmkf = mttfilter.cmkf(hd_mtnmodel_si,hd_msmodel);
hd_si_ekf = mttfilter.ekf(hd_mtnmodel_si,hd_msmodel);
% hd_si_soekf = mttfilter.soekf(hd_mtnmodel_si,hd_msmodel);
% hd_si_iekf = mttfilter.iekf(hd_mtnmodel_si,hd_msmodel);
hd_si_ukf = mttfilter.ukf(hd_mtnmodel_si,hd_msmodel);
% hd_si_pf = mttfilter.pf(hd_mtnmodel_si,hd_msmodel);

%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
Xhat_SI_CMKF = hd_si_cmkf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波
Xhat_SI_EKF = hd_si_ekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_SI_SOEKF = hd_si_soekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_SI_IEKF = hd_si_iekf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
Xhat_SI_UKF = hd_si_ukf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
% Xhat_SI_PF = hd_si_pf.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position 
IndexPosRMSE_SI_CMKF = rmse(Xhat_SI_CMKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_SI_EKF = rmse(Xhat_SI_EKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_SOEKF = rmse(Xhat_SI_SOKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_IEKF = rmse(Xhat_SI_IEKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_SI_UKF = rmse(Xhat_SI_UKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI_PF = rmse(Xhat_SI_PF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelRMSE_SI_CMKF = rmse(Xhat_SI_CMKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI_EKF = rmse(Xhat_SI_EKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_SOEKF = rmse(Xhat_SI_SOEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_IEKF = rmse(Xhat_SI_IEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI_UKF = rmse(Xhat_SI_UKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI_PF = rmse(Xhat_SI_PF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccRMSE_SI_CMKF = rmse(Xhat_SI_CMKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_SI_EKF = rmse(Xhat_SI_EKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_SOEKF = rmse(Xhat_SI_SOEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_IEKF = rmse(Xhat_SI_IEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_SI_UKF = rmse(Xhat_SI_UKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI_PF = rmse(Xhat_SI_PF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexPosGAE_SI_CMKF = gae(Xhat_SI_CMKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_SI_EKF = gae(Xhat_SI_EKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_SOEKF = gae(Xhat_SI_SOKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_IEKF = gae(Xhat_SI_IEKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_SI_UKF = gae(Xhat_SI_UKF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI_PF = gae(Xhat_SI_PF(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity 
IndexVelGAE_SI_CMKF = gae(Xhat_SI_CMKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI_EKF = gae(Xhat_SI_EKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_SOEKF = gae(Xhat_SI_SOEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_IEKF = gae(Xhat_SI_IEKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI_UKF = gae(Xhat_SI_UKF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI_PF = gae(Xhat_SI_PF(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccGAE_SI_CMKF = gae(Xhat_SI_CMKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_SI_EKF = gae(Xhat_SI_EKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_SOEKF = gae(Xhat_SI_SOEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_IEKF = gae(Xhat_SI_IEKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_SI_UKF = gae(Xhat_SI_UKF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI_PF = gae(Xhat_SI_PF(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
TotalTime = length(time)*T;
save demo_nonlinear_filters.mat
% system('shutdown -s');

%%
%%% filted trajectory
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_SI_CMKF(1,:,kk),Xhat_SI_CMKF(4,:,kk),Xhat_SI_CMKF(7,:,kk),'-r');
plot3(Xhat_SI_EKF(1,:,kk),Xhat_SI_EKF(4,:,kk),Xhat_SI_EKF(7,:,kk),'-b');
% plot3(Xhat_SI_SOEKF(1,:,kk),Xhat_SI_SOEKF(3,:,kk),Xhat_SI_SOEKF(5,:,kk),'-c');
% plot3(Xhat_SI_IEKF(1,:,kk),Xhat_SI_IEKF(3,:,kk),Xhat_SI_IEKF(5,:,kk),'-m');
plot3(Xhat_SI_UKF(1,:,kk),Xhat_SI_UKF(4,:,kk),Xhat_SI_UKF(7,:,kk),'-c');
% plot3(Xhat_SI_PF(1,:,kk),Xhat_SI_PF(3,:,kk),Xhat_SI_PF(5,:,kk),'-m');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
% legend('真实','CMKF','EKF','SOEKF','IEKF','UKF','PF','观测')
legend('真实','CMKF','EKF','UKF','观测')
% view(3)

%%
%%% RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexPosRMSE_SI_CMKF,'-r');
plot(time(3:end),IndexPosRMSE_SI_EKF,'--b');
% plot(time(3:end),IndexPosRMSE_SI_SOKF,'-c');
% plot(time(3:end),IndexPosRMSE_SI_IEKF,'-m');
plot(time(3:end),IndexPosRMSE_SI_UKF,'-.c');
% plot(time(3:end),IndexPosRMSE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','EKF','UKF')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_SI_CMKF,'-r');
plot(time(3:end),IndexVelRMSE_SI_EKF,'--b');
% plot(time(3:end),IndexVelRMSE_SI_SOEKF,'-c');
% plot(time(3:end),IndexVelRMSE_SI_IEKF,'-m');
plot(time(3:end),IndexVelRMSE_SI_UKF,'-.c');
% plot(time(3:end),IndexVelRMSE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','EKF','UKF')

figure(13)
hold on
plot(time(3:end),IndexAccRMSE_SI_CMKF,'-r');
plot(time(3:end),IndexAccRMSE_SI_EKF,'--b');
% plot(time(3:end),IndexAccRMSE_SI_SOKF,'-c');
% plot(time(3:end),IndexAccRMSE_SI_IEKF,'-m');
plot(time(3:end),IndexAccRMSE_SI_UKF,'-.c');
% plot(time(3:end),IndexAccRMSE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','EKF','UKF')

%%
figure(21)
hold on
plot(time(3:end),IndexPosRMSE_SI_CMKF-IndexPosRMSE_SI_EKF,'-r');
plot(time(3:end),IndexPosRMSE_SI_CMKF-IndexPosRMSE_SI_UKF,'--b');
plot(time(3:end),IndexPosRMSE_SI_EKF-IndexPosRMSE_SI_UKF,'-.c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
legend('CMKF-EKF','CMKF-UKF','EKF-UKF')

figure(22)
hold on
plot(time(3:end),IndexVelRMSE_SI_CMKF-IndexVelRMSE_SI_EKF,'-r');
plot(time(3:end),IndexVelRMSE_SI_CMKF-IndexVelRMSE_SI_UKF,'--b');
plot(time(3:end),IndexVelRMSE_SI_EKF-IndexVelRMSE_SI_UKF,'-.c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('CMKF-EKF','CMKF-UKF','EKF-UKF')

figure(23)
hold on
plot(time(3:end),IndexAccRMSE_SI_CMKF-IndexAccRMSE_SI_EKF,'-r');
plot(time(3:end),IndexAccRMSE_SI_CMKF-IndexAccRMSE_SI_UKF,'--b');
plot(time(3:end),IndexAccRMSE_SI_EKF-IndexAccRMSE_SI_UKF,'-.c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('CMKF-EKF','CMKF-UKF','EKF-UKF')

%%
%%% GAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_SI_CMKF,'-r');
plot(time(3:end),IndexPosGAE_SI_EKF,'-b');
% plot(time(3:end),IndexPosGAE_SI_SOKF,'-c');
% plot(time(3:end),IndexPosGAE_SI_IEKF,'-m');
plot(time(3:end),IndexPosGAE_SI_UKF,'-y');
% plot(time(3:end),IndexPosGAE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','EKF','UKF')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_SI_CMKF,'-r');
plot(time(3:end),IndexVelGAE_SI_EKF,'-b');
% plot(time(3:end),IndexVelGAE_SI_IEKF,'-m');
% plot(time(3:end),IndexVelGAE_SI_SOEKF,'-c');
plot(time(3:end),IndexVelGAE_SI_UKF,'-y');
% plot(time(3:end),IndexVelGAE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','EKF','UKF')

figure(33)
hold on
plot(time(3:end),IndexAccGAE_SI_CMKF,'-r');
plot(time(3:end),IndexAccGAE_SI_EKF,'-b');
% plot(time(3:end),IndexAccGAE_SI_SOKF,'-c');
% plot(time(3:end),IndexAccGAE_SI_IEKF,'-m');
plot(time(3:end),IndexAccGAE_SI_UKF,'-y');
% plot(time(3:end),IndexAccGAE_SI_PF,'-y');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
% legend('CMKF','EKF','SOEKF','IEKF','UKF','PF')
legend('CMKF','EKF','UKF')
