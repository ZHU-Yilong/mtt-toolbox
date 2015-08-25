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
Swz = 10;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_ca = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% Singer model
Alpha = 0.15;
Amax = 5*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_si = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% ct model
Omega = -5*9.8/300;
Sw = 1e1;
hd_mtnmodel_xy = b2model.ct(Omega,Sw,T);
hd_mtnmodel_ct1 = mtnmodel.dxydz(hd_mtnmodel_xy,hd_mtnmodel_z);
%%% ct model
Omega = -5*9.8/300;
Sw = 1e2;
hd_mtnmodel_xy = b2model.ct(Omega,Sw,T);
hd_mtnmodel_ct2 = mtnmodel.dxydz(hd_mtnmodel_xy,hd_mtnmodel_z);
%%% ct model
Omega = -5*9.8/300;
Sw = 1e3;
hd_mtnmodel_xy = b2model.ct(Omega,Sw,T);
hd_mtnmodel_ct3 = mtnmodel.dxydz(hd_mtnmodel_xy,hd_mtnmodel_z);
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_cv = mttfilter.cmkf(hd_mtnmodel_cv,hd_msmodel);
hd_ca = mttfilter.cmkf(hd_mtnmodel_ca,hd_msmodel);
hd_si = mttfilter.cmkf(hd_mtnmodel_si,hd_msmodel);
hd_ct1 = mttfilter.cmkf(hd_mtnmodel_ct1,hd_msmodel);
hd_ct2 = mttfilter.cmkf(hd_mtnmodel_ct2,hd_msmodel);
hd_ct3 = mttfilter.cmkf(hd_mtnmodel_ct3,hd_msmodel);
%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
Xhat_CV = hd_cv.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
Xhat_CT1 = hd_ct1.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
Xhat_CT2 = hd_ct2.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
Xhat_CT3 = hd_ct3.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
%%%
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
[Xhat_CA,Phat_CA] = hd_ca.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波
% [Xhat_SI,Phat_SI] = hd_si.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position
IndexPosRMSE_CV = rmse(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CT1 = rmse(Xhat_CT1(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CT2 = rmse(Xhat_CT2(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CT3 = rmse(Xhat_CT3(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_CA = rmse(Xhat_CA(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosRMSE_SI = rmse(Xhat_SI(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelRMSE_CV = rmse(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CT1 = rmse(Xhat_CT1(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CT2 = rmse(Xhat_CT2(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CT3 = rmse(Xhat_CT3(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CA = rmse(Xhat_CA(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelRMSE_SI = rmse(Xhat_SI(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
% IndexAccRMSE_CA = rmse(Xhat_CA(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccRMSE_SI = rmse(Xhat_SI(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexPosGAE_CV = gae(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CT1 = gae(Xhat_CT1(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CT2 = gae(Xhat_CT2(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CT3 = gae(Xhat_CT3(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_CA = gae(Xhat_CA(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
% IndexPosGAE_SI = gae(Xhat_SI(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelGAE_CV = gae(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CT1 = gae(Xhat_CT1(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CT2 = gae(Xhat_CT2(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CT3 = gae(Xhat_CT3(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CA = gae(Xhat_CA(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
% IndexVelGAE_SI = gae(Xhat_SI(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
% IndexAccGAE_CA = gae(Xhat_CA(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
% IndexAccGAE_SI = gae(Xhat_SI(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
save demo_single_motion_model_ct.mat

%%
%%% filted trajectory
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_CV(1,:,kk),Xhat_CV(3,:,kk),Xhat_CV(5,:,kk),'-y');
plot3(Xhat_CT1(1,:,kk),Xhat_CT1(3,:,kk),Xhat_CT1(5,:,kk),'-r');
plot3(Xhat_CT2(1,:,kk),Xhat_CT2(3,:,kk),Xhat_CT2(5,:,kk),'-b');
plot3(Xhat_CT3(1,:,kk),Xhat_CT3(3,:,kk),Xhat_CT3(5,:,kk),'-c');
plot3(Xhat_CA(1,:,kk),Xhat_CA(4,:,kk),Xhat_CA(7,:,kk),'-c');
% plot3(Xhat_SI(1,:,kk),Xhat_SI(4,:,kk),Xhat_SI(7,:,kk),'-m');
scatter3(MeasureX,MeasureY,MeasureZ);
hold off
axis equal, box on, grid on
legend('真实','CV模型','CT1模型','CT2模型','CT3模型','CA','观测')
% view(3)

%%
%%% RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexPosRMSE_CV,'-y');
plot(time(3:end),IndexPosRMSE_CT1,'-r');
plot(time(3:end),IndexPosRMSE_CT2,'-b');
plot(time(3:end),IndexPosRMSE_CT3,'-c');
plot(time(3:end),IndexPosRMSE_CA,'-k');
% plot(time(3:end),IndexPosRMSE_SI,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('CV模型','CT1模型','CT2模型','CT3模型','CA')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_CV,'-y');
plot(time(3:end),IndexVelRMSE_CT1,'-r');
plot(time(3:end),IndexVelRMSE_CT2,'-b');
plot(time(3:end),IndexVelRMSE_CT3,'-c');
plot(time(3:end),IndexVelRMSE_CA,'-k');
% plot(time(3:end),IndexVelRMSE_SI,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('CV模型','CT1模型','CT2模型','CT3模型','CA')

% figure(13)
% hold on
% plot(time(3:end),IndexAccRMSE_CA,'-c');
% plot(time(3:end),IndexAccRMSE_SI,'-m');
% hold off
% grid on, box on
% xlim([0,TotalTime]), ylim([0,50])
% legend('CA模型','辛格模型')

%%
%%% GAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_CV,'-y');
plot(time(3:end),IndexPosGAE_CT1,'-r');
plot(time(3:end),IndexPosGAE_CT2,'-b');
plot(time(3:end),IndexPosGAE_CT3,'-c');
plot(time(3:end),IndexPosGAE_CA,'-k');
% plot(time(3:end),IndexPosGAE_SI,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
legend('CV模型','CT1模型','CT2模型','CT3模型','CA')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_CV,'-y');
plot(time(3:end),IndexVelGAE_CT1,'-r');
plot(time(3:end),IndexVelGAE_CT2,'-b');
plot(time(3:end),IndexVelGAE_CT3,'-c');
plot(time(3:end),IndexVelGAE_CA,'-k');
% plot(time(3:end),IndexVelGAE_SI,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('CV模型','CT1模型','CT2模型','CT3模型','CA')

% figure(33)
% hold on
% plot(time(3:end),IndexAccGAE_CA,'-c');
% plot(time(3:end),IndexAccGAE_SI,'-m');
% hold off
% grid on, box on
% xlim([0,TotalTime])
% % ylim([0,100])
% legend('CA模型','辛格模型')
