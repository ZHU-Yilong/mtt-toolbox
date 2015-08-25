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
NumMC = 50;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);
 
%%
%%% 
%%% 运动模型
%%% CV model
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
%%% CT model
Omega = -5*9.8/300;
Sw = 1e2;
hd_mtnmodel_xy = b2model.ct(Omega,Sw,T);
hd_mtnmodel_ct = mtnmodel.dxydz(hd_mtnmodel_xy,hd_mtnmodel_z);
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_cv = mttfilter.cmkf(hd_mtnmodel_cv,hd_msmodel);
hd_ca = mttfilter.cmkf(hd_mtnmodel_ca,hd_msmodel);
hd_si = mttfilter.cmkf(hd_mtnmodel_si,hd_msmodel);
hd_ct = mttfilter.cmkf(hd_mtnmodel_ct,hd_msmodel);
%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
[Xhat_CV,Phat_CV] = hd_cv.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
[Xhat_CT,Phat_CT] = hd_ct.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);
%%%
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
[Xhat_CA,Phat_CA] = hd_ca.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波
[Xhat_SI,Phat_SI] = hd_si.mcfilter(PseudoMeasure(:,3:NumStep,:),Xinit,Pinit,flag);       % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position
IndexCVPosRMSE = rmse(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexCTPosRMSE = rmse(Xhat_CT(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexCAPosRMSE = rmse(Xhat_CA(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexSIPosRMSE = rmse(Xhat_SI(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexCVVelRMSE = rmse(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexCTVelRMSE = rmse(Xhat_CT(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexCAVelRMSE = rmse(Xhat_CA(2:3:9,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexSIVelRMSE = rmse(Xhat_SI(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexCAAccRMSE = rmse(Xhat_CA(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexSIAccRMSE = rmse(Xhat_SI(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexCVPosGAE = gae(Xhat_CV(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexCTPosGAE = gae(Xhat_CT(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexCAPosGAE = gae(Xhat_CA(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexSIPosGAE = gae(Xhat_SI(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexCVVelGAE = gae(Xhat_CV(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexCTVelGAE = gae(Xhat_CT(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexCAVelGAE = gae(Xhat_CA(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexSIVelGAE = gae(Xhat_SI(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexCAAccGAE = gae(Xhat_CA(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexSIAccGAE = gae(Xhat_SI(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
save demo_motion_models2.mat

%%
%%% filted trajectory
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_CV(1,:,kk),Xhat_CV(3,:,kk),Xhat_CV(5,:,kk),'-r');
plot3(Xhat_CT(1,:,kk),Xhat_CT(3,:,kk),Xhat_CT(5,:,kk),'-b');
plot3(Xhat_CA(1,:,kk),Xhat_CA(4,:,kk),Xhat_CA(7,:,kk),'-c');
plot3(Xhat_SI(1,:,kk),Xhat_SI(4,:,kk),Xhat_SI(7,:,kk),'-m');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','CV模型','CT模型','CA模型','辛格模型','观测')
% view(3)

%%
%%% RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexCVPosRMSE,'-r');
plot(time(3:end),IndexCTPosRMSE,'-b');
plot(time(3:end),IndexCAPosRMSE,'-c');
plot(time(3:end),IndexSIPosRMSE,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('CV模型','CT模型','CA模型','辛格模型')

figure(12)
hold on
plot(time(3:end),IndexCVVelRMSE,'-r');
plot(time(3:end),IndexCTVelRMSE,'-b');
plot(time(3:end),IndexCAVelRMSE,'-c');
plot(time(3:end),IndexSIVelRMSE,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('CV模型','CT模型','CA模型','辛格模型')

figure(13)
hold on
plot(time(3:end),IndexCAAccRMSE,'-c');
plot(time(3:end),IndexSIAccRMSE,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('CA模型','辛格模型')

%%
%%% GAE
figure(31)
hold on
plot(time(3:end),IndexCVPosGAE,'-r');
plot(time(3:end),IndexCTPosGAE,'-b');
plot(time(3:end),IndexCAPosGAE,'-c');
plot(time(3:end),IndexSIPosGAE,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('CV模型','CT模型','CA模型','辛格模型')

figure(32)
hold on
plot(time(3:end),IndexCVVelGAE,'-r');
plot(time(3:end),IndexCTVelGAE,'-b');
plot(time(3:end),IndexCAVelGAE,'-c');
plot(time(3:end),IndexSIVelGAE,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,50])
legend('CV模型','CT模型','CA模型','辛格模型')

figure(33)
hold on
plot(time(3:end),IndexCAAccGAE,'-c');
plot(time(3:end),IndexSIAccGAE,'-m');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,50])
legend('CA模型','辛格模型')
