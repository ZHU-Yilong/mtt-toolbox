%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiments of multiple sources 
%%% with radar and infrared measurements
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

% %%
% load measurement.mat
% SigmaRan_rad = SigmaRan;
% SigmaAzi_rad = SigmaAzi;
% SigmaEle_rad = SigmaEle;
% RadarMeasure1 = Measure1;
% RadarMeasure2 = Measure2;
% RadarMeasure3 = Measure3;
% PseudoRadarMeasure = PseudoMeasure;
% %%%
% SigmaAzi_inf = deg2rad(0.1);
% SigmaEle_inf = deg2rad(0.1);
% %%% pseudo infrared measurement vector
% InfraMeasure = repmat(MeasureTrue(2:3,:),[1,1,NumMC]) + ...
%     [randn(1,NumStep,NumMC)*SigmaAzi_inf;
%      randn(1,NumStep,NumMC)*SigmaEle_inf];
% InfraMeasure1 = InfraMeasure+repmat([LosAzimuth;LosElevation],[1,1,NumMC]);
% [InfraMeasure2(1,:,:),InfraMeasure2(2,:,:),InfraMeasure2(3,:,:)] = ...
%     sph2cart(InfraMeasure1(1,:,:),InfraMeasure1(2,:,:),RadarMeasure1(1,:,:));
% InfraMeasure3 = InfraMeasure2+repmat(PosMissile,[1,1,NumMC]);
% [PseudoInfraMeasure(2,:,:),PseudoInfraMeasure(3,:,:),PseudoInfraMeasure(1,:,:)] = ...
%     cart2sph(InfraMeasure3(1,:,:),InfraMeasure3(2,:,:),InfraMeasure3(3,:,:)); 
% 
% Zclutter = cell(2,NumStep,NumMC);
% for kk = 1:1:NumMC
%     for jj = 1:1:NumStep
%         Zclutter{1,jj,kk} = PseudoRadarMeasure(:,jj,kk);
%         Zclutter{2,jj,kk} = PseudoInfraMeasure(:,jj,kk);
%     end
% end
% 
% save measurement_multiple_sources.mat

load measurement_multiple_sources.mat
NumMC = 10;
Zclutter = Zclutter(:,:,1:NumMC);
PseudoRadarMeasure = PseudoRadarMeasure(:,:,1:NumMC);

%%
%%% 
%%% CV model
Swx = 1e2;                                                              % x轴过程噪声功率谱密度
Swy = 1e2;                                                              % y轴过程噪声功率谱密度
Swz = 10;
T = 0.02;                                                               % 时间采样间隔
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.cv(Swz,T);
hd_mtnmodel_cv = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% CT model
Omega = -5*9.8/300;
Sw = 1e2;
hd_mtnmodel_xy = b2model.ct(Omega,Sw,T);
hd_mtnmodel_ct = mtnmodel.dxydz(hd_mtnmodel_xy,hd_mtnmodel_z);
%%% 观测模型
R_rad = blkdiag(SigmaRan_rad^2,SigmaAzi_rad^2,SigmaEle_rad^2);                  % 观测噪声协方差阵
hd_msmodel_rad = msmodel.drbe(R_rad);                                           % 观测模型
R_inf = blkdiag(SigmaRan_rad^2,SigmaAzi_inf^2,SigmaEle_inf^2);                  % 观测噪声协方差阵
hd_msmodel_inf = msmodel.drbe(R_inf);                                           % 观测模型
%%% 滤波器模型
hd_cv_rad = mttfilter.ukf(hd_mtnmodel_cv,hd_msmodel_rad);
hd_ct_rad = mttfilter.ukf(hd_mtnmodel_ct,hd_msmodel_rad);
hd_cv_inf = mttfilter.ukf(hd_mtnmodel_cv,hd_msmodel_inf);
hd_ct_inf = mttfilter.ukf(hd_mtnmodel_ct,hd_msmodel_inf);
%%%
lambda = 0.0004;
gamma = 100;
Pg = 0.9997;
Pd = 1;
para = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];
hd_cv = mttfilter.mspdaf({hd_cv_rad;hd_cv_inf}, para);
hd_ct = mttfilter.mspdaf({hd_ct_rad;hd_ct_inf}, para);
MatrixPi = [0.99,0.01;0.01,0.99];
hd_ms = mttfilter.immmspdaf({hd_cv;hd_ct},MatrixPi);
hd_ss = mttfilter.imm({hd_cv_rad,hd_ct_rad},MatrixPi);
%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
[Xhat_MS,Phat_MS,Mu_MS] = hd_ms.mcfilter(Zclutter(:,3:end,:),Xinit,Pinit,flag);       % 滤波
[Xhat_SS,Phat_SS,Mu_SS] = hd_ss.mcfilter(PseudoRadarMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
IndexPosRMSE_SS = rmse(Xhat_SS(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SS = rmse(Xhat_SS(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosRMSE_MS = rmse(Xhat_MS(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_MS = rmse(Xhat_MS(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));

%%% index No.3 GAE / LGAE
IndexPosGAE_SS = gae(Xhat_SS(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SS = gae(Xhat_SS(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosGAE_MS = gae(Xhat_MS(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_MS = gae(Xhat_MS(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));


%%
%%% save data files
save demo_multiple_sources.mat
system('shutdown -s');

%%
%%% filtered trajectory
figure(1)
kk = 1;
[RadarMeasureX,RadarMeasureY,RadarMeasureZ] = sph2cart(PseudoRadarMeasure(2,:,kk),PseudoRadarMeasure(3,:,kk),PseudoRadarMeasure(1,:,kk));
[InfraMeasureX,InfraMeasureY,InfraMeasureZ] = sph2cart(PseudoInfraMeasure(2,:,kk),PseudoInfraMeasure(3,:,kk),PseudoInfraMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_SS(1,:,kk),Xhat_SS(3,:,kk),Xhat_SS(5,:,kk),'-r');
plot3(Xhat_MS(1,:,kk),Xhat_MS(3,:,kk),Xhat_MS(5,:,kk),'-b');
scatter3(RadarMeasureX,RadarMeasureY,RadarMeasureZ,3);
scatter3(InfraMeasureX,InfraMeasureY,InfraMeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','单传感器','多传感器','雷达观测','红外观测')
% view(3)

%%
%%% index 1 RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexPosRMSE_SS,'-r');
plot(time(3:end),IndexPosRMSE_MS,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('单传感器','多传感器')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_SS,'-r');
plot(time(3:end),IndexVelRMSE_MS,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('单传感器','多传感器')

%%
%%% index 3 GAE / LGAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_SS,'-r');
plot(time(3:end),IndexPosGAE_MS,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
legend('单传感器','多传感器')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_SS,'-r');
plot(time(3:end),IndexVelGAE_MS,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('单传感器','多传感器')

%%
kk = 1;
figure(41)
hold on
plot(time(3:end),Mu_MS(1,:,kk),'-r')
plot(time(3:end),Mu_MS(2,:,kk),'-b')
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,1])
legend('CV','CT')

figure(42)
hold on
plot(time(3:end),Mu_SS(1,:,kk),'-r')
plot(time(3:end),Mu_SS(2,:,kk),'-b')
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,1])
legend('CV','CT')

