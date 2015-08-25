%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiments of motion model CV 
%%% with different parameters Sw
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
%%% MODEL1 CV model
Swx = 1e2;                                                              % x轴过程噪声功率谱密度
Swy = 1e2;                                                              % y轴过程噪声功率谱密度
Swz = 10;
T = 0.02;                                                               % 时间采样间隔
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel_z = b1model.cv(Swz,T);
hd_mtnmodel_cv = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% MODEL2 CT model
Omega = -5*9.8/300;
Sw = 1e2;
hd_mtnmodel_xy = b2model.ct(Omega,Sw,T);
hd_mtnmodel_ct = mtnmodel.dxydz(hd_mtnmodel_xy,hd_mtnmodel_z);
%%% MODEL3 Singer model
Amax = 5*9.8;
P0 = 0.1;
Pmax = 0.05;
Alpha = 0.15;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_si = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_cv = mttfilter.cmkf(hd_mtnmodel_cv,hd_msmodel);
hd_ct = mttfilter.cmkf(hd_mtnmodel_ct,hd_msmodel);
hd_si = mttfilter.cmkf(hd_mtnmodel_si,hd_msmodel);
hd_amm = mttfilter.amm({hd_cv,hd_ct});
% MatrixPi = [0.90,0.05,0.05;0.05,0.90,0.05;0.05,0.05,0.90];
MatrixPi = [0.99,0.01;0.01,0.99];
hd_imm = mttfilter.imm({hd_cv,hd_ct},MatrixPi);
%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat([xtrue(1:2,2);xtrue(4:5,2);xtrue(7:8,2)],[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,200^2,100^2,200^2),[1,1,NumMC]);
[Xhat_AMM,Phat_AMM,Mu_AMM] = hd_amm.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波
[Xhat_IMM,Phat_IMM,Mu_IMM] = hd_imm.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波
%%%
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
[Xhat_SI,Phat_SI] = hd_si.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
IndexPosRMSE_SI = rmse(Xhat_SI(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_SI = rmse(Xhat_SI(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosRMSE_AMM = rmse(Xhat_AMM(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_AMM = rmse(Xhat_AMM(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosRMSE_IMM = rmse(Xhat_IMM(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_IMM = rmse(Xhat_IMM(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));

%%% index No.3 GAE / LGAE
IndexPosGAE_SI = gae(Xhat_SI(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_SI = gae(Xhat_SI(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosGAE_AMM = gae(Xhat_AMM(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_AMM = gae(Xhat_AMM(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosGAE_IMM = gae(Xhat_IMM(1:2:5,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_IMM = gae(Xhat_IMM(2:2:6,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));


%%
%%% save data files
save demo_multiple_models.mat
system('shutdown -s');

%%
%%% filtered trajectory
figure(1)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_SI(1,:,kk),Xhat_SI(4,:,kk),Xhat_SI(7,:,kk),'-r');
plot3(Xhat_AMM(1,:,kk),Xhat_AMM(3,:,kk),Xhat_AMM(5,:,kk),'-b');
plot3(Xhat_IMM(1,:,kk),Xhat_IMM(3,:,kk),Xhat_IMM(5,:,kk),'-c');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','单模型','AMM','IMM','观测')
% view(3)

%%
%%% index 1 RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexPosRMSE_SI,'-r');
plot(time(3:end),IndexPosRMSE_AMM,'-b');
plot(time(3:end),IndexPosRMSE_IMM,'-c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('单模型','AMM','IMM')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_SI,'-r');
plot(time(3:end),IndexVelRMSE_AMM,'-b');
plot(time(3:end),IndexVelRMSE_IMM,'-c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('单模型','AMM','IMM')

%%
%%% index 3 GAE / LGAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_SI,'-r');
plot(time(3:end),IndexPosGAE_AMM,'-b');
plot(time(3:end),IndexPosGAE_IMM,'-c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
legend('单模型','AMM','IMM')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_SI,'-r');
plot(time(3:end),IndexVelGAE_AMM,'-b');
plot(time(3:end),IndexVelGAE_IMM,'-c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('单模型','AMM','IMM')

%%
kk = 2;
figure(41)
hold on
plot(time(3:end),Mu_AMM(1,:,kk),'-r')
plot(time(3:end),Mu_AMM(2,:,kk),'-b')
% plot(time(3:end),Mu_AMM(3,:,kk),'-c')
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,1])
% legend('CV模型','CT模型','辛格模型')
legend('CV模型','CT模型')

figure(42)
hold on
plot(time(3:end),Mu_IMM(1,:,kk),'-r')
plot(time(3:end),Mu_IMM(2,:,kk),'-b')
% plot(time(3:end),Mu_IMM(3,:,kk),'-c')
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,1])
% legend('CV模型','CT模型','辛格模型')
legend('CV模型','CT模型')
