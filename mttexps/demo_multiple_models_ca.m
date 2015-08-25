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
NumMC = 1;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

%%
%%% 
%%% 运动模型
%%% CA model 1
Swx = 10;
Swy = 10;
Swz = 10;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_z = b1model.ca(Swz,T);
hd_mtnmodel_ca1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% CA model 2
Swx = 1e3;
Swy = 1e3;
hd_mtnmodel_x = b1model.ca(Swx,T);                     % x轴运动模型
hd_mtnmodel_y = b1model.ca(Swy,T);                     % y轴运动模型
hd_mtnmodel_ca2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R);                                               % 观测模型
%%% 滤波器模型
hd_ca1 = mttfilter.cmkf(hd_mtnmodel_ca1,hd_msmodel);
hd_ca2 = mttfilter.cmkf(hd_mtnmodel_ca2,hd_msmodel);
hd_amm = mttfilter.amm({hd_ca1,hd_ca2});
hd_imm = mttfilter.imm({hd_ca1,hd_ca2});
%%% 滤波过程
flag = 1;                                                               % 进度条标示
Xinit = repmat(xtrue(:,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2,100^2),[1,1,NumMC]);
[Xhat_CA2,Phat_CA2] = hd_ca2.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波
[Xhat_AMM,Phat_AMM,Mu_AMM] = hd_amm.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波
[Xhat_IMM,Phat_IMM,Mu_IMM] = hd_imm.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);       % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
IndexPosRMSE_CA2 = rmse(Xhat_CA2(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_CA2 = rmse(Xhat_CA2(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosRMSE_AMM = rmse(Xhat_AMM(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_AMM = rmse(Xhat_AMM(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosRMSE_IMM = rmse(Xhat_IMM(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_IMM = rmse(Xhat_IMM(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));

%%% index No.3 GAE / LGAE
IndexPosGAE_CA2 = gae(Xhat_CA2(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_CA2 = gae(Xhat_CA2(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosGAE_AMM = gae(Xhat_AMM(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_AMM = gae(Xhat_AMM(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%%
IndexPosGAE_IMM = gae(Xhat_IMM(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_IMM = gae(Xhat_IMM(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));


%%
%%% save data files
save demo_multiple_models_ca.mat

%%
%%% filtered trajectory
figure(1)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_CA2(1,:,kk),Xhat_CA2(3,:,kk),Xhat_CA2(5,:,kk),'-r');
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
plot(time(3:end),IndexPosRMSE_CA2,'-r');
plot(time(3:end),IndexPosRMSE_AMM,'-b');
plot(time(3:end),IndexPosRMSE_IMM,'-c');
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,20])
legend('单模型','AMM','IMM')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_CA2,'-r');
plot(time(3:end),IndexVelRMSE_AMM,'-b');
plot(time(3:end),IndexVelRMSE_IMM,'-c');
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,60])
legend('单模型','AMM','IMM')

%%
%%% index 3 GAE / LGAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_CA2,'-r');
plot(time(3:end),IndexPosGAE_AMM,':b');
plot(time(3:end),IndexPosGAE_IMM,'-.c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
legend('单模型','AMM','IMM')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_CA2,'-r');
plot(time(3:end),IndexVelGAE_AMM,':b');
plot(time(3:end),IndexVelGAE_IMM,'-.c');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('单模型','AMM','IMM')

%%
kk = 1;
figure(41)
hold on
plot(time(3:end),Mu_AMM(1,:,kk),'-r')
plot(time(3:end),Mu_AMM(2,:,kk),'-b')
% plot(time(3:end),Mu_AMM(3,:,kk),'-c')
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,1])
legend('CA1','CA2')

figure(42)
hold on
plot(time(3:end),Mu_IMM(1,:,kk),'-r')
plot(time(3:end),Mu_IMM(2,:,kk),'-b')
% plot(time(3:end),Mu_IMM(3,:,kk),'-c')
hold off
grid on, box on
xlim([0,TotalTime])
ylim([0,1])
legend('CA1','CA2')
