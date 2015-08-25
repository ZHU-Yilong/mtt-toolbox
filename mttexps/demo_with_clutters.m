%%
%%% 清除变量关闭打开的窗口
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

% %%
% load measurement.mat
% %%% add clutter
% nc = 5;
% lambda = 0.0004;
% Av = nc/10/lambda;
% q = sqrt(10*Av)/2;
% PseudoMeasureClutter = cell(NumStep,NumMC);
% for kk = 1:1:NumMC
%     for jj = 1:1:NumStep
% %         [xMeasure,yMeasure,zMeasure] = sph2cart(PseudoMeasure(2,jj,kk),PseudoMeasure(3,jj,kk),PseudoMeasure(1,jj,kk));
%         CartMeasure = repmat(Measure3(:,jj,kk)-[q;q;q],1,nc)+rand(DimMeasure,nc)*2*q;
%         [SphMeasureAzi,SphMeasureEle,SphMeasureRan] = cart2sph(CartMeasure(1),CartMeasure(2),CartMeasure(3));
%         PseudoMeasureClutter{jj,kk} = [SphMeasureRan;SphMeasureAzi;SphMeasureEle];        % 杂波条件下观测
%     end
% end
% 
% save measurement_with_clutter.mat

load measurement_with_clutter.mat
NumMC = 10;
PseudoMeasureClutter = PseudoMeasureClutter(:,1:NumMC);
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

%%
%%% 运动模型
%%% Singer model
Swz = 10;
T = 0.02;
Alpha = 0.15;
Amax = 5*9.8;
P0 = 0.1;
Pmax = 0.05;
hd_mtnmodel_x = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % x轴运动模型
hd_mtnmodel_y = b1model.singer(Alpha,T,Amax,P0,Pmax);                     % y轴运动模型
hd_mtnmodel_z = b1model.cv(Swz,T);
hd_mtnmodel = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);   % 运动模型
%%% 观测模型
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R); 
%%% 滤波器模型
hdd = mttfilter.ukf(hd_mtnmodel,hd_msmodel);                      % 滤波器模型
%%% 滤波器参数
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda;gamma;Pg;Pd];                                            % 滤波器参数
hd = mttfilter.pdaf(hdd, para);
%%% 滤波过程
flag = 1;   
Xinit = repmat(xtrue(1:8,2),[1,NumMC]);
Pinit = repmat(blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2),[1,1,NumMC]);
[Xhat_wo,Phat_wo] = hdd.mcfilter(PseudoMeasure(:,3:end,:),Xinit,Pinit,flag);
[Xhat_wc,Phat_wc] = hd.mcfilter(PseudoMeasureClutter(3:end,:),Xinit,Pinit,flag);            % 滤波

%%
%%% performance evaluation
%%% index No.1 RMSE
%%% position
IndexPosRMSE_WO = rmse(Xhat_wo(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosRMSE_WC = rmse(Xhat_wc(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelRMSE_WO = rmse(Xhat_wo(2:3:9,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelRMSE_WC = rmse(Xhat_wc(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccRMSE_WO = rmse(Xhat_wo(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccRMSE_WC = rmse(Xhat_wc(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
%%% index No.3 GAE / LGAE
%%% position
IndexPosGAE_WO = gae(Xhat_wo(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
IndexPosGAE_WC = gae(Xhat_wc(1:3:7,:,:)-repmat(PosTarget(:,3:end),[1,1,NumMC]));
%%% velocity
IndexVelGAE_WO = gae(Xhat_wo(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
IndexVelGAE_WC = gae(Xhat_wc(2:3:8,:,:)-repmat(VelTarget(:,3:end),[1,1,NumMC]));
%%% acceleration
IndexAccGAE_WO = gae(Xhat_wo(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));
IndexAccGAE_WC = gae(Xhat_wc(3:3:6,:,:)-repmat(AccTarget(1:2,3:end),[1,1,NumMC]));

%%
save demo_with_clutters.mat
% system('shutdown -s');

%%
%%% 滤波轨迹
figure(01)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_wo(1,:,kk),Xhat_wo(4,:,kk),Xhat_wo(7,:,kk),'-r');
plot3(Xhat_wc(1,:,kk),Xhat_wc(4,:,kk),Xhat_wc(7,:,kk),'-b');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','无杂波','有杂波','观测')

%%
%%% index 1 RMSE
TotalTime = length(time)*T;
figure(11)
hold on
plot(time(3:end),IndexPosRMSE_WO,'-r');
plot(time(3:end),IndexPosRMSE_WC,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,20])
legend('无杂波','有杂波')

figure(12)
hold on
plot(time(3:end),IndexVelRMSE_WO,'-r');
plot(time(3:end),IndexVelRMSE_WC,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('无杂波','有杂波')

figure(13)
hold on
plot(time(3:end),IndexAccRMSE_WO,'-r');
plot(time(3:end),IndexAccRMSE_WC,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,60])
legend('无杂波','有杂波')

%%
%%% index 3 GAE / LGAE
figure(31)
hold on
plot(time(3:end),IndexPosGAE_WO,'-r');
plot(time(3:end),IndexPosGAE_WC,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,200])
legend('无杂波','有杂波')

figure(32)
hold on
plot(time(3:end),IndexVelGAE_WO,'-r');
plot(time(3:end),IndexVelGAE_WC,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('无杂波','有杂波')

figure(33)
hold on
plot(time(3:end),IndexAccGAE_WO,'-r');
plot(time(3:end),IndexAccGAE_WC,'-b');
hold off
grid on, box on
xlim([0,TotalTime])
% ylim([0,100])
legend('无杂波','有杂波')
