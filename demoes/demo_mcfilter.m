%%
%%% 清除变量关闭打开的窗口
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

%%
%%% 运动模型
Swx = 50;                                                               % x轴过程噪声功率谱密度
Swy = 50;                                                               % y轴过程噪声功率谱密度
T = 0.1;                                                                % 时间采样间隔
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % 运动模型
%%% 观测模型1
H1 = [1,0,0,0; 0,0,1,0];                                                % 观测矩阵
R1 = [10000,0; 0,10000];                                                % 观测噪声协方差阵
hd_msmodel1 = msmodel.dxy(H1,R1);                                       % 观测模型
%%% 观测模型2
R2 = [10000,0; 0,0.0001];                                               % 观测噪声协方差阵
hd_msmodel2 = msmodel.drb(R2);                                          % 观测模型
%%% 滤波器模型1——卡尔曼滤波
hd1 = mttfilter.kalman(hd_mtnmodel,hd_msmodel1);                        % 滤波器模型1
%%% 滤波器模型2——扩展卡尔曼滤波
hd2 = mttfilter.ekf(hd_mtnmodel,hd_msmodel2);                           % 滤波器模型2
%%% 滤波器模型3——无迹卡尔曼滤波
hd3 = mttfilter.ukf(hd_mtnmodel,hd_msmodel2);                           % 滤波器模型3
%%% 滤波过程
NumMC = 10;
DimState = length(hd_mtnmodel.StateSym);
Xtrue = trajectory.dxy;                                                 % 目标真实状态
Num = size(Xtrue,2);
Z1 = repmat([Xtrue(1,:);Xtrue(4,:)],[1,1,NumMC])+...
    randn(2,Num,NumMC)*sqrt(R1(1,1));                                   % 直角坐标系观测向量序列
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));
Z2 = repmat([rho;theta],[1,1,NumMC])+...
    [randn(1,Num,NumMC)*sqrt(R2(1,1));
     randn(1,Num,NumMC)*sqrt(R2(2,2))];                                 % 极坐标系观测向量序列
Xinit = zeros(DimState,NumMC);
Pinit = zeros(DimState,DimState,NumMC);
for kk = 1:1:NumMC
    [Xinit(:,kk),Pinit(:,:,kk)] = twopointsinit(Z1(:,1:2,kk), [Swx,Swy], T, R1);
                                                                        % 滤波器起始值（两点起始法）
end
[Xhat1,Phat1] = mcfilter(hd1,Z1,Xinit,Pinit);                           % 标准卡尔曼滤波
[Xhat2,Phat2] = mcfilter(hd2,Z2,Xinit,Pinit);                           % 扩展卡尔曼滤波
% [Xhat3,Phat3] = mcfilter(hd3,Z2,Xinit,Pinit);                           % 不敏卡尔曼滤波

%%
%%% 滤波性能评估
ErrorPosition1 = flterr([Xtrue(1,:);Xtrue(4,:)],[Xhat1(1,:,:);Xhat1(3,:,:)]);
ErrorVelocity1 = flterr([Xtrue(2,:);Xtrue(5,:)],[Xhat1(2,:,:);Xhat1(4,:,:)]);

ErrorPosition2 = flterr([Xtrue(1,:);Xtrue(4,:)],[Xhat2(1,:,:);Xhat2(3,:,:)]);
ErrorVelocity2 = flterr([Xtrue(2,:);Xtrue(5,:)],[Xhat2(2,:,:);Xhat2(4,:,:)]);

% ErrorPosition3 = flterr([Xtrue(1,:);Xtrue(4,:)],[Xhat3(1,:,:);Xhat3(3,:,:)]);
% ErrorVelocity3 = flterr([Xtrue(2,:);Xtrue(5,:)],[Xhat3(2,:,:);Xhat3(4,:,:)]);

%%
%%% 滤波轨迹
figure(11)
kk = 1;
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat1(1,:,kk),Xhat1(3,:,kk),'-r');
plot(Xhat2(1,:,kk),Xhat2(3,:,kk),'-g');
% plot(Xhat3(1,:,kk),Xhat3(3,:,kk),'-k');
hold off
axis equal, box on, grid on
legend('真实','KF滤波','EKF')

%%
%%% 位置滤波误差
figure(21)
hold on
plot(Xtrue(1,1:end)-Xhat1(1,:,kk),'-.k');
plot(Xtrue(4,1:end)-Xhat1(3,:,kk),'-k');
hold off
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x轴','y轴')

%%
%%% 速度滤波误差
figure(22)
hold on
plot(Xtrue(2,1:end)-Xhat1(2,:,kk),'-.k');
plot(Xtrue(5,1:end)-Xhat1(4,:,kk),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x轴','y轴')

%%
%%% 位置滤波误差
figure(31)
hold on
plot(ErrorPosition1,'-r')
plot(ErrorPosition2,'--g')
% plot(ErrorPosition3,'-.k')
hold off
xlim([0,1300])
ylim([0,160])
box on, grid on
legend('SKF','EKF')

%%
%%% 速度滤波误差
figure(32)
hold on
plot(ErrorVelocity1,'-r')
plot(ErrorVelocity2,'--g')
% plot(ErrorVelocity3,'-k')
hold off
xlim([0,1300])
ylim([0,120])
box on, grid on
legend('SKF','EKF')
