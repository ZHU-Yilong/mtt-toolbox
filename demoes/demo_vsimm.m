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
Omega = deg2rad(6);
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel1 = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % 运动模型
hd_mtnmodel_sub = b2model.ct(Omega,Swx,T);
hd_mtnmodel2 = mtnmodel.dxy(hd_mtnmodel_sub);
%%% 观测模型
H = [1,0,0,0; 0,0,1,0];                                                 % 观测矩阵
R = [10000,0; 0,10000];                                                 % 观测噪声协方差阵
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel = msmodel.dxy(R,StateSym);                                   % 观测模型
%%% 滤波器模型
hd_filter1 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel);                  % 滤波器模型
hd_filter2 = mttfilter.kalman(hd_mtnmodel2,hd_msmodel);
hd = mttfilter.vsimm({hd_filter1,hd_filter2});
%%% 滤波过程
Xtrue = trajectory.dxy;                                                 % 目标真实状态
Z = [Xtrue(1,:);Xtrue(4,:)]+randn(2,size(Xtrue,2))*100;                 % 观测向量序列
MdlSqn = [repmat({ones(1,1)}, [500,1]); repmat({ones(1,1)*2}, [300,1]); repmat({ones(1,1)}, [500,1])];
[Xinit, Pinit] = twopointsinit(Z(:,1:2), [Swx,Swy], T, R);              % 滤波器起始值（两点起始法）
Minit = 1;

%%
flag = 1;                                                               % 进度条标示
[Xhat,Phat] = hd.filter(Xinit, Pinit, Minit, Z, MdlSqn, flag);          % 滤波

%%
%%% 利用mcfilter进行蒙特卡洛滤波
% NumMc = 2;                                                              % 蒙特卡洛仿真次数
% Zmc = repmat(Z,[1,1,NumMc]);                                            % 观测
% XhatMc = hd.mcfilter(Xinit, Pinit, Minit, Zmc, MdlSqn, flag);           % 滤波过程

%%
% save demo_vsimm.mat

%%
%%% 滤波轨迹
figure(11)
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(Z(1,:),Z(2,:),'-.k');
hold off
axis equal, box on, grid on
legend('真实','滤波','观测')

%%
%%% 位置滤波误差
figure(21)
hold on
plot(Xtrue(1,:)-Xhat(1,:),'-.k');
plot(Xtrue(4,:)-Xhat(3,:),'-k');
hold off
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x轴','y轴')

%%
%%% 速度滤波误差
figure(22)
hold on
plot(Xtrue(2,:)-Xhat(2,:),'-.k');
plot(Xtrue(5,:)-Xhat(4,:),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x轴','y轴')

%%
% kk = 2;
% %%% 位置滤波误差
% figure(31)
% hold on
% plot(Xtrue(1,:)-XhatMc(1,:,kk),'-.k');
% plot(Xtrue(4,:)-XhatMc(3,:,kk),'-k');
% hold off
% xlim([0,1300])
% ylim([-200,200])
% box on, grid on
% legend('x轴','y轴')

%%
%%% 速度滤波误差
% figure(32)
% hold on
% plot(Xtrue(2,:)-XhatMc(2,:,kk),'-.k');
% plot(Xtrue(5,:)-XhatMc(4,:,kk),'-k');
% hold off
% xlim([0,1300])
% ylim([-100,100])
% box on, grid on
% legend('x轴','y轴')

