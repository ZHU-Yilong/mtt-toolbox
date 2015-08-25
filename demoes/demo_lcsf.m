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
Swx = 100;                                                               % x轴过程噪声功率谱密度
Swy = 100;                                                               % y轴过程噪声功率谱密度
T = 0.1;                                                                % 时间采样间隔
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x轴运动模型
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y轴运动模型
hd_mtnmodel = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % 运动模型
%%% 观测模型
R = [10000,0; 0,0.0001];                                                % 观测噪声协方差阵
StateSym = hd_mtnmodel.StateSym;
hd_msmodel = msmodel.drb(R,StateSym);                                   % 观测模型
%%% 滤波器模型
hd = mttfilter.lcsf(hd_mtnmodel,hd_msmodel);
%%% 观测数据
Xtrue = trajectory.dxy;                                                 % 目标真实状态
Num = size(Xtrue,2);                                                    % 采样样本数量
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));                          % 真实距离和方位角
Z = [rho+randn(1,Num)*sqrt(R(1,1));theta+randn(1,Num)*sqrt(R(2,2))];    % 观测序列
Xinit = [Xtrue(1:2,2);Xtrue(4:5,2)];
Pinit = blkdiag(100^2,200^2,100^2,200^2);

%%
%%% 利用filter进行滤波
flag = 1; 
[Xhat,Phat] = hd.filter(Xinit,Pinit,Z,flag);                             % 滤波

%%
%%% 利用mcfilter进行蒙特卡洛滤波
NumMc = 2;                                                              % 蒙特卡洛仿真次数
Zmc = repmat(Z,[1,1,NumMc]);                                            % 观测
XhatMc = hd.mcfilter(Xinit,Pinit,Zmc,flag);                             % 滤波过程

%%
% save demo_lcsf.mat

%%
%%% 滤波轨迹
figure(11)
kk = 2;
[xx, yy] = pol2cart(Z(2,:),Z(1,:));
hold on
plot(Xtrue(1,:),Xtrue(4,:),'-c');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(XhatMc(1,:,kk),XhatMc(3,:,kk),'-m');
plot(xx,yy,'-b');
hold off
axis equal, box on, grid on
legend('真实','滤波1','滤波2','观测')

%%
%%% 位置滤波误差
figure(21)
hold on
plot(Xtrue(1,:)-Xhat(1,:),'-r');
plot(Xtrue(4,:)-Xhat(3,:),'-b');
hold off
xlim([0,1300])
% ylim([-200,200])
box on, grid on
legend('x轴','y轴')

%%
%%% 速度滤波误差
figure(22)
hold on
plot(Xtrue(2,:)-Xhat(2,:),'-r');
plot(Xtrue(5,:)-Xhat(4,:),'-b');
hold off
xlim([0,1300])
% ylim([-100,100])
box on, grid on
legend('x轴','y轴')


