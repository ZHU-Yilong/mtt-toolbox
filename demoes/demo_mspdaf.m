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
Swx1 = 50;                                                              % x轴过程噪声功率谱密度
Swy1 = 50;                                                              % y轴过程噪声功率谱密度
T = 0.1;                                                                % 时间采样间隔
hd_mtnmodel_x1 = b1model.cv(Swx1,T);                                    % x轴运动模型
hd_mtnmodel_y1 = b1model.cv(Swy1,T);                                    % y轴运动模型
hd_mtnmodel1 = mtnmodel.dxdy(hd_mtnmodel_x1,hd_mtnmodel_y1);            % 运动模型
Swx2 = 100;                                                               % x轴过程噪声功率谱密度
Swy2 = 100;                                                               % y轴过程噪声功率谱密度
hd_mtnmodel_x2 = b1model.cv(Swx2,T);                                    % x轴运动模型
hd_mtnmodel_y2 = b1model.cv(Swy2,T);                                    % y轴运动模型
hd_mtnmodel2 = mtnmodel.dxdy(hd_mtnmodel_x2,hd_mtnmodel_y2);            % 运动模型
%%% 观测模型
R1 = [10000,0; 0,10000];                                                % 观测噪声协方差阵
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel1 = msmodel.dxy(R1,StateSym);                                 % 观测模型
R2 = [10000,0; 0,10000];
hd_msmodel2 = msmodel.dxy(R2,StateSym); 
%%% 滤波器模型
hdd1 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel1);                      % 滤波器模型
hdd2 = mttfilter.kalman(hd_mtnmodel2,hd_msmodel2);
hdd = {hdd1;hdd2};
%%% 滤波器参数
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];                         % 滤波器参数
hd = mttfilter.mspdaf(hdd, para);
%%% 滤波过程
Xtrue = trajectory.dxy;                                                 % 目标真实状态
DimMeasure = 2;
NumStep = size(Xtrue,2);
Z1 = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*50;              % 观测向量序列
Z2 = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*10;
%%%
nc1 = 5;
Av1 = nc1/10/lambda;
q1 = sqrt(10*Av1)/2;
nc2 = 10;
Av2 = nc2/10/lambda;
q2 = sqrt(10*Av2)/2;
Zclutter = cell(2,NumStep);
for kk = 1:1:NumStep
    Zclutter{1,kk} = repmat(Z1(:,kk)-[q1;q1],1,nc1)+rand(DimMeasure,nc1)*2*q1;  % 杂波条件下观测
    Zclutter{2,kk} = repmat(Z2(:,kk)-[q2;q2],1,nc2)+rand(DimMeasure,nc2)*2*q2;
end
[Xinit, Pinit] = twopointsinit(Z2(:,1:2), [Swx2,Swy2], T, R2);          % 滤波器起始值（两点起始法）

%%
%%%
flag = 1;                                                               % 进度条标示
[Xhat,Phat] = hd.filter(Xinit, Pinit, Zclutter, flag);                  % 滤波

%%
% save demo_mspdaf.mat

%%
%%% 滤波轨迹
figure(11)
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(Z1(1,:),Z1(2,:),'-.b');
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

