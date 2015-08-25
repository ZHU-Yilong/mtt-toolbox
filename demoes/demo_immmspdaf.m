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
hd_mtnmodel1 = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % 运动模型
%%% 观测模型
H = [1,0,0,0; 0,0,1,0];                                                 % 观测矩阵
R1 = [10000,0; 0,10000];                                                % 观测噪声协方差阵
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel1 = msmodel.dxy(R1, StateSym);                                        % 观测模型
R2 = [10000,0; 0,10000];
hd_msmodel2 = msmodel.dxy(R2, StateSym); 
%%% 滤波器模型
hdd1 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel1);                      % 滤波器模型
hdd2 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel2);
%%% 滤波器参数
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];                         % 滤波器参数
hd1 = mttfilter.mspdaf({hdd1;hdd2}, para);
%%% 
Sw = 50;
Omega = deg2rad(6);
hd_mtnmodel_sub = b2model.ct(Omega,Sw,T);
hd_mtnmodel2 = mtnmodel.dxy(hd_mtnmodel_sub);
hdd3 = mttfilter.kalman(hd_mtnmodel2,hd_msmodel1);
hdd4 = mttfilter.kalman(hd_mtnmodel2,hd_msmodel2);
hd2 = mttfilter.mspdaf({hdd3,hdd4}, para);
%%% 
hd = mttfilter.immmspdaf({hd1;hd2});

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
[Xinit, Pinit] = twopointsinit(Z2(:,1:2), [Swx,Swy], T, R2);            % 滤波器起始值（两点起始法）
flag = 1;                                                               % 进度条标示
[Xhat,Phat,Mu] = hd.filter(Xinit,Pinit,Zclutter(:,3:end),flag);         % 滤波

%%
save demo_immmspdaf.mat

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
plot(Xtrue(1,3:end)-Xhat(1,:),'-.k');
plot(Xtrue(4,3:end)-Xhat(3,:),'-k');
hold off
% xlim([0,1300])
% ylim([-200,200])
box on, grid on
legend('x轴','y轴')

%%
%%% 速度滤波误差
figure(22)
hold on
plot(Xtrue(2,3:end)-Xhat(2,:),'-.k');
plot(Xtrue(5,3:end)-Xhat(4,:),'-k');
hold off
% xlim([0,1300])
% ylim([-100,100])
box on, grid on
legend('x轴','y轴')

%%
figure(31)
hold on
plot(Mu(1,:),'-k')
plot(Mu(2,:),'-.k')
hold off
% xlim([0,1300])
% ylim([0,1])
box on, grid on
legend('模型1','模型2')

