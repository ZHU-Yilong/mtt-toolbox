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
% H = [1,0,0,0; 0,0,1,0];                                                 % 观测矩阵
% R1 = [10000,0; 0,10000];                                                % 观测噪声协方差阵
% hd_msmodel1 = msmodel.dxy(H,R1);                                        % 观测模型
% R2 = [10000,0; 0,10000];
% hd_msmodel2 = msmodel.dxy(H,R2); 
R1 = [10000,0; 0,0.0001];                                                % 观测噪声协方差阵
hd_msmodel1 = msmodel.drb(R1);                                            % 观测模型
R2 = [10000,0; 0,0.00001];
hd_msmodel2 = msmodel.drb(R2);
%%% 滤波器模型
hdd1 = mttfilter.ukf(hd_mtnmodel1,hd_msmodel1);                      % 滤波器模型
hdd2 = mttfilter.ukf(hd_mtnmodel1,hd_msmodel2);
%%% 滤波器参数
lambda = 0.0004;
gamma = 100;
Pg = 0.9997;
Pd = 1;
para = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];                         % 滤波器参数
hd1 = mttfilter.mspdaf({hdd1;hdd2}, para);
%%% 
Sw = 50;
Omega = deg2rad(6);
hd_mtnmodel_sub = b2model.ct(Omega,Sw,T);
hd_mtnmodel2 = mtnmodel.dxy(hd_mtnmodel_sub);
hdd3 = mttfilter.ukf(hd_mtnmodel2,hd_msmodel1);
hdd4 = mttfilter.ukf(hd_mtnmodel2,hd_msmodel2);
hd2 = mttfilter.mspdaf({hdd3,hdd4}, para);
%%% 
hd = mttfilter.immmspdaf({hd1;hd2});

%%% 滤波过程
Xtrue = trajectory.dxy;                                                 % 目标真实状态
DimMeasure = 2;
NumStep = size(Xtrue,2);
% Z1 = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*50;              % 观测向量序列
% Z2 = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*10;
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));
Z1 = [rho+randn(1,NumStep)*sqrt(R1(1,1));theta+randn(1,NumStep)*sqrt(R1(2,2))];
Z2 = [rho+randn(1,NumStep)*sqrt(R2(1,1));theta+randn(1,NumStep)*sqrt(R2(2,2))];
%%%
nc1 = 2;
Av1 = nc1/10/lambda;
q1 = sqrt(10*Av1)/2;
nc2 = 2;
Av2 = nc2/10/lambda;
q2 = sqrt(10*Av2)/2;
Zclutter = cell(2,NumStep);
[Xmeasure1,Ymeasure1] = pol2cart(Z1(2,:),Z1(1,:));
[Xmeasure2,Ymeasure2] = pol2cart(Z2(2,:),Z2(1,:));
for kk = 1:1:NumStep
    CartMeasure1 = repmat([Xmeasure1(kk);Ymeasure1(kk)]-[q1;q1],1,nc1)+rand(DimMeasure,nc1)*2*q1;  % 杂波条件下观测
    [PolMeasureThe1,PolMeasureRho1] = cart2pol(CartMeasure1(1,:),CartMeasure1(2,:));
    Zclutter{1,kk} = [PolMeasureRho1;PolMeasureThe1];
    CartMeasure2 = repmat([Xmeasure2(kk);Ymeasure2(kk)]-[q2;q2],1,nc2)+rand(DimMeasure,nc2)*2*q2;  % 杂波条件下观测
    [PolMeasureThe2,PolMeasureRho2] = cart2pol(CartMeasure2(1,:),CartMeasure2(2,:));
    Zclutter{2,kk} = [PolMeasureRho2;PolMeasureThe2];
end
load demo_immmspdaf_ukf_zclutter.mat
% [Xinit, Pinit] = twopointsinit(Z2(:,1:2), [Swx,Swy], T, R2);            % 滤波器起始值（两点起始法）
Xinit = [Xtrue(1:2,2);Xtrue(4:5,2)];
Pinit = blkdiag(100^2,200^2,100^2,200^2);
flag = 1;                                                               % 进度条标示
[Xhat,Phat,Mu] = hd.filter(Zclutter(:,3:end),Xinit,Pinit,flag);         % 滤波

%%
save demo_immmspdaf_ukf.mat

%%
%%% 滤波轨迹
jj = 1;
Xmeasure1 = zeros(NumStep,1);
Ymeasure1 = zeros(NumStep,1);
for kk = 1:1:NumStep
    [Xmeasure1(kk),Ymeasure1(kk)] = pol2cart(Zclutter{1,kk}(2,jj),Zclutter{1,kk}(1,jj));
end
figure(11)
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(Xmeasure1,Ymeasure1,'-.b');
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
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x轴','y轴')

%%
%%% 速度滤波误差
figure(22)
hold on
plot(Xtrue(2,3:end)-Xhat(2,:),'-.k');
plot(Xtrue(5,3:end)-Xhat(4,:),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x轴','y轴')

%%
figure(31)
hold on
plot(Mu(1,:),'-k')
plot(Mu(2,:),'-.k')
hold off
xlim([0,1300])
ylim([0,1])
box on, grid on
legend('模型1','模型2')

