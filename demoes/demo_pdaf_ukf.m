%%
%%% 清除变量关闭打开的窗口
close all;
clear all;
% clear classes;
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
%%% 观测模型
R = [10000,0; 0,0.0001];                                                % 观测噪声协方差阵
hd_msmodel = msmodel.drb(R);                                            % 观测模型
% H = [1,0,0,0; 0,0,1,0];                                                 % 观测矩阵
% R = [10000,0; 0,10000];                                                 % 观测噪声协方差阵
% hd_msmodel = msmodel.dxy(H,R);                                          % 观测模型
%%% 滤波器模型
% hdd = mttfilter.kalman(hd_mtnmodel,hd_msmodel);                         % 滤波器模型
hdd = mttfilter.ukf(hd_mtnmodel,hd_msmodel);
%%% 滤波器参数
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda;gamma;Pg;Pd];                                            % 滤波器参数
hd = mttfilter.pdaf(hdd, para);
%%% 滤波过程
Xtrue = trajectory.dxy;                                                 % 目标真实状态
DimMeasure = 2;
NumStep = size(Xtrue,2);
% Z = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*50;               % 观测向量序列
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));                          % 真实距离和方位角
Z = [rho+randn(1,NumStep)*sqrt(R(1,1));theta+randn(1,NumStep)*sqrt(R(2,2))];    % 观测序列
%%%
nc = 3;
Av = nc/10/lambda;
q = sqrt(10*Av)/2;
Zclutter = cell(NumStep,1);
[Xmeasure,Ymeasure] = pol2cart(Z(2,:),Z(1,:));
for kk = 1:1:NumStep
    CartMeasure = repmat([Xmeasure(kk);Ymeasure(kk)]-[q;q],1,nc)+rand(DimMeasure,nc)*2*q;  % 杂波条件下观测
    [PolMeasureThe,PolMeasureRho] = cart2pol(CartMeasure(1,:),CartMeasure(2,:));
    Zclutter{kk} = [PolMeasureRho;PolMeasureThe];
end
% [Xinit, Pinit] = twopointsinit(Z(:,1:2), [Swx,Swy], T, R);              % 滤波器起始值（两点起始法）
% [Xinit, Pinit] = twopointsinit([Xtrue(1,1:2);Xtrue(4,1:2)]+randn(2,2)*100, [Swx,Swy], T, [10000,0;0,10000]);
Xinit = [Xtrue(1:2,2);Xtrue(4:5,2)];
Pinit = blkdiag(100^2,200^2,100^2,200^2);
flag = 1;                                                               % 进度条标示
[Xhat,Phat] = hd.filter(Zclutter(3:end),Xinit,Pinit,flag);              % 滤波

%%
save demo_pdaf_ukf.mat

%%
%%% 滤波轨迹
jj = 1;
Xmeasure = zeros(NumStep,1);
Ymeasure = zeros(NumStep,1);
for kk = 1:1:NumStep
    [Xmeasure(kk),Ymeasure(kk)] = pol2cart(Zclutter{kk}(2,jj),Zclutter{kk}(1,jj));
end
figure(11)
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(Xmeasure,Ymeasure,'-b');
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

