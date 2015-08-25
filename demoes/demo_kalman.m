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
%%% 观测模型
R = [10000,0; 0,10000];                                                 % 观测噪声协方差阵
StateSym = hd_mtnmodel.StateSym;                                        % 状态符号变量
hd_msmodel = msmodel.dxy(R,StateSym);                                   % 观测模型
%%% 滤波器模型
hd = mttfilter.kalman(hd_mtnmodel,hd_msmodel);                          % 滤波器模型
%%% 滤波过程
Xtrue = trajectory.dxy;                                                 % 目标真实状态
Z = [Xtrue(1,:);Xtrue(4,:)]+randn(2,size(Xtrue,2))*100;                 % 观测向量序列
[Xinit, Pinit] = twopointsinit(Z(:,1:2), [Swx,Swy], T, R);              % 滤波器起始值（两点起始法
%%% 利用predict1和update1进行滤波
DimState = 4;
DimMeasure = 2;
NumStep = size(Xtrue,2);
Xhat1 = zeros(DimState,NumStep);
Phat1 = zeros(DimState,DimState,NumStep);
h = waitbar(0,'0%','Name','Stadard Kalman Filtering Progress ...',...
            'CreateCancelBtn',...
            'setappdata(gcbf,''canceling'',1)');
setappdata(h,'canceling',0)
for kk = 1:1:NumStep
    if getappdata(h,'canceling')
        break
    end
    [XhatPre, PhatPre] = hd.predict(Xinit,Pinit,Z(:,kk));
    [Xhat, Phat] = hd.update(XhatPre,PhatPre,Z(:,kk));
    Xinit = Xhat; Pinit = Phat;
    Xhat1(:,kk) = Xhat; Phat1(:,:,kk) = Phat;
    waitbar(kk/NumStep,h,sprintf('%3.0f %%',kk*100/NumStep))
end
delete(h)
%%% 利用filter进行滤波
flag = 1;                                                               % 进度条标示
% [Xhat,Phat] = hd.filter(Z(:,3:end),Xinit,Pinit,flag);                   % 滤波
Xhat = hd.filter(Xinit,Pinit,Z(:,3:end),flag);
%%% 利用mcfilter进行蒙特卡洛滤波
NumMc = 10;                                                             % 蒙特卡洛仿真次数
Zmc = repmat(Z,[1,1,NumMc]);                                            % 观测
XhatMc = hd.mcfilter(Xinit,Pinit,Zmc,flag);                             % 滤波过程

%%
% save demo_kalman.mat

%%
%%% 滤波轨迹
figure(11)
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-k');
plot(Z(1,:),Z(2,:),'-.k');
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

