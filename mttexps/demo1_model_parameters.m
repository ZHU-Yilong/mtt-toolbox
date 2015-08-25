%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiments of motion model
%%% with different parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

%%
load measurement.mat
% NumMC = 50;
PseudoMeasure = PseudoMeasure(:,:,1:NumMC);

%%
%%%
prompt = ['请选择运动模型：[1]\n',...
    '  1. Constant Velocity (CV)\n',...
    '  2. Constant Acceleration (CA)\n',...
    '  3. Singer\n',...
    '  4. Current Statistic\n'];
ModelType = input(prompt);
while(~isempty(ModelType) && ModelType~=1 && ModelType~=2 && ModelType~=3 && ModelType~=4)
    clc;
    fprintf( '请输入正确的序号！\n');
    ModelType = input(prompt);
end
if (isempty(ModelType))
    ModelType = 1;
end
switch ModelType
    case 1
        fprintf('已选择CV运动模型\n');
        fprintf('待评估模型参数如下：\n type1: Sw = 10;\n type2: Sw = 50;\n type3: Sw = 100;\n type4: Sw = 1000;\nt type5: Sw = 100000.\n\n');
    case 2
        fprintf('已选择CA运动模型\n');
        fprintf('待评估模型参数如下：\n type1: Sw = 10;\n type2: Sw = 50;\n type3: Sw = 100;\n type4: Sw = 1000;\nt type5: Sw = 100000.\n\n');
    case 3
        fprintf('已选择Singer运动模型\n');
        fprintf('待评估模型参数如下：\n type1: Alpha = 0.01;\n type2: Alpha = 0.1;\n type3: Alpha = 0.5;\n type4: Alpha = 0.9;\n type5: Alpha = 0.99.\n\n');
    case 4
        fprintf('已选择当前统计运动模型\n');
        fprintf('待评估模型参数如下：\n type1: Alpha = 0.01;\n type2: Alpha = 0.1;\n type3: Alpha = 0.5;\n type4: Alpha = 0.9;\n type5: Alpha = 0.99.\n\n');
    otherwise
end

prompt = ['请选择滤波器：[1]\n',...
    '  1. Extended Kalman Filter (EKF)\n',...
    '  2. Second-Order EKF\n',...
    '  3. Iterated EKF\n',...
    '  4. Unscented Kalman Filter (UKF)\n',...
    '  5. Particle Filter (PF)\n',...
    '  6. Coverted Measurement Kalman Filter (CMKF)\n',...
    '  7. Line-of-sight Coordiante System Filter\n'];
FilterType = input(prompt);
while( ~isempty(FilterType) && (FilterType<1 || FilterType>7) )
    clc;
    fprintf('请输入正确的序号！\n');
    FilterType = input(prompt);
end
if (isempty(FilterType))
    FilterType = 1;
end
switch FilterType
    case 1
        fprintf('已选择扩展Kalman滤波器(EKF)\n');
    case 2
        fprintf('已选择二阶扩展Kalman滤波器\n');
    case 3
        fprintf('已选择迭代扩展Kalman滤波器\n');
    case 4
        fprintf('已选择无迹Kalman滤波器(UKF)\n');
    case 5
        fprintf('已选择粒子滤波器(PF)\n');
    case 6
        fprintf('已选择转换观测的Kalman滤波器(CMKF)\n');
    case 7
        fprintf('已选择视线坐标系的滤波器\n');
    otherwise
end

%%
%%% 
T = 0.02;                                                               % 时间采样间隔
%%% 运动模型
switch ModelType
    case 1        
        alpha_type1 = 10; Swy_type1 = 10; Swz_type1 = 10;
        hd_mtnmodel_x = b1model.cv(alpha_type1,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.cv(Swy_type1,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.cv(Swz_type1,T);
        hd_mtnmodel_type1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 2
        alpha_type2 = 50; Swy_type2 = 50; Swyz_type2 = 50;
        hd_mtnmodel_x = b1model.cv(alpha_type2,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.cv(Swy_type2,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.cv(Swy_type2,T);
        hd_mtnmodel_type2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 3
        alpha_type3 = 1e2; Swy_type3 = 1e2; Swz_type3 = 1e2;
        hd_mtnmodel_x = b1model.cv(alpha_type3,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.cv(Swy_type3,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.cv(Swz_type3,T);
        hd_mtnmodel_type3 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 4
        alpha_type4 = 1e3; Swy_type4 = 1e3; Swz_type4 = 1e3;
        hd_mtnmodel_x = b1model.cv(alpha_type4,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.cv(Swy_type4,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.cv(Swz_type4,T);
        hd_mtnmodel_type4 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 5
        Swx_type5 = 1e5; Swy_type5 = 1e5; Swz_type5 = 1e5;
        hd_mtnmodel_x = b1model.cv(Swx_type5,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.cv(Swy_type5,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.cv(Swz_type5,T);
        hd_mtnmodel_type5 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
    case 2
        alpha_type1 = 10; Swy_type1 = 10; Swz_type1 = 10;
        hd_mtnmodel_x = b1model.ca(alpha_type1,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.ca(Swy_type1,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.ca(Swz_type1,T);
        hd_mtnmodel_type1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 2
        alpha_type2 = 50; Swy_type2 = 50; Swyz_type2 = 50;
        hd_mtnmodel_x = b1model.ca(alpha_type2,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.ca(Swy_type2,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.ca(Swy_type2,T);
        hd_mtnmodel_type2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 3
        alpha_type3 = 1e2; Swy_type3 = 1e2; Swz_type3 = 1e2;
        hd_mtnmodel_x = b1model.ca(alpha_type3,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.ca(Swy_type3,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.ca(Swz_type3,T);
        hd_mtnmodel_type3 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 4
        alpha_type4 = 1e3; Swy_type4 = 1e3; Swz_type4 = 1e3;
        hd_mtnmodel_x = b1model.ca(alpha_type4,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.ca(Swy_type4,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.ca(Swz_type4,T);
        hd_mtnmodel_type4 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 5
        Swx_type5 = 1e5; Swy_type5 = 1e5; Swz_type5 = 1e5;
        hd_mtnmodel_x = b1model.ca(Swx_type5,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.ca(Swy_type5,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.ca(Swz_type5,T);
        hd_mtnmodel_type5 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
    case 3
        alpha_type1 = 0.01;
        hd_mtnmodel_x = b1model.singer(alpha_type1,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.singer(alpha_type1,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.singer(alpha_type1,T);
        hd_mtnmodel_type1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 2
        alpha_type2 = 0.1;
        hd_mtnmodel_x = b1model.singer(alpha_type2,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.singer(alpha_type2,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.singer(alpha_type2,T);
        hd_mtnmodel_type2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 3
        alpha_type3 = 0.5;
        hd_mtnmodel_x = b1model.singer(alpha_type3,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.singer(alpha_type3,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.singer(alpha_type3,T);
        hd_mtnmodel_type3 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 4
        alpha_type4 = 0.9;
        hd_mtnmodel_x = b1model.singer(alpha_type4,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.singer(alpha_type4,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.singer(alpha_type4,T);
        hd_mtnmodel_type4 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 5
        alpha_type5 = 0.99;
        hd_mtnmodel_x = b1model.singer(alpha_type5,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.singer(alpha_type5,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.singer(alpha_type5,T);
        hd_mtnmodel_type5 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
    case 4
        alpha_type1 = 0.01;
        hd_mtnmodel_x = b1model.current(alpha_type1,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.current(alpha_type1,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.current(alpha_type1,T);
        hd_mtnmodel_type1 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 2
        alpha_type2 = 0.1;
        hd_mtnmodel_x = b1model.current(alpha_type2,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.current(alpha_type2,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.current(alpha_type2,T);
        hd_mtnmodel_type2 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 3
        alpha_type3 = 0.5;
        hd_mtnmodel_x = b1model.current(alpha_type3,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.current(alpha_type3,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.current(alpha_type3,T);
        hd_mtnmodel_type3 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 4
        alpha_type4 = 0.9;
        hd_mtnmodel_x = b1model.current(alpha_type4,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.current(alpha_type4,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.current(alpha_type4,T);
        hd_mtnmodel_type4 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
        %%% CV model 5
        alpha_type5 = 0.99;
        hd_mtnmodel_x = b1model.current(alpha_type5,T);                                      % x轴运动模型
        hd_mtnmodel_y = b1model.current(alpha_type5,T);                                      % y轴运动模型
        hd_mtnmodel_z = b1model.current(alpha_type5,T);
        hd_mtnmodel_type5 = mtnmodel.dxdydz(hd_mtnmodel_x,hd_mtnmodel_y,hd_mtnmodel_z);
    otherwise        
end

%%% 观测模型
StateSym = hd_mtnmodel_type1.StateSym;
R = blkdiag(SigmaRan^2,SigmaAzi^2,SigmaEle^2);                              % 观测噪声协方差阵
hd_msmodel = msmodel.drbe(R, StateSym);                                     % 观测模型

%%% 滤波器模型
switch FilterType
    case 1
        hd_filter_type1 = mttfilter.ekf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.ekf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.ekf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.ekf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.ekf(hd_mtnmodel_type5, hd_msmodel);
    case 2
        hd_filter_type1 = mttfilter.soekf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.soekf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.soekf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.soekf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.soekf(hd_mtnmodel_type5, hd_msmodel);
    case 3
        hd_filter_type1 = mttfilter.iekf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.iekf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.iekf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.iekf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.iekf(hd_mtnmodel_type5, hd_msmodel);
    case 4
        hd_filter_type1 = mttfilter.ukf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.ukf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.ukf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.ukf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.ukf(hd_mtnmodel_type5, hd_msmodel);
    case 5
        hd_filter_type1 = mttfilter.pf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.pf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.pf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.pf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.pf(hd_mtnmodel_type5, hd_msmodel);
    case 6
        hd_filter_type1 = mttfilter.cmkf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.cmkf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.cmkf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.cmkf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.cmkf(hd_mtnmodel_type5, hd_msmodel);
    case 7
        hd_filter_type1 = mttfilter.lcsf(hd_mtnmodel_type1, hd_msmodel);
        hd_filter_type2 = mttfilter.lcsf(hd_mtnmodel_type2, hd_msmodel);
        hd_filter_type3 = mttfilter.lcsf(hd_mtnmodel_type3, hd_msmodel);
        hd_filter_type4 = mttfilter.lcsf(hd_mtnmodel_type4, hd_msmodel);
        hd_filter_type5 = mttfilter.lcsf(hd_mtnmodel_type5, hd_msmodel);
end

%%
%%% 滤波过程
flag = 1;       % 进度条标示
offset = length(hd_mtnmodel_type1.StateSym)/hd_mtnmodel_type1.Dimension;
if offset==1
    Xinit = [xtrue(1,1);xtrue(4,1);xtrue(7,1)];
    Pinit = blkdiag(100^2,100^2,100^2);
end
if offset==2
    Xinit = [xtrue(1:2,1);xtrue(4:5,1);xtrue(7:8,1)];
    Pinit = blkdiag(100^2,200^2,100^2,200^2,100^2,200^2);
end
if offset==3
    Xinit = xtrue(:,1);
    Pinit = blkdiag(100^2,200^2,200^2,100^2,200^2,200^2,100^2,200^2,200^2);
end

fprintf('请按任意键继续type1运动模型Monte Carlo滤波过程！\n');pause;
[Xhat_type1, Phat_type1] = hd_filter_type1.mcfilter(Xinit,Pinit,PseudoMeasure,flag);       % 滤波
fprintf('请按任意键继续type2运动模型Monte Carlo滤波过程！\n');pause;
[Xhat_type2, Phat_type2] = hd_filter_type2.mcfilter(Xinit,Pinit,PseudoMeasure,flag);       % 滤波
fprintf('请按任意键继续type3运动模型Monte Carlo滤波过程！\n');pause;
[Xhat_type3, Phat_type3] = hd_filter_type3.mcfilter(Xinit,Pinit,PseudoMeasure,flag);       % 滤波
fprintf('请按任意键继续type4运动模型Monte Carlo滤波过程！\n');pause;
[Xhat_type4, Phat_type4] = hd_filter_type4.mcfilter(Xinit,Pinit,PseudoMeasure,flag);       % 滤波
fprintf('请按任意键继续type5运动模型Monte Carlo滤波过程！\n');pause;
[Xhat_type5, Phat_type5] = hd_filter_type5.mcfilter(Xinit,Pinit,PseudoMeasure,flag);       % 滤波

%%
%%% performance evaluation
fprintf('请按任意键继续性能评估分析过程！\n');pause;

%%
%%% index No.1 RMSE
fprintf('正在分析指标1：均方根误差RMSE...\n');
if offset>0
    IndexPosRMSE_type1 = rmse(Xhat_type1(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosRMSE_type2 = rmse(Xhat_type2(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosRMSE_type3 = rmse(Xhat_type3(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosRMSE_type4 = rmse(Xhat_type4(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosRMSE_type5 = rmse(Xhat_type5(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
end
if offset>1
    IndexVelRMSE_type1 = rmse(Xhat_type1(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelRMSE_type2 = rmse(Xhat_type2(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelRMSE_type3 = rmse(Xhat_type3(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelRMSE_type4 = rmse(Xhat_type4(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelRMSE_type5 = rmse(Xhat_type5(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
end
if offset>2
    IndexAccRMSE_type1 = rmse(Xhat_type1(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccRMSE_type2 = rmse(Xhat_type2(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccRMSE_type3 = rmse(Xhat_type3(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccRMSE_type4 = rmse(Xhat_type4(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccRMSE_type5 = rmse(Xhat_type5(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
end

%%
%%% index No.2 AEE
fprintf('正在分析指标2：平均欧氏误差AEE...\n');
if offset>0
    IndexPosAEE_type1 = aee(Xhat_type1(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosAEE_type2 = aee(Xhat_type2(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosAEE_type3 = aee(Xhat_type3(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosAEE_type4 = aee(Xhat_type4(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosAEE_type5 = aee(Xhat_type5(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
end
if offset>1
    IndexVelAEE_type1 = aee(Xhat_type1(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelAEE_type2 = aee(Xhat_type2(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelAEE_type3 = aee(Xhat_type3(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelAEE_type4 = aee(Xhat_type4(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelAEE_type5 = aee(Xhat_type5(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
end
if offset>2
    IndexAccAEE_type1 = aee(Xhat_type1(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccAEE_type2 = aee(Xhat_type2(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccAEE_type3 = aee(Xhat_type3(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccAEE_type4 = aee(Xhat_type4(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccAEE_type5 = aee(Xhat_type5(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
end

%%
%%% index No.3 GAE / LGAE
fprintf('正在分析指标3：几何平均误差GAE/对数几何平均误差LGAE...\n');
if offset>0
    IndexPosGAE_type1 = gae(Xhat_type1(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosGAE_type2 = gae(Xhat_type2(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosGAE_type3 = gae(Xhat_type3(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosGAE_type4 = gae(Xhat_type4(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosGAE_type5 = gae(Xhat_type5(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));    
    IndexPosLGAE_type1 = lgae(Xhat_type1(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosLGAE_type2 = lgae(Xhat_type2(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosLGAE_type3 = lgae(Xhat_type3(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosLGAE_type4 = lgae(Xhat_type4(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
    IndexPosLGAE_type5 = lgae(Xhat_type5(1:offset:end,:,:)-repmat(PosTarget,[1,1,NumMC]));
end
if offset>1
    IndexVelGAE_type1 = gae(Xhat_type1(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));    
    IndexVelGAE_type2 = gae(Xhat_type2(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));    
    IndexVelGAE_type3 = gae(Xhat_type3(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));    
    IndexVelGAE_type4 = gae(Xhat_type4(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));   
    IndexVelGAE_type5 = gae(Xhat_type5(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelLGAE_type1 = lgae(Xhat_type1(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelLGAE_type2 = lgae(Xhat_type2(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelLGAE_type3 = lgae(Xhat_type3(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelLGAE_type4 = lgae(Xhat_type4(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
    IndexVelLGAE_type5 = lgae(Xhat_type5(2:offset:end,:,:)-repmat(VelTarget,[1,1,NumMC]));
end
if offset>2
    IndexAccGAE_type1 = gae(Xhat_type1(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccGAE_type2 = gae(Xhat_type2(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccGAE_type3 = gae(Xhat_type3(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccGAE_type4 = gae(Xhat_type4(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccGAE_type5 = gae(Xhat_type5(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccLGAE_type1 = lgae(Xhat_type1(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccLGAE_type2 = lgae(Xhat_type2(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccLGAE_type3 = lgae(Xhat_type3(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccLGAE_type4 = lgae(Xhat_type4(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
    IndexAccLGAE_type5 = lgae(Xhat_type5(3:offset:end,:,:)-repmat(AccTarget,[1,1,NumMC]));
end

%%
%%% index No.4 MERF1 / MERF2 / LMERF
fprintf('正在分析指标4：观测误差缩减系数MERF...\n');
[PseudoMeasureTrue(2,:),PseudoMeasureTrue(3,:),PseudoMeasureTrue(1,:)] = ...
    cart2sph(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:));
[PseudoMeasureEstimate_type1(2,:,:),PseudoMeasureEstimate_type1(3,:,:),PseudoMeasureEstimate_type1(1,:,:)] = ...
    cart2sph(Xhat_type1(1,:,:),Xhat_type1(1+offset,:,:),Xhat_type1(1+2*offset,:,:));
[PseudoMeasureEstimate_type2(2,:,:),PseudoMeasureEstimate_type2(3,:,:),PseudoMeasureEstimate_type2(1,:,:)] = ...
    cart2sph(Xhat_type2(1,:,:),Xhat_type2(1+offset,:,:),Xhat_type2(1+2*offset,:,:));
[PseudoMeasureEstimate_type3(2,:,:),PseudoMeasureEstimate_type3(3,:,:),PseudoMeasureEstimate_type3(1,:,:)] = ...
    cart2sph(Xhat_type3(1,:,:),Xhat_type3(1+offset,:,:),Xhat_type3(1+2*offset,:,:));
[PseudoMeasureEstimate_type4(2,:,:),PseudoMeasureEstimate_type4(3,:,:),PseudoMeasureEstimate_type4(1,:,:)] = ...
    cart2sph(Xhat_type4(1,:,:),Xhat_type4(1+offset,:,:),Xhat_type4(1+2*offset,:,:));
[PseudoMeasureEstimate_type5(2,:,:),PseudoMeasureEstimate_type5(3,:,:),PseudoMeasureEstimate_type5(1,:,:)] = ...
    cart2sph(Xhat_type5(1,:,:),Xhat_type5(1+offset,:,:),Xhat_type5(1+2*offset,:,:));
%%%
IndexMERF1_type1 = merf1(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type1,PseudoMeasure);
IndexMERF2_type1 = merf2(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type1,PseudoMeasure);
IndexLMERF_type1 = lmerf(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type1,PseudoMeasure);
%%%
IndexMERF1_type2 = merf1(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type2,PseudoMeasure);
IndexMERF2_type2 = merf2(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type2,PseudoMeasure);
IndexLMERF_type2 = lmerf(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type2,PseudoMeasure);
%%%
IndexMERF1_type3 = merf1(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type3,PseudoMeasure);
IndexMERF2_type3 = merf2(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type3,PseudoMeasure);
IndexLMERF_type3 = lmerf(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type3,PseudoMeasure);
%%%
IndexMERF1_type4 = merf1(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type4,PseudoMeasure);
IndexMERF2_type4 = merf2(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type4,PseudoMeasure);
IndexLMERF_type4 = lmerf(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type4,PseudoMeasure);
%%%
IndexMERF1_type5 = merf1(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type5,PseudoMeasure);
IndexMERF2_type5 = merf2(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type5,PseudoMeasure);
IndexLMERF_type5 = lmerf(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type5,PseudoMeasure);

%%
%%% index No.5 EERF1 / EERF2 / LEERF
fprintf('正在分析指标5：估计误差缩减系数EERF...\n');
[StateNoisy(1,:,:),StateNoisy(2,:,:),StateNoisy(3,:,:)] = ...
    sph2cart(PseudoMeasure(2,:,:),PseudoMeasure(3,:,:),PseudoMeasure(1,:,:));
%%%
IndexEERF1_type1 = eerf1(repmat(PosTarget,[1,1,NumMC]),Xhat_type1(1:offset:end,:,:),StateNoisy);
IndexEERF2_type1 = eerf2(repmat(PosTarget,[1,1,NumMC]),Xhat_type1(1:offset:end,:,:),StateNoisy);
IndexLEERF_type1 = leerf(repmat(PosTarget,[1,1,NumMC]),Xhat_type1(1:offset:end,:,:),StateNoisy);
%%%
IndexEERF1_type2 = eerf1(repmat(PosTarget,[1,1,NumMC]),Xhat_type2(1:offset:end,:,:),StateNoisy);
IndexEERF2_type2 = eerf2(repmat(PosTarget,[1,1,NumMC]),Xhat_type2(1:offset:end,:,:),StateNoisy);
IndexLEERF_type2 = leerf(repmat(PosTarget,[1,1,NumMC]),Xhat_type2(1:offset:end,:,:),StateNoisy);
%%%
IndexEERF1_type3 = eerf1(repmat(PosTarget,[1,1,NumMC]),Xhat_type3(1:offset:end,:,:),StateNoisy);
IndexEERF2_type3 = eerf2(repmat(PosTarget,[1,1,NumMC]),Xhat_type3(1:offset:end,:,:),StateNoisy);
IndexLEERF_type3 = leerf(repmat(PosTarget,[1,1,NumMC]),Xhat_type3(1:offset:end,:,:),StateNoisy);
%%%
IndexEERF1_type4 = eerf1(repmat(PosTarget,[1,1,NumMC]),Xhat_type4(1:offset:end,:,:),StateNoisy);
IndexEERF2_type4 = eerf2(repmat(PosTarget,[1,1,NumMC]),Xhat_type4(1:offset:end,:,:),StateNoisy);
IndexLEERF_type4 = leerf(repmat(PosTarget,[1,1,NumMC]),Xhat_type4(1:offset:end,:,:),StateNoisy);
%%%
IndexEERF1_type5 = eerf1(repmat(PosTarget,[1,1,NumMC]),Xhat_type5(1:offset:end,:,:),StateNoisy);
IndexEERF2_type5 = eerf2(repmat(PosTarget,[1,1,NumMC]),Xhat_type5(1:offset:end,:,:),StateNoisy);
IndexLEERF_type5 = leerf(repmat(PosTarget,[1,1,NumMC]),Xhat_type5(1:offset:end,:,:),StateNoisy);

%%
%%% index No.6 SR
fprintf('正在分析指标6：有效率SR...\n');
AlphaSucc = 2:2:200;
IndexSR_type1 = zeros(NumStep,length(AlphaSucc));
IndexSR_type2 = zeros(NumStep,length(AlphaSucc));
IndexSR_type3 = zeros(NumStep,length(AlphaSucc));
IndexSR_type4 = zeros(NumStep,length(AlphaSucc));
IndexSR_type5 = zeros(NumStep,length(AlphaSucc));
for kk = 1:1:length(AlphaSucc)
    IndexSR_type1(:,kk) = succrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type1,PseudoMeasure,AlphaSucc(kk));
    IndexSR_type2(:,kk) = succrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type2,PseudoMeasure,AlphaSucc(kk));
    IndexSR_type3(:,kk) = succrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type3,PseudoMeasure,AlphaSucc(kk)); 
    IndexSR_type4(:,kk) = succrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type4,PseudoMeasure,AlphaSucc(kk)); 
    IndexSR_type5(:,kk) = succrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type5,PseudoMeasure,AlphaSucc(kk)); 
end

%%
%%% index No.7 FR
fprintf('正在分析指标7：失效率FR...\n');
AlphaFail = 202:2:400;
IndexFR_type1 = zeros(NumStep,length(AlphaFail));
IndexFR_type2 = zeros(NumStep,length(AlphaFail));
IndexFR_type3 = zeros(NumStep,length(AlphaFail));
IndexFR_type4 = zeros(NumStep,length(AlphaFail));
IndexFR_type5 = zeros(NumStep,length(AlphaFail));
for kk = 1:1:length(AlphaFail)
    IndexFR_type1(:,kk) = failrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type1,PseudoMeasure,AlphaFail(kk));
    IndexFR_type2(:,kk) = failrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type2,PseudoMeasure,AlphaFail(kk));
    IndexFR_type3(:,kk) = failrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type3,PseudoMeasure,AlphaFail(kk));
    IndexFR_type4(:,kk) = failrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type4,PseudoMeasure,AlphaFail(kk));
    IndexFR_type5(:,kk) = failrate(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate_type5,PseudoMeasure,AlphaFail(kk));    
end

%%
%%% save data files
fprintf('请按任意键继续数据存储过程！\n');pause;
save demo_model_parameters.mat

%%
%%% filtered trajectory
fprintf('请按任意键继续展示滤波效果图\n');pause;
figure(1)
kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(PseudoMeasure(2,:,kk),PseudoMeasure(3,:,kk),PseudoMeasure(1,:,kk));
hold on
plot3(PosTarget(1,:),PosTarget(2,:),PosTarget(3,:),'-k');
plot3(Xhat_type1(1,:,kk),Xhat_type1(1+offset,:,kk),Xhat_type1(1+2*offset,:,kk),'-r');
plot3(Xhat_type2(1,:,kk),Xhat_type2(1+offset,:,kk),Xhat_type2(1+2*offset,:,kk),'-b');
plot3(Xhat_type3(1,:,kk),Xhat_type3(1+offset,:,kk),Xhat_type3(1+2*offset,:,kk),'-c');
plot3(Xhat_type4(1,:,kk),Xhat_type4(1+offset,:,kk),Xhat_type4(1+2*offset,:,kk),'-m');
plot3(Xhat_type5(1,:,kk),Xhat_type5(1+offset,:,kk),Xhat_type5(1+2*offset,:,kk),'-y');
scatter3(MeasureX,MeasureY,MeasureZ,3);
hold off
axis equal, box on, grid on
legend('真实','type1','type2','type3','type4','type5','观测')
view(3)

%%
%%%
TotalTime = length(time)*T;
prompt = ['请选择性能评估指标：[1]\n',...
    '  1.  均方根误差(RMSE)\n',...
    '  2.  平均欧氏误差(AEE)\n',...
    '  3.  几何平均误差(GAE / LGAE)\n',...
    '  4.  观测误差缩减系数(MERF1 / MERF2 / LMERF)\n',...
    '  5.  估计误差缩减系数(EERF1 / EERF2 / LEERF)\n',...
    '  6.  有效率(SR)\n',...
    '  7.  失效率(FR)\n',...
    '  8.  结束！\n'];
clc;
IndexInput = input(prompt);
while(1)
    if isempty(IndexInput)
        IndexInput = 1;
    end
    prompt1 = ['请继续选择性能评估指标：[1]\n',...
        '  1.  均方根误差(RMSE)\n',...
        '  2.  平均欧氏误差(AEE)\n',...
        '  3.  几何平均误差(GAE / LGAE)\n',...
        '  4.  观测误差缩减系数(MERF1 / MERF2 / LMERF)\n',...
        '  5.  估计误差缩减系数(EERF1 / EERF2 / LEERF)\n',...
        '  6.  有效率(SR)\n',...
        '  7.  失效率(FR)\n',...
        '  8.  结束！\n'];
switch IndexInput
    case 1
        %%% index 1 RMSE
        if offset>0
            figure(11)
            hold on
            plot(time,IndexPosRMSE_type1,'-r');
            plot(time,IndexPosRMSE_type2,'-b');
            plot(time,IndexPosRMSE_type4,'-c');
            plot(time,IndexPosRMSE_type4,'-m');
            plot(time,IndexPosRMSE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,60])
            legend('type1','type2','type3','type4','type5')
        end
        if offset>1
            figure(12)
            hold on
            plot(time,IndexVelRMSE_type1,'-r');
            plot(time,IndexVelRMSE_type2,'-b');
            plot(time,IndexVelRMSE_type4,'-c');
            plot(time,IndexVelRMSE_type4,'-m');
            plot(time,IndexVelRMSE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,140])
            legend('type1','type2','type3','type4','type5')
        end
        if offset>2
            figure(13)
            hold on
            plot(time,IndexAccRMSE_type1,'-r');
            plot(time,IndexAccRMSE_type2,'-b');
            plot(time,IndexAccRMSE_type4,'-c');
            plot(time,IndexAccRMSE_type4,'-m');
            plot(time,IndexAccRMSE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,140])
            legend('type1','type2','type3','type4','type5')
        end
        clc;
        IndexInput = input(prompt1);
        
    case 2
        %%% index 2 AEE
        if offset>0
            figure(21)
            hold on
            plot(time,IndexPosAEE_type1,'-r');
            plot(time,IndexPosAEE_type2,'-b');
            plot(time,IndexPosAEE_type3,'-c');
            plot(time,IndexPosAEE_type4,'-m');
            plot(time,IndexPosAEE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,200])
            legend('type1','type2','type3','type4','type5')
        end
        if offset>1
            figure(22)
            hold on
            plot(time,IndexVelAEE_type1,'-r');
            plot(time,IndexVelAEE_type2,'-b');
            plot(time,IndexVelAEE_type3,'-c');
            plot(time,IndexVelAEE_type4,'-m');
            plot(time,IndexVelAEE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,100])
            legend('type1','type2','type3','type4','type5')
        end
        if offset>2
            figure(23)
            hold on
            plot(time,IndexAccAEE_type1,'-r');
            plot(time,IndexAccAEE_type2,'-b');
            plot(time,IndexAccAEE_type3,'-c');
            plot(time,IndexAccAEE_type4,'-m');
            plot(time,IndexAccAEE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,100])
            legend('type1','type2','type3','type4','type5')
        end
        clc;
        IndexInput = input(prompt1);
        
    case 3
        %%% index 3 GAE / LGAE
        if offset>0
            figure(311)
            hold on
            plot(time,IndexPosGAE_type1,'-r');
            plot(time,IndexPosGAE_type2,'-b');
            plot(time,IndexPosGAE_type3,'-c');
            plot(time,IndexPosGAE_type4,'-m');
            plot(time,IndexPosGAE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,200])
            legend('type1','type2','type3','type4','type5')
            figure(312)
            hold on
            plot(time,IndexPosLGAE_type1,'-r');
            plot(time,IndexPosLGAE_type2,'-b');
            plot(time,IndexPosLGAE_type3,'-c');
            plot(time,IndexPosLGAE_type4,'-m');
            plot(time,IndexPosLGAE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,200])
            legend('type1','type2','type3','type4','type5')
        end
        if offset>1
            figure(321)
            hold on
            plot(time,IndexVelGAE_type1,'-r');
            plot(time,IndexVelGAE_type2,'-b');
            plot(time,IndexVelGAE_type3,'-c');
            plot(time,IndexVelGAE_type4,'-m');
            plot(time,IndexVelGAE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,100])
            legend('type1','type2','type3','type4','type5')
            figure(322)
            hold on
            plot(time,IndexVelLGAE_type1,'-r');
            plot(time,IndexVelLGAE_type2,'-b');
            plot(time,IndexVelLGAE_type3,'-c');
            plot(time,IndexVelLGAE_type4,'-m');
            plot(time,IndexVelLGAE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,100])
            legend('type1','type2','type3','type4','type5')
        end
        if offset>2
            figure(331)
            hold on
            plot(time,IndexAccGAE_type1,'-r');
            plot(time,IndexAccGAE_type2,'-b');
            plot(time,IndexAccGAE_type3,'-c');
            plot(time,IndexAccGAE_type4,'-m');
            plot(time,IndexAccGAE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,100])
            legend('type1','type2','type3','type4','type5')
            figure(332)
            hold on
            plot(time,IndexAccLGAE_type1,'-r');
            plot(time,IndexAccLGAE_type2,'-b');
            plot(time,IndexAccLGAE_type3,'-c');
            plot(time,IndexAccLGAE_type4,'-m');
            plot(time,IndexAccLGAE_type5,'-y');
            hold off
            grid on, box on
            xlim([0,TotalTime])
            % ylim([0,100])
            legend('type1','type2','type3','type4','type5')
        end
        clc;
        IndexInput = input(prompt1);
        
    case 4
        %%% index 4 MERF1 / MERF2 / LMERF
        figure(41)
        hold on
        plot(time,IndexMERF1_type1,'-r');
        plot(time,IndexMERF1_type2,'-b');
        plot(time,IndexMERF1_type3,'-c');
        plot(time,IndexMERF1_type4,'-m');
        plot(time,IndexMERF1_type5,'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        % ylim([0,200])
        legend('type1','type2','type3','type4','type5')

        figure(42)
        hold on
        plot(time,IndexMERF2_type1,'-r');
        plot(time,IndexMERF2_type2,'-b');
        plot(time,IndexMERF2_type3,'-c');
        plot(time,IndexMERF2_type4,'-m');
        plot(time,IndexMERF2_type5,'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        % ylim([0,200])
        legend('type1','type2','type3','type4','type5')

        figure(43)
        hold on
        plot(time,IndexLMERF_type1,'-r');
        plot(time,IndexLMERF_type2,'-b');
        plot(time,IndexLMERF_type3,'-c');
        plot(time,IndexLMERF_type4,'-m');
        plot(time,IndexLMERF_type5,'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        % ylim([0,200])
        legend('type1','type2','type3','type4','type5')
        clc;
        IndexInput = input(prompt);
        
    case 5
        %%% index 5 EERF1 / EERF2 / LEERF
        figure(51)
        hold on
        plot(time,IndexEERF1_type1,'-r');
        plot(time,IndexEERF1_type2,'-b');
        plot(time,IndexEERF1_type3,'-c');
        plot(time,IndexEERF1_type4,'-m');
        plot(time,IndexEERF1_type5,'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        % ylim([0,200])
        legend('type1','type2','type3','type4','type5')

        figure(52)
        hold on
        plot(time,IndexEERF2_type1,'-r');
        plot(time,IndexEERF2_type2,'-b');
        plot(time,IndexEERF2_type3,'-c');
        plot(time,IndexEERF2_type4,'-m');
        plot(time,IndexEERF2_type5,'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        % ylim([0,200])
        legend('type1','type2','type3','type4','type5')

        figure(53)
        hold on
        plot(time,IndexLEERF_type1,'-r');
        plot(time,IndexLEERF_type2,'-b');
        plot(time,IndexLEERF_type3,'-c');
        plot(time,IndexLEERF_type4,'-m');
        plot(time,IndexLEERF_type5,'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        % ylim([0,200])
        legend('type1','type2','type3','type4','type5')
        clc;
        IndexInput = input(prompt1);
        
    case 6
        %%% Index 6 Sucess Rate
        kk = 1;
        figure(61)
        hold on
        plot(time,IndexSR_type1(:,kk),'-r');
        plot(time,IndexSR_type2(:,kk),'-b');
        plot(time,IndexSR_type3(:,kk),'-c');
        plot(time,IndexSR_type4(:,kk),'-m');
        plot(time,IndexSR_type5(:,kk),'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        ylim([0.8,1])
        legend('type1','type2','type3','type4','type5')
        clc;
        IndexInput = input(prompt1);
        
    case 7
        %%% Index 7 Fail Rate
        kk = 1;
        figure(71)
        hold on
        plot(time,IndexFR_type1(:,kk),'-r');
        plot(time,IndexFR_type2(:,kk),'-b');
        plot(time,IndexFR_type3(:,kk),'-c');
        plot(time,IndexFR_type4(:,kk),'-m');
        plot(time,IndexFR_type5(:,kk),'-y');
        hold off
        grid on, box on
        xlim([0,TotalTime])
        ylim([0,0.2])
        legend('type1','type2','type3','type4','type5')
        clc;
        IndexInput = input(prompt1);
        
    case 8 
        break;
        
    otherwise
        clc;
        fprintf( '请输入正确的序号！\n');
        IndexInput = input(prompt);
end

end

