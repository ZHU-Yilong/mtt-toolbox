%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% demo experiment for mtt toolbox
%%%  2014-11-10
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

%%
%%% ����Ŀ���˶��켣
% �������ڡ�����ʱ��
T = 0.01;
TotalTime = 5;

% ��ʼλ�á��ٶ�
InitPosx = 3000;
InitPosy = 0;
InitPosz = 0;
InitSpeed = 0.8 * 340;
InitAngle1 = pi;
InitAngle2 = 0;
InitPos = [InitPosx; InitPosy; InitPosz];
InitVel = [InitSpeed; InitAngle1; InitAngle2];
InitState = [InitPos; InitVel];

% ���ٶ�
g = 9.8;
Acc = 6 * g;

% ��������
direct = 1;         % 1 bank left 2 bank right 3 up 4 down
switch direct
    case 1
        ddeg = 0;
    case 2
        ddeg = pi;
    case 3
        ddeg = pi/2;
    case 4
        ddeg = pi*3/2;
end

% ������ʼ������ʱ��
TimeBegin = 0.5;
TimeEnd   = TotalTime;

NumBegin  = floor(TimeBegin/T);
NumEnd    = floor(TimeEnd/T);
NumStep   = floor(TotalTime/T);

an1 = Acc*cos(ddeg);
an2 = Acc*sin(ddeg);

an1s = [zeros(1,NumBegin), an1*ones(1,NumEnd-NumBegin), zeros(1,NumStep-NumEnd)];
an2s = [zeros(1,NumBegin), an2*ones(1,NumEnd-NumBegin), zeros(1,NumStep-NumEnd)];
ats  = zeros(1,NumStep);
VectorAcc = [ats; an1s; an2s];

ParamTime = [T,TotalTime];
ParamNum  = 100;

StateTrue = trajectory.dxyz(InitState, VectorAcc, ParamTime, ParamNum);

%%
% Ŀ���˶��켣
figure(11)
plot3(StateTrue(1,:), StateTrue(4,:), StateTrue(7,:), '-r')
grid on
box on
axis equal
xlim([1800, 3000]); ylim([-400,200]); zlim([-100,100])
view(3)


%%
% Ŀ��״̬
PosTarget = StateTrue(1:3:7, :);
VelTarget = StateTrue(2:3:8, :);
AccTarget = StateTrue(3:3:9, :);

% ����״̬����
PosMissile = zeros(3, NumStep);
VelMissile = zeros(3, NumStep);
AccMissile = zeros(3, NumStep);

% �Ƶ���ʼ��
N  = 3;
Vm = 700;
PosMissile(:,1) = [0;0;0];
VelMissile(:,1) = [Vm;0;0];

for kk = 1:1:NumStep
    % �����Ƶ�
    AccCommand = pnglaw(PosTarget(:,kk), VelTarget(:,kk), PosMissile(:,kk), VelMissile(:,kk), N);
    
    % ����λ��
    PosMissile(:,kk+1) = PosMissile(:,kk) + VelMissile(:,kk).*T + 0.5.*AccCommand.*T^2;
    VelMissile(:,kk+1) = VelMissile(:,kk) + AccCommand.*T;
    AccMissile(:,kk)   = AccCommand;
    
    % ֹͣ����
    PosRelative = PosMissile(:,kk+1) - PosTarget(:,kk+1);
    RangeRelative = sqrt(PosRelative(1)^2 + PosRelative(2)^2 + PosRelative(3)^2);
    if RangeRelative < 50
        break;
    end
end
if kk < NumStep
    NumStep = kk+1;
    PosMissile = PosMissile(:,1:kk+1);
    VelMissile = VelMissile(:,1:kk+1);
    AccMissile = AccMissile(:,1:kk+1);
    AccMissile(:,kk+1) = AccCommand;
    StateTrue = StateTrue(:,1:kk+1);
end
PosTarget = StateTrue(1:3:7, :);
VelTarget = StateTrue(2:3:8, :);
AccTarget = StateTrue(3:3:9, :);

PosRelative = PosTarget - PosMissile;
VelRelative = VelTarget - VelMissile;
AccRelative = AccTarget - AccMissile;

%%
% ��Ŀ����˶��켣
figure(21)
hold on
plot3(PosMissile(1,:), PosMissile(2,:), PosMissile(3,:), '-b')
plot3(PosTarget(1,:),  PosTarget(2,:),  PosTarget(3,:), '-r')
hold off
grid on, box on
axis equal
xlim([0,3000]); ylim([-400,200]); zlim([-100,100])
legend('boxoff'),legend('����','Ŀ��','Location','NorthEast')
view(3)

%%
% �۲�����
% ENU����ϵ��
MeasurementTrue = zeros(3,NumStep);
[MeasurementTrue(2,:), MeasurementTrue(3,:), MeasurementTrue(1,:)] = ...
    cart2sph(PosRelative(1,:), PosRelative(2,:), PosRelative(3,:));

SigmaRange = 1;
SigmaAzimu = 0.005;
SigmaEleva = 0.005;

NumMC = 2;
MeasurementNoise = repmat(MeasurementTrue, [1,1,NumMC]) + ...
    [randn(1, NumStep, NumMC) * SigmaRange; 
    randn(1, NumStep, NumMC) * SigmaAzimu; 
    randn(1, NumStep, NumMC) * SigmaEleva];

% ����ƽ̨�˶�
[MeasurementNoiseCART(1,:,:), MeasurementNoiseCART(2,:,:), MeasurementNoiseCART(3,:,:) ] =...
    sph2cart(MeasurementNoise(2,:,:), MeasurementNoise(3,:,:), MeasurementNoise(1,:,:));
PseudoMeasurementNoiseCART = MeasurementNoiseCART + repmat(PosMissile, [1,1,NumMC]);

[PseudoMeasurementNoise(2,:,:), PseudoMeasurementNoise(3,:,:), PseudoMeasurementNoise(1,:,:)] = ...
    cart2sph(PseudoMeasurementNoiseCART(1,:,:), PseudoMeasurementNoiseCART(2,:,:), PseudoMeasurementNoiseCART(3,:,:));

%%
% �۲�
figure(22)
kk = 1;
hold on
plot3(PosMissile(1,:), PosMissile(2,:), PosMissile(3,:), '-r')
plot3(PosTarget(1,:),  PosTarget(2,:),  PosTarget(3,:), '-b')
plot3(PseudoMeasurementNoiseCART(1,:,kk), PseudoMeasurementNoiseCART(2,:,kk), PseudoMeasurementNoiseCART(3,:,kk), '-g')
hold off
grid on, box on
axis equal
xlim([0,3000]); ylim([-400,200]); zlim([-100,100])
% legend('boxoff'),legend('����','Ŀ��','Location','NorthEast')
view(3)

%%
% �˶�ģ��
Swx = 10;
Swy = 10;
Swz = 10;
hd_mtnmodel_x = b1model.cv(Swx, T);
hd_mtnmodel_y = b1model.cv(Swy, T); 
hd_mtnmodel_z = b1model.cv(Swz, T);
hd_mtnmodel1 = mtnmodel.dxdydz(hd_mtnmodel_x, hd_mtnmodel_y, hd_mtnmodel_z);
Omega = deg2rad(12.5);
Sw = 10;
hd_mtnmodel_xy = b2model.ct(Omega, Sw, T);
hd_mtnmodel2 = mtnmodel.dxydz(hd_mtnmodel_xy, hd_mtnmodel_z);
Omega = deg2rad(-12.5);
Sw = 10;
hd_mtnmodel_xy = b2model.ct(Omega, Sw, T);
hd_mtnmodel3 = mtnmodel.dxydz(hd_mtnmodel_xy, hd_mtnmodel_z);

% ģ��1��CA
% Swx = 1;
% Swy = 1;
% Swz = 1;
% hd_mtnmodel_x = b1model.ca(Swx, T);
% hd_mtnmodel_y = b1model.ca(Swy, T); 
% hd_mtnmodel_z = b1model.ca(Swz, T);
% hd_mtnmodel1 = mtnmodel.dxdydz(hd_mtnmodel_x, hd_mtnmodel_y, hd_mtnmodel_z);

% ģ��2��CT
% Omega = deg2rad(10);
% Sw = [1,1,1];
% hd_mtndodel2_xyz = b3model.ct(Omega, Sw, T);
% hd_mtnmodel2 = mtnmodel.dxyz(hd_mtndodel2_xyz);

% �۲�ģ��
R = blkdiag(SigmaRange^2, SigmaAzimu^2, SigmaEleva^2);
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel = msmodel.drbe(R, StateSym);

% �˲���ģ��
hd_filter1 = mttfilter.ekf(hd_mtnmodel1, hd_msmodel);
hd_filter2 = mttfilter.ekf(hd_mtnmodel2, hd_msmodel);
hd_filter3 = mttfilter.ekf(hd_mtnmodel3, hd_msmodel);
MatrixPi = [0.95 0.025 0.025; 0.025 0.95 0.025; 0.025 0.025 0.95];
hd = mttfilter.imm({hd_filter1,hd_filter2,hd_filter3},MatrixPi);

% hd = mttfilter.imm({hd_filter1,hd_filter2});

%%
% Monte Carlo �˲�����
flag = 1;
% for state dimension 6
StateInit = [StateTrue(1:2,1);StateTrue(4:5,1);StateTrue(7:8,1)];
CovarInit = blkdiag(100^2,200^2,100^2,200^2,100^2,200^2);

% for state dimension 9
% StateInit = StateTrue(:,1);
% CovarInit = blkdiag(100^2,200^2,100^2,100^2,200^2,100^2,100^2,200^2,100^2);

% for filtering test
% [StateFilter, CovarFilter, Mu] = hd.filter(StateInit, CovarInit, PseudoMeasurementNoise(:,:,1), flag);

% for Mento Carlo filtering
[StateFilter, CovarFilter, Mu] = hd.mcfilter(StateInit, CovarInit, PseudoMeasurementNoise, flag);

PosFilter = StateFilter(1:2:end,:,:);
VelFilter = StateFilter(2:2:end,:,:);

%%
% �˲������ʾ
figure(31)
kk = 1;
% [MeasurementNoiseCart(1,:), MeasurementNoiseCart(2,:), MeasurementNoiseCart(3,:)] = ...
%     sph2cart(PseudoMeasurementNoise(2,:,kk), PseudoMeasurementNoise(3,:,kk), PseudoMeasurementNoise(1,:,kk));
hold on
plot3(PosTarget(1,:),  PosTarget(2,:),  PosTarget(3,:), '-r')
plot3(PseudoMeasurementNoiseCART(1,:,kk), PseudoMeasurementNoiseCART(2,:,kk), PseudoMeasurementNoiseCART(3,:,kk), '-b');
plot3(PosFilter(1,:,kk), PosFilter(2,:,kk), PosFilter(3,:,kk), '-g');
hold off
grid on, box on
axis equal
view(3)

%%
% ģ�͸���
figure(32)
hold on
plot(Mu(1,:,kk),'-r')
plot(Mu(2,:,kk),'-g')
plot(Mu(3,:,kk),'-b')
hold off
xlim([1,NumStep])
ylim([0,1])
box on, grid on
legend('ģ��1','ģ��2','ģ��3')

%%
% �˲����ͳ��
% index No.1 RMSE
IndexRMSE_Pos = rmse(PosFilter - repmat(PosTarget, [1,1,NumMC]));
IndexRMSE_Vel = rmse(VelFilter - repmat(VelTarget, [1,1,NumMC]));

% index No.2 AEE
IndexAEE_Pos = aee(PosFilter - repmat(PosTarget,[1,1,NumMC]));
IndexAEE_Vel = aee(VelFilter - repmat(VelTarget,[1,1,NumMC]));

% index No.3 GAE / LGAE
IndexGAE_Pos = gae(PosFilter - repmat(PosTarget, [1,1,NumMC]));
IndexGAE_Vel = gae(VelFilter - repmat(VelTarget, [1,1,NumMC]));
IndexLGAE_Pos = lgae(PosFilter - repmat(PosTarget, [1,1,NumMC]));
IndexLGAE_Vel = lgae(VelFilter - repmat(VelTarget, [1,1,NumMC]));

% index No.4 MERF1 / MERF2 / LMERF
[MeasurementEstimate(2,:,:),MeasurementEstimate(3,:,:),MeasurementEstimate(1,:,:)] = ...
    cart2sph(PosFilter(1,:,:), PosFilter(2,:,:), PosFilter(3,:,:));
IndexMERF1 = merf1(repmat(MeasurementTrue,[1,1,NumMC]),MeasurementEstimate,PseudoMeasurementNoise);
IndexMERF2 = merf2(repmat(MeasurementTrue,[1,1,NumMC]),MeasurementEstimate,PseudoMeasurementNoise);
IndexLMERF = lmerf(repmat(MeasurementTrue,[1,1,NumMC]),MeasurementEstimate,PseudoMeasurementNoise);

% index No.5 EERF1 / EERF2 / LEERF
% [StateNoise(1,:,:),StateNoise(2,:,:),StateNoise(3,:,:)] = ...
%     sph2cart(PseudoMeasurementNoise(2,:,:),PseudoMeasurementNoise(3,:,:),PseudoMeasurementNoise(1,:,:));
IndexEERF1 = eerf1(repmat(PosTarget,[1,1,NumMC]),PosFilter,PseudoMeasurementNoiseCART);
IndexEERF2 = eerf2(repmat(PosTarget,[1,1,NumMC]),PosFilter,PseudoMeasurementNoiseCART);
IndexLEERF = leerf(repmat(PosTarget,[1,1,NumMC]),PosFilter,PseudoMeasurementNoiseCART);

%%
% index No.6 SR
AlphaSucc = 2:2:200;
IndexSR = zeros(NumStep,length(AlphaSucc));
for kk = 1:1:length(AlphaSucc)
    IndexSR(:,kk) = succrate(repmat(MeasurementTrue,[1,1,NumMC]),MeasurementEstimate,PseudoMeasurementNoise,AlphaSucc(kk));
end

%%
% index No.7 FR
AlphaFail = 2:2:200;
IndexFR = zeros(NumStep,length(AlphaFail));
for kk = 1:1:length(AlphaFail)
    IndexFR(:,kk) = failrate(repmat(MeasurementTrue,[1,1,NumMC]),MeasurementEstimate,PseudoMeasurementNoise,AlphaFail(kk)); 
end

%%
% ��������ָ��ͼʾ
n = 1:1:NumStep;
figure(41)
subplot(211)
plot(n,IndexRMSE_Pos, '-k')
box on, grid on
xlim([1,NumStep])
legend('λ��RMSE')
subplot(212)
plot(n,IndexRMSE_Vel, '-k')
box on, grid on
xlim([1,NumStep])
legend('�ٶ�RMSE')

%%
figure(42)
subplot(211)
plot(n,IndexAEE_Pos, '-k')
box on, grid on
xlim([1,NumStep])
legend('λ��AEE')
subplot(212)
plot(n,IndexAEE_Vel, '-k')
box on, grid on
xlim([1,NumStep])
legend('�ٶ�AEE')

%%
figure(43)
subplot(411)
plot(n,IndexGAE_Pos, '-k')
box on, grid on
xlim([1,NumStep])
legend('λ��GAE')
subplot(412)
plot(n,IndexGAE_Vel, '-k')
box on, grid on
xlim([1,NumStep])
legend('�ٶ�GAE')
subplot(413)
plot(n,IndexLGAE_Pos, '-k')
box on, grid on
xlim([1,NumStep])
legend('λ��LGAE')
subplot(414)
plot(n,IndexLGAE_Vel, '-k')
box on, grid on
xlim([1,NumStep])
legend('�ٶ�LGAE')

%%
figure(44)
subplot(311)
plot(n,IndexMERF1, '-k')
box on, grid on
xlim([1,NumStep])
legend('MERF1')
subplot(312)
plot(n,IndexMERF2, '-k')
box on, grid on
xlim([1,NumStep])
legend('MERF2')
subplot(313)
plot(n,IndexLMERF, '-k')
box on, grid on
xlim([1,NumStep])
legend('LMERF')

%%
figure(45)
kk = 5;
subplot(211)
plot(n,IndexSR(:,kk),'-k')
box on, grid on
xlim([1,NumStep])
legend('SR')
kk = 5;
subplot(212)
plot(n,IndexFR(:,kk),'-k')
box on, grid on
xlim([1,NumStep])
legend('FR')

%%
% �ļ�����
save result_demo_experiment11.mat
