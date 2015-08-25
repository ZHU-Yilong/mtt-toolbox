%%
%%% ��������رմ򿪵Ĵ���
close all;
clear all;
clear classes;
syms clear;
clear classes;
reset(symengine);

%%
%%% �˶�ģ��
Swx = 50;                                                               % x����������������ܶ�
Swy = 50;                                                               % y����������������ܶ�
T = 0.1;                                                                % ʱ��������
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x���˶�ģ��
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y���˶�ģ��
hd_mtnmodel = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % �˶�ģ��
%%% �۲�ģ��1
H1 = [1,0,0,0; 0,0,1,0];                                                % �۲����
R1 = [10000,0; 0,10000];                                                % �۲�����Э������
hd_msmodel1 = msmodel.dxy(H1,R1);                                       % �۲�ģ��
%%% �۲�ģ��2
R2 = [10000,0; 0,0.0001];                                               % �۲�����Э������
hd_msmodel2 = msmodel.drb(R2);                                          % �۲�ģ��
%%% �˲���ģ��1�����������˲�
hd1 = mttfilter.kalman(hd_mtnmodel,hd_msmodel1);                        % �˲���ģ��1
%%% �˲���ģ��2������չ�������˲�
hd2 = mttfilter.ekf(hd_mtnmodel,hd_msmodel2);                           % �˲���ģ��2
%%% �˲���ģ��3�����޼��������˲�
hd3 = mttfilter.ukf(hd_mtnmodel,hd_msmodel2);                           % �˲���ģ��3
%%% �˲�����
NumMC = 10;
DimState = length(hd_mtnmodel.StateSym);
Xtrue = trajectory.dxy;                                                 % Ŀ����ʵ״̬
Num = size(Xtrue,2);
Z1 = repmat([Xtrue(1,:);Xtrue(4,:)],[1,1,NumMC])+...
    randn(2,Num,NumMC)*sqrt(R1(1,1));                                   % ֱ������ϵ�۲���������
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));
Z2 = repmat([rho;theta],[1,1,NumMC])+...
    [randn(1,Num,NumMC)*sqrt(R2(1,1));
     randn(1,Num,NumMC)*sqrt(R2(2,2))];                                 % ������ϵ�۲���������
Xinit = zeros(DimState,NumMC);
Pinit = zeros(DimState,DimState,NumMC);
for kk = 1:1:NumMC
    [Xinit(:,kk),Pinit(:,:,kk)] = twopointsinit(Z1(:,1:2,kk), [Swx,Swy], T, R1);
                                                                        % �˲�����ʼֵ��������ʼ����
end
[Xhat1,Phat1] = mcfilter(hd1,Z1,Xinit,Pinit);                           % ��׼�������˲�
[Xhat2,Phat2] = mcfilter(hd2,Z2,Xinit,Pinit);                           % ��չ�������˲�
% [Xhat3,Phat3] = mcfilter(hd3,Z2,Xinit,Pinit);                           % �����������˲�

%%
%%% �˲���������
ErrorPosition1 = flterr([Xtrue(1,:);Xtrue(4,:)],[Xhat1(1,:,:);Xhat1(3,:,:)]);
ErrorVelocity1 = flterr([Xtrue(2,:);Xtrue(5,:)],[Xhat1(2,:,:);Xhat1(4,:,:)]);

ErrorPosition2 = flterr([Xtrue(1,:);Xtrue(4,:)],[Xhat2(1,:,:);Xhat2(3,:,:)]);
ErrorVelocity2 = flterr([Xtrue(2,:);Xtrue(5,:)],[Xhat2(2,:,:);Xhat2(4,:,:)]);

% ErrorPosition3 = flterr([Xtrue(1,:);Xtrue(4,:)],[Xhat3(1,:,:);Xhat3(3,:,:)]);
% ErrorVelocity3 = flterr([Xtrue(2,:);Xtrue(5,:)],[Xhat3(2,:,:);Xhat3(4,:,:)]);

%%
%%% �˲��켣
figure(11)
kk = 1;
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat1(1,:,kk),Xhat1(3,:,kk),'-r');
plot(Xhat2(1,:,kk),Xhat2(3,:,kk),'-g');
% plot(Xhat3(1,:,kk),Xhat3(3,:,kk),'-k');
hold off
axis equal, box on, grid on
legend('��ʵ','KF�˲�','EKF')

%%
%%% λ���˲����
figure(21)
hold on
plot(Xtrue(1,1:end)-Xhat1(1,:,kk),'-.k');
plot(Xtrue(4,1:end)-Xhat1(3,:,kk),'-k');
hold off
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x��','y��')

%%
%%% �ٶ��˲����
figure(22)
hold on
plot(Xtrue(2,1:end)-Xhat1(2,:,kk),'-.k');
plot(Xtrue(5,1:end)-Xhat1(4,:,kk),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x��','y��')

%%
%%% λ���˲����
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
%%% �ٶ��˲����
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
