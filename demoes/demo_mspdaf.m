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
Swx1 = 50;                                                              % x����������������ܶ�
Swy1 = 50;                                                              % y����������������ܶ�
T = 0.1;                                                                % ʱ��������
hd_mtnmodel_x1 = b1model.cv(Swx1,T);                                    % x���˶�ģ��
hd_mtnmodel_y1 = b1model.cv(Swy1,T);                                    % y���˶�ģ��
hd_mtnmodel1 = mtnmodel.dxdy(hd_mtnmodel_x1,hd_mtnmodel_y1);            % �˶�ģ��
Swx2 = 100;                                                               % x����������������ܶ�
Swy2 = 100;                                                               % y����������������ܶ�
hd_mtnmodel_x2 = b1model.cv(Swx2,T);                                    % x���˶�ģ��
hd_mtnmodel_y2 = b1model.cv(Swy2,T);                                    % y���˶�ģ��
hd_mtnmodel2 = mtnmodel.dxdy(hd_mtnmodel_x2,hd_mtnmodel_y2);            % �˶�ģ��
%%% �۲�ģ��
R1 = [10000,0; 0,10000];                                                % �۲�����Э������
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel1 = msmodel.dxy(R1,StateSym);                                 % �۲�ģ��
R2 = [10000,0; 0,10000];
hd_msmodel2 = msmodel.dxy(R2,StateSym); 
%%% �˲���ģ��
hdd1 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel1);                      % �˲���ģ��
hdd2 = mttfilter.kalman(hd_mtnmodel2,hd_msmodel2);
hdd = {hdd1;hdd2};
%%% �˲�������
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];                         % �˲�������
hd = mttfilter.mspdaf(hdd, para);
%%% �˲�����
Xtrue = trajectory.dxy;                                                 % Ŀ����ʵ״̬
DimMeasure = 2;
NumStep = size(Xtrue,2);
Z1 = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*50;              % �۲���������
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
    Zclutter{1,kk} = repmat(Z1(:,kk)-[q1;q1],1,nc1)+rand(DimMeasure,nc1)*2*q1;  % �Ӳ������¹۲�
    Zclutter{2,kk} = repmat(Z2(:,kk)-[q2;q2],1,nc2)+rand(DimMeasure,nc2)*2*q2;
end
[Xinit, Pinit] = twopointsinit(Z2(:,1:2), [Swx2,Swy2], T, R2);          % �˲�����ʼֵ��������ʼ����

%%
%%%
flag = 1;                                                               % ��������ʾ
[Xhat,Phat] = hd.filter(Xinit, Pinit, Zclutter, flag);                  % �˲�

%%
% save demo_mspdaf.mat

%%
%%% �˲��켣
figure(11)
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(Z1(1,:),Z1(2,:),'-.b');
hold off
axis equal, box on, grid on
legend('��ʵ','�˲�','�۲�')

%%
%%% λ���˲����
figure(21)
hold on
plot(Xtrue(1,:)-Xhat(1,:),'-.k');
plot(Xtrue(4,:)-Xhat(3,:),'-k');
hold off
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x��','y��')

%%
%%% �ٶ��˲����
figure(22)
hold on
plot(Xtrue(2,:)-Xhat(2,:),'-.k');
plot(Xtrue(5,:)-Xhat(4,:),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x��','y��')

