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
hd_mtnmodel1 = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % �˶�ģ��
%%% �۲�ģ��
H = [1,0,0,0; 0,0,1,0];                                                 % �۲����
R1 = [10000,0; 0,10000];                                                % �۲�����Э������
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel1 = msmodel.dxy(R1, StateSym);                                        % �۲�ģ��
R2 = [10000,0; 0,10000];
hd_msmodel2 = msmodel.dxy(R2, StateSym); 
%%% �˲���ģ��
hdd1 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel1);                      % �˲���ģ��
hdd2 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel2);
%%% �˲�������
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda,lambda;gamma,gamma;Pg,Pg;Pd,Pd];                         % �˲�������
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
[Xinit, Pinit] = twopointsinit(Z2(:,1:2), [Swx,Swy], T, R2);            % �˲�����ʼֵ��������ʼ����
flag = 1;                                                               % ��������ʾ
[Xhat,Phat,Mu] = hd.filter(Xinit,Pinit,Zclutter(:,3:end),flag);         % �˲�

%%
save demo_immmspdaf.mat

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
plot(Xtrue(1,3:end)-Xhat(1,:),'-.k');
plot(Xtrue(4,3:end)-Xhat(3,:),'-k');
hold off
% xlim([0,1300])
% ylim([-200,200])
box on, grid on
legend('x��','y��')

%%
%%% �ٶ��˲����
figure(22)
hold on
plot(Xtrue(2,3:end)-Xhat(2,:),'-.k');
plot(Xtrue(5,3:end)-Xhat(4,:),'-k');
hold off
% xlim([0,1300])
% ylim([-100,100])
box on, grid on
legend('x��','y��')

%%
figure(31)
hold on
plot(Mu(1,:),'-k')
plot(Mu(2,:),'-.k')
hold off
% xlim([0,1300])
% ylim([0,1])
box on, grid on
legend('ģ��1','ģ��2')

