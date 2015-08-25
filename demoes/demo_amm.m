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
Swx = 10;                                                               % x����������������ܶ�
Swy = 10;                                                               % y����������������ܶ�
T = 0.1;                                                                % ʱ��������
Omega = deg2rad(6);
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x���˶�ģ��
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y���˶�ģ��
hd_mtnmodel1 = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);              % �˶�ģ��
hd_mtnmodel_sub = b2model.ct(Omega,Swx,T);
hd_mtnmodel2 = mtnmodel.dxy(hd_mtnmodel_sub);
%%% �۲�ģ��
H = [1,0,0,0; 0,0,1,0];                                                 % �۲����
R = [10000,0; 0,10000];                                                 % �۲�����Э������
StateSym = hd_mtnmodel1.StateSym;
hd_msmodel = msmodel.dxy(R,StateSym);                                   % �۲�ģ��
%%% �˲���ģ��
hd_filter1 = mttfilter.kalman(hd_mtnmodel1,hd_msmodel);                  % �˲���ģ��
hd_filter2 = mttfilter.kalman(hd_mtnmodel2,hd_msmodel);
hd = mttfilter.amm({hd_filter1,hd_filter2});
%%% �˲�����
Xtrue = trajectory.dxy;                                                 % Ŀ����ʵ״̬
Z = [Xtrue(1,:);Xtrue(4,:)]+randn(2,size(Xtrue,2))*100;                 % �۲���������
[Xinit, Pinit] = twopointsinit(Z(:,1:2), [Swx,Swy], T, R);              % �˲�����ʼֵ��������ʼ����

%%
flag = 1;                                                               % ��������ʾ
[Xhat,Phat,Mu] = hd.filter(Xinit, Pinit, Z, flag);                      % �˲�

%%
%%% ����mcfilter�������ؿ����˲�
NumMc = 2;                                                              % ���ؿ���������
Zmc = repmat(Z,[1,1,NumMc]);                                            % �۲�
XhatMc = hd.mcfilter(Xinit, Pinit, Zmc, flag);                          % �˲�����

%%
% save demo_amm.mat

%%
%%% �˲��켣
figure(11)
kk = 1;
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(XhatMc(1,:,kk),XhatMc(3,:,kk),'-m');
plot(Z(1,:),Z(2,:),'-.k');
hold off
axis equal, box on, grid on
legend('��ʵ','�˲�1','�˲�2','�۲�')

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

%%
figure(31)
hold on
plot(Mu(1,:),'-r')
plot(Mu(2,:),'-.b')
hold off
xlim([0,1300])
ylim([-0.5,1.5])
box on, grid on
legend('ģ��1','ģ��2')
