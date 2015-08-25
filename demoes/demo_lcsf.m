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
Swx = 100;                                                               % x����������������ܶ�
Swy = 100;                                                               % y����������������ܶ�
T = 0.1;                                                                % ʱ��������
hd_mtnmodel_x = b1model.cv(Swx,T);                                      % x���˶�ģ��
hd_mtnmodel_y = b1model.cv(Swy,T);                                      % y���˶�ģ��
hd_mtnmodel = mtnmodel.dxdy(hd_mtnmodel_x,hd_mtnmodel_y);               % �˶�ģ��
%%% �۲�ģ��
R = [10000,0; 0,0.0001];                                                % �۲�����Э������
StateSym = hd_mtnmodel.StateSym;
hd_msmodel = msmodel.drb(R,StateSym);                                   % �۲�ģ��
%%% �˲���ģ��
hd = mttfilter.lcsf(hd_mtnmodel,hd_msmodel);
%%% �۲�����
Xtrue = trajectory.dxy;                                                 % Ŀ����ʵ״̬
Num = size(Xtrue,2);                                                    % ������������
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));                          % ��ʵ����ͷ�λ��
Z = [rho+randn(1,Num)*sqrt(R(1,1));theta+randn(1,Num)*sqrt(R(2,2))];    % �۲�����
Xinit = [Xtrue(1:2,2);Xtrue(4:5,2)];
Pinit = blkdiag(100^2,200^2,100^2,200^2);

%%
%%% ����filter�����˲�
flag = 1; 
[Xhat,Phat] = hd.filter(Xinit,Pinit,Z,flag);                             % �˲�

%%
%%% ����mcfilter�������ؿ����˲�
NumMc = 2;                                                              % ���ؿ���������
Zmc = repmat(Z,[1,1,NumMc]);                                            % �۲�
XhatMc = hd.mcfilter(Xinit,Pinit,Zmc,flag);                             % �˲�����

%%
% save demo_lcsf.mat

%%
%%% �˲��켣
figure(11)
kk = 2;
[xx, yy] = pol2cart(Z(2,:),Z(1,:));
hold on
plot(Xtrue(1,:),Xtrue(4,:),'-c');
plot(Xhat(1,:),Xhat(3,:),'-r');
plot(XhatMc(1,:,kk),XhatMc(3,:,kk),'-m');
plot(xx,yy,'-b');
hold off
axis equal, box on, grid on
legend('��ʵ','�˲�1','�˲�2','�۲�')

%%
%%% λ���˲����
figure(21)
hold on
plot(Xtrue(1,:)-Xhat(1,:),'-r');
plot(Xtrue(4,:)-Xhat(3,:),'-b');
hold off
xlim([0,1300])
% ylim([-200,200])
box on, grid on
legend('x��','y��')

%%
%%% �ٶ��˲����
figure(22)
hold on
plot(Xtrue(2,:)-Xhat(2,:),'-r');
plot(Xtrue(5,:)-Xhat(4,:),'-b');
hold off
xlim([0,1300])
% ylim([-100,100])
box on, grid on
legend('x��','y��')


