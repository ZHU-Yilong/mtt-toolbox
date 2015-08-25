
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
%%% �۲�ģ��
R = [10000,0; 0,0.0001];                                                % �۲�����Э������
StateSym = hd_mtnmodel.StateSym;
hd_msmodel = msmodel.drb(R,StateSym);                                   % �۲�ģ��
%%% �˲���ģ��
numparticle = 100;
hd = mttfilter.pf(hd_mtnmodel,hd_msmodel,numparticle);                  % �˲���ģ��
%%% �˲�����
Xtrue = trajectory.dxy;                                                 % Ŀ����ʵ״̬
Num = size(Xtrue,2);                                                    % ���г���
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));                          % ������ϵ����ʵλ��
Z = [rho+randn(1,Num)*sqrt(R(1,1));theta+randn(1,Num)*sqrt(R(2,2))];    % �۲���������
Xinit = [Xtrue(1,1);Xtrue(2,1);Xtrue(4,1);Xtrue(5,1)];                  % ��ʼ״̬
Pinit = diag([1e4,2e5,1e4,2e5]);                                        % ��ʼ�˲�Э����

%%
%%% ����filter�����˲�
flag = 1;                                                               % ��������־
Xhat = hd.filter(Xinit, Pinit, Z, flag);                                % �˲�

%%
%%% ����mcfilter�������ؿ����˲�
NumMc = 2;                                                              % ���ؿ���������
Zmc = repmat(Z,[1,1,NumMc]);                                            % �۲�
XhatMc = hd.mcfilter(Xinit, Pinit, Zmc, flag);                          % �˲�����

%%
%%% �˲��켣
figure(11)
[xmeasure,ymeasure] = pol2cart(Z(2,:),Z(1,:));
hold on
plot(Xtrue(1,:),Xtrue(4,:),'--k');
plot(Xhat(1,:),Xhat(3,:),'-k');
plot(xmeasure,ymeasure,'-.k');
hold off
axis equal, box on, grid on
legend('��ʵ','�˲�','�۲�')

%%
%%% λ���˲����
figure(21)
xx = 1:1:size(Xhat,2);
hold on
plot(xx,Xtrue(1,:)-Xhat(1,:),'-.k');
plot(xx,Xtrue(4,:)-Xhat(3,:),'-k');
hold off
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x��','y��')

%%
%%% �ٶ��˲����
figure(22)
hold on
plot(xx,Xtrue(2,:)-Xhat(2,:),'-.k');
plot(xx,Xtrue(5,:)-Xhat(4,:),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x��','y��')

