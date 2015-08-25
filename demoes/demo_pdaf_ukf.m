%%
%%% ��������رմ򿪵Ĵ���
close all;
clear all;
% clear classes;
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
hd_msmodel = msmodel.drb(R);                                            % �۲�ģ��
% H = [1,0,0,0; 0,0,1,0];                                                 % �۲����
% R = [10000,0; 0,10000];                                                 % �۲�����Э������
% hd_msmodel = msmodel.dxy(H,R);                                          % �۲�ģ��
%%% �˲���ģ��
% hdd = mttfilter.kalman(hd_mtnmodel,hd_msmodel);                         % �˲���ģ��
hdd = mttfilter.ukf(hd_mtnmodel,hd_msmodel);
%%% �˲�������
lambda = 0.0004;
gamma = 16;
Pg = 0.9997;
Pd = 1;
para = [lambda;gamma;Pg;Pd];                                            % �˲�������
hd = mttfilter.pdaf(hdd, para);
%%% �˲�����
Xtrue = trajectory.dxy;                                                 % Ŀ����ʵ״̬
DimMeasure = 2;
NumStep = size(Xtrue,2);
% Z = [Xtrue(1,:);Xtrue(4,:)]+randn(DimMeasure,NumStep)*50;               % �۲���������
[theta,rho] = cart2pol(Xtrue(1,:),Xtrue(4,:));                          % ��ʵ����ͷ�λ��
Z = [rho+randn(1,NumStep)*sqrt(R(1,1));theta+randn(1,NumStep)*sqrt(R(2,2))];    % �۲�����
%%%
nc = 3;
Av = nc/10/lambda;
q = sqrt(10*Av)/2;
Zclutter = cell(NumStep,1);
[Xmeasure,Ymeasure] = pol2cart(Z(2,:),Z(1,:));
for kk = 1:1:NumStep
    CartMeasure = repmat([Xmeasure(kk);Ymeasure(kk)]-[q;q],1,nc)+rand(DimMeasure,nc)*2*q;  % �Ӳ������¹۲�
    [PolMeasureThe,PolMeasureRho] = cart2pol(CartMeasure(1,:),CartMeasure(2,:));
    Zclutter{kk} = [PolMeasureRho;PolMeasureThe];
end
% [Xinit, Pinit] = twopointsinit(Z(:,1:2), [Swx,Swy], T, R);              % �˲�����ʼֵ��������ʼ����
% [Xinit, Pinit] = twopointsinit([Xtrue(1,1:2);Xtrue(4,1:2)]+randn(2,2)*100, [Swx,Swy], T, [10000,0;0,10000]);
Xinit = [Xtrue(1:2,2);Xtrue(4:5,2)];
Pinit = blkdiag(100^2,200^2,100^2,200^2);
flag = 1;                                                               % ��������ʾ
[Xhat,Phat] = hd.filter(Zclutter(3:end),Xinit,Pinit,flag);              % �˲�

%%
save demo_pdaf_ukf.mat

%%
%%% �˲��켣
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
legend('��ʵ','�˲�','�۲�')

%%
%%% λ���˲����
figure(21)
hold on
plot(Xtrue(1,3:end)-Xhat(1,:),'-.k');
plot(Xtrue(4,3:end)-Xhat(3,:),'-k');
hold off
xlim([0,1300])
ylim([-200,200])
box on, grid on
legend('x��','y��')

%%
%%% �ٶ��˲����
figure(22)
hold on
plot(Xtrue(2,3:end)-Xhat(2,:),'-.k');
plot(Xtrue(5,3:end)-Xhat(4,:),'-k');
hold off
xlim([0,1300])
ylim([-100,100])
box on, grid on
legend('x��','y��')

