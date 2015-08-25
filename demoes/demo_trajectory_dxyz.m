clear all
close all

xtrue=trajectory.dxyz;
 
%%
figure(11)
hold on
plot(xtrue(1,:),'-k')
plot(xtrue(4,:),'-.k')
plot(xtrue(7,:),'--k')
hold off
xlim([0,1300])
grid on, box on
legend('\it{x}','\it{y}','\it{z}')
 
%%
figure(12)
hold on
plot(xtrue(2,:),'-k')
plot(xtrue(5,:),'-.k')
plot(xtrue(8,:),'--k')
hold off
xlim([0,1300])
ylim([-400,400])
grid on, box on
legend('\it{v_{x}}','\it{v_{y}}','\it{v_{z}}')

%%
figure(13)
hold on
plot(xtrue(3,:),'-k')
plot(xtrue(6,:),'-.k')
plot(xtrue(9,:),'--k')
hold off
xlim([0,1300])
grid on, box on
legend('\it{a_{x}}','\it{a_{y}}','\it{a_{z}}')
