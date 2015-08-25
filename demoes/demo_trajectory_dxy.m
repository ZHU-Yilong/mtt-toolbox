xtrue=trajectory.dxy;
 
%%
figure(11)
hold on
% plot(xtrue(1,:),xtrue(4,:),'-k','LineWidth',2)
plot(xtrue(1,:),'-k','LineWidth',2)
plot(xtrue(4,:),'-.k','LineWidth',2)
hold off
xlim([0,1300])
grid on, box on
legend('\it{x}','\it{y}')
 
%%
figure(12)
hold on
% plot(xtrue(2,:),xtrue(5,:),'-k','LineWidth',2)
plot(xtrue(2,:),'-k','LineWidth',2)
plot(xtrue(5,:),'-.k','LineWidth',2)
hold off
xlim([0,1300])
ylim([-400,400])
grid on, box on
legend('\it{v_{x}}','\it{v_{y}}')

%%
figure(13)
hold on
% plot(xtrue(3,:),xtrue(6,:),'-k','LineWidth',2)
plot(xtrue(3,:),'-k','LineWidth',2)
plot(xtrue(6,:),'-.k','LineWidth',2)
hold off
xlim([0,1300])
grid on, box on
legend('\it{a_{x}}','\it{a_{y}}')
