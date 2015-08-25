%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%  target trajectory generation
%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
close all
clear all

%%
Xinit = [10000,0,0,300,pi,0];
g = 9.8;
T = 0.02;
TotalTime = 12;
ManeuverTime = 2;
NumStep = TotalTime/T;
NumManeuver = ManeuverTime/T;
a = [zeros(1,NumStep);zeros(1,NumManeuver),-ones(1,NumStep-NumManeuver)*5*g;zeros(1,NumStep)];
time = [T,TotalTime];
n = 100;
xtrue = trajectory.dxyz(Xinit,a,time,n);
% 
% vel = sqrt(xtrue(2,:).^2+xtrue(5,:).^2+xtrue(8,:).^2);
% acc = sqrt(xtrue(3,:).^2+xtrue(6,:).^2+xtrue(9,:).^2);

%%
% save targettrajectory.mat
load targettrajectory.mat

%%
figure(1)
plot3(xtrue(1,:), xtrue(4,:), xtrue(7,:),'-r')
grid on
box on
axis equal
xlim([7500,10000]); ylim([0,2000]); zlim([-200,200])
view(3)

%%
figure(2)
plot(xtrue(1,:), xtrue(4,:))
grid on
box on
axis equal
% xlim([2100,3000]); ylim([-50,200])

%%
%%% position
timeserise = T:T:TotalTime;
figure(11)
hold on
plot(timeserise,xtrue(1,:),'-r')
plot(timeserise,xtrue(4,:),'-.b')
plot(timeserise,xtrue(7,:),'--c')
hold off
xlim([0,TotalTime])
ylim([-100,10000])
grid on, box on
legend('\it{x}','\it{y}','\it{z}')

%%
%%% velocity
figure(12)
hold on
plot(timeserise,xtrue(2,:),'-r')
plot(timeserise,xtrue(5,:),'-.b')
plot(timeserise,xtrue(8,:),'--c')
hold off
xlim([0,TotalTime])
ylim([-300,300])
grid on, box on
legend('\it{v_{x}}','\it{v_{y}}','\it{v_{z}}')

%%
%%% acceleration
figure(13)
hold on
plot(timeserise,xtrue(3,:),'-r')
plot(timeserise,xtrue(6,:),'-.b')
plot(timeserise,xtrue(9,:),'--c')
hold off
xlim([0,TotalTime])
ylim([-10,60])
grid on, box on
legend('\it{a_{x}}','\it{a_{y}}','\it{a_{z}}')




