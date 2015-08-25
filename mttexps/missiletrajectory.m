%%%%%
clear all
close all

load('targettrajectory.mat')

%%
% NumStep = size(xtrue,2);
PosMissile = zeros(3,NumStep);
VelMissile = zeros(3,NumStep);
AccMissile = zeros(3,NumStep);
PosMissile(:,1) = [0;0;0];
VelMissile(:,1) = [700;0;0];
N = 3;
Vm = 700;
% T = 0.02;

for kk = 1:1:NumStep
    PosTarget = xtrue(1:3:7, kk);
    VelTarget = xtrue(2:3:8, kk);
    AccCommand = pnglaw(PosTarget, VelTarget, PosMissile(:,kk), VelMissile(:,kk), N);
    
    PosMissile(:,kk+1) = PosMissile(:,kk)+VelMissile(:,kk).*T+0.5.*AccCommand.*T^2;
    VelMissile(:,kk+1) = VelMissile(:,kk)+AccCommand.*T;
    AccMissile(:,kk) = AccCommand;
    relative = PosMissile(:,kk+1)-PosTarget;
    range = sqrt(relative(1)^2+relative(2)^2+relative(3)^2);
    if range < 50
        break;
    end
end
if kk < NumStep
    PosMissile = PosMissile(:,1:kk+1);
    VelMissile = VelMissile(:,1:kk+1);
    AccMissile = AccMissile(:,1:kk+1);
    AccMissile(:,kk+1) = AccCommand;
    xtrue = xtrue(:,1:kk+1);
end

%%
% save missiletrajectory.mat
load missiletrajectory.mat

%%
figure(1)
hold on
plot3(PosMissile(1,:),PosMissile(2,:),PosMissile(3,:),'-b')
plot3(xtrue(1,:),xtrue(4,:),xtrue(7,:),'-r')
hold off
grid on, box on
axis equal
xlim([0,10000]); ylim([0,2000]); zlim([-200,200])
legend('boxoff'),legend('µ¼µ¯','Ä¿±ê','Location','NorthEast')
view(3)

%%
figure(11)
hold on
plot(AccMissile(1,:),'-r')
plot(AccMissile(2,:),':k')
plot(AccMissile(3,:),'-.b')
hold off
grid on, box on

