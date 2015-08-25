function Xtrue = dxy(Xinit,a,time,n)
% TRAJECTORY2D generate two-dimensional target trajectory
% Xinit: initial target state including position (x, y), 
% speed v0, and head angle phi
% a: target tangential and normal accelerations
% in the form of vector or a single value
% time: time vector including sampling interval time
% and total simulation time
% n: upsampling factor
% Xtrue: output target state


%%% arguments default value
if nargin==0
    Xinit = [-15000,-9000/pi,300,0];
end
if nargin<=1
    a = [zeros(1,1300);zeros(1,500),ones(1,300)*10*pi,zeros(1,500)];
end
if nargin<=2
    time = [0.1,130];
end
if nargin<=3
    n = 100;
end

%%% check argument Xinit
if numel(Xinit)~=4 || length(Xinit)~=4
    error('initial state vector should contain four elements')
end
%%% check argument time
if time(1)>=time(2)
    error('sampling interval must less than total time')
end
NumSample = floor(time(2)/time(1));
%%% check argument a
if size(a,1)~=2 && size(a,2)==2
    a = a.';
end
if numel(a)==2
    a = repmat(a,1,NumSample);
end
if size(a,1)~=2 && size(a,2)~=NumSample
    error('target acceleration vector should contain two elements or two-row/column elements')
end
%%% check argument n
if uint32(n)~=n || n<=0
    error('up sample factor must be a positive integer')
end

TimeInt = time(1)/n;
NumSampleN = NumSample*n;
XtrueN = zeros(4,NumSampleN);
XtrueN(:,1) = Xinit;
for kk = 2:1:NumSampleN
    XtrueN(1,kk) = XtrueN(1,kk-1)+XtrueN(3,kk-1)*cos(XtrueN(4,kk-1))*TimeInt;
    XtrueN(2,kk) = XtrueN(2,kk-1)+XtrueN(3,kk-1)*sin(XtrueN(4,kk-1))*TimeInt;
    XtrueN(3,kk) = XtrueN(3,kk-1)+a(1,ceil((kk-1)/n))*TimeInt;
    XtrueN(4,kk) = XtrueN(4,kk-1)+a(2,ceil((kk-1)/n))/XtrueN(3,kk-1)*TimeInt;
end
XtrueTemp = downsample(XtrueN.',n).';
Xtrue(1:3:4,:) = XtrueTemp(1:2,:);
Xtrue(2,:) = XtrueTemp(3,:).*cos(XtrueTemp(4,:));
Xtrue(5,:) = XtrueTemp(3,:).*sin(XtrueTemp(4,:));
Xtrue(3,:) = a(1,:).*cos(XtrueTemp(4,:))+a(2,:).*cos(XtrueTemp(4,:)+pi/2*ones(1,NumSample));
Xtrue(6,:) = a(1,:).*sin(XtrueTemp(4,:))+a(2,:).*sin(XtrueTemp(4,:)+pi/2*ones(1,NumSample));

if nargout==0
    figure
    hold on
    plot(Xtrue(1,:),Xtrue(4,:),'-k')
    plot(0,0,'-k')
    hold off
    axis equal, box on, grid on
    title('trajectory')
end
end