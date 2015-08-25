function Xtrue = dxyz(Xinit,a,time,n)
% TRAJECTORY2D generate two-dimensional target trajectory
% Xinit: initial target state including position (x0, y0, z0), 
% speed v0, and head angle phi and theta
% a: target tangential and normal accelerations
% in the form of vector or a single value
% time: time vector including sampling interval time
% and total simulation time
% n: upsampling factor
% Xtrue: output target state


%%% arguments default value
if nargin==0
    Xinit = [-15000,-9000/pi,0,300,0,0];
end
if nargin<=1
    a = [zeros(1,1300);zeros(1,500),ones(1,300)*10*pi,zeros(1,500);zeros(1,1300)];
end
if nargin<=2
    time = [0.1,130];
end
if nargin<=3
    n = 100;
end

%%% check argument Xinit
if numel(Xinit)~=6 || length(Xinit)~=6
    error('initial state vector should contain six elements')
end
%%% check argument time
if time(1)>=time(2)
    error('sampling interval must less than total time')
end
NumSample = floor(time(2)/time(1));
%%% check argument a
if size(a,1)~=3 && size(a,2)==3
    a = a.';
end
if numel(a)==3
    a = repmat(a,1,NumSample);
end
if size(a,1)~=3 && size(a,2)~=NumSample
    error('target acceleration vector should contain three elements or three-row/column elements')
end
%%% check argument n
if uint32(n)~=n || n<=0
    error('up sample factor must be a positive integer')
end

TimeInt = time(1)/n;
NumSampleN = NumSample*n;
XtrueN = zeros(6,NumSampleN);
XtrueN(:,1) = Xinit;
for kk = 2:1:NumSampleN
    XtrueN(1,kk) = XtrueN(1,kk-1)+XtrueN(4,kk-1)*cos(XtrueN(5,kk-1))*cos(XtrueN(6,kk-1))*TimeInt;
    XtrueN(2,kk) = XtrueN(2,kk-1)+XtrueN(4,kk-1)*sin(XtrueN(5,kk-1))*cos(XtrueN(6,kk-1))*TimeInt;
    XtrueN(3,kk) = XtrueN(3,kk-1)+XtrueN(4,kk-1)*sin(XtrueN(6,kk-1))*TimeInt;
    XtrueN(4,kk) = XtrueN(4,kk-1)+a(1,ceil((kk-1)/n))*TimeInt;
    XtrueN(5,kk) = XtrueN(5,kk-1)+a(2,ceil((kk-1)/n))/XtrueN(4,kk-1)/cos(XtrueN(6,kk-1))*TimeInt;
    XtrueN(6,kk) = XtrueN(6,kk-1)+a(3,ceil((kk-1)/n))/XtrueN(4,kk-1)*TimeInt;
end
XtrueTemp = downsample(XtrueN.',n).';
Xtrue(1:3:7,:) = XtrueTemp(1:1:3,:);
Xtrue(2,:) = XtrueTemp(4,:).*cos(XtrueTemp(5,:)).*cos(XtrueTemp(6,:));
Xtrue(5,:) = XtrueTemp(4,:).*sin(XtrueTemp(5,:)).*cos(XtrueTemp(6,:));
Xtrue(8,:) = XtrueTemp(4,:).*sin(XtrueTemp(6,:));
dcm = angle2dcm(XtrueTemp(5,:),-XtrueTemp(6,:),zeros(1,NumSample));
XtrueTemp2 = zeros(3,NumSample);
for kk = 1:1:NumSample
    XtrueTemp2(:,kk) = dcm(:,:,kk).'*[a(1,kk);a(2,kk);a(3,kk)];
    Xtrue(3,kk) = XtrueTemp2(1,kk);
    Xtrue(6,kk) = XtrueTemp2(2,kk);
    Xtrue(9,kk) = XtrueTemp2(3,kk);
end
if nargout==0
    figure
    plot3(Xtrue(1,:),Xtrue(4,:),Xtrue(7,:),'-k')
    axis equal, box on, grid on
    title('trajectory')
end
end