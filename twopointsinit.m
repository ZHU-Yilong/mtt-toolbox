function [Xinit, Pinit] = twopointsinit(Z, Sw, T, R)

%%% check argument Z
if size(Z,1)==2 && size(Z,2)~=2
    Z = Z.';
elseif size(Z,1)~=2 && size(Z,2)~=2
    error('two measurements must be provided')
end
DimMeasure = size(Z,1);
DimState = 2*DimMeasure;
%%% check argument Sw
if length(Sw)~=DimMeasure
    error('elements of process noise covariance should agree with measurements')
end
%%% check argument R
if length(R)~=DimMeasure
    error('elements of measurement noise covariance should agree with measurements')
end

%%% initialization state vector
Xinit(1:2:DimState,1) = Z(:,2);
Xinit(2:2:DimState,1) = (Z(:,2)-Z(:,1))/T;

%%% initialization covariance matrix
Pinit = [];
for kk = 1:1:DimMeasure
    P1 = [R(kk,kk), R(kk,kk)/T; R(kk,kk)/T, 2*R(kk,kk)/T+T*Sw(kk)/3];
    Pinit = blkdiag(Pinit, P1);
end

end