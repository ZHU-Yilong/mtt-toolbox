function [Xhat, Phat, Stil, K, Ztil] = update(obj, XhatPre, PhatPre, Z)
% FILTER run unscented kalman filter with 
% input arguments
%  obj:   kalman filter object
%  Z:     measurement sequence
%  Xinit: initial state vector
%  Pinit: initial covariance matrix
% and output arguments
%  Xhat:  filtered state
%  Phat:  filtered covariance

if nargin < 4
    error('not enough input arguments');
end

%%% check input argument obj
if isnumeric(obj.Fx) && isnumeric(obj.Hx)
    warning('both motion model and measurement model are linear, there is no need to employ unscented kalman filter')
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must be agree');
end
if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
    error('state symbolic variables motion model and measurement model must be agree')
end

%%% check input argument XhatPre
DimState = length(obj.StateSym);
if ~isvector(XhatPre)
    error('first input argument must be a vector')
end
if isrow(XhatPre)
    XhatPre = XhatPre.';
end
if length(XhatPre)~=DimState
    error('size of first input argument is not appropriate')
end

%%% check input argument Pinit
if ~ismatrix(PhatPre) || size(PhatPre,1)~=DimState || size(PhatPre,2)~=DimState
    error('size of second input argument is not appropriate')
end

%%% check input argument Z
DimMeasure = length(obj.MeasureSym);
if ~isvector(Z)
    error('third input argument must be a vector')
end
if isrow(Z)
    Z = Z.';
end
if size(Z,1)~=DimMeasure
    error('size of third input argument is not appropriate')
end

kappa = DimState-3;
%%% Kalman gain
if isnumeric(obj.Hv)
    Hv = obj.Hv;
elseif isa(obj.Hv, 'function_handle')
    Hv = feval(obj.Hv, obj, XhatPre);
end
if isnumeric(obj.R)
    R = obj.R;
elseif isa(obj.R, 'function_handle')
    R = feval(obj.R, obj, XhatPre);
end
if isempty(obj.h) && isnumeric(obj.Hx)
    Hx = obj.Hx;
    Pxz = PhatPre*Hx.';
    Stil = Hx*PhatPre*Hx.' + Hv*R*Hv.';
    ZPre = Hx*XhatPre;
elseif isa(obj.h, 'function_handle')
    matrixXi = chol((DimState+kappa)*PhatPre,'lower');
    xi = zeros(DimState,2*DimState+1);
    for jj = 1:1:DimState
        xi(:,jj) = XhatPre+matrixXi(:,jj);
        xi(:,jj+DimState) = XhatPre-matrixXi(:,jj);
    end
    xi(:,end) = XhatPre;
    weight = [ones(2*DimState,1)/2/(DimState+kappa); kappa/(DimState+kappa)];
    zeta = zeros(DimMeasure,2*DimState+1);
    ZPre = 0;
    for jj = 1:1:2*DimState+1
        zeta(:,jj) = feval(obj.h, obj, xi(:,jj));
        ZPre = ZPre + weight(jj)*zeta(:,jj);
    end
    Pzz = 0; Pxz = 0;
    for jj = 1:1:2*DimState+1
        Pzz = Pzz + weight(jj)*(zeta(:,jj)-ZPre)*(zeta(:,jj)-ZPre).';
        Pxz = Pxz + weight(jj)*(xi(:,jj)-XhatPre)*(zeta(:,jj)-ZPre).';
    end
    Stil = Pzz + Hv*R*Hv.';
end
K = Pxz/Stil;

%%% filtering
Ztil = Z - ZPre;
Xhat = XhatPre + K*Ztil;

%%% covariance fitlering
Phat = PhatPre - K*Stil*K.';

end

