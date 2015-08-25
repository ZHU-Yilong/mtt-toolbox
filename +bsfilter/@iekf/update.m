function [Xhat, Phat, Stil, K, Ztil] = update(obj, XhatPre, PhatPre, Z)
% FILTER run iterated extended kalman filter with 
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
    warning('both motion model and measurement model are linear, there is no need to employ extended kalman filter')
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must agree');
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

NumIteration = obj.NumIteration;
%%% iterated filtering
XhatIte = XhatPre;
for jj = 1:1:NumIteration
    %%% Kalman gain
    if isnumeric(obj.Hx)
        Hx = obj.Hx;
    elseif isa(obj.Hx, 'function_handle')
        Hx = feval(obj.Hx, obj, XhatIte);
    end
    if isnumeric(obj.Hv)
        Hv = obj.Hv;
    elseif isa(obj.Hv, 'function_handle')
        Hv = feval(obj.Hv, obj, XhatIte);
    end
    if isnumeric(obj.R)
        R = obj.R;
    elseif isa(obj.R, 'function_handle')
        R = feval(obj.R, obj, XhatIte);
    end
    Stil = Hx*PhatPre*Hx.' + Hv*R*Hv.';
    K = PhatPre*Hx.'/Stil;

    %%% filtering
    if isempty(obj.h) && isnumeric(obj.Hx)
        ZPre = Hx*XhatPre + Hx*(XhatPre-XhatIte);
    elseif isa(obj.h, 'function_handle')
        ZPre = feval(obj.h, obj, XhatIte) + Hx*(XhatPre-XhatIte);
    end
    Ztil = Z - ZPre;
    XhatIte = XhatPre + K*Ztil; 
end
Xhat = XhatIte;

%%% covariance fitlering
Phat = (eye(DimState)-K*Hx)*PhatPre*(eye(DimState)-K*Hx).' + K*R*K.';

end

