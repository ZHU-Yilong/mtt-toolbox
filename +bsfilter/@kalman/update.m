function [Xhat, Phat, Stil, K, Ztil] = update(obj, XhatPre, PhatPre, Z)
% FILTER run kalman filter with input arguments
%  obj:   kalman filter object
%  Z:     measurement vector
%  Xpre : prediction state vector
%  Ppre : prediction covariance matrix
% and output arguments
%  Xhat:  filtered state
%  Phat:  filtered covariance

if nargin<4
    error('not enough input arguments');
end

%%% check input argument obj
if (~isempty(obj.f)) || (~isnumeric(obj.Fx))
    error('motion model must be a linear one');
end
if (~isempty(obj.h)) || (~isnumeric(obj.Hx))
    error('measurement model must be a linear one');
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must be agree');
end
if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
    error('state symbolic variables motion model and measurement model must be agree')
end

%%% check input argument Xpre
DimState = length(obj.StateSym);                % state vector dimension
if ~isvector(XhatPre)
    error('first input argument must be a vector')
end
if isrow(XhatPre)
    XhatPre = XhatPre.';
end
if length(XhatPre)~=DimState
    error('size of first input argument is not approriate')
end

%%% check input argument Ppre
if ~ismatrix(PhatPre) || size(PhatPre,1)~=DimState || size(PhatPre,2)~=DimState
    error('size of second input argument is not appropriate')
end

%%% check input argument Z
DimMeasure = length(obj.MeasureSym);    % measurement dimension
if ~isvector(Z)
    error('third input argument must be a vector');
end
if isrow(Z)
    Z = Z.';
end
if ~iscolumn(Z) || size(Z,1)~=DimMeasure
    error('size of third input argument is not appropriate')
end

%%% constant matrix
Hx = obj.Hx;
%%% Kalman gain
if isnumeric(obj.Hv)
    Hv = obj.Hv;
elseif isa(obj.Hv,'function_handle')
    Hv = feval(obj.Hv, obj, XhatPre);
end
if isnumeric(obj.R)
    R = obj.R;
elseif isa(obj.R, 'function_handle')
    R = feval(obj.R, obj, XhatPre);
end

Stil = Hx*PhatPre*Hx.' + Hv*R*Hv.';
K = PhatPre*Hx.'/Stil;

%%% filtering state
Ztil = Z-Hx*XhatPre;
Xhat = XhatPre+K*Ztil;

%%% fitlering covariance 
Phat = (eye(DimState)-K*Hx)*PhatPre*(eye(DimState)-K*Hx).' + K*R*K.';

end
    


