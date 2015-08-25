function [XhatPre, PhatPre, Stil, K, Ztil] = predict(obj, Xinit, Pinit, Z)
% FILTER run kalman filter one step predict with input arguments
%  obj   : kalman filter object
%  Xinit : initial state vector
%  Pinit : initial covariance matrix
%  Z     : measurement vector
% and output arguments
%  Xpre  : predicted state
%  Ppre  : predicted covariance
%  Stil  : 
%  K     : kalman gain

if nargin<3
    error('not enough input arguments');
end
if nargin<4
    Z = [];
end
if isempty(Z) && nargout==5
    error('not enough input arguments')
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

%%% check input argument Xinit
DimState = length(obj.StateSym);                % state vector dimension
if ~isvector(Xinit)
    error('first input argument must be a vector')
end
if isrow(Xinit)
    Xinit = Xinit.';
end
if length(Xinit)~=DimState
    error('size of first input argument is not approriate')
end

%%% check input argument Pinit
if ~ismatrix(Pinit) || size(Pinit,1)~=DimState || size(Pinit,2)~=DimState
    error('size of second input argument is not appropriate')
end

%%% check input argument Z
if ~isempty(Z)
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
end

%%% constant matrix
Fx = obj.Fx;
Hx = obj.Hx;
%%% state prediction
XhatPre = Fx*Xinit;

%%% covariance prediction
if isnumeric(obj.Fw)
    Fw = obj.Fw;
elseif isa(obj.Fw, 'function_handle')
    Fw = feval(obj.Fw, obj, Xinit);
end
if isnumeric(obj.Q)
    Q = obj.Q;
elseif isa(obj.Q, 'function_handle')
    Q = feval(obj.Q, obj, Xinit);
end
PhatPre = Fx*Pinit*Fx.' + Fw*Q*Fw.';

if nargout>2
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
end
if nargout>3
    K = PhatPre*Hx.'/Stil;
end
if nargout>4
    %%%
    Ztil = Z-Hx*XhatPre;
end

end
