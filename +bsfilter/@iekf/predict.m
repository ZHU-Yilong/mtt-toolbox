function [XhatPre, PhatPre, Stil, K, Ztil] = predict(obj, Xinit, Pinit, Z)
% FILTER run iterated extended kalman filter with 
% input arguments
%  obj:   kalman filter object
%  Z:     measurement sequence
%  Xinit: initial state vector
%  Pinit: initial covariance matrix
% and output arguments
%  Xhat:  filtered state
%  Phat:  filtered covariance

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
if isnumeric(obj.Fx) && isnumeric(obj.Hx)
    warning('both motion model and measurement model are linear, there is no need to employ extended kalman filter')
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must be agree');
end
if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
    error('state symbolic variables motion model and measurement model must be agree')
end

%%% check input argument Xinit
DimState = length(obj.StateSym);
if ~isvector(Xinit)
    error('first input argument must be a vector')
end
if isrow(Xinit)
    Xinit = Xinit.';
end
if length(Xinit)~=DimState
    error('size of first input argument is not appropriate')
end

%%% check input argument Pinit
if ~ismatrix(Pinit) || size(Pinit,1)~=DimState || size(Pinit,2)~=DimState
    error('size of second input argument is not appropriate')
end

%%% check input argument Z
if ~isempty(Z)
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
end

%%% prediction
if isempty(obj.f)
    XhatPre = obj.Fx*Xinit;
elseif isa(obj.f, 'function_handle')
    XhatPre = feval(obj.f, obj, Xinit);
end

%%% covariance prediction
if isnumeric(obj.Fx)
    Fx = obj.Fx;
elseif isa(obj.Fx, 'function_handle')
    Fx = feval(obj.Fx, obj, Xinit);    
end
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
    if isnumeric(obj.Hx)
        Hx = obj.Hx;
    elseif isa(obj.Hx, 'function_handle')
        Hx = feval(obj.Hx, obj, XhatPre);
    end
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
    Stil = Hx*PhatPre*Hx.' + Hv*R*Hv.';
end

%%% Kalman gain
if nargout>3
    K = PhatPre*Hx.'/Stil;
end


%%% filtering
if nargout>4
    if isempty(obj.h) && isnumeric(obj.Hx)
        ZPre = Hx*XhatPre;
    elseif isa(obj.h, 'function_handle')
        ZPre = feval(obj.h, obj, XhatPre);
    end
    Ztil = Z - ZPre;
end

end

