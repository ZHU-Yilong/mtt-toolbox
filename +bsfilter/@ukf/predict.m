function [XhatPre, PhatPre, Stil, K, Ztil] = predict(obj, Xinit, Pinit, Z)
% FILTER run unscented kalman filter with 
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
    warning('both motion model and measurement model are linear, there is no need to employ unscented kalman filter')
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

%%%
kappa = DimState-3;
%%% prediction
if isempty(obj.f) && isnumeric(obj.Fx)
    XhatPre = obj.Fx*Xinit;
elseif isa(obj.f, 'function_handle')
    matrixChi = chol((DimState+kappa)*Pinit,'lower');
    chi = zeros(DimState,2*DimState+1);
    for jj = 1:1:DimState
        chi(:,jj) = Xinit+matrixChi(:,jj);
        chi(:,jj+DimState) = Xinit-matrixChi(:,jj);
    end
    chi(:,end) = Xinit;
    weight = [ones(1,2*DimState)/2/(DimState+kappa), kappa/(DimState+kappa)];
    xi = zeros(DimState,2*DimState+1);
    XhatPre = zeros(DimState,1);
    for jj = 1:1:2*DimState+1
        xi(:,jj) = feval(obj.f, obj, chi(:,jj));
        XhatPre = XhatPre+weight(jj)*xi(:,jj);
    end
end

%%% covariance prediction
if isempty(obj.f) && isnumeric(obj.Fx)
    Fx = obj.Fx;
    PhatPre1 = Fx*Pinit*Fx.';
elseif isa(obj.f, 'function_handle')
    PhatPre1 = 0;
    for jj = 1:1:2*DimState+1
        PhatPre1 = PhatPre1+weight(jj)*(xi(:,jj)-XhatPre)*(xi(:,jj)-XhatPre).';
    end
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
PhatPre = PhatPre1 + Fw*Q*Fw.';

%%% Kalman gain
if nargout>2
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
        if isnumeric(obj.Fx)
            matrixXi = chol((DimState+kappa)*PhatPre,'lower');
            xi = zeros(DimState,2*DimState+1);
            for jj = 1:1:DimState
                xi(:,jj) = XhatPre+matrixXi(:,jj);
                xi(:,jj+DimState) = XhatPre-matrixXi(:,jj);
            end
            xi(:,end) = XhatPre;
            weight = [ones(2*DimState,1)/2/(DimState+kappa); kappa/(DimState+kappa)];
        end
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
end

if nargout>3
    K = Pxz/Stil;
end

if nargout>4
    Ztil = Z-ZPre;
end

end

