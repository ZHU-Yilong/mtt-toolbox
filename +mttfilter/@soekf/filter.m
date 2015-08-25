function [Xhat, Phat, Stil, K, Ztil] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run extended kalman filter with 
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
if nargin == 4
    flag = 0;
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
DimMeasure = length(obj.MeasureSym);
if size(Z,1)~=DimMeasure && size(Z,2)==DimMeasure
    Z = Z.';
end
if size(Z,1)~=DimMeasure
    error('size of third input argument is not appropriate')
end

%%%
NumStep = size(Z,2);
%%% 
Xhat = zeros(DimState,NumStep+1);
Phat = zeros(DimState,DimState,NumStep+1);
Stil = zeros(DimMeasure,DimMeasure,NumStep);
K    = zeros(DimState,DimMeasure,NumStep);
Ztil = zeros(DimMeasure,NumStep);
%%%
Xhat(:,1) = Xinit;
Phat(:,:,1) = Pinit;
%%%
if flag
    h = waitbar(0,'0%','Name','Second-Order Extended Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
%%% filtering steps
for kk = 1:1:NumStep
    if flag
        if getappdata(h,'canceling')
            break
        end
    end
    %%% prediction ---- first order item
    if isempty(obj.f) && isnumeric(obj.Fx)
        XhatPre = obj.Fx*Xhat(:,kk);
    elseif isa(obj.f, 'function_handle')
        XhatPre = feval(obj.f, obj, Xhat(:,kk));
    end
    
    %%% prediction ---- second order item
    if isnumeric(obj.Fxx)
        Fxx = obj.Fxx;
    elseif isa(obj.Fxx, 'function_handle')
        Fxx = feval(obj.Fxx, obj, Xhat(:,kk));
    end
    if ~isequal(Fxx, zeros(DimState,DimState,DimState))
        for jj = 1:1:DimState
            Fxx_jj = squeeze(Fxx(:,:,jj));
            XhatPre(jj) = XhatPre(jj) + 0.5*trace(Fxx_jj*Phat(:,:,kk));
        end
    end
    
    %%% covariance prediction
    if isnumeric(obj.Fx)
        Fx = obj.Fx;
    elseif isa(obj.Fx, 'function_handle')
        Fx = feval(obj.Fx, obj, Xhat(:,kk));    
    end
    if isnumeric(obj.Fw)
        Fw = obj.Fw;
    elseif isa(obj.Fw, 'function_handle')
        Fw = feval(obj.Fw, obj, Xhat(:,kk));
    end
    if isnumeric(obj.Q)
        Q = obj.Q;
    elseif isa(obj.Q, 'function_handle')
        Q = feval(obj.Q, obj, Xhat(:,kk));
    end
    PhatPre = Fx*Phat(:,:,kk)*Fx.' + Fw*Q*Fw.';
    
    %%% covariance prediction ---- sencond order item
    if ~isequal(Fxx, zeros(DimState,DimState,DimState))
        for jj = 1:1:DimState
            for ii = 1:1:DimState
                Fxx_ii = squeeze(Fxx(:,:,ii));
                Fxx_jj = squeeze(Fxx(:,:,jj));
                PhatPre(ii,jj) = PhatPre(ii,jj) + 0.5*trace(Fxx_ii*Phat(:,:,kk)*Fxx_jj*Phat(:,:,kk));
            end
        end
    end
    
    %%% Kalman    
    if isnumeric(obj.Hx)
        Hx = obj.Hx;
    elseif isa(obj.Hx, 'function_handle')
        Hx = feval(obj.Hx, obj, XhatPre);
    end
    if isnumeric(obj.Hxx)
        Hxx = obj.Hxx;
    elseif isa(obj.Hxx, 'function_handle')
        Hxx = feval(obj.Hxx, obj, XhatPre);
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
    Stil(:,:,kk) = Hx*PhatPre*Hx.' + Hv*R*Hv.';
    if ~isequal(Hxx, zeros(DimState,DimState,DimMeasure))
        for jj = 1:1:DimMeasure
            for ii = 1:1:DimMeasure
                Hxx_ii = squeeze(Hxx(:,:,ii));
                Hxx_jj = squeeze(Hxx(:,:,jj));
                Stil(ii,jj,kk) = Stil(ii,jj,kk) + 0.5*trace(Hxx_ii*PhatPre*Hxx_jj*PhatPre);
            end
        end
    end
    K(:,:,kk) = PhatPre*Hx.'/Stil(:,:,kk);

    %%% filtering
    if isempty(obj.h) && isnumeric(obj.Hx)
        ZPre = obj.Hx*XhatPre;
    elseif isa(obj.h, 'function_handle')
        ZPre = feval(obj.h, obj, XhatPre);
    end
    if ~isequal(Hxx, zeros(DimState,DimState,DimMeasure))
        for jj = 1:1:DimMeasure
            Hxx_jj = squeeze(Hxx(:,:,jj));
            ZPre(jj) = ZPre(jj) + 0.5*trace(Hxx_jj*PhatPre);
        end
    end
    Ztil(:,kk) = Z(:,kk) - ZPre;
    Xhat(:,kk+1) = XhatPre + K(:,:,kk)*Ztil(:,kk);
    
    %%% covariance fitlering
    Phat(:,:,kk+1) = (eye(DimState)-K(:,:,kk)*Hx)*PhatPre*(eye(DimState)-K(:,:,kk)*Hx).' + K(:,:,kk)*R*K(:,:,kk).';
    
    if flag
        waitbar(kk/NumStep, h, sprintf('%3.0f %%', kk*100/NumStep))
    end
end
if flag
    delete(h)
end
Xhat = Xhat(:,2:end);
Phat = Phat(:,:,2:end);
end

