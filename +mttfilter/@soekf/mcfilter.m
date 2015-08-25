function [Xhat, Phat] = mcfilter(obj, Xinit, Pinit, Z, flag)
% MTTFILTER.MCFILTER Monte Carlo filtering function
% input arguments:
% obj: handle of filter in the mttfitler package
% Z: measurement vector sequence with dimension
% DimensionMeasure*NumberStep*NumberMonteCarlo
% Xinit: intial state vector with dimesion
% DimensionState*NumberMonteCarlo
% Pinit: intial error variance matrix with dimension
% DimensionState*DimensionState*NumberMonteCarlo

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 1;
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

%%%
DimState = length(obj.StateSym);
DimMeasure = length(obj.MeasureSym);
NumStep = size(Z,2);
NumMc = size(Z,3);
if NumMc==1
    warning('Monte Carlo filtering times is only one, there is no need to employ this function')
end

%%% check input argument Xinit
if isrow(Xinit)
    Xinit = repmat(Xinit.',[1,NumMc]);
elseif iscolumn(Xinit)
    Xinit = repmat(Xinit,[1,NumMc]);
end
if size(Xinit,1)~=DimState || size(Xinit,2)~=NumMc
    error('size of first input argument is not appropriate')
end

%%% check input argument Pinit
if ismatrix(Pinit)
    Pinit = repmat(Pinit,[1,1,NumMc]);
end
if size(Pinit,1)~=DimState || size(Pinit,2)~=DimState || size(Pinit,3)~=NumMc
    error('size of second input argument is not appropriate')
end


%%% check input argument Z
if size(Z,1)~=DimMeasure
    error('size of third input argument is not appropriate')
end

%%% initialize variables
Xhat = zeros(DimState,NumStep+1,NumMc);
Phat = zeros(DimState,DimState,NumStep+1,NumMc);
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Monte Carlo Extended Kalman Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
NumTotal = NumStep*NumMc;
for kk = 1:1:NumMc
    Xhat(:,1,kk) = Xinit(:,kk);
    Phat(:,:,1,kk) = Pinit(:,:,kk);
    for jj = 1:1:NumStep
        %%% progress bar
        if flag
            if getappdata(h,'canceling')
                break
            end
        end
        
        %%% prediction
        if isempty(obj.f) && isnumeric(obj.Fx)
            XhatPre = obj.Fx*Xhat(:,jj,kk);
        elseif isa(obj.f, 'function_handle')
            XhatPre = feval(obj.f, obj, Xhat(:,jj,kk));
        end
        
        %%% prediction ---- second order item
        if isnumeric(obj.Fxx)
            Fxx = obj.Fxx;
        elseif isa(obj.Fxx, 'function_handle')
            Fxx = feval(obj.Fxx, obj, Xhat(:,kk));
        end
        if ~isequal(Fxx, zeros(DimState,DimState,DimState))
            for ii = 1:1:DimState
                Fxx_ii = squeeze(Fxx(:,:,ii));
                XhatPre(ii) = XhatPre(ii) + 0.5*trace(Fxx_ii*Phat(:,:,jj,kk));
            end
        end

        %%% covariance prediction
        if isnumeric(obj.Fx)
            Fx = obj.Fx;
        elseif isa(obj.Fx, 'function_handle')
            Fx = feval(obj.Fx, obj, Xhat(:,jj,kk));    
        end
        if isnumeric(obj.Fw)
            Fw = obj.Fw;
        elseif isa(obj.Fw, 'function_handle')
            Fw = feval(obj.Fw, obj, Xhat(:,jj,kk));
        end
        if isnumeric(obj.Q)
            Q = obj.Q;
        elseif isa(obj.Q, 'function_handle')
            Q = feval(obj.Q, obj, Xhat(:,jj,kk));    
        end
        PhatPre = Fx*Phat(:,:,jj,kk)*Fx.' + Fw*Q*Fw.';
        
        %%% covariance prediction ---- sencond order item
        if ~isequal(Fxx, zeros(DimState,DimState,DimState))
            for mm = 1:1:DimState
                for nn = 1:1:DimState
                    Fxx_mm = squeeze(Fxx(:,:,mm));
                    Fxx_nn = squeeze(Fxx(:,:,nn));
                    PhatPre(mm,nn) = PhatPre(mm,nn) + 0.5*trace(Fxx_mm*Phat(:,:,jj,kk)*Fxx_nn*Phat(:,:,jj,kk));
                end
            end
        end

        %%% Kalman gain
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
        Stil = Hx*PhatPre*Hx.' + Hv*R*Hv.';
        if ~isequal(Hxx, zeros(DimState,DimState,DimMeasure))
            for mm = 1:1:DimMeasure
                for nn = 1:1:DimMeasure
                    Hxx_mm = squeeze(Hxx(:,:,mm));
                    Hxx_nn = squeeze(Hxx(:,:,nn));
                    Stil(mm,nn) = Stil(mm,nn) + 0.5*trace(Hxx_mm*PhatPre*Hxx_nn*PhatPre);
                end
            end
        end
        K = PhatPre*Hx.'/Stil;

        %%% filtering
        if isempty(obj.h) && isnumeric(obj.Hx)
            ZPre = obj.Hx*XhatPre;
        elseif isa(obj.h, 'function_handle')
            ZPre = feval(obj.h, obj, XhatPre);
        end
        if ~isequal(Hxx, zeros(DimState,DimState,DimMeasure))
            for ii = 1:1:DimMeasure
                Hxx_ii = squeeze(Hxx(:,:,ii));
                ZPre(ii) = ZPre(ii) + 0.5*trace(Hxx_ii*PhatPre);
            end
        end
        Ztil = Z(:,jj,kk) - ZPre;
        Xhat(:,jj+1,kk) = XhatPre + K*Ztil;

        %%% covariance fitlering
        Phat(:,:,jj+1,kk) = (eye(DimState)-K*Hx)*PhatPre*(eye(DimState)-K*Hx).' + K*R*K.';

        %%% progress bar
        num = (kk-1)*NumStep+jj;
        if flag
            waitbar(num/NumTotal,h,sprintf('%3.0f %%',num*100/NumTotal))
        end
    end
end
if flag
    delete(h)
end
Xhat = Xhat(:,2:end,:);
Phat = Phat(:,:,2:end,:);
end