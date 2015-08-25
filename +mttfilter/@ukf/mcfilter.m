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
    warning('both motion model and measurement model are linear, there is no need to employ unscented kalman filter')
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
kappa = DimState-3;

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

%%% filtering
Xhat = zeros(DimState,NumStep+1,NumMc);
Phat = zeros(DimState,DimState,NumStep+1,NumMc);
if flag
    h = waitbar(0,'0%','Name','Monte Carlo Unscented Kalman Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
NumTotal = NumStep*NumMc;
for kk = 1:NumMc
    Xhat(:,1,kk) = Xinit(:,kk);
    Phat(:,:,1,kk) = Pinit(:,:,kk);
    for jj = 1:1:NumStep
        %%% progress
        if flag
            if getappdata(h,'canceling')
                break
            end
        end

        %%% prediction
        if isempty(obj.f) && isnumeric(obj.Fx)
            XhatPre = obj.Fx*Xhat(:,jj,kk);
        elseif isa(obj.f, 'function_handle')
            matrixChi = chol((DimState+kappa)*Phat(:,:,jj,kk),'lower');
            chi = zeros(DimState,2*DimState+1);
            for ii = 1:1:DimState
                chi(:,ii) = Xhat(:,jj,kk)+matrixChi(:,ii);
                chi(:,ii+DimState) = Xhat(:,jj,kk)-matrixChi(:,ii);
            end
            chi(:,end) = Xhat(:,jj,kk);
            weight = [ones(1,2*DimState)/2/(DimState+kappa), kappa/(DimState+kappa)];
            xi = zeros(DimState,2*DimState+1);
            XhatPre = zeros(DimState,1);
            for ii = 1:1:2*DimState+1
                xi(:,ii) = feval(obj.f, obj, chi(:,ii));
                XhatPre = XhatPre+weight(ii)*xi(:,ii);
            end
        end

        %%% covariance prediction
        if isempty(obj.f) && isnumeric(obj.Fx)
            Fx = obj.Fx;
            PhatPre1 = Fx*Phat(:,:,jj,kk)*Fx.';
        elseif isa(obj.f, 'function_handle')
            PhatPre1 = 0;
            for ii = 1:1:2*DimState+1
                PhatPre1 = PhatPre1+weight(ii)*(xi(:,ii)-XhatPre)*(xi(:,ii)-XhatPre).';
            end
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
        PhatPre = PhatPre1 + Fw*Q*Fw.';

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
            if isempty(obj.f) && isnumeric(obj.Fx)
                matrixXi = chol((DimState+kappa)*PhatPre,'lower');
                xi = zeros(DimState,2*DimState+1);
                for ii = 1:1:DimState
                    xi(:,ii) = XhatPre+matrixXi(:,ii);
                    xi(:,ii+DimState) = XhatPre-matrixXi(:,ii);
                end
                xi(:,end) = XhatPre;
                weight = [ones(2*DimState,1)/2/(DimState+kappa); kappa/(DimState+kappa)];
            end
            zeta = zeros(DimMeasure,2*DimState+1);
            ZPre = 0;
            for ii = 1:1:2*DimState+1
                zeta(:,ii) = feval(obj.h, obj, xi(:,ii));
                ZPre = ZPre + weight(ii)*zeta(:,ii);
            end
            Pzz = 0; Pxz = 0;
            for ii = 1:1:2*DimState+1
                Pzz = Pzz + weight(ii)*(zeta(:,ii)-ZPre)*(zeta(:,ii)-ZPre).';
                Pxz = Pxz + weight(ii)*(xi(:,ii)-XhatPre)*(zeta(:,ii)-ZPre).';
            end
            Stil = Pzz + Hv*R*Hv.';
        end
        K = Pxz/Stil;

        %%% filtering
        Ztil = Z(:,jj,kk) - ZPre;
        Xhat(:,jj+1,kk) = XhatPre + K*Ztil;

        %%% covariance fitlering
        Phat(:,:,jj+1,kk) = PhatPre - K*Stil*K.';
        %%% waitbar
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
