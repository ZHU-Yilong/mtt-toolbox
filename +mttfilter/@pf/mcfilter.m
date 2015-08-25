function [Xhat, Phat, XParticle] = mcfilter(obj, Xinit, Pinit, Z, flag)
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
    warning('both motion model and measurement model are linear, there is no need to employ particle filter');
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must agree');
end
if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
    error('state symbolic variables motion model and measurement model must be agree');
end

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

%%%
NumParticle = obj.NumberParticle;
Xhat = zeros(DimState,NumStep,NumMc);
Phat = zeros(DimState,DimState,NumStep,NumMc);
XParticle = zeros(DimState,NumParticle,NumStep,NumMc);

%%%
if flag
    h = waitbar(0,'0%','Name','Monte Carlo Particle Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end

%%% filtering
for kk = 1:1:NumMc

    %%% progress
    if flag
        if getappdata(h,'canceling')
            break
        end
    end
    
    %%%
    Zkk = squeeze(Z(:,:,kk));
    Xinitkk = squeeze(Xinit(:,kk));
    Pinitkk = squeeze(Pinit(:,:,kk));
    [Xhat(:,:,kk), Phat(:,:,:,kk), XParticle(:,:,:,kk)] = obj.filter(Xinitkk, Pinitkk, Zkk, 0);
    
    %%% waitbar
    if flag
        waitbar(kk/NumMc,h,sprintf('%3.0f %%',kk*100/NumMc))
    end
end
if flag
    delete(h)
end
end
