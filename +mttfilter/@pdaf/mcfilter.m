function [Xhat, Phat] = mcfilter(obj, Xinit, Pinit, Z, flag)
% MCFILTER run Monte Carlo kalman filter with
% input arguments:
% obj   : handle of kalman filter
% Z     : measurement vector sequence
% Xinit : intial state vector
% Pinit : intial error variance matrix
% flag  : flag for progress bar (default = 1)
% and output arguments
%  Xhat : filtered state
%  Phat : filtered covariance

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 1;
end

%%% check input argument obj
if isempty(obj.MttFilter)
    error('filter is empty');
end

%%% check input argument Z
DimState = length(obj.StateSym);
DimMeasure = length(obj.MeasureSym);
NumStep = size(Z,1);
NumMc = size(Z,2);
if ~iscell(Z)
    error('measurement must be in cell form')
end
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

%%%
for kk = 1:1:NumMc
    for jj = 1:1:NumStep
        if size(Z{jj,kk},1)~=DimMeasure
            error('dimension of measurement vector should agree with measurment model')
        end
    end
end

%%%
Xhat = zeros(DimState,NumStep,NumMc);
Phat = zeros(DimState,DimState,NumStep,NumMc);

%%%
if flag
    h = waitbar(0,'0%','Name','Monte Carlo PDAF Filtering Progress ...',...
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
    Zkk = squeeze(Z(:,kk));
    Xinitkk = squeeze(Xinit(:,kk));
    Pinitkk = squeeze(Pinit(:,:,kk));
    [Xhat(:,:,kk), Phat(:,:,:,kk)] = obj.filter(Xinitkk, Pinitkk, Zkk, 0);
    
    %%% waitbar
    if flag
        waitbar(kk/NumMc,h,sprintf('%3.0f %%',kk*100/NumMc))
    end
end
if flag
    delete(h)
end
end
