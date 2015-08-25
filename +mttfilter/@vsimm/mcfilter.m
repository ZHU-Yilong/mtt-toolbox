function [Xhat, Phat] = mcfilter(obj, Xinit, Pinit, Minit, Z, MdlSqn, flag)
% MCFILTER run Monte Carlo kalman filter with
% input arguments:
%  obj   : handle of variable structure interacting multiple model filter
%  Xinit  : intial state vector
%  Pinit  : intial error variance matrix
%  Minit  : intial model set
%  Z      : measurement vector sequence
%  MdlSqn : model set squence
%  flag   : flag for progress bar (default = 1)
% and output arguments
%  Xhat   : filtered state
%  Phat   : filtered covariance

if nargin < 6
    error('not enough input arguments');
end
if nargin == 6
    flag = 1;
end

%%% check input argument obj
% if isempty(obj.MttFilterSet)
%     error('filter set is empty');
% end
% NumFilter = length(obj.MttFilterSet);
% if size(obj.MatrixPi,1)~=NumFilter || size(obj.MatrixPi,2)~=NumFilter
%     error('dimensions of MatrixPi and length of MttFilterSet must be agree with each other')
% end

%%%
DimState = length(obj.StateSym);
% DimMeasure = length(obj.MeasureSym);
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
% if size(Xinit,1)~=DimState || size(Xinit,2)~=NumMc
%     error('size of first input argument is not appropriate')
% end

%%% check input argument Pinit
if ismatrix(Pinit)
    Pinit = repmat(Pinit,[1,1,NumMc]);
end
% if size(Pinit,1)~=DimState || size(Pinit,2)~=DimState || size(Pinit,3)~=NumMc
%     error('size of second input argument is not appropriate')
% end

%%% check input argument Minit
if isrow(Minit)
    Minit = repmat(Minit.', [1,NumMc]);
elseif iscolumn(Minit)
    Minit = repmat(Minit,[1,NumMc]);
end
% MinitVec = cell2mat(Minit.');
% if max(MinitVec)>NumFilter || min(MinitVec)<1
%     error('elements of third input argument beyond the range of filter numbers')
% end

%%% check input argument Z
% if size(Z,1)~=DimMeasure
%     error('size of fourth input argument is not appropriate')
% end

%%% check input argument MdlSqn
% if ~iscell(MdlSqn) || ~ismatrix(MdlSqn)
%     error('fifth input argument must be a cell and matrix')
% end
if isrow(MdlSqn)
    MdlSqn = repmat(MdlSqn.', [1,NumMc]);
elseif iscolumn(MdlSqn)
    MdlSqn = repmat(MdlSqn, [1,NumMc]);
end
% if size(MdlSqn,1)~=NumStep || size(MdlSqn,2)~=NumMc
%     error('size of fifth input argument is not appropriate')
% end


%%% filtering
Xhat = zeros(DimState,NumStep,NumMc);
Phat = zeros(DimState,DimState,NumStep,NumMc);
if flag
    h = waitbar(0,'0%','Name','Monte Carlo VS-IMM Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
for kk = 1:1:NumMc
    %%% progress
    if flag
        if getappdata(h,'canceling')
            break
        end
    end    
    
    %%% filtering steps
    Xinitkk  = squeeze(Xinit(:,kk));    
    Pinitkk  = squeeze(Pinit(:,:,kk));
    Minitkk  = squeeze(Minit(:,kk));
    Zkk      = squeeze(Z(:,:,kk));
    MdlSqnkk = squeeze(MdlSqn(:,kk));
    [Xhat(:,:,kk), Phat(:,:,:,kk)] = obj.filter(Xinitkk, Pinitkk, Minitkk, Zkk, MdlSqnkk, 0);
    %%% waitbar
    if flag
        waitbar(kk/NumMc,h,sprintf('%3.0f %%',kk*100/NumMc))
    end
end
if flag
    delete(h)
end
end
