function [Xhat, Phat, Mu] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run kalman filter with input arguments
%  obj   : kalman filter object
%  Z     : measurement sequence
%  Xinit : initial state vector
%  Pinit : initial covariance matrix
%  flag  : flag for progress bar
% and output arguments
%  Xhat  : filtered state
%  Phat  : filtered covariance
%  Mu    : model probability

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 0;
end

%%% check input argument obj
if isempty(obj.MttFilterSet)
    error('filter set is empty');
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
NumFilter = length(obj.MttFilterSet);
%%% initial variables
Xhat = zeros(DimState,NumStep);
Phat = zeros(DimState,DimState,NumStep);
Mu = zeros(NumFilter,NumStep+1);

XhatSub = repmat(Xinit,[1,NumFilter]);
PhatSub = repmat(Pinit,[1,1,NumFilter]);
StilSub = zeros(DimMeasure,DimMeasure,NumFilter);
KSub    = zeros(DimState,DimMeasure,NumFilter);
ZtilSub = zeros(DimMeasure,NumFilter);

Mu(:,1) = ones(NumFilter,1)/NumFilter;
Lhood = zeros(NumFilter,1);
PhatFusion = zeros(DimState,DimState,NumFilter);
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Autonous Multiple Model Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
%%% filtering steps
for kk = 1:1:NumStep
    %%% progress bar
    if flag
        if getappdata(h,'canceling')
            break
        end
    end
    
    %%% S1. filtering each one model
    for jj = 1:1:NumFilter
        [XhatSub(:,jj), PhatSub(:,:,jj), StilSub(:,:,jj), KSub(:,:,jj), ZtilSub(:,jj)] = ...
            obj.MttFilterSet{jj}.filter(XhatSub(:,jj), PhatSub(:,:,jj), Z(:,kk), 0);
        %%% likelihood
        Lhood(jj) = 1/sqrt(det(2*pi*StilSub(:,:,jj)))*exp(-0.5*ZtilSub(:,jj)'/StilSub(:,:,jj)*ZtilSub(:,jj));
    end
    
    %%% S2. model probability update
    Mu(:,kk+1) = Mu(:,kk).*Lhood/sum(Mu(:,kk).*Lhood);
    
    %%% S3. estimation fusion
    Xhat(:,kk) = sum(XhatSub.*repmat(Mu(:,kk+1).',DimState,1),2);
    for jj = 1:1:NumFilter
        PhatFusion(:,:,jj) = (PhatSub(:,:,jj)+(Xhat(:,kk)-XhatSub(:,jj))*(Xhat(:,kk)-XhatSub(:,jj)).').*...
            repmat(Mu(jj,kk+1),DimState,DimState);
    end
    Phat(:,:,kk) = sum(PhatFusion,3);
    
    %%% progress bar
    if flag
        waitbar(kk/NumStep,h,sprintf('%3.0f %%',kk*100/NumStep))
    end
end
if flag
    delete(h)
end
Mu = Mu(:,2:end);
end
