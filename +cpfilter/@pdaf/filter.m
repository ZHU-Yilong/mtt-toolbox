function [Xhat, Phat, Stil, K, Ztil] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run kalman filter with input arguments
%  obj  : kalman filter object
%  Z    : measurement sequence
%  Xinit: initial state vector
%  Pinit: initial covariance matrix
%  flag : flag for progress bar
% and output arguments
%  Xhat : filtered state
%  Phat : filtered covariance

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 0;
end

%%% check input argument obj
if isempty(obj.MttFilter)
    error('filter is empty');
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
NumStep = length(Z);
DimMeasure = length(obj.MeasureSym); 
if ~iscell(Z)
    error('measurement must be in cell form')
end
for kk = 1:1:NumStep
    if size(Z{kk},1)~=DimMeasure
        error('dimension of measurement vector should agree with measurment model')
    end
end

%%% initial variables
Xhat = zeros(DimState,NumStep+1);
Phat = zeros(DimState,DimState,NumStep+1);
Stil = zeros(DimMeasure,DimMeasure,NumStep);
K    = zeros(DimState,DimMeasure,NumStep);
Ztil = zeros(DimMeasure,NumStep);

%%%
Xhat(:,1) = Xinit;
Phat(:,:,1) = Pinit;
lambda = obj.lambda;
gamma = obj.gamma;
Pg = obj.Pg;
Pd = obj.Pd;
b = lambda*(1-Pd*Pg)/Pd;

%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Probability Data Association Filtering Progress ...',...
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
    
    %%% S1. filtering with each one measurement
    NumMeasurement = size(Z{kk},2);
    XhatSub = zeros(DimState,NumMeasurement+1);
    PhatSub = zeros(DimState,DimState,NumMeasurement+1);
    StilSub = zeros(DimMeasure,DimMeasure,NumMeasurement);
    KSub    = zeros(DimState,DimMeasure,NumMeasurement);
    ZtilSub = zeros(DimMeasure,NumMeasurement);    
    e = zeros(NumMeasurement,1);
    for jj = 1:1:NumMeasurement
        [XhatSub(:,jj), PhatSub(:,:,jj), StilSub(:,:,jj), KSub(:,:,jj), ZtilSub(:,jj)] = obj.MttFilter.filter(Xhat(:,kk), Phat(:,:,kk), Z{kk}(:,jj), 0);
        gate = ZtilSub(:,jj)'/StilSub(:,:,jj)*ZtilSub(:,jj);
        if gate > gamma
            e(jj) = 0;
        else
            e(jj) = 1/sqrt(det(2*pi*StilSub(:,:,jj)))*exp(-0.5*gate);
        end
    end
    [XhatSub(:,end), PhatSub(:,:,end)] = obj.MttFilter.predict(Xhat(:,kk), Phat(:,:,kk), Z{kk}(:,end));
    
    %%% S2. association probability
    Beta = e./(sum(e)+b);
    Beta(NumMeasurement+1,1) = b/(sum(e)+b);
    
    %%% S3. estimation fusion
    Xhat(:,kk+1) = sum(XhatSub.*repmat(Beta.',DimState,1),2);
    
    PhatFusion = zeros(DimState,DimState,NumMeasurement+1);
    for jj = 1:1:NumMeasurement+1
        PhatFusion(:,:,jj) = (PhatSub(:,:,jj)+(Xhat(:,kk+1)-XhatSub(:,jj))*(Xhat(:,kk+1)-XhatSub(:,jj)).').*...
            repmat(Beta(jj),DimState,DimState);
    end
    Phat(:,:,kk+1) = sum(PhatFusion,3);
    
    Ztil(:,kk) = sum(ZtilSub.*repmat(Beta(1:NumMeasurement,1).',DimMeasure,1),2);
    K(:,:,kk) = mean(KSub,3);
    StilFusion = zeros(DimMeasure,DimMeasure,NumMeasurement);
    for ii = 1:1:NumMeasurement
    StilFusion(:,:,ii) = (StilSub(:,:,ii)+(Ztil(:,kk)-ZtilSub(:,ii))*(Ztil(:,kk)-ZtilSub(:,ii)).').*...
        repmat(Beta(ii),DimMeasure,DimMeasure);
    end
    Stil(:,:,kk) = sum(StilFusion,3);
    
    %%% progress bar
    if flag
        waitbar(kk/NumStep,h,sprintf('%3.0f %%',kk*100/NumStep))
    end
end
if flag
    delete(h)
end
Xhat = real(Xhat(:,2:end));
Phat = real(Phat(:,:,2:end));
end
