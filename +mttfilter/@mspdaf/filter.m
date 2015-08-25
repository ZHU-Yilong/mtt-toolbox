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
NumFilter = length(obj.MttFilterSet);
if NumFilter==1
    warning('it is unnecessary to use mspdaf')
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
NumStep = size(Z,2);
if ~iscell(Z)                                       % check is a cell
    error('measurement must be in cell form')
end
if size(Z,1)~=NumFilter                             % check rows
    error('number of rows of measurement must be agree with number of filters')
end
DimMeasure = zeros(NumFilter,1);
for jj = 1:1:NumFilter
    DimMeasure(jj) = length(obj.MttFilterSet{jj}.MeasurementModel.MeasureSym);
end
for kk = 1:1:NumStep
    for jj = 1:1:NumFilter
        if size(Z{jj,kk},1)~=DimMeasure(jj)        % check measurement dimension
            error('dimension of measurement vector should agree with measurment model')
        end
    end
end

%%% initial variables
Xhat = zeros(DimState,NumStep+1);
Phat = zeros(DimState,DimState,NumStep+1);
Stil = zeros(DimMeasure(end),DimMeasure(end),NumStep);
K    = zeros(DimState,DimMeasure(end),NumStep);
Ztil = zeros(DimMeasure(end),NumStep);
%%%
XhatFilter = zeros(DimState,1);
PhatFilter = zeros(DimState,DimState);
Xhat(:,1) = Xinit;
Phat(:,:,1) = Pinit;
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Multiple Source Probability Data Association Filtering Progress ...',...
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
    
    for jj = 1:1:NumFilter
        lambda = obj.ParametersSet(1,jj);
        gamma = obj.ParametersSet(2,jj);
        Pg = obj.ParametersSet(3,jj);
        Pd = obj.ParametersSet(4,jj);        
        NumMeasurement = size(Z{jj,kk},2);
        %%%
        XhatSub = zeros(DimState,NumMeasurement+1);
        PhatSub = zeros(DimState,DimState,NumMeasurement+1);        
        StilSub = zeros(DimMeasure(jj),DimMeasure(jj),NumMeasurement);
        KSub    = zeros(DimState,DimMeasure(jj),NumMeasurement);
        ZtilSub = zeros(DimMeasure(jj),NumMeasurement);
        e = zeros(NumMeasurement,1);
        for ii = 1:1:NumMeasurement
            %%% prediction 
            if jj==1
                [XpreFilter, PpreFilter] = obj.MttFilterSet{1}.predict(Xhat(:,kk), Phat(:,:,kk),Z{1,kk}(:,ii));
            end
            %%% S1. update with each one measurement
            [XhatSub(:,ii), PhatSub(:,:,ii), StilSub(:,:,ii), KSub(:,:,ii), ZtilSub(:,ii)] = ...
                obj.MttFilterSet{jj}.update(XpreFilter, PpreFilter, Z{jj,kk}(:,ii));
            gate = ZtilSub(:,ii)'/StilSub(:,:,ii)*ZtilSub(:,ii);
            if gate > gamma
                e(ii) = 0;
            else
                e(ii) = 1/sqrt(det(2*pi*StilSub(:,:,ii)))*exp(-0.5*gate);
            end
        end
        XhatSub(:,end) = XpreFilter;
        PhatSub(:,:,end) = PpreFilter;

        %%% S2. association probability
        b = lambda*(1-Pd*Pg)/Pd;
        Beta = e./(sum(e)+b);
        Beta(NumMeasurement+1,1) = b/(sum(e)+b);

        %%% S3. estimation fusion
        XhatFilter = sum(XhatSub.*repmat(Beta.',DimState,1),2);
        PhatFusion = zeros(DimState,DimState,NumMeasurement+1);
        for ii = 1:1:NumMeasurement+1
            PhatFusion(:,:,ii) = (PhatSub(:,:,ii)+(XhatFilter-XhatSub(:,ii))*(XhatFilter-XhatSub(:,ii)).').*...
                repmat(Beta(ii),DimState,DimState);
        end
        PhatFilter = sum(PhatFusion,3);
        
        XpreFilter = XhatFilter;
        PpreFilter = PhatFilter;
    
    end
    
    Xhat(:,kk+1) = XhatFilter;
    Phat(:,:,kk+1) = PhatFilter;
    
    %%% Ztil and Stil fusion / error when measurement are out of gate
    Ztil(:,kk) = sum(ZtilSub.*repmat(Beta(1:NumMeasurement,1).',DimMeasure(end),1),2);
    K(:,:,kk) = mean(KSub,3);
    StilFusion = zeros(DimMeasure(end),DimMeasure(end),NumMeasurement);
    for ii = 1:1:NumMeasurement
    StilFusion(:,:,ii) = (StilSub(:,:,ii)+(Ztil(:,kk)-ZtilSub(:,ii))*(Ztil(:,kk)-ZtilSub(:,ii)).').*...
        repmat(Beta(ii),DimMeasure(end),DimMeasure(end));
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
