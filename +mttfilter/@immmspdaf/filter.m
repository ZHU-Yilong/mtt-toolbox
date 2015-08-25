function [Xhat, Phat, Mu] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run kalman filter with input arguments
%  obj  : kalman filter object
%  Z    : measurement sequence
%  Xinit: initial state vector
%  Pinit: initial covariance matrix
%  flag : flag for progress bar
% and output arguments
%  Xhat : filtered state
%  Phat : filtered covariance
%  Mu   : model probability

%%% default value for flag
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
if ~ismatrix(obj.MatrixPi)
    error('MatrixPi is not a matrix');
end
NumFilter = length(obj.MttFilterSet);
if NumFilter==1
    warning('it is unnecessary to use imm/mspdaf')
end
if size(obj.MatrixPi,1)~=NumFilter || size(obj.MatrixPi,2)~=NumFilter
    error('dimensions of MatrixPi and length of MttFilterSet must be agree with each other')
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

%%% check and manipulate Z
NumStep = size(Z,2);
if ~iscell(Z)                                       % check is a cell
    error('measurement must be in cell form')
end
if size(Z,1)~=NumFilter                             % check rows
    error('number of rows of measurement must be agree with number of filters')
end
DimMeasure = zeros(NumFilter,1);
for jj = 1:1:NumFilter
    DimMeasure(jj) = length(obj.MttFilterSet{1}.MttFilterSet{jj}.MeasurementModel.MeasureSym);
end
for kk = 1:1:NumStep
    for jj = 1:1:NumFilter
        if size(Z{jj,kk},1)~=DimMeasure(jj)        % check measurement dimension
            error('dimension of measurement vector should agree with measurment model')
        end
    end
end

%%% initial variables
Xhat = zeros(DimState,NumStep);
Phat = zeros(DimState,DimState,NumStep);
StilSub = zeros(DimMeasure(end),DimMeasure(end),NumFilter);
KSub    = zeros(DimState,DimMeasure,NumFilter);
ZtilSub = zeros(DimMeasure(end),NumFilter);
Mu = zeros(NumFilter,NumStep+1);
Lhood = zeros(NumFilter,1);
PhatFusion = zeros(DimState,DimState,NumFilter);
XhatRe = zeros(DimState,NumFilter);
PhatRe = zeros(DimState,DimState,NumFilter);
PhatReSub = zeros(DimState,DimState,NumFilter);
%%% initial value
XhatSub = repmat(Xinit,1,NumFilter);
PhatSub = repmat(Pinit,[1,1,NumFilter]);
Mu(:,1) = ones(NumFilter,1)/NumFilter;
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','IMM/MSPDAF Filtering Progress ...',...
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
    
    %%% S1. reintialization
    MuPre = sum(obj.MatrixPi.*repmat(Mu(:,kk),1,NumFilter),1).';
    MuRe = obj.MatrixPi.*(Mu(:,kk)*(1./MuPre.'));
    for jj = 1:1:NumFilter
        XhatRe(:,jj) = sum(XhatSub.*repmat(MuRe(:,jj).',DimState,1),2);
        for ii = 1:1:NumFilter
            PhatReSub(:,:,ii) = (PhatSub(:,:,jj)+(XhatRe(:,jj)-XhatSub(:,jj))*(XhatRe(:,jj)-XhatSub(:,jj)).').*...
                repmat(MuRe(ii,jj),DimState,DimState);
        end
        PhatRe(:,:,jj) = sum(PhatReSub,3);
    end
    
    %%% S2. filtering each one model
    for jj = 1:1:NumFilter
        [XhatSub(:,jj), PhatSub(:,:,jj), StilSub(:,:,jj), KSub(:,:,jj), ZtilSub(:,jj)] = ...
            obj.MttFilterSet{jj}.filter(XhatRe(:,jj), PhatRe(:,:,jj), Z(:,kk), 0);
        %%% likelihood
        Lhood(jj) = 1/sqrt(det(2*pi*StilSub(:,:,jj)))*exp(-0.5*ZtilSub(:,jj)'/StilSub(:,:,jj)*ZtilSub(:,jj));
%         if isnan(Lhood(jj)) % for the case of out of gate measurement
%             Lhood(jj) = 0;
%         end
    end
    
    %%% S3. model probability update
    Mu(:,kk+1) = MuPre.*Lhood/sum(MuPre.*Lhood);
%     for jj = 1:1:NumFilter
%         if Lhood(jj)==0
%             Mu(jj,kk+1) = MuPre(jj);
%         end
%     end
    if any(isnan(Lhood))
        Mu(:,kk+1) = MuPre;
    end
    
    %%% S4. estimation fusion
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
