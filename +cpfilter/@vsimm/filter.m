function [Xhat, Phat] = filter(obj, Xinit, Pinit, Minit, Z, MdlSqn, flag)
% FILTER run kalman filter with input arguments
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
%%% default value for flag
if nargin < 6
    error('not enough input arguments');
end
if nargin == 6
    flag = 0;
end

%%% check input argument obj
if isempty(obj.MttFilterSet)
    error('filter set is empty');
end
NumFilter = length(obj.MttFilterSet);
if size(obj.MatrixPi,1)~=NumFilter || size(obj.MatrixPi,2)~=NumFilter
    error('dimensions of MatrixPi and number of MttFilterSet must be agree with each other')
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

%%% check input argument Minit
if ~isvector(Minit)
    error('third input argument must be a vector')
end
if iscell(Minit)
    Minit = cell2mat(Minit);
end
if max(Minit)>NumFilter || min(Minit)<1
    error('elements of third input argument beyond the range of filter numbers')
end

%%% check input argument Z
DimMeasure = length(obj.MeasureSym);
if size(Z,1)~=DimMeasure && size(Z,2)==DimMeasure
    Z = Z.';
end
if size(Z,1)~=DimMeasure
    error('size of fourth input argument is not appropriate')
end
NumStep = size(Z,2);

%%% check input argument MdlSqn
if ~iscell(MdlSqn) || ~isvector(MdlSqn)                         % MdlSqn 必须是包元向量
    error('fifth input argument must be a cell and vector')
end
if length(MdlSqn) < NumStep                                      % MdlSqn 长度必须大于等于观测向量长度
    error('size of fifth input argument is not appropriate')
end
MdlSqn = MdlSqn(1:NumStep);                                      % 对 MdlSqn 截断
    function B = reshape1(A)
        B = reshape(A, [], 1);
    end
MdlSqn = cellfun(@reshape1, MdlSqn);
MdlSqn = reshape(MdlSqn, [], 1);
MdlVec = cell2mat(MdlSqn);
% try                                                             % MdlSqn 元素大小必须在 1 -- NumFilter 之间
%     MdlVec = cell2mat(MdlSqn);
% catch err
%     try
%         MdlVec = cell2mat(MdlSqn.');
%     catch err2
%         rethrow(err)
%     end
% end
if isempty(MdlVec)
    error('elements of fifth input argument must be row or column vectors')
end
if max(MdlVec)>NumFilter || min(MdlVec)<1
    error('elements of fifth input argument beyond the range of filter numbers')
end

%%%


%%% initial variables
Xhat = zeros(DimState,NumStep);
Phat = zeros(DimState,DimState,NumStep);
% 
Models_curr = unique(Minit);
NumFilter_curr = length(Models_curr);
XhatSub = repmat(Xinit, [1,NumFilter_curr]);
PhatSub = repmat(Pinit, [1,1,NumFilter_curr]);
Mu = ones(NumFilter_curr,1)/NumFilter_curr;
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','VS-IMM Filtering Progress ...',...
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
    
    Models_prev = Models_curr;
    Models_curr = unique(MdlSqn{kk});
    MatrixPi_curr = obj.MatrixPi(Models_curr, Models_prev);
    NumFilter_curr = length(Models_curr);
    
    %%% S1. reintialization
    XhatRe    = zeros(DimState,NumFilter_curr);
    PhatRe    = zeros(DimState,DimState,NumFilter_curr);
    PhatReSub = zeros(DimState,DimState,NumFilter_curr);
    MuPre = sum(MatrixPi_curr.*repmat(Mu, [1, NumFilter_curr]), 1).';
    MuRe  = MatrixPi_curr .* (Mu*(1./MuPre.'));
    for jj = 1:1:NumFilter_curr
        XhatRe(:,jj) = sum(XhatSub.*repmat(MuRe(:,jj).',DimState,1),2);
        for ii = 1:1:NumFilter_curr
            PhatReSub(:,:,ii) = (PhatSub(:,:,jj)+(XhatRe(:,jj)-XhatSub(:,jj))*(XhatRe(:,jj)-XhatSub(:,jj)).').*...
                repmat(MuRe(ii,jj),DimState,DimState);
        end
        PhatRe(:,:,jj) = sum(PhatReSub,3);
    end
    
    %%% S2. filtering each one model
    StilSub = zeros(DimMeasure,DimMeasure,NumFilter_curr);
    KSub    = zeros(DimState,DimMeasure,NumFilter_curr);
    ZtilSub = zeros(DimMeasure,NumFilter_curr);
    Lhood = zeros(NumFilter_curr,1);
    for jj = 1:1:NumFilter_curr
        [XhatSub(:,jj), PhatSub(:,:,jj), StilSub(:,:,jj), KSub(:,:,jj), ZtilSub(:,jj)] = ...
            obj.MttFilterSet{Models_curr(jj)}.filter(XhatRe(:,jj), PhatRe(:,:,jj), Z(:,kk), 0);
        %%% likelihood
        Lhood(jj) = 1/sqrt(det(2*pi*StilSub(:,:,jj)))*exp(-0.5*ZtilSub(:,jj)'/StilSub(:,:,jj)*ZtilSub(:,jj));
    end
    
    %%% S3. model probability update
    Mu = MuPre.*Lhood/sum(MuPre.*Lhood);
    
    %%% S4. estimation fusion
    PhatFusion = zeros(DimState,DimState,NumFilter_curr);
    Xhat(:,kk) = sum(XhatSub.*repmat(Mu.',DimState,1),2);
    for jj = 1:1:NumFilter_curr
        PhatFusion(:,:,jj) = (PhatSub(:,:,jj)+(Xhat(:,kk)-XhatSub(:,jj))*(Xhat(:,kk)-XhatSub(:,jj)).').*...
            repmat(Mu,DimState,DimState);
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
end
