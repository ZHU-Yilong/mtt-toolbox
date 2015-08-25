function [Xhat, Phat, XParticle] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run particle filter with 
% input arguments
%  obj:   particle filter object
%  Z:     measurement sequence
%  Xinit: initial state vector
%  Pinit: initial covariance matrix
% and output arguments
%  Xhat:  filtered state
%  Phat:  filtered covariance

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 0;
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
NumParticle = obj.NumberParticle;
%%%
Xhat = zeros(DimState,NumStep+1);
Phat = zeros(DimState,DimState,NumStep+1);
XParticle = zeros(DimState,NumParticle,NumStep+1);
Xhat(:,1) = Xinit;
Phat(:,:,1) = Pinit;
XParticle(:,:,1) = obj.samplegaussian(Xinit, Pinit, NumParticle).';
Weight = ones(NumParticle,1)./NumParticle;
%%%
if flag
    h = waitbar(0,'0%','Name','Particle Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
%%% filtering steps
for kk = 1:1:NumStep
    if flag
        if getappdata(h,'canceling')
            break
        end
    end
    
    %%% prediction
    XParticlePre = zeros(DimState,NumParticle);
    for jj = 1:1:NumParticle
        if isempty(obj.f) && isnumeric(obj.Fx)
            XParticlePre(:,jj) = obj.Fx*XParticle(:,jj,kk);
        elseif isa(obj.f, 'function_handle')
            XParticlePre(:,jj) = feval(obj.f, obj, XParticle(:,jj,kk));
        end
    end
    if isnumeric(obj.Q)
        Q = obj.Q;
    elseif isa(obj.Q, 'function_handle')
        Q = feval(obj.Q, obj, Xhat(:,kk));
    end
    ProcessNoise = obj.samplegaussian(zeros(DimState,1), Q, NumParticle).';    
    XParticlePre = XParticlePre + ProcessNoise;
    XhatPre = mean(XParticlePre,2);
    
    %%% 
    if isnumeric(obj.Hx)
        Hx = obj.Hx;
    elseif isa(obj.Hx, 'function_handle')
        Hx = feval(obj.Hx, obj, XhatPre);
    end
    if isnumeric(obj.R)
        R = obj.R;
    elseif isa(obj.R, 'function_handle')
        R = feval(obj.R, obj, XhatPre);
    end
    
    %%% measurement prediction
    ZPre = zeros(DimMeasure,NumParticle);
    for jj = 1:1:NumParticle
        if isempty(obj.h) && isnumeric(obj.Hx)
            ZPre(:,jj) = Hx*XParticlePre(:,jj);
        elseif isa(obj.h, 'function_handle')
            ZPre(:,jj) = feval(obj.h, obj, XParticlePre(:,jj));
        end
    end

    %%% weights
    for jj = 1:1:NumParticle
        Weight(jj) = exp(-0.5*(Z(:,kk)-ZPre(:,jj))'/R*(Z(:,kk)-ZPre(:,jj)));
    end
    Weight = Weight./sum(Weight);
    
    %%% resampling
    WeightCumsum = cumsum(Weight);
    aux = rand(1);
    u = aux:1:(NumParticle-1+aux);
    u = u./NumParticle;
    nn = 1;
    for jj = 1:1:NumParticle
        while (u(jj)>WeightCumsum(nn))
            nn = nn+1;
        end
        XParticle(:,jj,kk+1) = XParticlePre(:,nn);
    end
    
    %%% update
    Xhat(:,kk+1) = mean(XParticle(:,:,kk+1),2);
    Phat(:,:,kk+1) = cov(XParticle(:,:,kk+1).');

    %%%
    if flag
        waitbar(kk/NumStep,h,sprintf('%3.0f %%',kk*100/NumStep))
    end

end
if flag
    delete(h)
end
Xhat = Xhat(:,2:end);
Phat = Phat(:,:,2:end);
XParticle = XParticle(:,:,2:end);
end


