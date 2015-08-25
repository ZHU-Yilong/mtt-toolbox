function [Xhat, Phat, Stil, K, Ztil] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run kalman filter with input arguments
%  obj   : kalman filter object
%  Z     : measurement sequence
%  Xinit : initial state vector
%  Pinit : initial covariance matrix
% and output arguments
%  Xhat  : filtered state vector
%  Phat  : filtered covariance matrix
%  Ztil  : measurement  
%  Stil  :  

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 0;
end

%%% check input argument obj
if (~isempty(obj.f)) || (~isnumeric(obj.Fx))
    error('motion model must be a linear one');
end
if (~isempty(obj.h)) || (~isnumeric(obj.Hx))
    error('measurement model must be a linear one');
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must be agree');
end
if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
    error('state symbolic variables motion model and measurement model must be agree')
end

%%% check input argument Xinit
DimState = length(obj.StateSym);                % state vector dimension
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
DimMeasure = length(obj.MeasureSym);                        % measurement dimension
if size(Z,2)==DimMeasure && size(Z,1)~=DimMeasure
    Z = Z.';                                                % reshape measurement matrix
end
if size(Z,1)~=DimMeasure
    error('size of third input argument is not appropriate')
end

%%% constant matrix
Fx = obj.Fx;
Hx = obj.Hx;
%%% 
NumStep = size(Z,2);
%%% initial variables
Xhat = zeros(DimState,NumStep+1);
Phat = zeros(DimState,DimState,NumStep+1);
Stil = zeros(DimMeasure,DimMeasure,NumStep);
K    = zeros(DimState,DimMeasure,NumStep);
Ztil = zeros(DimMeasure,NumStep);
Xhat(:,1) = Xinit;
Phat(:,:,1) = Pinit;
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Stadard Kalman Filtering Progress ...',...
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
    %%% state prediction
    XhatPre = Fx*Xhat(:,kk);
    
    %%% covariance prediction
    if isnumeric(obj.Fw)
        Fw = obj.Fw;
    elseif isa(obj.Fw, 'function_handle')
        Fw = feval(obj.Fw, obj, Xhat(:,kk));
    end
    if isnumeric(obj.Q)
        Q = obj.Q;
    elseif isa(obj.Q, 'function_handle')
        Q = feval(obj.Q, obj, Xhat(:,kk));
    end
    PhatPre = Fx*Phat(:,:,kk)*Fx.' + Fw*Q*Fw.';
    
    %%% Kalman gain
    if isnumeric(obj.Hv)
        Hv = obj.Hv;
    elseif isa(obj.Hv,'function_handle')
        Hv = feval(obj.Hv, obj, XhatPre);
    end
    if isnumeric(obj.R)
        R = obj.R;
    elseif isa(obj.R, 'function_handle')
        R = feval(obj.R, obj, XhatPre);
    end
    Stil(:,:,kk) = Hx*PhatPre*Hx.' + Hv*R*Hv.';
    K(:,:,kk) = PhatPre*Hx.'/Stil(:,:,kk);
    
    %%% state filtering
    Ztil(:,kk) = Z(:,kk)-Hx*XhatPre;
    Xhat(:,kk+1) = XhatPre+K(:,:,kk)*Ztil(:,kk);
    
    %%% covariance fitlering
    Phat(:,:,kk+1) = (eye(DimState)-K(:,:,kk)*Hx)*PhatPre*(eye(DimState)-K(:,:,kk)*Hx).' + K(:,:,kk)*R*K(:,:,kk).';
    
    %%% progress bar
    if flag
        waitbar(kk/NumStep,h,sprintf('%3.0f %%',kk*100/NumStep))
    end
end
%%% progress bar
if flag
    delete(h)
end
Xhat = Xhat(:,2:end);
Phat = Phat(:,:,2:end);
end
