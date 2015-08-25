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
%%%
DimState = length(obj.StateSym);                            % state vector dimension
DimMeasure = length(obj.MeasureSym);                        % measurement dimension
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

%%% constant matrix
Fx = obj.Fx;
Hx = obj.Hx;
%%% initialize variables
Xhat = zeros(DimState,NumStep+1,NumMc);
Phat = zeros(DimState,DimState,NumStep+1,NumMc);
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Monte Carlo Standard Kalman Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
NumTotal = NumStep*NumMc;
%%% filtering
for kk = 1:1:NumMc
    Xhat(:,1,kk) = Xinit(:,kk);
    Phat(:,:,1,kk) = Pinit(:,:,kk);
    for jj = 1:1:NumStep
        %%% progress
        if flag
            if getappdata(h,'canceling')
                break
            end
        end
        
        %%% state prediction
        XhatPre = Fx*Xhat(:,jj,kk);

        %%% covariance prediction
        if isnumeric(obj.Fw)
            Fw = obj.Fw;
        elseif isa(obj.Fw, 'function_handle')
            Fw = feval(obj.Fw, obj, Xhat(:,jj,kk));
        end
        if isnumeric(obj.Q)
            Q = obj.Q;
        elseif isa(obj.Q, 'function_handle')
            Q = feval(obj.Q, obj, Xhat(:,jj,kk));
        end
        PhatPre = Fx*Phat(:,:,jj,kk)*Fx.' + Fw*Q*Fw.';

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
        Stil = Hx*PhatPre*Hx.' + Hv*R*Hv.';
        K = PhatPre*Hx.'/Stil;

        %%% state filtering
        Ztil = Z(:,jj,kk)-Hx*XhatPre;
        Xhat(:,jj+1,kk) = XhatPre+K*Ztil;

        %%% covariance fitlering
        Phat(:,:,jj+1,kk) = (eye(DimState)-K*Hx)*PhatPre*(eye(DimState)-K*Hx).' + K*R*K.';

        %%% progress bar
        num = (kk-1)*NumStep+jj;
        if flag
            waitbar(num/NumTotal,h,sprintf('%3.0f %%',num*100/NumTotal))
        end
    end
end
if flag
    delete(h)
end
Xhat = Xhat(:,2:end,:);
Phat = Phat(:,2:end,:);
end
