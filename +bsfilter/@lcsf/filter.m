function [Xhat, Phat, Stil, K, Ztil] = filter(obj, Xinit, Pinit, Z, flag)
% FILTER run converted measurement kalman filter with 
% input arguments
%  obj   : kalman filter object
%  Z     : measurement sequence
%  Xinit : initial state vector
%  Pinit : initial covariance matrix
%  flag  : flag for progress bar (default = 0)
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
if (~isnumeric(obj.Fx)) || isnumeric(obj.Hx)
    error('motion model must be linear, and measurement model must be nonlinear')
end
DimMeasure = length(obj.MeasureSym);
if DimMeasure~=2 && DimMeasure~=3
    error('only two or three dimensional measurement model suport this filter')
end
if obj.Hv~=eye(DimMeasure)
    error('measurement variables must be nonlinear with noise')
end
if obj.MeasurementModel.Dimension~=obj.MotionModel.Dimension
    error('dimension(s) of motion model and measurement model must agree');
end
if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
    error('state symbolic variables motion model and measurement model must be agree')
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
if size(Z,1)~=DimMeasure && size(Z,2)==DimMeasure
    Z = Z.';
end
if size(Z,1)~=DimMeasure
    error('size of third input argument is not appropriate')
end

%%%
if DimMeasure == 3
     HcSym = [sym('x_position');
              sym('y_position');
              sym('z_position')];
elseif DimMeasure == 2
    HcSym = [sym('x_position');
             sym('y_position')];
end
Hcx = zeros(DimMeasure,DimState);
for jj = 1:1:DimMeasure
    for ii = 1:1:DimState
        Hcx(jj,ii) = isequal(HcSym(jj),obj.StateSym(ii));
    end
end
for ii = 1:1:DimState
    if Hcx(1,ii) == 1
        IndexXpos = ii;
    end
    if Hcx(2,ii) == 1
        IndexYpos = ii;
    end
    if (DimMeasure == 3) && (Hcx(3,ii) == 1)
        IndexZpos = ii;
    end
end
DimX = IndexYpos-1;
if (DimMeasure == 3)
    DimY = IndexZpos-IndexYpos;
    DimZ = DimState-IndexZpos+1;
elseif (DimMeasure == 2)
    DimY = DimState-IndexYpos+1;
end
if (DimX~=DimY)
    error('state dimension must be equal along different axes')
elseif (DimMeasure==3) && (DimX~=DimZ)
    error('state dimension must be equal along different axes')
end
Dim = DimX;
%%%
Fx = obj.Fx;
NumStep = size(Z,2);
%%%
Xhat = zeros(DimState,NumStep+1);
Phat = zeros(DimState,DimState,NumStep+1);
Stil = zeros(DimMeasure,DimMeasure,NumStep);
K    = zeros(DimState,DimMeasure,NumStep);
Ztil = zeros(DimMeasure,NumStep);
%%%
Xhat(:,1) = Xinit;
Phat(:,:,1) = Pinit;
%%% progress bar
if flag
    h = waitbar(0,'0%','Name','Line-of-Sight Kalman Filtering Progress ...',...
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
    %%% prediction in RCS
    if isempty(obj.f)
        XhatPre = Fx*Xhat(:,kk);
    elseif isa(obj.f, 'function_handle')
        XhatPre = feval(obj.f, obj, Xhat(:,kk));
    end
    
    %%% covariance prediction in RCS
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
    
    %%% coordinate update
    if DimMeasure == 2
        Azi = cart2pol(XhatPre(IndexXpos),XhatPre(IndexYpos));
        T = [cos(Azi),sin(Azi); -sin(Azi),cos(Azi)];
    elseif DimMeasure == 3
        [Azi,Ele] = cart2sph(XhatPre(IndexXpos),XhatPre(IndexYpos),XhatPre(IndexZpos));
        T = angle2dcm(Azi,-Ele,0,'ZYX');
    end
    Tall = kron(T,eye(Dim));
    
    %%% coordinate transform
    XhatPreLcs = Tall*XhatPre;
    PhatPreLcs = Tall*PhatPre*Tall.';

    %%% update measurements    
    if isnumeric(obj.R)
        Rm = obj.R;
    elseif isa(obj.R, 'function_handle')
        Rm = feval(obj.R, obj, XhatPre);
    end
    
    r = Z(1,kk);
    b = Z(2,kk);
    SigmaR = Rm(1,1);
    SigmaB = Rm(2,2);
    if DimMeasure == 3
        e = Z(3,kk);
        SigmaE = Rm(3,3);
    elseif DimMeasure == 2
        e = 0;
        SigmaE = 0;
    end
    R1 = SigmaR;
    R2 = (r*cos(e))^2*SigmaB;
    R3 = r^2*SigmaE;
    if DimMeasure == 3
        Rc = blkdiag(R1, R2, R3);
        [xMeasure,yMeasure,zMeasure] = sph2cart(b,e,r);
        ZLcs = T*[xMeasure;yMeasure;zMeasure];
    elseif DimMeasure == 2
        Rc = blkdiag(R1, R2);
        [xMeasure,yMeasure] = pol2cart(b,r);
        ZLcs = T*[xMeasure;yMeasure];
    end

    %%% Kalman in LCS
    Stil(:,:,kk) = Hcx*PhatPreLcs*Hcx'+Rc;
    K(:,:,kk) = PhatPreLcs*Hcx'/Stil(:,:,kk);
    
    %%% filtering in LCS
    ZPreLcs = Hcx*XhatPreLcs;
    Ztil(:,kk) = ZLcs-ZPreLcs;
    XhatLcs = XhatPreLcs + K(:,:,kk)*Ztil(:,kk);
    PhatLcs = (eye(DimState)-K(:,:,kk)*Hcx)*PhatPreLcs*(eye(DimState)-K(:,:,kk)*Hcx).' + K(:,:,kk)*Rc*K(:,:,kk).';
    
    %%% coordinate transform
    Xhat(:,kk+1) = Tall.'*XhatLcs;
    Phat(:,:,kk+1) = Tall.'*PhatLcs*Tall;
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
end

