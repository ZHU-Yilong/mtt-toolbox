function [Xhat, Phat] = mcfilter(obj, Xinit, Pinit, Z, flag)
% MCFILTER run Monte Carlo coverted measurement kalman filter with
% input arguments:
%  obj   : handle of autonomous multiple model filter
%  Z     : measurement vector sequence
%  Xinit : intial state vector
%  Pinit : intial error variance matrix
%  flag  : flag for progress bar (default = 1)
% and output arguments
%  Xhat  : filtered state
%  Phat  : filtered covariance

if nargin < 4
    error('not enough input arguments');
end
if nargin == 4
    flag = 1;
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

%%%
DimState = length(obj.StateSym);
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
%%%
Fx = obj.Fx;

%%% filtering
Xhat = zeros(DimState,NumStep+1,NumMc);
Phat = zeros(DimState,DimState,NumStep+1,NumMc);
if flag
    h = waitbar(0,'0%','Name','Monte Carlo Converted Measurements Kalman Filtering Progress ...',...
                'CreateCancelBtn',...
                'setappdata(gcbf,''canceling'',1)');
    setappdata(h,'canceling',0)
end
NumTotal = NumStep*NumMc;
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
        
        %%% prediction
        if isempty(obj.f)
            XhatPre = Fx*Xhat(:,jj,kk);
        elseif isa(obj.f, 'function_handle')
            XhatPre = feval(obj.f, obj, Xhat(:,jj,kk));
        end

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

        %%% Converted measurements
        if isnumeric(obj.R)
            Rm = obj.R;
        elseif isa(obj.R, 'function_handle')
            Rm = feval(obj.R, obj, XhatPre);
        end

        R = Z(1,jj,kk);
        B = Z(2,jj,kk);
        SigmaR = Rm(1,1);
        SigmaB = Rm(2,2);
        if DimMeasure == 3
            E = Z(3,jj,kk);
            SigmaE = Rm(3,3);
        elseif DimMeasure == 2
            E = 0;
            SigmaE = 0;
        end

        Lambda = 1-exp(-SigmaB-SigmaE)+exp(-0.5*SigmaB-0.5*SigmaE);
        Mu = 1-exp(-SigmaE)+exp(-0.5*SigmaE);

        Zc(1,1) = Lambda*R*cos(B)*cos(E);
        Zc(2,1) = Lambda*R*sin(B)*cos(E);
        if DimMeasure == 3
            Zc(3,1) = Mu*R*sin(E);
        end

        AlphaX = sin(B)^2*sinh(SigmaB)+cos(B)^2*cosh(SigmaB);
        AlphaY = sin(B)^2*cosh(SigmaB)+cos(B)^2*sinh(SigmaB);
        AlphaZ = sin(E)^2*cosh(SigmaE)+cos(E)^2*sinh(SigmaE);
        AlphaXY = sin(E)^2*sinh(SigmaE)+cos(E)^2*cosh(SigmaE);
        BetaX = sin(B)^2*sinh(2*SigmaB)+cos(B)^2*cosh(2*SigmaB);
        BetaY = sin(B)^2*cosh(2*SigmaB)+cos(B)^2*sinh(2*SigmaB);
        BetaZ = sin(E)^2*cosh(2*SigmaE)+cos(E)^2*sinh(2*SigmaE);
        BetaXY = sin(E)^2*sinh(2*SigmaE)+cos(E)^2*cosh(2*SigmaE);

        Rxx = (R^2*(BetaX*BetaXY-AlphaX*AlphaXY)+SigmaR*(2*BetaX*BetaXY-AlphaX*AlphaXY))*exp(-2*SigmaB-2*SigmaE);
        Rxy = (R^2*(BetaXY-AlphaXY*exp(SigmaB))+SigmaR*(2*BetaXY-AlphaXY*exp(SigmaB)))*sin(B)*cos(B)*exp(-4*SigmaB-2*SigmaE);
        Rxz = (R^2*(1-exp(SigmaE))+SigmaR*(2-exp(SigmaE)))*cos(B)*sin(E)*cos(E)*exp(-SigmaB-4*SigmaE);
        Ryy = (R^2*(BetaY*BetaXY-AlphaY*AlphaXY)+SigmaR*(2*BetaY*BetaXY-AlphaY*AlphaXY))*exp(-2*SigmaB-2*SigmaE);
        Ryz = (R^2*(1-exp(SigmaE))+SigmaR*(2-exp(SigmaE)))*sin(B)*sin(E)*cos(E)*exp(-SigmaB-4*SigmaE);
        Rzz = (R^2*(BetaZ-AlphaZ)+SigmaR*(2*BetaZ-AlphaZ))*exp(-2*SigmaE);

        if DimMeasure == 3
            Rc = [Rxx, Rxy, Rxz;
                  Rxy, Ryy, Ryz;
                  Rxz, Ryz, Rzz];
        elseif DimMeasure == 2
            Rc = [Rxx, Rxy; Rxy, Ryy];
        end

        %%% Kalman
        Stil = Hcx*PhatPre*Hcx.' + Rc;
        K = PhatPre*Hcx.'/Stil;

        %%% filtering
        ZPre = Hcx*XhatPre;
        Ztil = Zc-ZPre;
        Xhat(:,jj+1,kk) = XhatPre + K*Ztil;

        %%% fitlering covariance
        Phat(:,:,jj+1,kk) = (eye(DimState)-K*Hcx)*PhatPre*(eye(DimState)-K*Hcx).' + K*Rc*K.';

        %%% waitbar
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
Phat = Phat(:,:,2:end,:);
end
