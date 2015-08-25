function [XhatPre, PhatPre, Stil, K, Ztil] = predict(obj, Xinit, Pinit, Z)
% FILTER run converted measurement kalman filter one step predict with 
% input arguments
%  obj   : kalman filter object
%  Z     : measurement vector
%  Xinit : initial state vector
%  Pinit : initial covariance matrix
%  num   : the number of step
% and output arguments
%  Xhat : filtered state
%  Phat : filtered covariance

if nargin<3
    error('not enough input arguments');
end
if nargin<4
    Z = [];
end
if isempty(Z) && nargout==5
    error('not enough input arguments')
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
if ~isempty(Z)
    if size(Z,1)~=DimMeasure && size(Z,2)==DimMeasure
        Z = Z.';
    end
    if size(Z,1)~=DimMeasure
        error('size of third input argument is not appropriate')
    end
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
Fx = obj.Fx;

%%% prediction
if isempty(obj.f)
    XhatPre = Fx*Xinit;
elseif isa(obj.f, 'function_handle')
    XhatPre = feval(obj.f, obj, Xinit);
end

%%% covariance prediction
if isnumeric(obj.Fw)
    Fw = obj.Fw;
elseif isa(obj.Fw, 'function_handle')
    Fw = feval(obj.Fw, obj, Xinit);
end
if isnumeric(obj.Q)
    Q = obj.Q;
elseif isa(obj.Q, 'function_handle')
    Q = feval(obj.Q, obj, Xinit);
end
PhatPre = Fx*Pinit*Fx.' + Fw*Q*Fw.';

%%% Converted measurements
if nargout>2
    if isnumeric(obj.R)
        Rm = obj.R;
    elseif isa(obj.R, 'function_handle')
        Rm = feval(obj.R, obj, XhatPre);
    end

    R = Z(1);
    B = Z(2);
    SigmaR = Rm(1,1);
    SigmaB = Rm(2,2);
    if DimMeasure == 3
        E = Z(3);
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
end

if nargout>3
    K = PhatPre*Hcx.'/Stil;
end
    
    %%% filtering
if nargout>4
    ZPre = Hcx*XhatPre;
    Ztil = Zc-ZPre;
end

end


