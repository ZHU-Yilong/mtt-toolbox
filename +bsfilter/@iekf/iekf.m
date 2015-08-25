classdef iekf
    
    properties(Constant=true)
        Description = 'iterated extended kalman filter';
    end
    properties
        MotionModel = mtnmodel.dxdy;
        MeasurementModel = msmodel.drb;
    end
    properties
        NumIteration = 3;
    end
    properties(Dependent=true,SetAccess=private)
        f
        Fx
        Fxx
        Fw
        Q
        %%%
        h
        Hx
        Hxx
        Hv
        R
        %%%
        StateSym
        MeasureSym
    end
    properties(Dependent=true,SetAccess=private,Hidden=true)
        HSym
    end
    
    methods
        %%% constructor
        function obj = iekf(MotionModel, MeasurementModel, NumIteration)
            switch nargin
                case 1
                    obj.MotionModel = MotionModel;
                case 2
                    obj.MotionModel = MotionModel;
                    obj.MeasurementModel = MeasurementModel;
                case 3
                    obj.MotionModel = MotionModel;
                    obj.MeasurementModel = MeasurementModel;
                    obj.NumIteration = NumIteration;
                otherwise
            end
        end
        
        %%% set property functions
        function obj = set.MotionModel(obj, MotionModel)
            if ~strncmp(class(MotionModel),'mtnmodel',length('mtnmodel'))
                error('first input argument must be class handle within mtnmodel package')
            end
            obj.MotionModel = MotionModel;
        end
        
        function obj = set.MeasurementModel(obj, MeasurementModel)
            if ~strncmp(class(MeasurementModel),'msmodel',length('msmodel'))
                error('second input argument must be class handle within msmodel package')
            end
            obj.MeasurementModel = MeasurementModel;
        end
        
        function obj = set.NumIteration(obj, NumIteration)
            if NumIteration < 1
                error('third input argument must be greater than 0')
            elseif NumIteration == 1
                warning('iteration times equal one, there is no need to employ iterated EKF')
            end
            obj.NumIteration = NumIteration;
        end
        
        %%% get dependent property functions
        function f = get.f(obj)
            if isempty(obj.MotionModel.f)
                f = [];
            elseif isa(obj.MotionModel.f, 'function_handle')
                f = @iekf_f;
            end
        end
        function Fx = get.Fx(obj)
            if isnumeric(obj.MotionModel.Fx)
                Fx = obj.MotionModel.Fx;
            elseif isa(obj.MotionModel.Fx, 'function_handle')
                Fx = @iekf_Fx;
            end
        end
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.MotionModel.Fxx)
                Fxx = obj.MotionModel.Fxx;
            elseif isa(obj.MotionModel.Fxx, 'function_handle')
                Fxx = @iekf_Fxx;
            end
        end
        function Fw = get.Fw(obj)
            if isnumeric(obj.MotionModel.Fw)
                Fw = obj.MotionModel.Fw;
            elseif isa(obj.MotionModel.Fw, 'function_handle')
                Fw = @iekf_Fw;
            end
        end
        function Q = get.Q(obj)
            if isnumeric(obj.MotionModel.Q)
                Q = obj.MotionModel.Q;
            elseif isa(obj.MotionModel.Q, 'function_handle')
                Q = @iekf_Q;
            end
        end
        %%%
        function h = get.h(obj)
            if isempty(obj.MeasurementModel.h)
                h = [];
            elseif isa(obj.MeasurementModel.h, 'function_handle')
                h = @iekf_h;
            end
        end
        function Hx = get.Hx(obj)
            if isnumeric(obj.MeasurementModel.Hx)
                Hx = obj.MeasurementModel.Hx;
            elseif isa(obj.MeasurementModel.Hx, 'function_handle')
                Hx = @iekf_Hx;
            end
        end
        function Hxx = get.Hxx(obj)
            if isnumeric(obj.MeasurementModel.Hxx)
                Hxx = obj.MeasurementModel.Hxx;
            elseif isa(obj.MeasurementModel.Hxx, 'function_handle')
                Hxx = @iekf_Hxx;
            end
        end
        function Hv = get.Hv(obj)
            if isnumeric(obj.MeasurementModel.Hv)
                Hv = obj.MeasurementModel.Hv;
            elseif isa(obj.MeasurementModel.Hv, 'function_handle')
                Hv = @iekf_Hv;
            end
        end
        function R = get.R(obj)
            if isnumeric(obj.MeasurementModel.R)
                R = obj.MeasurementModel.R;
            elseif isa(obj.MeasurementModel.R, 'function_handle')
                R = @iekf_R;
            end
        end
        %%%
        function StateSym = get.StateSym(obj)
            if ~isequal(obj.MotionModel.StateSym,obj.MeasurementModel.StateSym)
                error('state symbolic variables motion model and measurement model must be agree')
            else
                StateSym = obj.MotionModel.StateSym;
            end
        end
        function MeasureSym = get.MeasureSym(obj)
            MeasureSym = obj.MeasurementModel.MeasureSym;
        end
        function HSym = get.HSym(obj)
            HSym = obj.MeasurementModel.HSym;
        end
    end
    
    methods(Access = private)
        function xnext = iekf_f(obj, xcurrent)
            xnext = feval(obj.MotionModel.f, obj.MotionModel, xcurrent);
        end
        function Fx = iekf_Fx(obj, xcurrent)
            Fx = feval(obj.MotionModel.Fx, obj.MotionModel, xcurrent);
        end
        function Fxx = iekf_Fxx(obj, xcurrent)
            Fxx = feval(obj.MotionModel.Fxx, obj.MotionModel, xcurrent);
        end
        function Fw = iekf_Fw(obj, xcurrent)
            Fw = feval(obj.MotionModel.Fw, obj.MotionModel, xcurrent);
        end
        function Q = iekf_Q(obj, xcurrent)
            Q = feval(obj.MotionModel.Q, obj.MotionModel, xcurrent);
        end
        %%%
        function zpredict = iekf_h(obj, xpredict)
            zpredict = feval(obj.MeasurementModel.h, obj.MeasurementModel, xpredict);
        end
        function Hx = iekf_Hx(obj, xpredict)
            Hx = feval(obj.MeasurementModel.Hx, obj.MeasurementModel, xpredict);
        end
        function Hxx = iekf_Hxx(obj, xpredict)
            Hxx = feval(obj.MeasurementModel.Hxx, obj.MeasurementModel, xpredict);
        end
        function Hv = iekf_Hv(obj, xpredict)
            Hv = feval(obj.MeasurementModel.Hv, obj.MeasurementModel, xpredict);
        end
        function R = iekf_R(obj, xpredict)
            R = feval(obj.MeasurementModel.R, obj.MeasurementModel, xpredict);
        end
    end
    
    methods        
        %%%
        [XhatPre, PhatPre, Stil, K, Ztil] = predict (obj, Xinit,   Pinit,   Z);           % one step prediction
        [Xhat,    Phat,    Stil, K, Ztil] = update  (obj, XhatPre, PhatPre, Z);           % one step update
        [Xhat,    Phat,    Stil, K, Ztil] = filter  (obj, Xinit,   Pinit,   Z, flag);     % filtering
        [Xhat,    Phat                  ] = mcfilter(obj, Xinit,   Pinit,   Z, flag);     % Monte Carlo filtering
    end
    
end