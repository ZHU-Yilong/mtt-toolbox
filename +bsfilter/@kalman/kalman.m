classdef kalman
    
    properties(Constant=true)
        Description = 'standard kalman filter';
    end
    properties
        MotionModel = mtnmodel.dx;
        MeasurementModel = msmodel.dx;
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
        function obj = kalman(MotionModel, MeasurementModel)
            switch nargin
                case 1
                    obj.MotionModel = MotionModel;
                case 2
                    obj.MotionModel = MotionModel;
                    obj.MeasurementModel = MeasurementModel;
                otherwise
            end
        end
        
        %%% set property functions
        function obj = set.MotionModel(obj, MotionModel)
            if ~strncmp(class(MotionModel),'mtnmodel',length('mtnmodel'))
                error('first input argument must be class handle within mtnmodel package')
            end
            if (~isempty(MotionModel.f)) || (~isnumeric(MotionModel.Fx))
                error('motion model must be a linear one')
            end
            obj.MotionModel = MotionModel;
        end
     
        function obj = set.MeasurementModel(obj, MeasurementModel)
            if ~strncmp(class(MeasurementModel),'msmodel',length('msmodel'))
                error('second input argument must be class handle within msmodel package')
            end
            if (~isempty(MeasurementModel.h)) || (~isnumeric(MeasurementModel.Hx))
                error('measurement model must be a linear one')
            end
            obj.MeasurementModel = MeasurementModel;
        end
        
        %%% get dependent property functions
        function f = get.f(obj)
            if isempty(obj.MotionModel.f)
                f = [];
            end
        end
        function Fx = get.Fx(obj)
            if isnumeric(obj.MotionModel.Fx)
                Fx = obj.MotionModel.Fx;
            end
        end
        function Fxx = get.Fxx(obj)
            if all(all(all(obj.MotionModel.Fxx==0)))
                Fxx = obj.MotionModel.Fxx;
            end
        end
        function Fw = get.Fw(obj)
            if isnumeric(obj.MotionModel.Fw)
                Fw = obj.MotionModel.Fw;
            elseif isa(obj.MotionModel.Fw, 'function_handle')
                Fw = @kalman_Fw;
            end
        end
        function Q = get.Q(obj)
            if isnumeric(obj.MotionModel.Q)
                Q = obj.MotionModel.Q;
            elseif isa(obj.MotionModel.Q, 'function_handle')
                Q = @kalman_Q;
            end
        end
        %
        function h = get.h(obj)
            if isempty(obj.MeasurementModel.h)
                h = [];
            end
        end
        function Hx = get.Hx(obj)
            if isnumeric(obj.MeasurementModel.Hx)
                Hx = obj.MeasurementModel.Hx;
            end
        end
        function Hxx = get.Hxx(obj)
            if all(all(all(obj.MeasurementModel.Hxx)))
                Hxx = obj.MeasurementModel.Hxx;
            end
        end
        function Hv = get.Hv(obj)
            if isnumeric(obj.MeasurementModel.Hv)
                Hv = obj.MeasurementModel.Hv;
            elseif isa(obj.MeasurementModel.Hv, 'function_handle')
                Hv = @kalman_Hv;
            end
        end
        function R = get.R(obj)
            if isnumeric(obj.MeasurementModel.R)
                R = obj.MeasurementModel.R;
            elseif isa(obj.MeasurementModel.R, 'function_handle')
                R = @kalman_R;
            end
        end
        
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
        function Fw = kalman_Fw(obj, xcurrent)
            Fw = feval(obj.MotionModel.Fw, obj.MotionModel, xcurrent);
        end
        function Q = kalman_Q(obj, xcurrent)
            Q = feval(obj.MotionModel.Q, obj.MotionModel, xcurrent);
        end
        %%%
        function Hv = kalman_Hv(obj, xpredict)
            Hv = feval(obj.MeasurementModel.Hv, obj.MeasurementModel, xpredict);
        end
        function R = kalman_R(obj, xpredict)
            R = feval(obj.MeasurementModel.R, obj.MeasurementModel, xpredict);
        end
    end
    
    methods
        %%%
        [XhatPre, PhatPre, Stil, K, Ztil] = predict (obj, Xinit,   Pinit,   Z);           % one step prediction
        [Xhat,    Phat,    Stil, K, Ztil] = update  (obj, XhatPre, PhatPre, Z);           % one step update
        [Xhat,    Phat,    Stil, K, Ztil] = filter  (obj, Xinit,   Pinit,   Z, flag);     % filtering
        [Xhat,    Phat]                   = mcfilter(obj, Xinit,   Pinit,   Z, flag);     % Monte Carlo filtering
    end
    
end