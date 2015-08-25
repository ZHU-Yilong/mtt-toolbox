classdef dxy
    % MTNMODEL.DXY two dimensional motion model
    % HD = MTNMODEL.DXY(b2model.**) 
    % construct a two-dimensional motion model object, 
    % with a basic model **. 
    % The basic model can be any model object handle within 'b2model' package.
    %
    % % EXAMPLE
    % Omega = deg2rad(10);
    % Sw = 1;
    % T = 1;
    % hd = mtnmodel.dxy(b2model.ct(Omega,Sw,T))
    % 
    
    properties(Constant=true)
        Dimension = 2;
        Description = 'two-dimensional motion model with one 2D basic model';
    end
    properties
       ModelXY = b2model.ct;
    end
    properties(Dependent=true,SetAccess=private)
        f
        Fx
        Fxx
        Fw
        Q
        StateSym
    end
    
    %%% methods
    methods
        %%% constructor
        function obj = dxy(ModelXY)
            switch nargin
                case 1
                    obj.ModelXY = ModelXY;
                otherwise
            end
        end
        
        %%% set property functions 
        function obj = set.ModelXY(obj, ModelXY)
            if (~strncmp(class(ModelXY),'b2model',length('b2model')))
                error('input argument must be class handle within b2model package')
            else
                obj.ModelXY = ModelXY;
            end
        end
        
        %%% get dependent property functions
        function f = get.f(obj)
            if isempty(obj.ModelXY.f)
                f = [];
            elseif isa(obj.ModelXY.f, 'function_handle')
                f = @dxy_f;
            end
        end
        function Fx = get.Fx(obj)
            if isnumeric(obj.ModelXY.Fx)
                Fx = obj.ModelXY.Fx;
            elseif isa(obj.ModelXY.Fx, 'function_handle')
                Fx = @dxy_Fx;
            end
        end
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.ModelXY.Fxx)
                Fxx = obj.ModelXY.Fxx;
            elseif isa(obj.ModelXY.Fxx, 'function_handle')
                Fxx = @dxy_Fxx;
            end
        end

        function Fw = get.Fw(obj)
            if isnumeric(obj.ModelXY.Fw)
                Fw = obj.ModelXY.Fw;
            elseif isa(obj.ModelXY.Fw, 'function_handle')
                Fw = @dxy_Fw;
            end
        end
        function Q = get.Q(obj)
            if isnumeric(obj.ModelXY.Q)
                Q = obj.ModelXY.Q;
            elseif isa(obj.ModelXY.Q, 'function_handle')
                Q = @dxy_Q;
            end
        end

        function StateSym = get.StateSym(obj)
            StateSym = obj.ModelXY.StateSym;
        end
    end
    %%%
    methods (Access = private)
        function xnext = dxy_f(obj, xcurrent)
            xnext = feval(obj.ModelXY.f, obj.ModelXY, xcurrent);
        end
        function Fx = dxy_Fx(obj, xcurrent)
            Fx = feval(obj.ModelXY.Fx, obj.ModelXY, xcurrent);
        end
        function Fxx = dxy_Fxx(obj, xcurrent)
            Fxx = feval(obj.ModelXY.Fxx, obj.ModelXY, xcurrent);
        end
        function Fw = dxy_Fw(obj, xcurrent)
            Fw = feval(obj.ModelXY.Fw, obj.ModelXY, xcurrent);
        end
        function Q = dxy_Q(obj, xcurrent)
            Q = feval(obj.ModelXY.Q, obj.ModelXY, xcurrent);
        end
    end
end

