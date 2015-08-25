classdef dx
    % MTNMODEL.DX one dimensional motion model
    % HD = MTNMODEL.DX(b1model.**) 
    % construct a one-dimensional motion model object, 
    % with a basic model **. 
    % The basic model can be any model object handle within 'b1model' package.
    %
    % % EXAMPLE
    % Sw = 1;
    % T = 1;
    % hd = mtnmodel.dx(b1model.cv(Sw,T))
    % 
    
    properties(Constant=true)
        Dimension = 1;
        Description = 'one-dimensional motion model with one 1D basic model';
    end
    properties
       ModelX = b1model.cv;
    end
    properties(Dependent=true,SetAccess=private)
        f
        Fx
        Fxx
        Fw
        Q
        StateSym
    end
    
    methods
        %%% constructor
        function obj = dx(ModelX)
            switch nargin
                case 1
                    obj.ModelX = ModelX;
                otherwise
            end
        end
        
        %%% set property functions 
        function obj = set.ModelX(obj, ModelX)
            if (~strncmp(class(ModelX),'b1model',length('b1model')))
                error('input argument must be class handle within b1model package')
            else
                obj.ModelX = ModelX;
            end
        end
        
        %%% get dependent property functions
        function f = get.f(obj)
            if isempty(obj.ModelX.f)
                f = [];
            elseif isa(obj.ModelX.f, 'function_handle')
                f = @dx_f;
            end
        end
        function Fx = get.Fx(obj)
            if isnumeric(obj.ModelX.Fx)
                Fx = obj.ModelX.Fx;
            elseif isa(obj.ModelX.Fx, 'function_handle')
                Fx = @dx_Fx;
            end
        end
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.ModelX.Fxx)
                Fxx = obj.ModelX.Fxx;
            elseif isa(obj.ModelX.Fxx, 'function_handle')
                Fxx = @dx_Fxx;
            end
        end

        function Fw = get.Fw(obj)
            if isnumeric(obj.ModelX.Fw)
                Fw = obj.ModelX.Fw;
            elseif isa(obj.ModelX.Fw, 'function_handle')
                Fw = @dx_Fw;
            end
        end
        function Q = get.Q(obj)
            if isnumeric(obj.ModelX.Q)
                Q = obj.ModelX.Q;
            elseif isa(obj.ModelX.Q, 'function_handle')
                Q = @dx_Q;
            end
        end
        function StateSym = get.StateSym(obj)
            StateSym = obj.ModelX.StateSym;
        end
    end
    
    methods (Access = private)
        function xnext = dx_f(obj, xcurrent)
            xnext = feval(obj.ModelX.f, obj.ModelX, xcurrent);
        end
        function Fx = dx_Fx(obj, xcurrent)
            Fx = feval(obj.ModelX.Fx, obj.ModelX, xcurrent);
        end
        function Fxx = dx_Fxx(obj, xcurrent)
            Fxx = feval(obj.ModelX.Fxx, obj.ModelX, xcurrent);
        end
        function Fw = dx_Fw(obj, xcurrent)
            Fw = feval(obj.ModelX.Fw, obj.ModelX, xcurrent);
        end
        function Q = dx_Q(obj, xcurrent)
            Q = feval(obj.ModelX.Q, obj.ModelX, xcurrent);
        end        
    end
end

