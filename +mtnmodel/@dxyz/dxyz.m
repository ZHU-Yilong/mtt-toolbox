classdef dxyz
    % MTNMODEL.DXDYDZ three dimensional motion model
    
    properties(Constant=true)
        Dimension = 3;
        Description = 'three-dimensional motion model with one 3D basic model';
    end
    properties
       ModelXYZ = b3model.ct;
    end
    properties(Dependent=true,SetAccess=private)
        f
        Fx
        Fxx
        Fw
        Q
        StateSym
    end
    
    %%%
    methods
        %%% constructor
        function obj = dxyz(ModelXYZ)
            switch nargin
                case 1
                    obj.ModelXYZ = ModelXYZ;
                otherwise
            end
        end
        
        %%% set property functions 
        function obj = set.ModelXYZ(obj, ModelXYZ)
            if (~strncmp(class(ModelXYZ),'b3model',length('b3model')))
                error('input argument must be class handle within b3model package')
            else
                obj.ModelXYZ = ModelXYZ;
            end
        end
        
        %%% get dependent property functions
        function f = get.f(obj)
            if isempty(obj.ModelXYZ.f)
                f = [];
            elseif isa(obj.ModelXYZ.f, 'function_handle')
                f = @dxyz_f;
            end
        end
        function Fx = get.Fx(obj)
            if isnumeric(obj.ModelXYZ.Fx)
                Fx = obj.ModelXYZ.Fx;
            elseif isa(obj.ModelXYZ.Fx, 'function_handle')
                Fx = @dxyz_Fx;
            end
        end
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.ModelXYZ.Fxx)
                Fxx = obj.ModelXYZ.Fxx;
            elseif isa(obj.ModelXYZ.Fxx, 'function_handle')
                Fxx = @dxyz_Fxx;
            end
        end

        function Fw = get.Fw(obj)
            if isnumeric(obj.ModelXYZ.Fw)
                Fw = obj.ModelXYZ.Fw;
            elseif isa(obj.ModelXYZ.Fw, 'function_handle')
                Fw = @dxyz_Fw;
            end
        end
        function Q = get.Q(obj)
            if isnumeric(obj.ModelXYZ.Q)
                Q = obj.ModelXYZ.Q;
            elseif isa(obj.ModelXYZ.Q, 'function_handle')
                Q = @dxyz_Q;
            end
        end
        
        function StateSym = get.StateSym(obj)
            StateSym = obj.ModelXYZ.StateSym;
        end
    end
    %%%
    methods (Access = private)
        function xnext = dxyz_f(obj, xcurrent)
            xnext = feval(obj.ModelXYZ.f, obj.ModelXYZ, xcurrent);
        end
        function Fx = dxyz_Fx(obj, xcurrent)
            Fx = feval(obj.ModelXYZ.Fx, obj.ModelXYZ, xcurrent);
        end
        function Fxx = dxyz_Fxx(obj, xcurrent)
            Fxx = feval(obj.ModelXYZ.Fxx, obj.ModelXYZ, xcurrent);
        end
        function Fw = dxyz_Fw(obj, xcurrent)
            Fw = feval(obj.ModelXYZ.Fw, obj.ModelXYZ, xcurrent);
        end
        function Q = dxyz_Q(obj, xcurrent)
            Q = feval(obj.ModelXYZ.Q, obj.ModelXYZ, xcurrent);
        end
    end
end

