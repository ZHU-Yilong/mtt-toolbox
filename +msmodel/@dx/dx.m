classdef dx
    % MSMODEL.DX one dimensional measurement model 
    % 
    
    properties(Constant=true)
        Dimension = 1;
        Description = 'one-dimensional Cartesian coordinate system measurement model';
    end
    properties(Constant=true)
        h = [];
    end
    properties(Dependent=true)
        Hx;
        Hxx;
    end
    properties(Constant=true)
        Hv = eye(1);
    end
    properties
        R = 1;
    end
    properties(Constant=true)
        MeasureSym = sym('position','real');
    end
    properties
        StateSym = [ sym('position','real');
                     sym('velocity','real')];
    end
    properties(Constant=true,Hidden=true)
        HSym = sym('position','real');
    end
    
    %%% methods
    methods
        %%% constructor
        function obj = dx(R, StateSym)
            switch nargin
                case 1
                    obj.R = R;
                case 2
                    obj.R = R;
                    obj.StateSym = StateSym;
                otherwise
            end
        end
        
        %%% set property functions 
        function obj = set.R(obj,R)
            if isnumeric(R)
                if size(R,1)~=1 || size(R,2)~=1
                    error('measurement noise covariance must contain only one element')
                elseif R(1,1)<0 
                    error('the element of measurement noise covariance must be positive')
                else
                    obj.R = R;
                end
            elseif isa(R, 'function_handle')
                obj.R = R;
            end
        end
        
        function obj = set.StateSym(obj, StateSym)
            if ~iscolumn(StateSym) || ~isa(StateSym,'sym')
                error('state varabile must be a column vector and symbolic')
            else
                obj.StateSym = StateSym;
            end
        end
        
        %%% get dependent property functions
        function Hx = get.Hx(obj)
            Hx = double(jacobian(obj.HSym,obj.StateSym));
        end
        
        function Hxx = get.Hxx(obj)
            DimState = length(obj.StateSym);
            DimMeasure = length(obj.MeasureSym);
            Hxx = zeros([DimState,DimState,DimMeasure]);
        end
        
    end
end

