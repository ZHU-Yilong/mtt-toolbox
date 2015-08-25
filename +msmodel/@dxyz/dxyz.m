classdef dxyz
    % MSMODEL.DXYZ three dimensional linear measurement model 
    % 
    
    properties(Constant=true)
        Dimension = 3;
        Description = 'three-dimensional Cartesian coordinate system measurement model';
    end
    properties(Constant=true)
        h = [];
    end
    properties(Dependent=true)
        Hx;
        Hxx;
    end
    properties(Constant=true)
        Hv = eye(3);
    end
    properties
        R = diag([1, 1, 1]);
    end
    properties(Constant=true)
        MeasureSym = [ sym('x_position', 'real');
                       sym('y_position', 'real');
                       sym('z_position', 'real') ];
    end
    properties
        StateSym = [ sym('x_position',  'real');
                     sym('x_velocity',  'real');
                     sym('y_position',  'real');
                     sym('y_velocity',  'real');
                     sym('z_position',  'real');
                     sym('z_velocity',  'real')];
    end
    properties(Constant=true,Hidden=true)
        HSym = [ sym('x_position', 'real');
                 sym('y_position', 'real');
                 sym('z_position', 'real') ];
    end
    
    %%% methods
    methods
        %%% constructor
        function obj = dxy(R, StateSym)
            switch nargin
                case 1
                    obj.R = R;
                case 2
                    obj.R = R;
                    obj.StateSym = StateSym;
                otherwise
            end
        end
        
        %%% set properties functions 
        function obj = set.R(obj, R)
            if isnumeric(R)
                if size(R,1)~=3 || size(R,2)~=3
                    error('measurement noise covariance must be a 3*3 matrix')
                elseif R(1,1)<0 || R(2,2)<0 || R(3,3)<0
                    error('all diagonal elements of measurement noise covariance must be positive')
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

