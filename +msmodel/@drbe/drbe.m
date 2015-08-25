classdef drbe
    % MSMODEL.DRBE three dimensional spherical measurement model 
    % HD = MSMODEL.DRBE(R,StateSym) 
    % construct a measurement model object
    % with range, bear, and elevation.
    % R is measurement noise covariance,
    % and StateSym is the symbolic variable of state
    %
    % EXAMPLE
    % R = diag([1, 0.01, 0.01]);
    % StateSym = [ sym('x_position','real');
    %              sym('x_velocity','real');
    %              sym('y_position','real');
    %              sym('y_velocity','real');
    %              sym('z_position','real');
    %              sym('z_velocity','real')];
    % hd = msmodel.drbe(R,StateSym)
    
    properties(Constant=true)
        Dimension = 3;
        Description = 'three-dimensional spherical coordinate system measurement model';
    end
    properties(Constant=true)
        h   = @drb_h;
        Hx  = @drb_Hx;
        Hxx = @drb_Hxx;
    end
    properties(Constant=true)
        Hv = eye(3);
    end
    properties
        R = diag([1, 0.01, 0.01]);
    end
    properties(Constant=true)
        MeasureSym = [ sym('range',     'real');
                       sym('bear',      'real');
                       sym('elevation', 'real')];
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
        HSym = [ sym('sqrt(x_position^2+y_position^2+z_position^2)');
                 sym('arg(x_position+y_position*i)');
                 sym('arg(sqrt(x_position^2+y_position^2)+z_position*i)')];
    end    
    %%% methods
    methods
        %%% constructor
        function obj = drbe(R,StateSym)
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
    end
    %  private methods
    methods (Access = private)
        function zpredict = drb_h(obj,xpredict)
            zpredict = double(subs(obj.HSym,obj.StateSym,xpredict));
        end
        %
        function Hx = drb_Hx(obj,xpredict)
            HxSym = jacobian(obj.HSym,obj.StateSym);
            Hx = double(subs(HxSym,obj.StateSym,xpredict));
        end
        %
        function Hxx = drb_Hxx(obj,xpredict)
            DimState = length(obj.StateSym);
            DimMeasure = length(obj.MeasureSym);
            HxxSym = sym(zeros([DimState,DimState,DimMeasure]));
            for kk = 1:1:DimMeasure
                HxxSym(:,:,kk) = hessian(obj.HSym,obj.StateSym);
            end
            Hxx = double(subs(HxxSym,obj.StateSym,xpredict));
        end
    end
end

