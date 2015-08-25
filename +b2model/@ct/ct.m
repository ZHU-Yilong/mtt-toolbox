classdef ct
    %CT constant turn motion model
    % Hd = B2MODEL.CT(Omega, Sw,T) construct a constant turn motion model
    % object Hd, with turn rate Omega, power spectral denstiy Sw, and 
    % sampling interval T. 
    %
    % % EXAMPLE 
    % Omega = deg2rad(10);
    % Sw = 1;
    % T = 1;
    % Hd = b2model.ct(Omega, Sw, T)
    
    properties (Constant=true)
        ModelType = 'Constant Turn (CT)';
    end
    properties
        Omega = deg2rad(10);           % turn rate
        Sw = 1;                        % power spectral density in continous-time
        T = 1;                         % sampling interval
    end
    
    properties(Constant=true)
        f = [];
    end
    properties(Dependent=true)
        Fx                               % state transferring matrix
    end
    properties(Constant=true)
        Fxx = zeros(4,4,4);
    end
    properties(Constant=true)
        Fw = eye(4);
    end
    properties(Dependent=true)
        Q                               % covariance matrix of process noise
    end
    properties(Constant=true)
        StateSym = [ sym('x_position','real');
                     sym('x_velocity','real');
                     sym('y_position','real');
                     sym('y_velocity','real')]; % state symbolic
    end
   
    methods
        
        % constructor
        function obj = ct(Omega, Sw, T)
            switch nargin
                case 1
                    obj.Omega = Omega;
                case 2
                    obj.Omega = Omega;
                    obj.Sw = Sw;
                case 3
                    obj.Omega = Omega;
                    obj.Sw = Sw;
                    obj.T = T;
                otherwise
            end
        end
        
        % set property functions
        function obj = set.Omega(obj, Omega)
            if numel(Omega)~=1 || Omega==0
                error('set first input argument error')
            else
                obj.Omega = Omega;
            end
        end
        
        function obj = set.Sw(obj, Sw)
            if numel(Sw)~=1 || Sw<=0
                error('set second input argument error')
            else
                obj.Sw = Sw;
            end
        end
        
        function obj = set.T(obj, T)
            if numel(T)~=1 || T<=0
                error('set third input argument error')
            else
                obj.T = T;
            end
        end

        % get property functions
        function Fx = get.Fx(obj)
            varsin = sin(obj.Omega*obj.T);
            varcos = cos(obj.Omega*obj.T);
            Fx = [1, varsin/obj.Omega, 0, -(1-varcos)/obj.Omega; 
                  0, varcos, 0, -varsin;
                  0, (1-varcos)/obj.Omega, 1, varsin/obj.Omega;
                  0, varsin, 0, varcos];
        end
        
        function Q = get.Q(obj)
            var = obj.Omega*obj.T;
            varsin = sin(obj.Omega*obj.T);
            varcos = cos(obj.Omega*obj.T);
            q11 = 2*(var-varsin)/obj.Omega^3;
            q12 = (1-varcos)/obj.Omega^2;
            q14 = (var-varsin)/obj.Omega^2;
            q23 = -q14;
            Q = obj.Sw*[q11, q12,   0,   q14;
                        q12, obj.T, q23, 0;
                        0,   q23,   q11, q12;
                        q14, 0,     q12, obj.T];
        end

    end
    % end methods
end


