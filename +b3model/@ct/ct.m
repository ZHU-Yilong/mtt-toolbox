classdef ct
    %CT constant velocity motion model
    % HD = B3MODEL.CT(Omega, Sw, T) construct a constant turn motion model
    % object Hd, with turn rate Omega, power spectral denstiy Sw, and 
    % sampling interval T. 
    %
    % % EXAMPLE 
    % Omega = deg2rad(10);
    % Sw = [1,1,1];
    % T = 1;
    % Hd = b3model.ct(Omega, Sw, T)
    
    properties (Constant=true)
        ModelType = 'Constant Turn (CT)';
    end
    properties
        Omega = deg2rad(10);           % turn rate
        Sw = [1, 1, 1];                % power spectral density in continous-time
        T = 1;                         % sampling interval
    end
    
    properties(Constant=true)
        f = [];
    end
    properties(Dependent=true)
        Fx                               % state transferring matrix
    end
    properties(Constant=true)
        Fxx = zeros(9,9,9);
    end
    properties(Constant=true)
        Fw = eye(9);
    end
    properties(Dependent=true)
        Q                               % covariance matrix of process noise
    end
    
    properties(Constant=true)
        StateSym = [ sym('x_position','real');
                     sym('x_velocity','real');
                     sym('x_acceleration','real');
                     sym('y_position','real');
                     sym('y_velocity','real');
                     sym('y_acceleration','real')
                     sym('z_position','real');
                     sym('z_velocity','real');
                     sym('z_acceleration','real')]; % state symbolic
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
            if numel(Sw)~=3 || any(Sw<=0)
                error('set second input argument error')
            else
                obj.Sw = [Sw(1);Sw(2);Sw(3)];
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
            MatrixF = [1, varsin/obj.Omega,  (1-varcos^2)/obj.Omega^2;
                       0, varcos,            varsin/obj.Omega;
                       0, -obj.Omega*varsin, varcos];
            Fx = blkdiag(MatrixF, MatrixF, MatrixF);
        end

        function Q = get.Q(obj)
            var = obj.Omega*obj.T;
            varsin = sin(obj.Omega*obj.T);
            varsin2 = sin(2*obj.Omega*obj.T);
            varsin5 = sin(obj.Omega*obj.T/2);
            q11 = (6*var-8*varsin+varsin2)/4/obj.Omega^5;
            q12 = 2*varsin5^4/obj.Omega^4;
            q13 = (-2*var+4*varsin-varsin2)/4/obj.Omega^3;
            q22 = (2*var-varsin2)/4/obj.Omega^3;
            q23 = varsin^2/2/obj.Omega^2;
            q33 = (2*var+varsin2)/4/obj.Omega;
            Q3 = [q11, q12, q13;
                  q12, q22, q23;
                  q13, q23, q33];
            Q = blkdiag(obj.Sw(1)*Q3,obj.Sw(2)*Q3,obj.Sw(3)*Q3);
        end

    end
    % end methods
end


