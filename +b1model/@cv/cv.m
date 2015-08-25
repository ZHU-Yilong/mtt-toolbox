classdef cv    
    %CV constant velocity motion model
    % Hd = B1MODEL.CV(Sw,T) construct a constant velocity motion model
    % object Hd, with power spectral denstiy Sw, and sampling interval T. 
    %
    % % EXAMPLE 
    % Sw = 1;
    % T = 1;
    % Hd = b1model.cv(Sw, T)
    
    properties (Constant=true)
        ModelType = 'Constant Velocity (CV)';
    end
    properties
        Sw = 1;                        % power spectral density in continous-time
        T = 1;                         % sampling interval
    end
    
    properties(Constant=true)
        f = [];
    end
    properties(Dependent=true)
        Fx                             % state transferring matrix 
    end    
    properties(Constant=true)
        Fxx = zeros(2,2,2);
    end    
    properties(Constant=true)
        Fw = eye(2);
    end
    properties(Dependent=true)
        Q                               % covariance matrix of process noise
    end
    
    properties(Constant=true)
        StateSym = [sym('position','real');
                    sym('velocity','real')];
    end
   
    methods
        
        % constructor
        function obj = cv(Sw, T)
            switch nargin
                case 1
                    obj.Sw = Sw;
                case 2
                    obj.Sw = Sw;
                    obj.T = T;
                otherwise
            end
        end
        
        %%% set property functions
        function obj = set.Sw(obj, Sw)
            if numel(Sw)~=1 || Sw<=0
                error('set first input argument error')
            else
                obj.Sw = Sw;
            end
        end
        
        function obj = set.T(obj, T)
            if numel(T)~=1 || T<=0
                error('set second input argument error')
            else
                obj.T = T;
            end
        end

        % get depedent property functions
        function Fx = get.Fx(obj)
            Fx = [1, obj.T; 0, 1];
        end
        
        function Q = get.Q(obj)
            Q2 = [obj.T^4/3, obj.T^3/2; obj.T^3/2, obj.T^2];
            Q = obj.Sw/obj.T*Q2;
        end
        %
    end
    % end methods
end


