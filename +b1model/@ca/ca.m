classdef ca 
    %CA constant acceleration motion model
    % Hd = B1MODEL.CA(Sw,T) construct a constant acceleration motion model
    % object Hd, with power spectral denstiy Sw, and sampling interval T. 
    %
    % % EXAMPLE 
    % Sw = 1;
    % T = 1;
    % Hd = b1model.ca(Sw, T)
    
    properties (Constant=true)
        ModelType = 'Constant Acceleration (CA)';
    end
    properties
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
        Fxx = zeros(3,3,3);
    end  
    properties(Constant=true)
        Fw = eye(3);
    end
    properties(Dependent=true)
        Q                               % covariance matrix of process noise
    end
    properties(Constant=true)
        StateSym = [ sym('position','real');
                     sym('velocity','real');
                     sym('acceleration','real')];
    end
   
    methods
        
        % constructor
        function obj = ca(Sw, T)
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

        % get dependent property functions
        function Fx = get.Fx(obj)
            Fx = [1, obj.T, obj.T^2/2; 
                  0, 1,     obj.T; 
                  0, 0,     1];
        end
        
        function Q = get.Q(obj)
            Q3 = [obj.T^5/20, obj.T^4/8, obj.T^3/6; 
                  obj.T^4/8, obj.T^3/3, obj.T^2/2; 
                  obj.T^3/6, obj.T^2/2, obj.T];
            Q = obj.Sw*Q3;
        end
        
    end
    % end methods
end


