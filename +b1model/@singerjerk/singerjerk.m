classdef singerjerk
    %SINGERJERK singer jerk motion model
    % Hd = B1MODEL.SINGERJERK(Alpha, T, Amax, P0, Pmax) construct a 
    % singer jerk motion model object Hd, with reciprocal of maneuver time 
    % constant Alpha, maximum acceleration Amax, probability of zero 
    % acceleration P0, probability of maximum acceleration Pmax, 
    % and sampling interval T. 
    %
    % % EXAMPLE 
    % Alpha = 0.1;
    % T = 1;
    % Amax = 100;
    % P0 = 0.5;
    % Pmax = 0.1;
    % Hd = b1model.singerjerk(Alpha, T, Amax, P0, Pmax)
    
    properties (Constant=true)
        ModelType = 'Singer Jerk';
    end
    properties
        Alpha = 0.1;            % reciprocal of maneuver time constant
        T = 1;                  % sampling interval
        Amax = 100;             % maximum acceleration
        P0 = 0.5                % probability of zero acceleration
        Pmax = 0.1;             % probability of maximum acceleration Amax
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
        StateSym = [ sym('position','real');
                     sym('velocity','real');
                     sym('acceleration','real');
                     sym('jerk','real')]; % state vector
    end
   
    methods        
        % constructor
        function obj = singerjerk(Alpha, T, Amax, P0, Pmax)
            switch nargin
                case 1
                    obj.Alpha = Alpha;
                case 2
                    obj.Alpha = Alpha;
                    obj.T = T;
                case 3
                    obj.Alpha = Alpha;
                    obj.T = T;
                    obj.Amax = Amax;
                case 4
                    obj.Alpha = Alpha;
                    obj.T = T;
                    obj.Amax = Amax;
                    obj.P0 = P0;
                case 5
                    obj.Alpha = Alpha;
                    obj.T = T;
                    obj.Amax = Amax;
                    obj.P0 = P0;
                    obj.Pmax = Pmax;
                otherwise
            end
        end
        
        %%% set properties functions
        function obj = set.Alpha(obj, Alpha)
            if numel(Alpha)~=1 || Alpha<=0
                error('set first input argument error')
            else
                obj.Alpha = Alpha;
            end
        end
        
        function obj = set.T(obj, T)
            if numel(T)~=1 || T<=0
                error('set second input argument error')
            else
                obj.T = T;
            end
        end
        
        function obj = set.Amax(obj, Amax)
            if numel(Amax)~=1 || Amax==0
                error('set third input argument error')
            else
                obj.Amax = Amax;
            end
        end
        
        function obj = set.P0(obj, P0)
            if numel(P0)~=1 || P0<0 || P0>1
                error('set fourth input argument error')
            else
                obj.P0 = P0;
            end
        end
        
        function obj = set.Pmax(obj, Pmax)
            if numel(Pmax)~=1 || Pmax<0 || Pmax>1
                error('set fifth input argument error')
            else
                obj.Pmax = Pmax;
            end
        end
        
        %%% get depedent properties functions
        function Fx = get.Fx(obj)
            var1 = obj.Alpha*obj.T;
            var2 = exp(-var1);
            p = (2-2*var1+var1^2-2*var2)/2/obj.Alpha^3;
            q = (var2-1+var1)/obj.Alpha^2;
            r = (1-var2)/obj.Alpha;
            s = var2;
            Fx = [1, obj.T, obj.T^2/2, p;
                  0, 1,     obj.T,     q;
                  0, 0,     1,         r;
                  0, 0,     0,         s];
        end
        
        function Q = get.Q(obj)
            var1 = obj.Alpha*obj.T;
            var2 = exp(-var1);
            q11 = (var1^5/10-var1^4/2+4*var1^3/3-2*var1^2+2*var1-3+4*var2+2*var1^2*var2-var2^2)/2/obj.Alpha^7;
            q12 = (1-2*var1+2*var1^2-var1^3+var1^4/4+var2^2+2*var1*var2-2*var2-var1^2*var2)/2/obj.Alpha^6;
            q13 = (2*var1-var1^2+var1^3/3-3-var2^2+4*var2+var1^2*var2)/2/obj.Alpha^5;
            q14 = (1+var2^2-2*var2-var1^2*var2)/2/obj.Alpha^4;
            q22 = (1-var2^2+2*var1^3/3+2*var1-2*var1^2-4*var1*var2)/2/obj.Alpha^5;
            q23 = (1+var1^2-2*var1+2*var1*var2+var2^2-2*var2)/2/obj.Alpha^4;
            q24 = (1-var2^2-var1*var2^2)/2/obj.Alpha^3;
            q33 = (4*var2-var2^2+2*var1-3)/2/obj.Alpha^3;
            q34 = (1-2*var2+var2^2)/2/obj.Alpha^2;
            q44 = (1-var2^2)/2/obj.Alpha;
            Q4 = [q11, q12, q13, q14; 
                  q12, q22, q23, q24; 
                  q13, q23, q33, q34; 
                  q14, q24, q34, q44];
            SigmaSquare = obj.Amax^2/3*(1+4*obj.Pmax-obj.P0);
            Sw = 2*obj.Alpha*SigmaSquare;
            Q = Sw*Q4;
        end
    end
    % end methods
end


