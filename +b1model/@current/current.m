classdef current
    %CURRENT statiscal current motion model
    % Hd = B1MODEL.CURRENT(Alpha, T, AmaxPlus, AmaxMinus) construct a 
    % current statistical motion model object Hd, with reciprocal of maneuver time 
    % constant Alpha, maximum acceleration AmaxPlus, maximum minus acceleration AmaxMinus, 
    % and sampling interval T. 
    %
    % % EXAMPLE 
    % Alpha = 0.1;
    % T = 1;
    % AmaxPlus = 100;
    % AmaxMinus = -50;
    % Hd = b1model.singer(Alpha, T, AmaxPlus, Amaxminus)
    
    properties (Constant=true)
        ModelType = 'statistical current';
    end
    properties
        Alpha = 0.1;            % reciprocal of maneuver time constant
        T = 1;                  % sampling interval
        AmaxPlus = 100;         % maximum acceleration
        AmaxMinus = -50         % maximum minus acceleration
    end
    
    properties(Constant=true)
        f = @current_f;
    end
    properties(Dependent=true)
        Fx                              % state transferring matrix
    end
    properties(Constant=true)
        Fxx = zeros(3,3,3);
    end
    properties(Constant=true)
        Fw = eye(3);
    end
    properties(Constant=true)
        Q = @current_Q;
    end

    properties(Constant=true)
        StateSym = [ sym('position','real');
                     sym('velocity','real');
                     sym('acceleration','real')]; % state symbolic
    end
   
    methods        
        % constructor
        function obj = current(Alpha, T, AmaxPlus, AmaxMinus)
            switch nargin
                case 1
                    obj.Alpha = Alpha;
                case 2
                    obj.Alpha = Alpha;
                    obj.T = T;
                case 3
                    obj.Alpha = Alpha;
                    obj.T = T;
                    obj.AmaxPlus = AmaxPlus;
                case 4
                    obj.Alpha = Alpha;
                    obj.T = T;
                    obj.AmaxPlus = AmaxPlus;
                    obj.AmaxMinus = AmaxMinus;
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
        
        function obj = set.AmaxPlus(obj, AmaxPlus)
            if numel(AmaxPlus)~=1 || AmaxPlus<=0
                error('set third input argument error')
            else
                obj.AmaxPlus = AmaxPlus;
            end
        end
        
        function obj = set.AmaxMinus(obj, AmaxMinus)
            if numel(AmaxMinus)~=1 || AmaxMinus>=0
                error('set fourth input argument error')
            else
                obj.AmaxMinus = AmaxMinus;
            end
        end

        %%% get depedent properties functions
        function Fx = get.Fx(obj)
            Fx = [1, obj.T, (obj.Alpha*obj.T-1+exp(-obj.Alpha*obj.T))/obj.Alpha^2;
                  0, 1,     (1-exp(-obj.Alpha*obj.T))/obj.Alpha;
                  0, 0,     exp(-obj.Alpha*obj.T)];
        end
    end
    
    methods (Access = private)
        function xnext = current_f(obj, xcurrent)
            Fx = [1, obj.T, obj.T^2/2; 
                  0, 1,     obj.T; 
                  0, 0,     1];
            xnext = Fx*xcurrent;
        end
        
        function Q = current_Q(obj, xcurrent)
            acc = xcurrent(3);
            var1 = obj.Alpha*obj.T;
            var2 = exp(-var1);
            q11 = (1-var2^2+2*var1+2/3*var1^3-2*var1^2-4*var1*var2)/2/obj.Alpha^5;
            q12 = (var2^2+1-2*var2+2*var1*var2-2*var1+var1^2)/2/obj.Alpha^4;
            q13 = (1-var2^2-2*var1*var2)/2/obj.Alpha^3;
            q22 = (4*var2-3-var2^2+2*var1)/2/obj.Alpha^3;
            q23 = (var2^2+1-2*var2)/2/obj.Alpha^2;
            q33 = (1-var2^2)/2/obj.Alpha;
            Q3 = [q11, q12, q13; q12, q22, q23; q13, q23, q33];
            if acc>0
                SigmaSquare = (4-pi)/pi*(obj.AmaxPlus-acc)^2;
            else
                SigmaSquare = (4-pi)/pi*(obj.AmaxMinus+acc)^2;
            end
            Sw = 2*obj.Alpha*SigmaSquare;
            Q = Sw*Q3;
        end
    end
    % end methods
end


