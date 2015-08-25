classdef dxydz
    % MTNMODEL.DXYDZ three dimensional motion model
    % HD = MTNMODEL.DXYDZ(b2model.**,b1model.##) 
    % construct a three-dimensional motion model object, 
    % with two basic models ** and ##. 
    % The basic model ** can be any model object handle within 'b2model'
    % package, and the basic model ## can be any model object handle within
    % 'b1model' package.
    %
    % % EXAMPLE
    % Omega = deg2rad(10);
    % Swxy = 1; Txy = 1;
    % Swz = 1; Tz = 1;
    % hdxy = b2model.ct(Omega,Swxy,Txy);
    % hdz = b1model.cv(Swz,Tz);
    % hd = mtnmodel.dxydz(hdxy,hdz)
    % 
    
    properties(Constant=true)
        Dimension = 3;
        Description = 'three-dimensional motion mode with one 2D model and one 1D basic models';
    end
    properties
       ModelXY = b2model.ct;
       ModelZ = b1model.cv;
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
        function obj = dxydz(ModelXY, ModelZ)
            switch nargin
                case 1
                    obj.ModelXY = ModelXY;
                case 2
                    obj.ModelXY = ModelXY;
                    obj.ModelZ = ModelZ;
                otherwise
            end
        end
        
        %%% set property functions 
        function obj = set.ModelXY(obj, ModelXY)
            if (~strncmp(class(ModelXY),'b2model',length('b2model')))
                error('input argument must be class handle within b2model package')
            else
                obj.ModelXY = ModelXY;
            end
        end
        
        function obj = set.ModelZ(obj, ModelZ)
            if (~strncmp(class(ModelZ),'b1model',length('b1model')))
                error('second input argument must be class handle within b1model package')
            else
                obj.ModelZ = ModelZ;
            end
        end
        
        %%% get dependent property functions
        function f = get.f(obj)
            if isempty(obj.ModelXY.f) && isempty(obj.ModelZ.f)
                f = [];
            else
                f = @dxydz_f;
            end
        end
        
        function Fx = get.Fx(obj)
            if isnumeric(obj.ModelXY.Fx) && isnumeric(obj.ModelZ.Fx)
                Fx = blkdiag(obj.ModelXY.Fx,obj.ModelZ.Fx);
            else
                Fx = @dxydz_Fx;
            end
        end
        
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.ModelXY.Fxx) && isnumeric(obj.ModelZ.Fxx)
                Fxx = n3blkdiag(obj.ModelXY.Fxx,obj.ModelZ.Fxx);
            else
                Fxx = @dxydz_Fxx;
            end
        end
        
        function Fw = get.Fw(obj)
            if isnumeric(obj.ModelXY.Fw) && isnumeric(obj.ModelZ.Fw)
                Fw = blkdiag(obj.ModelXY.Fw,obj.ModelZ.Fw);
            else
                Fw = @dxydz_Fw;
            end
        end
        
        function Q = get.Q(obj)
            if isnumeric(obj.ModelXY.Q) && isnumeric(obj.ModelZ.Q)
                Q = blkdiag(obj.ModelXY.Q,obj.ModelZ.Q);
            else
                Q = @dxydz_Q;
            end
        end

        function StateSym = get.StateSym(obj)
            dimxy = length(obj.ModelXY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            StateSym = sym('StateSym',[dimxy+dimz,1]);
            StateSym(1:1:dimxy) = obj.ModelXY.StateSym;
            for kk = 1:1:dimz
                StateSym(dimxy+kk) = sym(['z_',char(obj.ModelZ.StateSym(kk))]);
            end
        end
    end
    % end public methods
    
    methods (Access = private)
        function xnext = dxydz_f(obj, xcurrent)
            dimxy = length(obj.ModelXY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isempty(obj.ModelXY.f)
                fXY = obj.ModelXY.Fx*xcurrent(1:dimxy);
            elseif isa(obj.ModelXY.f,'function_handle')
                fXY = feval(obj.ModelXY.f,xcurrent(1:dimxy));
            end
            if isempty(obj.ModelZ.f)
                fZ = obj.ModelZ.Fx*current(dimxy+1:dimxy+dimz);
            elseif isa(obj.ModelZ.f,'function_handle')
                fZ = feval(obj.ModelZ.f,xcurrent(dimxy+1:dimxy+dimz));
            end
            xnext = [fXY;fZ];
        end

        function Fx = dxydz_Fx(obj, xcurrent)
            dimxy = length(obj.ModelXY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelXY.Fx)
                FxXY = obj.ModelXY.Fx;
            elseif isa(obj.ModelXY.Fx,'function_handle')
                FxXY = feval(obj.ModelXY.Fx,xcurrent(1:dimxy));
            end
            if isnumeric(obj.ModelZ.Fx)
                FxZ = obj.ModelZ.Fx;
            elseif isa(obj.ModelZ.Fx,'function_handle')
                FxZ = feval(obj.ModelZ.Fx,xcurrent(dimxy+1:dimxy+dimz));
            end
            Fx = blkdiag(FxXY,FxZ);
        end
        
        function Fxx = dxydz_Fxx(obj, xcurrent)
            dimxy = length(obj.ModelXY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelXY.Fxx)
                FxxXY = obj.ModelXY.Fxx;
            elseif isa(obj.ModelXY.Fxx,'function_handle')
                FxxXY = feval(obj.ModelXY.Fxx,xcurrent(1:dimxy));
            end
            if isnumeric(obj.ModelZ.Fxx)
                FxxZ = obj.ModelZ.Fxx;
            elseif isa(obj.ModelZ.Fxx,'function_handle')
                FxxZ = feval(obj.ModelZ.Fxx,xcurrent(dimxy+1:dimxy+dimz));
            end
            Fxx = n3blkdiag(FxxXY,FxxZ);
        end
        
        function Fw = dxydz_Fw(obj, xcurrent)
            dimxy = length(obj.ModelXY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelXY.Fw)
                FwXY = obj.ModelXY.Fw;
            elseif isa(obj.ModelXY.Fw,'function_handle')
                FwXY = feval(obj.ModelXY.Fw,xcurrent(1:dimxy));
            end
            if isnumeric(obj.ModelZ.Fw)
                FwZ = obj.ModelZ.Fw;
            elseif isa(obj.ModelZ.Fw,'function_handle')
                FwZ = feval(obj.ModelZ.Fw,xcurrent(dimxy+1:dimxy+dimz));
            end
            Fw = blkdiag(FwXY,FwZ);
        end
        
        function Q = dxydz_Q(obj, xcurrent)
            dimxy = length(obj.ModelXY.Q);
            dimz = length(obj.ModelZ.Q);
            if isnumeric(obj.ModelXY.Q)
                QXY = obj.ModelXY.Q;
            elseif isa(obj.ModelXY.Q,'function_handle')
                QXY = feval(obj.ModelXY.Q,xcurrent(1:dimxy));
            end
            if isnumeric(obj.ModelZ.Q)
                QZ = obj.ModelZ.Q;
            elseif isa(obj.ModelZ.Q,'function_handle')
                QZ = feval(obj.ModelZ.Q,xcurrent(dimxy+1:dimxy+dimz));
            end
            Q = blkdiag(QXY,QZ);
        end
    end
    % end private methods
end

