classdef dxdydz
    % MTNMODEL.DXDYDZ three-dimensional motion model
    % HD = MTNMODEL.DXDYDZ(b1model.**,b1model.##,b1model.$$) 
    % construct a three-dimensional motion model object, 
    % with three basic models **, ##, and $$.
    % The basic model can be any model object handle within 'b1model' package.
    %
    % % EXAMPLE
    % Swx = 1; Tx = 1;
    % Swy = 1; Ty = 1;
    % Swz = 1; Tz = 1;
    % hdx = b1model.cv(Swx,Tx);
    % hdy = b1model.cv(Swy,Ty);
    % hdz = b1model.cv(Swz,Tz);
    % hd = mtnmodel.dxdydz(hdx,hdy,hdz)
    % 
    
    properties(Constant=true)
        Dimension = 3;
        Description = 'three-dimensional motion model with three 1D basic models';
    end
    properties
       ModelX = b1model.cv;
       ModelY = b1model.cv;
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
        function obj = dxdydz(ModelX, ModelY, ModelZ)
            switch nargin
                case 1
                    obj.ModelX = ModelX;
                case 2
                    obj.ModelX = ModelX;
                    obj.ModelY = ModelY;
                case 3
                    obj.ModelX = ModelX;
                    obj.ModelY = ModelY;
                    obj.ModelZ = ModelZ;
                otherwise
            end
        end
        
        %%% set properties functions 
        function obj = set.ModelX(obj, ModelX)
            if (~strncmp(class(ModelX),'b1model',length('b1model')))
                error('first input argument must be class handle within b1model package')
            else
                obj.ModelX = ModelX;
            end
        end
        
        function obj = set.ModelY(obj, ModelY)
            if (~strncmp(class(ModelY),'b1model',length('b1model')))
                error('second input argument must be class handle within b1model package')
            else
                obj.ModelY = ModelY;
            end
        end
        
        function obj = set.ModelZ(obj, ModelZ)
            if (~strncmp(class(ModelZ),'b1model',length('b1model')))
                error('second input argument must be class handle within b1model package')
            else
                obj.ModelZ = ModelZ;
            end
        end
        
        %%% get properties functions
        function f = get.f(obj)
            if isempty(obj.ModelX.f) && isempty(obj.ModelY.f) && isempty(obj.ModelZ.f)
                f = [];
            else
                f = @dxdydz_f;
            end
        end
        
        function Fx = get.Fx(obj)
            if isnumeric(obj.ModelX.Fx) && isnumeric(obj.ModelY.Fx) && isnumeric(obj.ModelZ.Fx)
                Fx = blkdiag(obj.ModelX.Fx,obj.ModelY.Fx,obj.ModelZ.Fx);
            else
                Fx = @dxdydz_Fx;
            end
        end
        
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.ModelX.Fxx) && isnumeric(obj.ModelY.Fxx) && isnumeric(obj.ModelZ.Fxx)
                Fxx = n3blkdiag(obj.ModelX.Fxx,obj.ModelY.Fxx,obj.ModelZ.Fxx);
            else
                Fxx = @dxdydz_Fxx;
            end
        end
        
        function Fw = get.Fw(obj)
            if isnumeric(obj.ModelX.Fw) && isnumeric(obj.ModelY.Fw) && isnumeric(obj.ModelZ.Fw)
                Fw = blkdiag(obj.ModelX.Fw,obj.ModelY.Fw,obj.ModelZ.Fw);
            else
                Fw = @dxdydz_Fw;
            end
        end
        
        function Q = get.Q(obj)
            if isnumeric(obj.ModelX.Q) && isnumeric(obj.ModelY.Q) && isnumeric(obj.ModelZ.Q)
                Q = blkdiag(obj.ModelX.Q,obj.ModelY.Q,obj.ModelZ.Q);
            else
                Q = @dxdydz_Q;
            end
        end
        
        function StateSym = get.StateSym(obj)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            StateSym = sym('StateSym',[dimx+dimy+dimz,1]);
            for kk = 1:1:dimx
                StateSym(kk) = sym(['x_',char(obj.ModelX.StateSym(kk))]);
            end
            for kk = 1:1:dimy
                StateSym(dimx+kk) = sym(['y_',char(obj.ModelY.StateSym(kk))]);
            end
            for kk = 1:1:dimz
                StateSym(dimx+dimy+kk) = sym(['z_',char(obj.ModelZ.StateSym(kk))]);
            end
        end
    end
    % end public methods
    methods (Access = private)
        function xnext = dxdydz_f(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isempty(obj.ModelX.f)
                fX = obj.ModelX.Fx*xcurrent(1:dimx);
            elseif isa(obj.ModelX.f,'function_handle')
                fX = feval(obj.ModelX.f, obj.ModelX, xcurrent(1:dimx));
            end
            if isempty(obj.ModelY.f)
                fY = obj.ModelY.Fx*current(dimx+1:dimx+dimy);
            elseif isa(obj.ModelY.f,'function_handle')
                fY = feval(obj.ModelY.f, obj.ModelY, xcurrent(dimx+1:dimx+dimy));
            end
            if isempty(obj.ModelZ.f)
                fZ = obj.ModelZ.Fx*current(dimx+dimy+1:dimx+dimy+dimz);
            elseif isa(obj.ModelZ.f,'function_handle')
                fZ = feval(obj.ModelZ.f, obj.ModelZ, xcurrent(dimx+dimy+1:dimx+dimy+dimz));
            end
            xnext = [fX;fY;fZ];
        end
        
        function Fx = dxdydz_Fx(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelX.Fx)
                FxX = obj.ModelX.Fx;
            elseif isa(obj.ModelX.Fx,'function_handle')
                FxX = feval(obj.ModelX.Fx, obj.ModelX, xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Fx)
                FxY = obj.ModelY.Fx;
            elseif isa(obj.ModelY.Fx,'function_handle')
                FxY = feval(obj.ModelY.Fx, obj.ModelY, xcurrent(dimx+1:dimx+dimy));
            end
            if isnumeric(obj.ModelZ.Fx)
                FxZ = obj.ModelZ.Fx;
            elseif isa(obj.ModelZ.Fx,'function_handle')
                FxZ = feval(obj.ModelZ.Fx, obj.ModelZ, xcurrent(dimx+dimy+1:dimx+dimy+dimz));
            end
            Fx = blkdiag(FxX,FxY,FxZ);
        end
        
        function Fxx = dxdydz_Fxx(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelX.Fxx)
                FxxX = obj.ModelX.Fxx;
            elseif isa(obj.ModelX.Fxx,'function_handle')
                FxxX = feval(obj.ModelX.Fxx, obj.ModelX, xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Fxx)
                FxxY = obj.ModelY.Fxx;
            elseif isa(obj.ModelY.Fxx,'function_handle')
                FxxY = feval(obj.ModelY.Fxx, obj.ModelY, xcurrent(dimx+1:dimx+dimy));
            end
            if isnumeric(obj.ModelZ.Fxx)
                FxxZ = obj.ModelZ.Fxx;
            elseif isa(obj.ModelZ.Fxx,'function_handle')
                FxxZ = feval(obj.ModelZ.Fxx, obj.ModelZ, xcurrent(dimx+dimy+1:dimx+dimy+dimz));
            end
            Fxx = n3blkdiag(FxxX,FxxY,FxxZ);
        end
        
        function Fw = dxdydz_Fw(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelX.Fw)
                FwX = obj.ModelX.Fw;
            elseif isa(obj.ModelX.Fw,'function_handle')
                FwX = feval(obj.ModelX.Fw, obj.ModelX, xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Fw)
                FwY = obj.ModelY.Fw;
            elseif isa(obj.ModelY.Fw,'function_handle')
                FwY = feval(obj.ModelY.Fw, obj.ModelY, xcurrent(dimx+1:dimx+dimy));
            end
            if isnumeric(obj.ModelZ.Fw)
                FwZ = obj.ModelZ.Fw;
            elseif isa(obj.ModelZ.Fw,'function_handle')
                FwZ = feval(obj.ModelZ.Fw, obj.ModelZ, xcurrent(dimx+dimy+1:dimx+dimy+dimz));
            end
            Fw = blkdiag(FwX,FwY,FwZ);
        end
        
        function Q = dxdydz_Q(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            dimz = length(obj.ModelZ.StateSym);
            if isnumeric(obj.ModelX.Q)
                QX = obj.ModelX.Q;
            elseif isa(obj.ModelX.Q,'function_handle')
                QX = feval(obj.ModelX.Q, obj.ModelX, xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Q)
                QY = obj.ModelY.Q;
            elseif isa(obj.ModelY.Q,'function_handle')
                QY = feval(obj.ModelY.Q, obj.ModelY, xcurrent(dimx+1:dimx+dimy));
            end
            if isnumeric(obj.ModelZ.Q)
                QZ = obj.ModelZ.Q;
            elseif isa(obj.ModelZ.Q,'function_handle')
                QZ = feval(obj.ModelZ.Q, obj.ModelZ, xcurrent(dimx+dimy+1:dimx+dimy+dimz));
            end
            Q = blkdiag(QX,QY,QZ);
        end
    end
    % end private methods
end