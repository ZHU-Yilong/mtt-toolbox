classdef dxdy
    % MTNMODEL.DXDY two dimensional motion model
    % HD = MTNMODEL.DXDY(b1model.**,b1model.##) 
    % construct a two-dimensional motion model object, 
    % with two basic models ** and ##.
    % The basic model can be any model object handle within 'b1model' package.
    %
    % % EXAMPLE
    % Swx = 1; Tx = 1;
    % Swy = 1; Ty = 1;
    % hdx = b1model.cv(Swx,Tx);
    % hdy = b1model.cv(Swy,Ty);
    % hd = mtnmodel.dxdy(hdx,hdy)
    % 
    
    properties(Constant=true)
        Dimension = 2;
        Description = 'two-dimensional motion model with two 1D basic models';
    end
    properties
       ModelX = b1model.cv;
       ModelY = b1model.cv;
    end
    properties(Dependent=true,SetAccess=private)
        f
        Fx
        Fxx
        Fw
        Q
        StateSym
    end
    
    methods
        %%% constructor
        function obj = dxdy(ModelX, ModelY)
            switch nargin
                case 1
                    obj.ModelX = ModelX;
                case 2
                    obj.ModelX = ModelX;
                    obj.ModelY = ModelY;
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
        
        %%% get property functions
        function f = get.f(obj)
            if isempty(obj.ModelX.f) && isempty(obj.ModelY.f)
                f = [];
            else
                f = @dxdy_f;
            end
        end
        
        function Fx = get.Fx(obj)
            if isnumeric(obj.ModelX.Fx) && isnumeric(obj.ModelY.Fx)
                Fx = blkdiag(obj.ModelX.Fx,obj.ModelY.Fx);
            else
                Fx = @dxdy_Fx;
            end
        end
        
        function Fxx = get.Fxx(obj)
            if isnumeric(obj.ModelX.Fxx) && isnumeric(obj.ModelY.Fxx)
                Fxx = n3blkdiag(obj.ModelX.Fxx,obj.ModelY.Fxx);
            else
                Fxx = @dxdy_Fxx;
            end
        end
        
        function Fw = get.Fw(obj)
            if isnumeric(obj.ModelX.Fw) && isnumeric(obj.ModelY.Fw)
                Fw = blkdiag(obj.ModelX.Fw,obj.ModelY.Fw);
            else
                Fw = @dxdy_Fw;
            end
        end
        function Q = get.Q(obj)
            if isnumeric(obj.ModelX.Q) && isnumeric(obj.ModelY.Q)
                Q = blkdiag(obj.ModelX.Q,obj.ModelY.Q);
            else
                Q = @dxdy_Q;
            end
        end
        function StateSym = get.StateSym(obj)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            StateSym = sym('StateSym',[dimx+dimy,1]);
            for kk = 1:1:dimx
                StateSym(kk) = sym(['x_',char(obj.ModelX.StateSym(kk))]);
            end
            for kk = 1:1:dimy
                StateSym(dimx+kk) = sym(['y_',char(obj.ModelY.StateSym(kk))]);
            end
        end

    end
    
    methods (Access = private)
        function xnext = dxdy_f(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            if isempty(obj.ModelX.f)
                fX = obj.ModelX.Fx*xcurrent(1:dimx);
            elseif isa(obj.ModelX.f,'function_handle')
                fX = feval(obj.ModelX.f,xcurrent(1:dimx));
            end
            if isempty(obj.ModelY.f)
                fY = obj.ModelY.Fx*current(dimx+1:dimx+dimy);
            elseif isa(obj.ModelY.f,'function_handle')
                fY = feval(obj.ModelY.f,xcurrent(dimx+1:dimx+dimy));
            end
            xnext = [fX;fY];
        end
        
        function Fx = dxdy_Fx(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            if isnumeric(obj.ModelX.Fx)
                FxX = obj.ModelX.Fx;
            elseif isa(obj.ModelX.Fx,'function_handle')
                FxX = feval(obj.ModelX.Fx,xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Fx)
                FxY = obj.ModelY.Fx;
            elseif isa(obj.ModelY.Fx,'function_handle')
                FxY = feval(obj.ModelY.Fx,xcurrent(dimx+1:dimx+dimy));
            end
            Fx = blkdiag(FxX,FxY);
        end
        
        function Fxx = dxdy_Fxx(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            if isnumeric(obj.ModelX.Fxx)
                FxxX = obj.ModelX.Fxx;
            elseif isa(obj.ModelX.Fxx,'function_handle')
                FxxX = feval(obj.ModelX.Fxx,xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Fxx)
                FxxY = obj.ModelY.Fxx;
            elseif isa(obj.ModelY.Fxx,'function_handle')
                FxxY = feval(obj.ModelY.Fxx,xcurrent(dimx+1:dimx+dimy));
            end
            Fxx = n3blkdiag(FxxX,FxxY);
        end
        
        function Fw = dxdy_Fw(obj, xcurrent)
            dimx = length(obj.ModelX.StateSym);
            dimy = length(obj.ModelY.StateSym);
            if isnumeric(obj.ModelX.Fw)
                FwX = obj.ModelX.Fw;
            elseif isa(obj.ModelX.Fw,'function_handle')
                FwX = feval(obj.ModelX.Fw,xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Fw)
                FwY = obj.ModelY.Fw;
            elseif isa(obj.ModelY.Fw,'function_handle')
                FwY = feval(obj.ModelY.Fw,xcurrent(dimx+1:dimx+dimy));
            end
            Fw = blkdiag(FwX,FwY);
        end
        
        function Q = dxdy_Q(obj, xcurrent)
            dimx = length(obj.ModelX.Q);
            dimy = length(obj.ModelY.Q);
            if isnumeric(obj.ModelX.Q)
                QX = obj.ModelX.Q;
            elseif isa(obj.ModelX.Q,'function_handle')
                QX = feval(obj.ModelX.Q,xcurrent(1:dimx));
            end
            if isnumeric(obj.ModelY.Q)
                QY = obj.ModelY.Q;
            elseif isa(obj.ModelY.Q,'function_handle')
                QY = feval(obj.ModelY.Q,xcurrent(dimx+1:dimx+dimy));
            end
            Q = blkdiag(QX,QY);
        end
        
    end
end

