classdef pdaf
    
    properties(Constant=true)
        Description = 'probability data association filter';
    end
    properties
        MttFilter = mttfilter.kalman;
    end
    properties
        lambda = 0.0004;
        gamma = 16;
        Pg = 0.9997;
        Pd = 1;
    end
    properties(Dependent=true,SetAccess=private)
        MotionModel
        MeasurementModel
        StateSym
        MeasureSym
    end
    
    methods
        %%% constructor
        function obj = pdaf(MttFilter, lambda, gamma, Pg, Pd)
            if nargin>0
                obj.MttFilter = MttFilter;
            end
            if nargin>1
                obj.lambda = lambda;
            end
            if nargin>2
                obj.gamma = gamma;
            end
            if nargin>3
                obj.Pg = Pg;
            end
            if nargin>4
                obj.Pd = Pd;
            end
        end
        
        %%% set property functions
        function obj = set.MttFilter(obj, MttFilter)
            hd = metaclass(mttfilter.kalman);
            strExceptionFilter = {'amm','imm','pf','cmkf', 'pdaf'};
            Info = findobj(hd.ContainingPackage.ClassList,'Name',class(MttFilter));             
            if isempty(Info.Name)
                error('MttFilter must be within mttfilter package')
            elseif any(strcmp(Info.Name,strExceptionFilter))
                error('%s is an exception filter', Info.Name)
            end
            %%% 
            obj.MttFilter = MttFilter;
        end
        
        function obj = set.lambda(obj, lambda)         
            if size(lambda,1)~=1 || size(lambda,2)~=1
                error('size of lambda is not approriate')
            end
            if lambda<=0 || lambda>=1
                error('value of lambda is not approriate');
            end
            obj.lambda = lambda;
        end
        
        function obj = set.gamma(obj, gamma)         
            if size(gamma,1)~=1 || size(gamma,2)~=1
                error('size of gamma is not approriate')
            end
            if gamma<=0
                error('value of gamma is not approriate');
            end
            obj.gamma = gamma;
        end
        
        function obj = set.Pg(obj, Pg)         
            if size(Pg,1)~=1 || size(Pg,2)~=1
                error('size of Pg is not approriate')
            end
            if Pg<=0 || Pg>1
                error('value of Pg is not approriate');
            end
            obj.Pg = Pg;
        end
        
        function obj = set.Pd(obj, Pd)         
            if size(Pd,1)~=1 || size(Pd,2)~=1
                error('size of Pd is not approriate')
            end
            if Pd<=0 || Pd>1
                error('value of Pd is not approriate');
            end
            obj.Pd = Pd;
        end
        
        %%% get dependent property functions
        function MotionModel = get.MotionModel(obj)
            MotionModel = obj.MttFilter.MotionModel;
        end
        function MeasurementModel = get.MeasurementModel(obj)
            MeasurementModel = obj.MttFilter.MeasurementModel;
        end
        function StateSym = get.StateSym(obj)
            StateSym = obj.MttFilter.MotionModel.StateSym;
        end
        function MeasureSym = get.MeasureSym(obj)
            MeasureSym = obj.MttFilter.MeasurementModel.MeasureSym;
        end
        
        
        %%%
        [Xhat, Phat, Stil, K, Ztil] = filter   (obj, Xinit, Pinit, Z, flag)     % filtering algorithm
        [Xhat, Phat               ] = mcfilter (obj, Xinit, Pinit, Z, flag)
    end
    
end