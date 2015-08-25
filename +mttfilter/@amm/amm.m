classdef amm
    
    properties(Constant=true)
        Description = 'autonomous multiple model filter';
    end
    properties
        MttFilterSet = {mttfilter.kalman; mttfilter.kalman};
    end
    properties(Dependent=true,SetAccess=private)
        MotionModelSet
        MeasurementModelSet
        StateSym
        MeasureSym
    end
    
    methods
        %%% constructor
        function obj = amm(MttFilterSet)
            if nargin>0
                obj.MttFilterSet = MttFilterSet;
            end
        end
        
        %%% set property functions
        function obj = set.MttFilterSet(obj, MttFilterSet)
            hd = metaclass(mttfilter.kalman);
            NumFilter = length(MttFilterSet);
            strExceptionFilter = {'amm','imm','pf','pdaf','mspdaf','immmspdaf'};
            for kk = 1:1:NumFilter
                Info = findobj(hd.ContainingPackage.ClassList,'Name',class(MttFilterSet{kk}));             
                if isempty(Info.Name)
                    error('MttFilterSet must be within mttfilter package')
                elseif any(strcmp(Info.Name,strExceptionFilter))
                    error('%s is an exception filter', Info.Name)
                end
            end
            %%% check input arguments
            
            %%% 
            obj.MttFilterSet = MttFilterSet;
        end
        
        %%% get dependent property functions
        function MotionModelSet = get.MotionModelSet(obj)
            NumFilter = length(obj.MttFilterSet);
            MotionModelSet = cell(NumFilter,1);
            for kk = 1:1:NumFilter
                MotionModelSet{kk} = obj.MttFilterSet{kk}.MotionModel;
            end
        end
        function MeasurementModelSet = get.MeasurementModelSet(obj)
            NumFilter = length(obj.MttFilterSet);
            MeasurementModelSet = cell(NumFilter,1);
            for kk = 1:1:NumFilter
                MeasurementModelSet{kk} = obj.MttFilterSet{kk}.MeasurementModel;
            end
        end
        function StateSym = get.StateSym(obj)
            StateSym = obj.MttFilterSet{1}.MotionModel.StateSym;
        end
        function MeasureSym = get.MeasureSym(obj)
            MeasureSym = obj.MttFilterSet{1}.MeasurementModel.MeasureSym;
        end
        
        %%% filtering functions
        [Xhat, Phat, Mu] = filter   (obj, Xinit, Pinit, Z, flag)     % filtering
        [Xhat, Phat, Mu] = mcfilter (obj, Xinit, Pinit, Z, flag)     % monte carlo filtering
    end
    
end
