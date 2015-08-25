classdef mspdaf
    
    properties(Constant=true)
        Description = 'multiple source probability data association filter';
    end
    properties
        MttFilterSet = {mttfilter.kalman; mttfilter.kalman};
    end
    properties
        ParametersSet = [0.0004,  0.0004;
                         16,      16;
                         0.9997,  0.9997;
                         1,       1];
    end
    properties(Dependent=true, SetAccess=private)
        MotionModelSet
        MeasurementModelSet
        StateSym
        MeasureSym
    end
    
    methods
        %%% constructor
        function obj = mspdaf(MttFilterSet,ParametersSet)
            if nargin>0
                obj.MttFilterSet = MttFilterSet;
            end
            if nargin>1
                obj.ParametersSet = ParametersSet;
            end
        end
        
        %%% set property functions
        function obj = set.MttFilterSet(obj, MttFilterSet)
            hd = metaclass(mttfilter.kalman);
            NumFilter = length(MttFilterSet);
            strExceptionFilter = {'amm','imm','pf','cmkf','pdaf'};
            for kk = 1:1:NumFilter
                Info = findobj(hd.ContainingPackage.ClassList,'Name',class(MttFilterSet{kk}));             
                if isempty(Info.Name)
                    error('MttFilterSet must be within mttfilter package')
                elseif any(strcmp(Info.Name,strExceptionFilter))
                    error('%s is an exception filter', Info.Name)
                end
            end
            %%% check input arguments
            % 相同的运动模型
            %%% 
            obj.MttFilterSet = MttFilterSet;
        end
        
        function obj = set.ParametersSet(obj, ParametersSet)         
            if ~ismatrix(ParametersSet)
                error('ParametersSet must be a two dimensional matrix')
            elseif size(ParametersSet,1)~=4 && size(ParametersSet,2)~=4
                error('ParametersSet must be four rows or columns')
            end
            %%% 
            obj.ParametersSet = ParametersSet;
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
            NumFilter = length(obj.MttFilterSet);
            MeasureSym = [];
            for kk = 1:1:NumFilter
                if iscolumn(obj.MttFilterSet{kk}.MeasurementModel.MeasureSym)
                    MeasureSym = [MeasureSym; obj.MttFilterSet{kk}.MeasurementModel.MeasureSym];
                elseif isrow(obj.MttFilterSet{kk}.MeasurementModel.MeasureSym)
                    MeasureSym = [MeasureSym, obj.MttFilterSet{kk}.MeasurementModel.MeasureSym];
                end
            end
        end
        
        
        %%%
        [Xhat, Phat, Stil, K, Ztil] = filter   (obj, Xinit, Pinit, Z, flag)     % filtering algorithm
        [Xhat, Phat               ] = mcfilter (obj, Xinit, Pinit, Z, flag)
    end
    
end