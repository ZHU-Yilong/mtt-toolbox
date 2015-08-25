classdef imm
    
    properties(Constant=true)
        Description = 'interacting multiple model filter';
    end
    properties        
        MttFilterSet = {mttfilter.kalman; mttfilter.kalman};
    end
    properties
        MatrixPi = [0.95,0.05;0.05,0.95];
    end
    properties(Dependent=true,SetAccess=private)
        MotionModelSet
        MeasurementModelSet
        StateSym
        MeasureSym
    end
    
    methods
        %%% constructor
        function obj = imm(MttFilterSet, MatrixPi)
            if nargin>0
                obj.MttFilterSet = MttFilterSet;
            end
            if nargin>1
                obj.MatrixPi = MatrixPi;
            end
        end
        
        %%% set property functions
        function obj = set.MttFilterSet(obj, MttFilterSet)
            hd = metaclass(mttfilter.kalman);
            NumFilter = length(MttFilterSet);
            strExceptionFilter = {'amm','imm','pf','cmkf'};
            for kk = 1:1:NumFilter
                Info = findobj(hd.ContainingPackage.ClassList,'Name',class(MttFilterSet{kk}));             
                if isempty(Info.Name)
                    error('MttFilterSet must be within mttfilter package')
                elseif any(strcmp(Info.Name,strExceptionFilter))
                    error('%s is an exception filter', Info.Name)
                end
            end
            %%% check input arguments
            % 相同的观测模型
            % 运动模型的状态变量相同
            %%% 
            obj.MttFilterSet = MttFilterSet;
        end
        function obj = set.MatrixPi(obj, MatrixPi)
            if ~ismatrix(MatrixPi)
                error('MatrixPi must be two dimensions');
            end
            obj.MatrixPi = MatrixPi;
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
        
        %%%
        [Xhat, Phat, Mu] = filter   (obj, Xinit, Pinit, Z, flag)     % filtering algorithm
        [Xhat, Phat, Mu] = mcfilter (obj, Xinit, Pinit, Z, flag)
    end
    
end