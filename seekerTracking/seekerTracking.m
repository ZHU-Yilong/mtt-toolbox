function varargout = seekerTracking(varargin)
% SEEKERTRACKING MATLAB code for seekerTracking.fig
%      SEEKERTRACKING, by itself, creates a new SEEKERTRACKING or raises the existing
%      singleton*.
%
%      H = SEEKERTRACKING returns the handle to a new SEEKERTRACKING or the handle to
%      the existing singleton*.
%
%      SEEKERTRACKING('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SEEKERTRACKING.M with the given input arguments.
%
%      SEEKERTRACKING('Property','Value',...) creates a new SEEKERTRACKING or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before seekerTracking_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to seekerTracking_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help seekerTracking

% Last Modified by GUIDE v2.5 09-Oct-2014 23:19:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @seekerTracking_OpeningFcn, ...
                   'gui_OutputFcn',  @seekerTracking_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before seekerTracking is made visible.
function seekerTracking_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to seekerTracking (see VARARGIN)

% defaultSenario
defaultSenario.tPos = [18;0;0];
defaultSenario.tVel = [600;15;0];
defaultSenario.tManu = [10;1;2;4];
defaultSenario.mMotion = [1200;1;0.2]; 
defaultSenario.decoy = [1;2;2];
defaultSenario.chkbox = logical([0,0,0,0]);
defaultSenario.tPos = 1000*defaultSenario.tPos; %km转换为m
defaultSenario.tVel(2:3) = deg2rad(defaultSenario.tVel(2:3));%转为rad
defaultSenario.tManu(1) =  defaultSenario.tManu(1)*9.8; %转为m/s^2
setappdata(handles.senarioSet,'senarioIn',defaultSenario); %储存默认场景

% defaultSensor
defaultSensor.Sensors       = 0;
defaultSensor.radarChkbox   = [true;true;true;false];
defaultSensor.radarView     = [450;3;3;20];
defaultSensor.radarSigma    = [15;0.3;0.3;0.5];
defaultSensor.infraredView  = [2;2];
defaultSensor.infraredPixel = [64;64];
defaultSensor.infraredSigma = [0.03;0.03];
setappdata(handles.sensorSet, 'sensorIn', defaultSensor); % 默认传感器设置

% defaultFilter
defaultFilter.Filter = 1;
% defaultFilter.FilterParameters.T = 0.02;
% defaultFilter.FilterParameters.SigmaRan = 450;
% defaultFilter.FilterParameters.SigmaAzi = deg2rad(3);
% defaultFilter.FilterParameters.SigmaEle = deg2rad(3);
% defaultFilter.FilterHandle = filter1(defaultFilter.FilterParameters);
setappdata(handles.filterSet, 'filterIn', defaultFilter); % 缺省滤波器设置

% defaultSeeker
% defaultSeeker.SimulateStep = 0.02;
% defaultSeeker.SimulateTimes = 1;
% defaultSeeker.SimulateTime = 5;
% setappdata(hObject,'seekerIn',defaultSeeker); %储存默认场景

handles.SimulateStep  = 0.02;
handles.SimulateTimes = 1;
handles.SimulateTime  = 5;

% Choose default command line output for seekerTracking
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes seekerTracking wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = seekerTracking_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in senarioSet.
function senarioSet_Callback(hObject, eventdata, handles)
% hObject    handle to senarioSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
senarioStru = getappdata(hObject,'senarioIn');          %读取当前场景配置
senarioStru = senarioDef( 'senarioIn', senarioStru);    %open the tracking senarioDef.m，用户重新设置
setappdata(hObject,'senarioIn',senarioStru);            %保留用户设置


% --- Executes on button press in sensorSet.
function sensorSet_Callback(hObject, eventdata, handles)
% hObject    handle to sensorSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
sensorStru = getappdata(hObject,'sensorIn');            %读取当前场景配置
sensorStru = sensorDef( 'sensorIn', sensorStru);        %open the tracking senarioDef.m，用户重新设置
setappdata(hObject,'sensorIn',sensorStru);              %保留用户设置

% --- Executes on button press in filterSet.
function filterSet_Callback(hObject, eventdata, handles)
% hObject    handle to filterSet (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

filterStru  = getappdata(hObject, 'filterIn');          %读取当前滤波器配置
filterStru  = filterDef( 'filterIn', filterStru);       %open the tracking filterDef.m，用户重新设置
setappdata(hObject, 'filterIn', filterStru);            %保留用户设置

% --- Executes on button press in resultEvaluate.
function resultEvaluate_Callback(hObject, eventdata, handles)
% hObject    handle to resultEvaluate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
Xhat = handles.Xhat;
xState = handles.xState;
offset = handles.offset;
NumStep = handles.NumStep;
NumMC  = handles.SimulateTimes;
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
RadarMeasure = handles.RadarMeasure;
filterParam  = getappdata(handles.filterSet,  'filterIn');
time = 0:T:TotalTime-T;

% 非蒙特卡洛仿真
if NumMC==1
    if offset > 0
        IndexPosRMSE = rmse(Xhat(1:offset:end,:) - xState(1:3:end,:));
    end
    if offset > 1
        IndexVelRMSE = rmse(Xhat(2:offset:end,:) - xState(2:3:end,:));
    end
    if offset > 2
        IndexAccRMSE = rmse(Xhat(3:offset:end,:) - xState(3:3:end,:));
    end
    
    set(handles.figure1,'CurrentAxes',handles.axes3);
    switch offset
        case 1
            plot(time, IndexPosRMSE, '-r');
            legend('位置');
        case 2
            plot(time, IndexPosRMSE, '-r');
            hold on
            plot(time, IndexVelRMSE, '-g');
            hold off
            box on, grid on
            legend('位置', '速度');
        case 3
            plot(time, IndexPosRMSE, '-r');
            hold on
            plot(time, IndexVelRMSE, '-g');
            plot(time, IndexAccRMSE, '-b');
            hold off
            box on, grid on
            legend('位置', '速度', '加速度');
        otherwise
    end
    
    set(handles.axes3, 'UIContextMenu', []);   
    return;
end

set(handles.axes3, 'UIContextMenu', handles.Index);

% Index 1: RMSE
% Index 2: AEE
% Index 3: GAE / LGAE
if offset > 0
    IndexPosRMSE = rmse(Xhat(1:offset:end,:,:)-repmat(xState(1:3:end,:),[1,1,NumMC]));
    IndexPosAEE  = aee(Xhat(1:offset:end,:,:)-repmat(xState(1:3:end,:),[1,1,NumMC]));
    IndexPosGAE  = gae(Xhat(1:offset:end,:,:)-repmat(xState(1:3:end,:),[1,1,NumMC]));
    IndexPosLGAE = lgae(Xhat(1:offset:end,:,:)-repmat(xState(1:3:end,:),[1,1,NumMC]));
end
if offset > 1
    IndexVelRMSE = rmse(Xhat(2:offset:end,:,:)-repmat(xState(2:3:end,:),[1,1,NumMC]));
    IndexVelAEE  = aee(Xhat(2:offset:end,:,:)-repmat(xState(2:3:end,:),[1,1,NumMC]));
    IndexVelGAE  = gae(Xhat(2:offset:end,:,:)-repmat(xState(2:3:end,:),[1,1,NumMC]));
    IndexVelLGAE = lgae(Xhat(2:offset:end,:,:)-repmat(xState(2:3:end,:),[1,1,NumMC]));
end
if offset > 2
    IndexAccRMSE = rmse(Xhat(3:offset:end,:,:)-repmat(xState(3:3:end,:),[1,1,NumMC]));
    IndexAccAEE  = aee(Xhat(3:offset:end,:,:)-repmat(xState(3:3:end,:),[1,1,NumMC]));
    IndexAccGAE  = gae(Xhat(3:offset:end,:,:)-repmat(xState(3:3:end,:),[1,1,NumMC]));
    IndexAccLGAE = lgae(Xhat(3:offset:end,:,:)-repmat(xState(3:3:end,:),[1,1,NumMC]));
end

% Index 4: MERF1 / MERF2 / LMERF
[PseudoMeasureTrue(2,:),PseudoMeasureTrue(3,:),PseudoMeasureTrue(1,:)] = ...
    cart2sph(xState(1,:),xState(4,:),xState(7,:));
[PseudoMeasureEstimate(2,:,:),PseudoMeasureEstimate(3,:,:),PseudoMeasureEstimate(1,:,:)] = ...
    cart2sph(Xhat(1,:,:),Xhat(1+offset,:,:),Xhat(1+2*offset,:,:));
switch filterParam.Filter
    case {1, 2, 99}
        IndexMERF1 = merf1(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate,RadarMeasure);
        IndexMERF2 = merf2(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate,RadarMeasure);
        IndexLMERF = lmerf(repmat(PseudoMeasureTrue,[1,1,NumMC]),PseudoMeasureEstimate,RadarMeasure);
    otherwise
        IndexMERF1 = [];
        IndexMERF2 = [];
        IndexLMERF = [];
end
% Index 5: EERF1 / EERF2 / LEERF
switch filterParam.Filter
    case {1, 2, 99}
        [StateNoisy(1,:,:),StateNoisy(2,:,:),StateNoisy(3,:,:)] = ...
            sph2cart(RadarMeasure(2,:,:),RadarMeasure(3,:,:),RadarMeasure(1,:,:));
        IndexEERF1 = eerf1(repmat(xState(1:3:end,:),[1,1,NumMC]), Xhat(1:offset:end,:,:), StateNoisy);
        IndexEERF2 = eerf2(repmat(xState(1:3:end,:),[1,1,NumMC]), Xhat(1:offset:end,:,:), StateNoisy);
        IndexLEERF = leerf(repmat(xState(1:3:end,:),[1,1,NumMC]), Xhat(1:offset:end,:,:), StateNoisy);
    otherwise
        IndexEERF1 = [];
        IndexEERF2 = [];
        IndexLEERF = [];
end
% Index 6: SR
switch filterParam.Filter
    case {1, 2, 99}
        AlphaSucc = 2:2:200;
        IndexSR = zeros(NumStep,length(AlphaSucc));
        for kk = 1:1:length(AlphaSucc)
            IndexSR(:,kk) = succrate(repmat(PseudoMeasureTrue,[1,1,NumMC]), PseudoMeasureEstimate, RadarMeasure, AlphaSucc(kk));
        end
    otherwise
        IndexSR = [];
end
% Index 7: FR
switch filterParam.Filter
    case {1, 2, 99}
        AlphaFail = 202:2:400;
        IndexFR = zeros(NumStep,length(AlphaFail));
        for kk = 1:1:length(AlphaFail)
            IndexFR(:,kk) = failrate(repmat(PseudoMeasureTrue,[1,1,NumMC]), PseudoMeasureEstimate, RadarMeasure, AlphaFail(kk));
        end
    otherwise
        IndexFR = [];
end

% plot 
set(handles.figure1,'CurrentAxes',handles.axes3);
switch offset
    case 1
        plot(time, IndexPosRMSE, '-r');
        legend('位置');
    case 2
        plot(time, IndexPosRMSE, '-r');
        hold on
        plot(time, IndexVelRMSE, '-g');
        hold off
        box on, grid on
        legend('位置', '速度');
    case 3
        plot(time, IndexPosRMSE, '-r');
        hold on
        plot(time, IndexVelRMSE, '-g');
        plot(time, IndexAccRMSE, '-b');
        hold off
        box on, grid on
        legend('位置', '速度', '加速度');
    otherwise
end
set(handles.Index_RMSE,  'Checked', 'on');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

% save data
if offset > 0
    handles.IndexPosRMSE = IndexPosRMSE;
    handles.IndexPosAEE  = IndexPosAEE;
    handles.IndexPosGAE  = IndexPosGAE;
    handles.IndexPosLGAE = IndexPosLGAE;
end
if offset > 1
    handles.IndexVelRMSE = IndexVelRMSE;
    handles.IndexVelAEE  = IndexVelAEE;
    handles.IndexVelGAE  = IndexVelGAE;
    handles.IndexVelLGAE = IndexVelLGAE;
end
if offset > 2
    handles.IndexAccRMSE = IndexAccRMSE;
    handles.IndexAccAEE  = IndexAccAEE;
    handles.IndexAccGAE  = IndexAccGAE;
    handles.IndexAccLGAE = IndexAccLGAE;
end
handles.IndexMERF1 = IndexMERF1;
handles.IndexMERF2 = IndexMERF2;
handles.IndexLMERF = IndexLMERF;
handles.IndexEERF1 = IndexEERF1;
handles.IndexEERF2 = IndexEERF2;
handles.IndexLEERF = IndexLEERF;
handles.IndexSR = IndexSR;
handles.IndexFR = IndexFR;
guidata(hObject, handles);



% --- Executes on button press in startSimulate.
function startSimulate_Callback(hObject, eventdata, handles)
% hObject    handle to startSimulate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% set(handles.senarioSet,      'Enable', 'off');
% set(handles.sensorSet,       'Enable', 'off');
% set(handles.filterSet,       'Enable', 'off');
% set(handles.senarioPreview,  'Enable', 'off');
% set(handles.startSimulate,   'Enable', 'off');
% set(handles.resultEvaluate,  'Enable', 'off');

% 获取设定参数
senarioParam = getappdata(handles.senarioSet, 'senarioIn');
sensorParam  = getappdata(handles.sensorSet,  'sensorIn');
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
NumMC = handles.SimulateTimes;
RadarSigmaRan = sensorParam.radarSigma(1);
RadarSigmaAzi = deg2rad(sensorParam.radarSigma(2));
RadarSigmaEle = deg2rad(sensorParam.radarSigma(3));
InfraredSigmaRan = RadarSigmaRan;
InfraredSigmaAzi = deg2rad(sensorParam.infraredSigma(1));
InfraredSigmaEle = deg2rad(sensorParam.infraredSigma(2));
% 滤波器参数设置
% 仅下列参数可以设置，其他参数固定
FilterParameters{1}.T = T;
FilterParameters{1}.SigmaRan = RadarSigmaRan;
FilterParameters{1}.SigmaAzi = RadarSigmaAzi;
FilterParameters{1}.SigmaEle = RadarSigmaEle;
FilterParameters{2}.T = T;
FilterParameters{2}.SigmaRan = InfraredSigmaRan;
FilterParameters{2}.SigmaAzi = InfraredSigmaAzi;
FilterParameters{2}.SigmaEle = InfraredSigmaEle;
% 创建滤波器
switch filterParam.Filter
    case 1        
        filterHandle = filter1(FilterParameters{1});        % UKF
    case 2
        filterHandle = filter2(FilterParameters{1});        % IMM
    case 3
        filterHandle = filter3(FilterParameters{1});        % PDAF
    case 4
        filterHandle = filter4(FilterParameters);           % MSPDAF
    case 5
        filterHandle = filter5(FilterParameters);           % IMM/MSPDAF
    case 99
        filterHandle = filter99(FilterParameters{1});       % CMKF
    otherwise
        filterHandle = filter99(FilterParameters{1});
end

% simulate target trajectory

N = floor(TotalTime/T);
xinit = [senarioParam.tPos; senarioParam.tVel];
acc    = senarioParam.tManu(1);
direct = senarioParam.tManu(2);
tstart = senarioParam.tManu(3);
tlast  = senarioParam.tManu(4);
switch direct
    case 1
        ddeg = 0;
    case 2
        ddeg = pi;
    case 3
        ddeg = pi/2;
    case 4
        ddeg = pi*3/2;
    otherwise
        ddeg = 0;
end
an1 = acc*cos(ddeg);
an2 = acc*sin(ddeg);
N1 = floor(tstart/T);
N2 = floor((tstart+tlast)/T);
if N2<N
    an1s = [zeros(1,N1), an1*ones(1,N2-N1), zeros(1,N-N2)];
    an2s = [zeros(1,N1), an2*ones(1,N2-N1), zeros(1,N-N2)];    
else
    an1s = [zeros(1,N1), an1*ones(1,N-N1)];
    an2s = [zeros(1,N1), an2*ones(1,N-N1)];
end
ats  = zeros(1,N);
a = [ats; an1s; an2s];
time = [T, TotalTime];
xState = trajectory.dxyz(xinit, a, time);

% measurements of radar and infrared
[xAzimuth, xElevation, xRange] = cart2sph(xState(1,:), xState(4,:), xState(7,:));
RadarMeasure = repmat([xRange; xAzimuth; xElevation], [1,1,NumMC]) + ...
    [randn(1,N,NumMC)*RadarSigmaRan; 
     randn(1,N,NumMC)*RadarSigmaAzi; 
     randn(1,N,NumMC)*RadarSigmaEle];
InfraredMeasure = repmat([xAzimuth; xElevation], [1,1,NumMC]) + ...
    [randn(1,N,NumMC)*InfraredSigmaAzi; 
     randn(1,N,NumMC)*InfraredSigmaEle];
PseudoInfraredMeasure = [RadarMeasure(1,:,:); InfraredMeasure];
 
% clutter measurements
lambda = 0.0004;
RadarMeasureClutter = cell(N,NumMC);
InfraredMeasureClutter = cell(N,NumMC);
PseudoInfraredMeasureClutter= cell(N,NumMC);
for kk = 1:1:NumMC
    for jj = 1:1:N
        nc = randi(3, 1);
        Av = nc/10/lambda;
        q = sqrt(10*Av)/2;
        CartMeasure = repmat(xState(1:3:end,jj)-[q;q;q],[1,nc]) + rand(3,nc)*2*q;
        [SphMeasureAzi,SphMeasureEle,SphMeasureRan] = cart2sph(CartMeasure(1),CartMeasure(2),CartMeasure(3));
        
        RadarMeasureClutter{jj,kk} = repmat([SphMeasureRan;SphMeasureAzi;SphMeasureEle], [1,nc]) + ...
            [randn(1,nc)*RadarSigmaRan; 
             randn(1,nc)*RadarSigmaAzi; 
             randn(1,nc)*RadarSigmaEle];
         InfraredMeasureClutter{jj,kk} = repmat([SphMeasureAzi;SphMeasureEle], [1,nc]) + ...
            [randn(1,nc)*InfraredSigmaAzi; 
             randn(1,nc)*InfraredSigmaEle];
         PseudoInfraredMeasureClutter{jj,kk} = [RadarMeasureClutter{jj,kk}(1,:); InfraredMeasureClutter{jj,kk}];
    end
end

MeasureClutter = cell(2, N, NumMC);
for kk = 1:1:NumMC
    for jj = 1:1:N
        MeasureClutter{1,jj,kk} = RadarMeasureClutter{jj,kk};
        MeasureClutter{2,jj,kk} = PseudoInfraredMeasureClutter{jj,kk};
    end
end


% initial value
offset = length(filterHandle.StateSym)/3;
switch offset
    case 1
        Xinit = [xState(1,1);xState(4,1);xState(7,1)];
        Pinit = blkdiag( (10*RadarSigmaRan)^2, (10*RadarSigmaRan)^2, (10*RadarSigmaRan)^2 );
    case 2
        Xinit = [xState(1:2,1);xState(4:5,1);xState(7:8,1)];
        Pinit = blkdiag((10*RadarSigmaRan)^2, (20*RadarSigmaRan)^2, ...
            (10*RadarSigmaRan)^2, (20*RadarSigmaRan)^2, ...
            (10*RadarSigmaRan)^2, (20*RadarSigmaRan)^2);
    case 3
        Xinit = xState(:,1);
        Pinit = blkdiag( (10*RadarSigmaRan)^2, (20*RadarSigmaRan)^2, (20*RadarSigmaRan)^2, ...
            (10*RadarSigmaRan)^2, (20*RadarSigmaRan)^2, (20*RadarSigmaRan)^2,...
            (10*RadarSigmaRan)^2, (20*RadarSigmaRan)^2, (20*RadarSigmaRan)^2 );
    otherwise
end


% filtering progress
switch filterParam.Filter
    case {1, 2, 99}
        if handles.SimulateTimes == 1
            [Xhat, Phat] = filterHandle.filter(Xinit, Pinit, RadarMeasure, 0);
        else
            [Xhat, Phat] = filterHandle.mcfilter(Xinit, Pinit, RadarMeasure, 0);
        end
    case 3
        if handles.SimulateTimes == 1
            [Xhat, Phat] = filterHandle.filter(Xinit, Pinit, RadarMeasureClutter, 0);
        else
            [Xhat, Phat] = filterHandle.mcfilter(Xinit, Pinit, RadarMeasureClutter, 0);
        end
    case {4, 5}
        if handles.SimulateTimes == 1
            [Xhat, Phat] = filterHandle.filter(Xinit, Pinit, MeasureClutter, 0);
        else
            [Xhat, Phat] = filterHandle.mcfilter(Xinit, Pinit, MeasureClutter, 0);
        end
    otherwise
end
        

% plot filtered target trajectory
set(handles.figure1,'CurrentAxes',handles.axes1);

kk = 1;
[MeasureX,MeasureY,MeasureZ] = sph2cart(RadarMeasure(2,:,kk),RadarMeasure(3,:,kk),RadarMeasure(1,:,kk));
hold off
plot3(xState(1,:),xState(4,:),xState(7,:),'-r');
hold on
plot3(Xhat(1,:,kk),Xhat(1+offset,:,kk),Xhat(1+2*offset,:,kk),'-g');
scatter3(MeasureX, MeasureY, MeasureZ, 3);
hold off
axis equal, box on, grid on
% legend('真实','滤波','观测')
view(3)

% save data
handles.NumStep = N;
handles.offset = offset;
handles.xState = xState;
handles.Xhat   = Xhat;
handles.RadarMeasure = RadarMeasure;
handles.PseudoInfraredMeasure = PseudoInfraredMeasure;
handles.RadarMeasureClutter = RadarMeasureClutter;
handles.PseudoInfraredMeasureClutter = PseudoInfraredMeasureClutter;

guidata(hObject, handles);

% set(handles.senarioSet,     'Enable', 'on');
% set(handles.sensorSet,      'Enable', 'on');
% set(handles.filterSet,      'Enable', 'on');
% set(handles.senarioPreview, 'Enable', 'on');
% set(handles.startSimulate,  'Enable', 'on');
% set(handles.resultEvaluate, 'Enable', 'on');




function simulateTime_Callback(hObject, eventdata, handles)
% hObject    handle to simulateTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of simulateTime as text
%        str2double(get(hObject,'String')) returns contents of simulateTime as a double
SimulateTime = str2double(get(hObject,'String'));
if isnan(SimulateTime) || (SimulateTime<0) || (SimulateTime<handles.SimulateStep)
    errordlg('数值设置错误！','Error','modal');
    uicontrol(hObject);
else
    handles.SimulateTime = SimulateTime;
    guidata(hObject, handles);
end
    
    
% --- Executes during object creation, after setting all properties.
function simulateTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simulateTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function simulateStep_Callback(hObject, eventdata, handles)
% hObject    handle to simulateStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of simulateStep as text
%        str2double(get(hObject,'String')) returns contents of simulateStep as a double
SimulateStep = str2double(get(hObject,'String'));
if isnan(SimulateStep) || (SimulateStep<0) || (SimulateStep>handles.SimulateTime)
    errordlg('数值设置错误！','Error','modal');
    uicontrol(hObject);
else
    handles.SimulateStep = SimulateStep;
    guidata(hObject, handles);
end



% --- Executes during object creation, after setting all properties.
function simulateStep_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simulateStep (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function simulateTimes_Callback(hObject, eventdata, handles)
% hObject    handle to simulateTimes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of simulateTimes as text
%        str2double(get(hObject,'String')) returns contents of simulateTimes as a double
SimulateTimes = str2double(get(hObject,'String'));
if uint64(SimulateTimes) ~= SimulateTimes
    errordlg('数值设置错误！','Error','modal');
    uicontrol(hObject);
else
    handles.SimulateTimes = SimulateTimes;
    guidata(hObject, handles);
end


% --- Executes during object creation, after setting all properties.
function simulateTimes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to simulateTimes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in senarioPreview.
function senarioPreview_Callback(hObject, eventdata, handles)
% hObject    handle to senarioPreview (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

N = floor(handles.SimulateTime/handles.SimulateStep);
senarioParam = getappdata(handles.senarioSet, 'senarioIn');
xinit = [senarioParam.tPos; senarioParam.tVel];
acc    = senarioParam.tManu(1);
direct = senarioParam.tManu(2);
tstart = senarioParam.tManu(3);
tlast  = senarioParam.tManu(4);
switch direct
    case 1
        ddeg = 0;
    case 2
        ddeg = pi;
    case 3
        ddeg = pi/2;
    case 4
        ddeg = pi*3/2;
    otherwise
        ddeg = 0;
end
an1 = acc*cos(ddeg);
an2 = acc*sin(ddeg);
N1 = floor(tstart/handles.SimulateStep);
N2 = floor((tstart+tlast)/handles.SimulateStep);
if N2<N
    an1s = [zeros(1,N1), an1*ones(1,N2-N1), zeros(1,N-N2)];
    an2s = [zeros(1,N1), an2*ones(1,N2-N1), zeros(1,N-N2)];    
else
    an1s = [zeros(1,N1), an1*ones(1,N-N1)];
    an2s = [zeros(1,N1), an2*ones(1,N-N1)];
end
ats  = zeros(1,N);
a = [ats; an1s; an2s];
time = [handles.SimulateStep, handles.SimulateTime];
xState = trajectory.dxyz(xinit, a, time);
set(handles.figure1, 'CurrentAxes', handles.axes1);
plot3(xState(1,:),xState(4,:),xState(7,:));
% scatter3(0,0,0);
view(3);
grid on, axis on
axis equal


% --------------------------------------------------------------------
function Index_RMSE_Callback(hObject, eventdata, handles)
% hObject    handle to Index_RMSE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

offset = handles.offset;
if offset > 0
    IndexPosRMSE = handles.IndexPosRMSE;
end
if offset > 1
    IndexVelRMSE = handles.IndexVelRMSE;
end
if offset > 2
    IndexAccRMSE = handles.IndexAccRMSE;
end

TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch offset
    case 1
        plot(time, IndexPosRMSE, '-r');
        legend('位置');
    case 2
        plot(time, IndexPosRMSE, '-r');
        hold on
        plot(time, IndexVelRMSE, '-g');
        hold off
        box on, grid on
        legend('位置', '速度');
    case 3
        plot(time, IndexPosRMSE, '-r');
        hold on
        plot(time, IndexVelRMSE, '-g');
        plot(time, IndexAccRMSE, '-b');
        hold off
        box on, grid on
        legend('位置', '速度', '加速度');
    otherwise
end

set(handles.Index_RMSE,  'Checked', 'on');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'RMSE';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_AEE_Callback(hObject, eventdata, handles)
% hObject    handle to Index_AEE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

offset = handles.offset;
if offset > 0
    IndexPosAEE  = handles.IndexPosAEE;
end
if offset > 1
    IndexVelAEE  = handles.IndexVelAEE;
end
if offset > 2
    IndexAccAEE  = handles.IndexAccAEE;
end

TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch offset
    case 1
        plot(time, IndexPosAEE, '-r');
        legend('位置');
    case 2
        plot(time, IndexPosAEE, '-r');
        hold on
        plot(time, IndexVelAEE, '-g');
        hold off
        box on, grid on
        legend('位置', '速度');
    case 3
        plot(time, IndexPosAEE, '-r');
        hold on
        plot(time, IndexVelAEE, '-g');
        plot(time, IndexAccAEE, '-b');
        hold off
        box on, grid on
        legend('位置', '速度', '加速度');
    otherwise
end

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'on');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'AEE';
guidata(hObject, handles);


% --------------------------------------------------------------------
function Index_GAE_Callback(hObject, eventdata, handles)
% hObject    handle to Index_GAE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
offset = handles.offset;
if offset > 0
    IndexPosGAE  = handles.IndexPosGAE;
end
if offset > 1
    IndexVelGAE  = handles.IndexVelGAE;
end
if offset > 2
    IndexAccGAE  = handles.IndexAccGAE;
end

TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;

switch offset
    case 1
        plot(time, IndexPosGAE, '-r');
        legend('位置');
    case 2
        plot(time, IndexPosGAE, '-r');
        hold on
        plot(time, IndexVelGAE, '-g');
        hold off
        box on, grid on
        legend('位置', '速度');
    case 3
        plot(time, IndexPosGAE, '-r');
        hold on
        plot(time, IndexVelGAE, '-g');
        plot(time, IndexAccGAE, '-b');
        hold off
        box on, grid on
        legend('位置', '速度', '加速度');
    otherwise
end

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'on');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'GAE';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_LGAE_Callback(hObject, eventdata, handles)
% hObject    handle to Index_LGAE (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
offset = handles.offset;
if offset > 0
    IndexPosLGAE = handles.IndexPosLGAE;
end
if offset > 1
    IndexVelLGAE = handles.IndexVelLGAE;
end
if offset > 2
    IndexAccLGAE = handles.IndexAccLGAE;
end

TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;

switch offset
    case 1
        plot(time, IndexPosLGAE, '-r');
        legend('位置');
    case 2
        plot(time, IndexPosLGAE, '-r');
        hold on
        plot(time, IndexVelLGAE, '-g');
        hold off
        box on, grid on
        legend('位置', '速度');
    case 3
        plot(time, IndexPosLGAE, '-r');
        hold on
        plot(time, IndexVelLGAE, '-g');
        plot(time, IndexAccLGAE, '-b');
        hold off
        box on, grid on
        legend('位置', '速度', '加速度');
    otherwise
end

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'on');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'LGAE';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_MERF1_Callback(hObject, eventdata, handles)
% hObject    handle to Index_MERF1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% offset = handles.offset;
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;

time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        IndexMERF1 = handles.IndexMERF1;
        plot(time, IndexMERF1, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'on');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'MERF1';
guidata(hObject, handles);


% --------------------------------------------------------------------
function Index_MERF2_Callback(hObject, eventdata, handles)
% hObject    handle to Index_MERF2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        IndexMERF2 = handles.IndexMERF2;
        plot(time, IndexMERF2, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'on');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'MERF2';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_LMERF_Callback(hObject, eventdata, handles)
% hObject    handle to Index_LMERF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        IndexLMERF = handles.IndexLMERF;
        plot(time, IndexLMERF, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'on');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'LMERF';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_EERF1_Callback(hObject, eventdata, handles)
% hObject    handle to Index_EERF1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        IndexEERF1 = handles.IndexEERF1;
        plot(time, IndexEERF1, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'on');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'EERF1';
guidata(hObject, handles);


% --------------------------------------------------------------------
function Index_EERF2_Callback(hObject, eventdata, handles)
% hObject    handle to Index_EERF2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        IndexEERF2 = handles.IndexEERF2;
        plot(time, IndexEERF2, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'on');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'EERF2';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_LEERF_Callback(hObject, eventdata, handles)
% hObject    handle to Index_LEERF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;
time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        IndexLEERF = handles.IndexLEERF;
        plot(time, IndexLEERF, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'on');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'LEERF';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_SR_Callback(hObject, eventdata, handles)
% hObject    handle to Index_SR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;

IndexSR = handles.IndexSR;

time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        plot(time, IndexSR, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'on');
set(handles.Index_FR,    'Checked', 'off');

handles.CurrentIndexDisplay = 'SR';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_FR_Callback(hObject, eventdata, handles)
% hObject    handle to Index_FR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
filterParam  = getappdata(handles.filterSet,  'filterIn');
TotalTime = handles.SimulateTime;
T = handles.SimulateStep;

IndexFR = handles.IndexFR;

time = 0:T:TotalTime-T;
switch filterParam.Filter
    case {1, 2, 99}
        plot(time, IndexFR, '-r');
    otherwise
end
box on, grid on

set(handles.Index_RMSE,  'Checked', 'off');
set(handles.Index_AEE,   'Checked', 'off');
set(handles.Index_GAE,   'Checked', 'off');
set(handles.Index_LGAE,  'Checked', 'off');
set(handles.Index_MERF1, 'Checked', 'off');
set(handles.Index_MERF2, 'Checked', 'off');
set(handles.Index_LMERF, 'Checked', 'off');
set(handles.Index_EERF1, 'Checked', 'off');
set(handles.Index_EERF2, 'Checked', 'off');
set(handles.Index_LEERF, 'Checked', 'off');
set(handles.Index_SR,    'Checked', 'off');
set(handles.Index_FR,    'Checked', 'on');

handles.CurrentIndexDisplay = 'SR';
guidata(hObject, handles);

% --------------------------------------------------------------------
function Index_Callback(hObject, eventdata, handles)
% hObject    handle to Index (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Data_Callback(hObject, eventdata, handles)
% hObject    handle to Data (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isfield(handles, 'Xhat')
    errordlg('无数据可保存！','Error','modal');
    uicontrol(hObject);
end

File = 'matlab.mat';

senarioParam = getappdata(handles.senarioSet, 'senarioIn');
sensorParam  = getappdata(handles.sensorSet,  'sensorIn');
filterParam  = getappdata(handles.filterSet,  'filterIn');
save(File, 'senarioParam', 'sensorParam', 'filterParam');

SimulateTime  = handles.SimulateTime;
SimulateStep  = handles.SimulateStep;
SimulateTimes = handles.SimulateTimes;
save(File, 'SimulateTime', 'SimulateStep', 'SimulateTimes', '-append');

switch filterParam.Filter
    case {1, 2, 99}
        Measure = handles.RadarMeasure;
    case 3
        Measure = handles.RadarMeasureClutter;
    case {4, 5}
        Measure = handles.MeasureClutter;
    otherwise
        Measure = handles.RadarMeasure;
end

StateEstimate = handles.Xhat;
StateTrue     = handles.xState;

save(File, 'Measure', 'StateEstimate', 'StateTrue', '-append');

if SimulateTimes>1
    offset = handles.offset;
    if offset > 0
        IndexPosRMSE = handles.IndexPosRMSE;
        IndexPosAEE  = handles.IndexPosAEE;
        IndexPosGAE  = handles.IndexPosGAE;
        IndexPosLGAE = handles.IndexPosLGAE;
        save(File, 'IndexPosRMSE', 'IndexPosAEE', 'IndexPosGAE', 'IndexPosLGAE', '-append');
    end
    if offset > 1
        IndexVelRMSE = handles.IndexVelRMSE;
        IndexVelAEE  = handles.IndexVelAEE;
        IndexVelGAE  = handles.IndexVelGAE;
        IndexVelLGAE = handles.IndexVelLGAE;
        save(File, 'IndexVelRMSE', 'IndexVelAEE', 'IndexVelGAE', 'IndexVelLGAE', '-append');
    end
    if offset > 2
        IndexAccRMSE = handles.IndexAccRMSE;
        IndexAccAEE  = handles.IndexAccAEE;
        IndexAccGAE  = handles.IndexAccGAE;
        IndexAccLGAE = handles.IndexAccLGAE;
        save(File, 'IndexAccRMSE', 'IndexAccAEE', 'IndexAccGAE', 'IndexAccLGAE', '-append');
    end

    switch filterParam.Filter
        case {1, 2, 99}
            IndexMERF1 = handles.IndexMERF1;
            IndexMERF2 = handles.IndexMERF2;
            IndexLMERF = handles.IndexLMERF;
            IndexEERF1 = handles.IndexEERF1;
            IndexEERF2 = handles.IndexEERF2;
            IndexLEERF = handles.IndexLEERF;
            IndexSR = handles.IndexSR;
            IndexFR = handles.IndexFR;
            save(File, 'IndexMERF1', 'IndexMERF2', 'IndexLMERF', ...
                'IndexEERF1', 'IndexEERF2', 'IndexLEERF', ...
                'IndexSR', 'IndexFR', '-append');
        otherwise
    end
end


% --------------------------------------------------------------------
function saveas_Callback(hObject, eventdata, handles)
% hObject    handle to saveas (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isfield(handles, 'Xhat')
    errordlg('无数据可保存！','Error','modal');
    uicontrol(hObject);
end

[filename, pathname] = uiputfile( ...
    {'*.mat';'*.*'}, ...
    'Save as');	

if isequal([filename,pathname],[0,0])
    return
else
    % Construct the full path and save
    File = fullfile(pathname,filename);
    
    senarioParam = getappdata(handles.senarioSet, 'senarioIn');
    sensorParam  = getappdata(handles.sensorSet,  'sensorIn');
    filterParam  = getappdata(handles.filterSet,  'filterIn');
    save(File, 'senarioParam', 'sensorParam', 'filterParam');

    SimulateTime  = handles.SimulateTime;
    SimulateStep  = handles.SimulateStep;
    SimulateTimes = handles.SimulateTimes;
    save(File, 'SimulateTime', 'SimulateStep', 'SimulateTimes', '-append');

    switch filterParam.Filter
        case {1, 2, 99}
            Measure = handles.RadarMeasure;
        case 3
            Measure = handles.RadarMeasureClutter;
        case {4, 5}
            Measure = handles.MeasureClutter;
        otherwise
            Measure = handles.RadarMeasure;
    end

    StateEstimate = handles.Xhat;
    StateTrue     = handles.xState;

    save(File, 'Measure', 'StateEstimate', 'StateTrue', '-append');

    if SimulateTimes>1
        offset = handles.offset;
        if offset > 0
            IndexPosRMSE = handles.IndexPosRMSE;
            IndexPosAEE  = handles.IndexPosAEE;
            IndexPosGAE  = handles.IndexPosGAE;
            IndexPosLGAE = handles.IndexPosLGAE;
            save(File, 'IndexPosRMSE', 'IndexPosAEE', 'IndexPosGAE', 'IndexPosLGAE', '-append');
        end
        if offset > 1
            IndexVelRMSE = handles.IndexVelRMSE;
            IndexVelAEE  = handles.IndexVelAEE;
            IndexVelGAE  = handles.IndexVelGAE;
            IndexVelLGAE = handles.IndexVelLGAE;
            save(File, 'IndexVelRMSE', 'IndexVelAEE', 'IndexVelGAE', 'IndexVelLGAE', '-append');
        end
        if offset > 2
            IndexAccRMSE = handles.IndexAccRMSE;
            IndexAccAEE  = handles.IndexAccAEE;
            IndexAccGAE  = handles.IndexAccGAE;
            IndexAccLGAE = handles.IndexAccLGAE;
            save(File, 'IndexAccRMSE', 'IndexAccAEE', 'IndexAccGAE', 'IndexAccLGAE', '-append');
        end

        switch filterParam.Filter
            case {1, 2, 99}
                IndexMERF1 = handles.IndexMERF1;
                IndexMERF2 = handles.IndexMERF2;
                IndexLMERF = handles.IndexLMERF;
                IndexEERF1 = handles.IndexEERF1;
                IndexEERF2 = handles.IndexEERF2;
                IndexLEERF = handles.IndexLEERF;
                IndexSR = handles.IndexSR;
                IndexFR = handles.IndexFR;
                save(File, 'IndexMERF1', 'IndexMERF2', 'IndexLMERF', ...
                    'IndexEERF1', 'IndexEERF2', 'IndexLEERF', ...
                    'IndexSR', 'IndexFR', '-append');
            otherwise
        end
    end

end


% --------------------------------------------------------------------
function loadws_Callback(hObject, eventdata, handles)
% hObject    handle to loadws (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

vars = evalin('base','who');
for k = 1:1:length(vars)
    if strcmp(vars{k}, 'Measure')
        handles.Measure = evalin('base', vars{k});
        return;
    end
end
errordlg('导入数据失败！','Error','modal');
uicontrol(hObject);

% --------------------------------------------------------------------
function loadfile_Callback(hObject, eventdata, handles)
% hObject    handle to loadfile (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname] = uigetfile( ...
    {'*.mat', 'All MAT-Files (*.mat)'; ...
        '*.*','All Files (*.*)'}, ...
    'Select Measurement Data');
% If "Cancel" is selected then return
if isequal([filename,pathname],[0,0])
    return
    % Otherwise construct the full file name.
else
    File = fullfile(pathname,filename);
end

if exist(File,'file') == 2
    data = load(File);
end

flds = fieldnames(data);

for k = 1:1:length(flds)
    if strcmp(flds{k}, 'Measure')
        handles.Measure = data.Measure;
        return;
    end
end
errordlg('导入数据失败！','Error','modal');
uicontrol(hObject);
