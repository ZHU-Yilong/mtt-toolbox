function varargout = sensorDef(varargin)
% SENSORDEF MATLAB code for sensorDef.fig
%      SENSORDEF, by itself, creates a new SENSORDEF or raises the existing
%      singleton*.
%
%      H = SENSORDEF returns the handle to a new SENSORDEF or the handle to
%      the existing singleton*.
%
%      SENSORDEF('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SENSORDEF.M with the given input arguments.
%
%      SENSORDEF('Property','Value',...) creates a new SENSORDEF or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before sensorDef_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to sensorDef_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help sensorDef

% Last Modified by GUIDE v2.5 12-Sep-2014 12:56:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @sensorDef_OpeningFcn, ...
                   'gui_OutputFcn',  @sensorDef_OutputFcn, ...
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


% --- Executes just before sensorDef is made visible.
function sensorDef_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sensorDef (see VARARGIN)


% Choose default command line output for sensorDef
% 
defaultSensor.Sensors       = 0;
defaultSensor.radarChkbox   = [true;true;true;false];
defaultSensor.radarView     = [450;3;3;20];
defaultSensor.radarSigma    = [15;0.3;0.3;0.5];
defaultSensor.infraredView  = [2;2];
defaultSensor.infraredPixel = [64;64];
defaultSensor.infraredSigma = [0.03;0.03];
set(hObject,'UserData',defaultSensor);

%读取主GUI的参数输入
mainGuiInput = find(strcmp(varargin, 'sensorIn')); %寻找初始输入场景对应字符串
if (isempty(mainGuiInput)) ...
    || (length(varargin) <= mainGuiInput) ...
    || (~isstruct(varargin{mainGuiInput+1}))
   %没有合法场景输入，用默认场景初始化输出
    handles.curVal = defaultSensor; 
%     handles.curVal.radarView(3:4)  = deg2rad(handles.curVal.radarView(3:4)); %转为rad
%     handles.curVal.radarSigma(3:4) = deg2rad(handles.curVal.radarView(3:4)); %转为rad
    handles.curVal.infraredView    = deg2rad(handles.curVal.infraredView);
    handles.curVal.infraredSigma   = deg2rad(handles.curVal.infraredSigma);
    
    handles.tmpVal = defaultSensor;%用默认场景初始化临时显示变量 
else
    %有来自主GUI的场景输入，用其初始化
    inputSenario = varargin{mainGuiInput+1};
     %将输入场景设为默认输出场景
    handles.curVal = inputSenario;    
    %用输入场景初始化临时显示变量
    handles.tmpVal = inputSenario;
%     handles.tmpVal.radarView(3:4)  = rad2deg(handles.tmpVal.radarView(3:4)); %转为deg
%     handles.tmpVal.radarSigma(3:4) = rad2deg(handles.tmpVal.radarView(3:4)); %转为deg
    handles.tmpVal.infraredView    = rad2deg(handles.tmpVal.infraredView);
    handles.tmpVal.infraredSigma   = rad2deg(handles.tmpVal.infraredSigma);
end

% Update handles structure
guidata(hObject, handles);

%Update initialized senario
updateUI(handles);

% UIWAIT makes sensorDef wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = sensorDef_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.curVal;
delete(hObject);


% --- Executes on button press in confirm_pb.
function confirm_pb_Callback(hObject, eventdata, handles)
% hObject    handle to confirm_pb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%保存数据
handles.curVal = handles.tmpVal;
guidata(hObject,handles);
uiresume(handles.figure1);

% --- Executes on button press in cancel_pb.
function cancel_pb_Callback(hObject, eventdata, handles)
% hObject    handle to cancel_pb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 uiresume(handles.figure1);

 
 % --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
choice = questdlg('Are you sure to quit?', ...
	'QuitDialog','Save&Quit', 'Quit','Cancel','Quit');
switch choice
    case 'Save&Quit'
        %保存数据
        uiresume(handles.figure1);
    case 'Quit'
        uiresume(handles.figure1);
    case 'Cancel'
        return;
end
% Hint: delete(hObject) closes the figure
%delete(hObject);



% --- Executes on button press in single_rb.
function single_rb_Callback(hObject, eventdata, handles)
% hObject    handle to single_rb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of single_rb
handles.tmpVal.Sensors = 0;
% set(handles.single_rb, 'Value', 1);
% set(handles.double_rb, 'Value', 0);
guidata(hObject,handles);
updateUI(handles)


% --- Executes on button press in double_rb.
function double_rb_Callback(hObject, eventdata, handles)
% hObject    handle to double_rb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of double_rb
handles.tmpVal.Sensors = 1;
% set(handles.single_rb, 'Value', 0);
% set(handles.double_rb, 'Value', 1);
guidata(hObject,handles);
updateUI(handles)


%%%%%%%%%%%
function updateUI(handles)

if handles.tmpVal.Sensors           % 双模传感器
    % radio button
    set(handles.single_rb, 'Value', 0);
    set(handles.double_rb, 'Value', 1);
    % uitable
    set(handles.infrared_uitable, 'Enable', 'on');
else                             % 单模传感器
    set(handles.single_rb, 'Value', 1);
    set(handles.double_rb, 'Value', 0);
    % infrared_uitable
    set(handles.infrared_uitable, 'Enable', 'off');
end

% data = cell(4,3);
data = [handles.tmpVal.radarChkbox, handles.tmpVal.radarView, handles.tmpVal.radarSigma];
data = num2cell(data);
data{1,1} = logical(data{1,1});
data{2,1} = logical(data{2,1});
data{3,1} = logical(data{3,1});
data{4,1} = logical(data{4,1});
set(handles.radar_uitable, 'Data', data);


% --- Executes when entered data in editable cell(s) in radar_uitable.
function radar_uitable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to radar_uitable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
if(eventdata.Indices(2) ~= 1)
    temp = str2double(eventdata.EditData);     
    if isnan(temp) || (temp < 0)
        %
        %set(hObject, 'String', num2str(handles.temp.tVel(1)));
        errordlg('设置数值错误！','Error','modal');
        uicontrol(hObject);
        return;
    end
    data = get(handles.radar_uitable, 'Data');
    handles.tmpVal.radarChkbox = cell2mat(data(:,1));
    handles.tmpVal.radarView   = cell2mat(data(:,2));
    handles.tmpVal.radarSigma  = cell2mat(data(:,3));
    guidata(hObject, handles);
end


% --- Executes during object creation, after setting all properties.
function radar_uitable_CreateFcn(hObject, eventdata, handles)
% hObject    handle to radar_uitable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called


% --- Executes when entered data in editable cell(s) in infrared_uitable.
function infrared_uitable_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to infrared_uitable (see GCBO)
% eventdata  structure with the following fields (see UITABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

temp = str2double(eventdata.EditData);     
if isnan(temp) || (temp < 0)
    errordlg('设置数值错误！','Error','modal');
    uicontrol(hObject);
    return;
end
data = get(handles.infrared_uitable, 'Data');
handles.tmpVal.infraredView   = cell2mat(data(:,1));
handles.tmpVal.infraredPixel  = cell2mat(data(:,2));
handles.tmpVal.infraredSigma  = cell2mat(data(:,3));
guidata(hObject, handles);
