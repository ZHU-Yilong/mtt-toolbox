function varargout = filterDef(varargin)
% FILTERDEF MATLAB code for filterDef.fig
%      FILTERDEF, by itself, creates a new FILTERDEF or raises the existing
%      singleton*.
%
%      H = FILTERDEF returns the handle to a new FILTERDEF or the handle to
%      the existing singleton*.
%
%      FILTERDEF('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FILTERDEF.M with the given input arguments.
%
%      FILTERDEF('Property','Value',...) creates a new FILTERDEF or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before filterDef_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to filterDef_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help filterDef

% Last Modified by GUIDE v2.5 10-Sep-2014 13:50:06

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @filterDef_OpeningFcn, ...
                   'gui_OutputFcn',  @filterDef_OutputFcn, ...
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


% --- Executes just before filterDef is made visible.
function filterDef_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to filterDef (see VARARGIN)

% Choose default command line output for filterDef


defaultFilter.Filter = 1;
defaultFilter.FilterParameters.T = 0.02;
defaultFilter.FilterParameters.SigmaRan = 450;
defaultFilter.FilterParameters.SigmaAzi = deg2rad(3);
defaultFilter.FilterParameters.SigmaEle = deg2rad(3);
defaultFilter.FilterHandle = filter1(defaultFilter.FilterParameters);

mainGuiInput = find(strcmp(varargin, 'filterIn'));
if (isempty(mainGuiInput)) ...
    || (length(varargin) <= mainGuiInput) ...
    || (~isstruct(varargin{mainGuiInput+1}))
    handles.curVal = defaultFilter;
    handles.tmpVal = defaultFilter;
else
    inputFilter = varargin{mainGuiInput+1};
    handles.curVal = inputFilter;
    handles.tmpVal = inputFilter;
end

% handles.tmpVal.Filter = 1;

% handles.tmpVal.FilterHandle = filter1(handles.tmpVal.FilterParameters);

guidata(hObject, handles);
updateUI(handles);

% UIWAIT makes filterDef wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = filterDef_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.curVal;
delete(hObject);



function updateUI(handles)
% radiobutton


% static text
switch handles.tmpVal.Filter
    case 1
        textStrings = sprintf('滤波器1：\n 运动模型：辛格模型 (alpha = 0.1)\n 观测模型：距离、方位、俯仰\n 滤 波 器 ：无迹卡尔曼滤波器 (UKF)\n');
        set(handles.filter1_radiobutton,'Value',1);
        set(handles.filter2_radiobutton,'Value',0);
        set(handles.filter3_radiobutton,'Value',0);
        set(handles.filter4_radiobutton,'Value',0);
        set(handles.filter5_radiobutton,'Value',0);
        set(handles.filterdefined_radiobutton,'Value',0);
    case 2
        textStrings = sprintf('滤波器2：\n 运动模型1：辛格模型 (alpha = 0.1)\n 运动模型2：辛格模型 (alpha = 0.9)\n 观测模型 ：距离、方位、俯仰\n 滤 波 器  ：交互多模滤波器 (IMM)\n');
        set(handles.filter1_radiobutton,'Value',0);
        set(handles.filter2_radiobutton,'Value',1);
        set(handles.filter3_radiobutton,'Value',0);
        set(handles.filter4_radiobutton,'Value',0);
        set(handles.filter5_radiobutton,'Value',0);
        set(handles.filterdefined_radiobutton,'Value',0);
    case 3
        textStrings = sprintf('滤波器3：\n 运动模型：辛格模型 (alpha = 0.1)\n 观测模型：距离、方位、俯仰\n 滤 波 器 ：概率数据关联滤波器 (PDAF)\n 参      数：\n        lambda = 0.0004;\n        gamma  = 16;\n        Pg     = 0.9997;\n        Pd     = 1;\n');
        set(handles.filter1_radiobutton,'Value',0);
        set(handles.filter2_radiobutton,'Value',0);
        set(handles.filter3_radiobutton,'Value',1);
        set(handles.filter4_radiobutton,'Value',0);
        set(handles.filter5_radiobutton,'Value',0);        
        set(handles.filterdefined_radiobutton,'Value',0);
    case 4
        textStrings = sprintf('滤波器4：\n 运动模型 ：辛格模型 (alpha = 0.1)\n 观测模型1：距离、方位、俯仰\n 观测模型2：距离、方位、俯仰\n 滤 波 器 ：多模概率数据关联滤波器 (MSPDAF)\n 参      数：\n        lambda = 0.0004; gamma  = 16;\n        Pg     = 0.9997; Pd     = 1;\n');
        set(handles.filter1_radiobutton,'Value',0);
        set(handles.filter2_radiobutton,'Value',0);
        set(handles.filter3_radiobutton,'Value',0);
        set(handles.filter4_radiobutton,'Value',1);
        set(handles.filter5_radiobutton,'Value',0);
        set(handles.filterdefined_radiobutton,'Value',0);
    case 5
        textStrings = sprintf('滤波器5：\n 运动模型1：辛格模型 (alpha = 0.1)\n 运动模型2：辛格模型 (alpha = 0.9)\n 观测模型1：距离、方位、俯仰\n 观测模型2：距离、方位、俯仰\n 滤 波 器 ：交互多模型/多模概率数据关联滤波器 (IMM/MSPDAF)\n 参      数：\n        lambda = 0.0004; gamma  = 16;\n        Pg     = 0.9997; Pd     = 1;\n');
        set(handles.filter1_radiobutton,'Value',0);
        set(handles.filter2_radiobutton,'Value',0);
        set(handles.filter3_radiobutton,'Value',0);
        set(handles.filter4_radiobutton,'Value',0);
        set(handles.filter5_radiobutton,'Value',1);
        set(handles.filterdefined_radiobutton,'Value',0);
    case 99
        textStrings = sprintf('自定义滤波器：\n 运动模型：辛格模型 (alpha = 0.1)\n 观测模型：距离、方位、俯仰\n 滤 波 器 ：转换观测的卡尔曼滤波器 (CMKF)\n\n 可自行修改滤波器定义文件，设置滤波器类型，但需要满足单个运动模型，单个观测源的条件。');
        set(handles.filter1_radiobutton,'Value',0);
        set(handles.filter2_radiobutton,'Value',0);
        set(handles.filter3_radiobutton,'Value',0);
        set(handles.filter4_radiobutton,'Value',0);
        set(handles.filter5_radiobutton,'Value',0);
        set(handles.filterdefined_radiobutton,'Value',1);
    otherwise
        
end
set(handles.filterDescription, 'String', textStrings);

% edit box


% --- Executes on button press in confirm.
function confirm_Callback(hObject, eventdata, handles)
% hObject    handle to confirm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.curVal = handles.tmpVal;
guidata(hObject,handles);
uiresume(handles.figure1);

% --- Executes on button press in save.
function save_Callback(hObject, eventdata, handles)
% hObject    handle to save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.curVal = handles.tmpVal;
guidata(hObject,handles);


% --- Executes on button press in cancel.
function cancel_Callback(hObject, eventdata, handles)
% hObject    handle to cancel (see GCBO)
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
        handles.curVal = handles.tmpVal;
        guidata(hObject,handles);
        uiresume(handles.figure1);
    case 'Quit'
        uiresume(handles.figure1);
    case 'Cancel'
        return;
end


% --- Executes on button press in filter1_radiobutton.
function filter1_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to filter1_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of filter1_radiobutton
% set(handles.filter2_radiobutton,'Value',0);
% set(handles.filter3_radiobutton,'Value',0);
% set(handles.filter4_radiobutton,'Value',0);
% set(handles.filter5_radiobutton,'Value',0);
% set(handles.filterdefined_radiobutton,'Value',0);

handles.tmpVal.Filter = 1;
guidata(hObject, handles);
updateUI(handles);



% --- Executes on button press in filter2_radiobutton.
function filter2_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to filter2_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of filter2_radiobutton
% set(handles.filter1_radiobutton,'Value',0);
% set(handles.filter3_radiobutton,'Value',0);
% set(handles.filter4_radiobutton,'Value',0);
% set(handles.filter5_radiobutton,'Value',0);
% set(handles.filterdefined_radiobutton,'Value',0);

handles.tmpVal.Filter = 2;
guidata(hObject, handles);
updateUI(handles);

% --- Executes on button press in filter3_radiobutton.
function filter3_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to filter3_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of filter3_radiobutton
% set(handles.filter1_radiobutton,'Value',0);
% set(handles.filter2_radiobutton,'Value',0);
% set(handles.filter4_radiobutton,'Value',0);
% set(handles.filter5_radiobutton,'Value',0);
% set(handles.filterdefined_radiobutton,'Value',0);

handles.tmpVal.Filter = 3;
guidata(hObject, handles);
updateUI(handles);

% --- Executes on button press in filter4_radiobutton.
function filter4_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to filter4_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of filter4_radiobutton
% set(handles.filter1_radiobutton,'Value',0);
% set(handles.filter2_radiobutton,'Value',0);
% set(handles.filter3_radiobutton,'Value',0);
% set(handles.filter5_radiobutton,'Value',0);
% set(handles.filterdefined_radiobutton,'Value',0);

handles.tmpVal.Filter = 4;
guidata(hObject, handles);
updateUI(handles);

% --- Executes on button press in filter5_radiobutton.
function filter5_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to filter5_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of filter5_radiobutton
% set(handles.filter1_radiobutton,'Value',0);
% set(handles.filter2_radiobutton,'Value',0);
% set(handles.filter3_radiobutton,'Value',0);
% set(handles.filter4_radiobutton,'Value',0);
% set(handles.filterdefined_radiobutton,'Value',0);

handles.tmpVal.Filter = 5;
guidata(hObject, handles);
updateUI(handles);

% --- Executes on button press in filterdefined_radiobutton.
function filterdefined_radiobutton_Callback(hObject, eventdata, handles)
% hObject    handle to filterdefined_radiobutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of filterdefined_radiobutton
% set(handles.filter1_radiobutton,'Value',0);
% set(handles.filter2_radiobutton,'Value',0);
% set(handles.filter3_radiobutton,'Value',0);
% set(handles.filter4_radiobutton,'Value',0);
% set(handles.filter5_radiobutton,'Value',0);

handles.tmpVal.Filter = 99;
guidata(hObject, handles);
updateUI(handles);


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

switch handles.tmpVal.Filter
    case 1
        evalin('base', 'open filter1.m');
        uicontrol(hObject);
    case 2
        open('filter2.m');
    case 3
        open('filter3.m');
    case 4
        open('filter4.m');
    case 5
        open('filter5.m');
    case 99
        open('filter99.m');
    otherwise
        
end
    
