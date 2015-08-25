function varargout = senarioDef(varargin)
% SENARIODEF MATLAB code for senarioDef.fig
%      SENARIODEF, by itself, creates a new SENARIODEF or raises the existing
%      singleton*.
%
%      H = SENARIODEF returns the handle to a new SENARIODEF or the handle to
%      the existing singleton*.
%
%      SENARIODEF('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SENARIODEF.M with the given input arguments.
%
%      SENARIODEF('Property','Value',...) creates a new SENARIODEF or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before senarioDef_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to senarioDef_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help senarioDef

% Last Modified by GUIDE v2.5 08-Dec-2013 20:39:33

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @senarioDef_OpeningFcn, ...
                   'gui_OutputFcn',  @senarioDef_OutputFcn, ...
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


% --- Executes just before senarioDef is made visible.
function senarioDef_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to senarioDef (see VARARGIN)

% Choose default command line output for senarioDef
%设置并储存默认场景
defaultSenario.tPos = [18;0;0];
defaultSenario.tVel = [600;15;0];
defaultSenario.tManu = [10;1;2;4];
defaultSenario.mMotion = [1200;1;0.2]; 
defaultSenario.decoy = [1;2;2];
defaultSenario.chkbox = logical([0,0,0,0]);
set(hObject,'UserData',defaultSenario); %储存默认场景

% Determine the position of the dialog - centered on the callback figure
% if available, else, centered on the screen
FigPos=get(0,'DefaultFigurePosition');
OldUnits = get(hObject, 'Units');
set(hObject, 'Units', 'pixels');
OldPos = get(hObject,'Position');
FigWidth = OldPos(3);
FigHeight = OldPos(4);
%%%%
if isempty(gcbf)  %是否由其他GUI打开
    ScreenUnits=get(0,'Units');
    set(0,'Units','pixels');
    ScreenSize=get(0,'ScreenSize');
    set(0,'Units',ScreenUnits);

    FigPos(1)=1/2*(ScreenSize(3)-FigWidth);
    FigPos(2)=2/3*(ScreenSize(4)-FigHeight);
else 
    GCBFOldUnits = get(gcbf,'Units');
    set(gcbf,'Units','pixels');
    GCBFPos = get(gcbf,'Position');
    set(gcbf,'Units',GCBFOldUnits);
    FigPos(1:2) = [(GCBFPos(1) + GCBFPos(3) / 2) - FigWidth / 2, ...
                   (GCBFPos(2) + GCBFPos(4) / 2) - FigHeight / 2];
end
FigPos(3:4)=[FigWidth FigHeight];
set(hObject, 'Position', FigPos);
set(hObject, 'Units', OldUnits);

%%%%%%%%%%%%%%%%%%%%%
%读取主GUI的参数输入
mainGuiInput = find(strcmp(varargin, 'senarioIn')); %寻找初始输入场景对应字符串
if (isempty(mainGuiInput)) ...
    || (length(varargin) <= mainGuiInput) ...
    || (~isstruct(varargin{mainGuiInput+1}))
   %没有合法场景输入，用默认场景初始化输出
    handles.curVal = defaultSenario; 
    handles.curVal.tPos = 1000*handles.curVal.tPos; %km转换为m
    handles.curVal.tVel(2:3) = deg2rad(handles.curVal.tVel(2:3));%转为rad
    handles.curVal.tManu(1) =  handles.curVal.tManu(1)*9.8; %转为m/s^2
    handles.temp = defaultSenario;%用默认场景初始化临时显示变量 
else
    %有来自主GUI的场景输入，用其初始化
    inputSenario = varargin{mainGuiInput+1};
     %将输入场景设为默认输出场景
    handles.curVal = inputSenario;
    %用输入场景初始化临时显示变量
    handles.temp = inputSenario;
    handles.temp.tPos = handles.temp.tPos/1000; %km转换为km
    handles.temp.tVel(2:3) = rad2deg(handles.temp.tVel(2:3));%转为deg
    handles.temp.tManu(1) =  handles.temp.tManu(1)/9.8; %转为g
end

% Update handles structure
guidata(hObject, handles);

%Update initialized senario
updateUI(handles);

% UIWAIT makes senarioDef wait for user response (see UIRESUME)
uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = senarioDef_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.curVal;
delete(hObject);


function tSojournTime_Callback(hObject, eventdata, handles)
% hObject    handle to tSojournTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tSojournTime as text
%        str2double(get(hObject,'String')) returns contents of tSojournTime as a double
tSojournTime = str2double(get(hObject, 'String'));     
if isnan(tSojournTime) || (tSojournTime < 0)
    %
    set(hObject, 'String', num2str(handles.temp.tManu(4)));
    errordlg('Please input a positive real number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value 
    handles.temp.tManu(4) = tSojournTime;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function tSojournTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tSojournTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tManuStart_Callback(hObject, eventdata, handles)
% hObject    handle to tManuStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tManuStart as text
%        str2double(get(hObject,'String')) returns contents of tManuStart as a double
tManuStart = str2double(get(hObject, 'String'));     
if isnan(tManuStart) || (tManuStart < 0)
    %
    set(hObject, 'String', num2str(handles.temp.tManu(3)));
    errordlg('Please input a positive real number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value 
    handles.temp.tManu(3) = tManuStart;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function tManuStart_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tManuStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tMaxLoad_Callback(hObject, eventdata, handles)
% hObject    handle to tMaxLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tMaxLoad as text
%        str2double(get(hObject,'String')) returns contents of tMaxLoad as a double
tMaxLoad = str2double(get(hObject, 'String'));     
if isnan(tMaxLoad) || (tMaxLoad < 0)
    %
    set(hObject, 'String', num2str(handles.temp.tManu(1)));
    errordlg('Please input a positive real number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value 
    handles.temp.tManu(1) = tMaxLoad;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function tMaxLoad_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tMaxLoad (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in tManuRandom.
function tManuRandom_Callback(hObject, eventdata, handles)
% hObject    handle to tManuRandom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of tManuRandom
if (get(hObject,'Value') == get(hObject,'Max'))
    %目标逃逸方向和逃逸时间随机，禁用逃逸方向、起始时间、逗留时间输入
    tManuRandom = true;
    set([handles.tManu_popupmenu,handles.tManuStart,handles.tSojournTime],...
        'Enable','off');    
else
    % 目标初始速度矢量确定，启用航向角与航迹倾角输入
     tManuRandom = false;
    set([handles.tManu_popupmenu,handles.tManuStart,handles.tSojournTime],...
        'Enable','on');
end
handles.temp.chkbox(2) = tManuRandom; %缓存输入
guidata(hObject,handles);

% --- Executes on selection change in tManu_popupmenu.
function tManu_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to tManu_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents =  returns tManu_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from tManu_popupmenu
%contents = cellstr(get(hObject,'String'));
handles.temp.tManu(2) = get(hObject,'Value'); %缓存输入
guidata(hObject,handles);



% --- Executes during object creation, after setting all properties.
function tManu_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tManu_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in tVRandom.
function tVRandom_Callback(hObject, eventdata, handles)
% hObject    handle to tVRandom (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of tVRandom
if (get(hObject,'Value') == get(hObject,'Max'))
    %目标初始速度矢量随机，禁用航向角与航迹倾角输入
    tVRandom = true;
    set([handles.tYaw,handles.tPitch],'Enable','off');    
else
    % 目标初始速度矢量确定，启用航向角与航迹倾角输入
    tVRandom = false;
    set([handles.tYaw,handles.tPitch],'Enable','on');
end
handles.temp.chkbox(1) = tVRandom; %缓存输入
guidata(hObject,handles);


function tSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to tSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tSpeed as text
%        str2double(get(hObject,'String')) returns contents of tSpeed as a double
tSpeed = str2double(get(hObject, 'String'));     
if isnan(tSpeed) || (tSpeed <= 0) || (tSpeed >= handles.temp.mMotion(1))
    %
    set(hObject, 'String', num2str(handles.temp.tVel(1)));
    errordlg('目标速度(Speed)必须为正且小于平台速度','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.tVel(1) = tSpeed;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function tSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tYaw_Callback(hObject, eventdata, handles)
% hObject    handle to tYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tYaw as text
%        str2double(get(hObject,'String')) returns contents of tYaw as a double
tYaw = str2double(get(hObject, 'String'));     
if isnan(tYaw) || (tYaw < 0) || (tYaw > 360)
    %
    set(hObject, 'String', num2str(handles.temp.tVel(2)));
    errordlg(['航向角必须在0-360度之间'],'Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.tVel(2) = tYaw;
    guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function tYaw_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tYaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tPitch_Callback(hObject, eventdata, handles)
% hObject    handle to tPitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tPitch as text
%        str2double(get(hObject,'String')) returns contents of tPitch as a double
tPitch = str2double(get(hObject, 'String'));     
if isnan(tPitch) || (abs(tPitch)>90)
    %
    set(hObject, 'String', num2str(handles.temp.tVel(3)));
    errordlg(['航向角必须在-90度到90度之间'],'Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.tVel(3) = tPitch;
    guidata(hObject,handles);
end

% --- Executes during object creation, after setting all properties.
function tPitch_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tPitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function zPosition_Callback(hObject, eventdata, handles)
% hObject    handle to zPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of zPosition as text
%        str2double(get(hObject,'String')) returns contents of zPosition as a double
zPosition = str2double(get(hObject, 'String'));     
if isnan(zPosition) 
    %
    set(hObject, 'String', num2str(handles.temp.tPos(3)));
    errordlg('Input must be a number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.tPos(3) = zPosition;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function zPosition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to zPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yPosition_Callback(hObject, eventdata, handles)
% hObject    handle to yPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yPosition as text
%        str2double(get(hObject,'String')) returns contents of yPosition as a double
yPosition = str2double(get(hObject, 'String'));     
if isnan(yPosition) 
    %
    set(hObject, 'String', num2str(handles.temp.tPos(2)));
    errordlg('Input must be a number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.tPos(2) = yPosition;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function yPosition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xPosition_Callback(hObject, eventdata, handles)
% hObject    handle to xPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xPosition as text
%        str2double(get(hObject,'String')) returns contents of xPosition as a double
xPosition = str2double(get(hObject, 'String'));     
if isnan(xPosition) 
    %
    set(hObject, 'String', num2str(handles.temp.tPos(1)));
    errordlg('Input must be a number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.tPos(1) = xPosition;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function xPosition_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xPosition (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in savePB.
function savePB_Callback(hObject, eventdata, handles)
% hObject    handle to savePB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
choice = questdlg('Are you sure to save current input contents?', ...
	'SaveDialog','Yes', 'No','Yes');
% Handle response
switch choice
    case 'Yes' %保存temp变量值至current,单位全部转换为SI
        handles.curVal = handles.temp;        
        handles.curVal.tPos = 1000*handles.curVal.tPos; %km转换为m
        handles.curVal.tVel(2:3) = deg2rad(handles.curVal.tVel(2:3));%转为rad
        handles.curVal.tManu(1) =  handles.curVal.tManu(1)*9.8; %转为m/s^2 
        guidata(hObject,handles);
    case 'No' %保存变量值
        return;
end

% --- Executes on button press in resetPB.
function resetPB_Callback(hObject, eventdata, handles)
% hObject    handle to resetPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
choice = questdlg('Which senario do you want to reset to?', ...
	'ResetSenarioDialog','Default', 'Last Saved','Cancel','Last Saved');
% Handle response
switch choice
    case 'Default' %复位至默认值,,并更新GUI输入
        handles.temp = get(handles.figure1,'UserData');        
        guidata(hObject,handles);
        updateUI(handles);
    case  'Last Saved' %复位至上次保存值或初始值,并更新GUI输入
        handles.temp = handles.curVal;        
        handles.temp.tPos = handles.temp.tPos/1000; %km转换为m
        handles.temp.tVel(2:3) = rad2deg(handles.temp.tVel(2:3));%转为rad
        handles.temp.tManu(1) =  handles.temp.tManu(1)/9.8; %转为m/s^2 
        guidata(hObject,handles);
        updateUI(handles);
    case 'Cancel' %保存变量值        
        return;
end

% --- Executes on button press in quitPB.
function quitPB_Callback(hObject, eventdata, handles)
% hObject    handle to quitPB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
myQuit(hObject, handles);

% --- Executes on selection change in decoyType_popupmenu.
function decoyType_popupmenu_Callback(hObject, eventdata, handles)
% hObject    handle to decoyType_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns decoyType_popupmenu contents as cell array
%        contents{get(hObject,'Value')} returns selected item from decoyType_popupmenu
handles.temp.decoy(1) = get(hObject,'Value'); %缓存输入
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function decoyType_popupmenu_CreateFcn(hObject, eventdata, handles)
% hObject    handle to decoyType_popupmenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in decoyChkBox.
function decoyChkBox_Callback(hObject, eventdata, handles)
% hObject    handle to decoyChkBox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of decoyChkBox
if (get(hObject,'Value') == get(hObject,'Max'))
    %存在干扰，启用相关参数设置UI
    decoyExsit = true;
    set([handles.decoyType_popupmenu,handles.decoyStart,handles.decoySojournTime],...
       'Enable','on');
else
    %场景中无干扰，禁用干扰参数输入UI
   decoyExsit = false;
   set([handles.decoyType_popupmenu,handles.decoyStart,handles.decoySojournTime],...
        'Enable','off');
end
handles.temp.chkbox(4) = decoyExsit; %缓存输入
guidata(hObject,handles);


function decoyStart_Callback(hObject, eventdata, handles)
% hObject    handle to decoyStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of decoyStart as text
%        str2double(get(hObject,'String')) returns contents of decoyStart as a double
decoyStart = str2double(get(hObject, 'String'));     
if isnan(decoyStart) || (decoyStart <= 0)
    %
    set(hObject, 'String', num2str(handles.temp.decoy(2)));
    errordlg('Please input a positive number.','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.decoy(2) = decoyStart;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function decoyStart_CreateFcn(hObject, eventdata, handles)
% hObject    handle to decoyStart (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function decoySojournTime_Callback(hObject, eventdata, handles)
% hObject    handle to decoySojournTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of decoySojournTime as text
%        str2double(get(hObject,'String')) returns contents of decoySojournTime as a double
decoyST = str2double(get(hObject, 'String'));     
if isnan(decoyST) || (decoyST <= 0)
    %
    set(hObject, 'String', num2str(handles.temp.decoy(3)));
    errordlg('Please input a positive number.','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.decoy(3) = decoyST;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function decoySojournTime_CreateFcn(hObject, eventdata, handles)
% hObject    handle to decoySojournTime (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function mSpeed_Callback(hObject, eventdata, handles)
% hObject    handle to mSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mSpeed as text
%        str2double(get(hObject,'String')) returns contents of mSpeed as a double
mSpeed = str2double(get(hObject, 'String'));     
if isnan(mSpeed) || (mSpeed<=0) || (mSpeed <= handles.temp.tVel(1))
    %
    set(hObject, 'String', num2str(handles.temp.mMotion(1)));
    errordlg('平台速度(speed)必须为正且大于目标速度','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.mMotion(1) = mSpeed;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function mSpeed_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mSpeed (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in mMotionless.
function mMotionless_Callback(hObject, eventdata, handles)
% hObject    handle to mMotionless (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of mMotionless
if (get(hObject,'Value') == get(hObject,'Max'))
    %平台静止，禁用平台运动参数设置输入UI
    mMotionless = true;
    set([handles.mGL,handles.mSpeed,handles.mCtrlTimeConst],...
        'Enable','off');    
else
    % 平台按设定导引律运动，启用相关参数设置UI
     mMotionless = false;
    set([handles.mGL,handles.mSpeed,handles.mCtrlTimeConst],...
        'Enable','on');
end
handles.temp.chkbox(3) = mMotionless; %缓存输入
guidata(hObject,handles);

% --- Executes on selection change in mGL.
function mGL_Callback(hObject, eventdata, handles)
% hObject    handle to mGL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns mGL contents as cell array
%        contents{get(hObject,'Value')} returns selected item from mGL
handles.temp.mMotion(2) = get(hObject,'Value'); %缓存输入
guidata(hObject,handles);

% --- Executes during object creation, after setting all properties.
function mGL_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mGL (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function mCtrlTimeConst_Callback(hObject, eventdata, handles)
% hObject    handle to mCtrlTimeConst (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of mCtrlTimeConst as text
%        str2double(get(hObject,'String')) returns contents of mCtrlTimeConst as a double
mCtrlTimeConst = str2double(get(hObject, 'String'));     
if isnan(mCtrlTimeConst) || (mCtrlTimeConst<=0)
    %
    set(hObject, 'String', num2str(handles.temp.mMotion(3)));
    errordlg('Input must be a positive number','Error','modal');
    uicontrol(hObject);
else
    % Save the new input value  
    handles.temp.mMotion(3) = mCtrlTimeConst;
    guidata(hObject,handles);
end


% --- Executes during object creation, after setting all properties.
function mCtrlTimeConst_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mCtrlTimeConst (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure
myQuit(hObject, handles);

%%
%Customized Function
function updateUI(handles)
% handles   structure with handles and user data (see GUIDATA)
%This function use handles.temp update the input ui controls

%Postion Update
set(handles.xPosition, 'String', num2str(handles.temp.tPos(1)));
set(handles.yPosition, 'String', num2str(handles.temp.tPos(2)));
set(handles.zPosition, 'String', num2str(handles.temp.tPos(3)));

%Velocity Update
set(handles.tSpeed, 'String', num2str(handles.temp.tVel(1)));
if handles.temp.chkbox(1)%随机场景
    set(handles.tVRandom,'Value',get(handles.tVRandom,'Max'));
    set(handles.tYaw, 'String', num2str(handles.temp.tVel(2)),...
        'Enable','off');
    set(handles.tPitch, 'String', num2str(handles.temp.tVel(3)),...
        'Enable','off');
else %确定性场景
    set(handles.tVRandom,'Value',get(handles.tVRandom,'Min'));
    set(handles.tYaw, 'String', num2str(handles.temp.tVel(2)),...
        'Enable','on');
    set(handles.tPitch, 'String', num2str(handles.temp.tVel(3)),...
        'Enable','on');
end

%Target Maneuvering Update
set(handles.tMaxLoad, 'String', num2str(handles.temp.tManu(1)));
if handles.temp.chkbox(2)%随机场景
    set(handles.tManuRandom,'Value',get(handles.tManuRandom,'Max'));
    set(handles.tManu_popupmenu,'Value',handles.temp.tManu(2),...
        'Enable','off');
    set(handles.tManuStart, 'String', num2str(handles.temp.tManu(3)),...
        'Enable','off');
    set(handles.tSojournTime, 'String', num2str(handles.temp.tManu(4)),...
        'Enable','off');
else
    set(handles.tManuRandom,'Value',get(handles.tManuRandom,'Min'));
    set(handles.tManu_popupmenu,'Value',handles.temp.tManu(2),...
        'Enable','on');
    set(handles.tManuStart, 'String', num2str(handles.temp.tManu(3)),...
        'Enable','on');
    set(handles.tSojournTime, 'String', num2str(handles.temp.tManu(4)),...
        'Enable','on');    
end

%Platform Motion Update
if handles.temp.chkbox(3)%平台静止
    set(handles.mMotionless,'Value',get(handles.mMotionless,'Max'));
    set(handles.mSpeed, 'String', num2str(handles.temp.mMotion(1)),...
        'Enable','off');
    set(handles.mGL, 'Value',handles.temp.mMotion(2),...
        'Enable','off');
    set(handles.mCtrlTimeConst, 'String', num2str(handles.temp.mMotion(3)),...
        'Enable','off');    
else %平台运动
    set(handles.mMotionless,'Value',get(handles.mMotionless,'Min'));
    set(handles.mSpeed, 'String', num2str(handles.temp.mMotion(1)),...
        'Enable','on');
    set(handles.mGL, 'Value',handles.temp.mMotion(2),...
        'Enable','on');
    set(handles.mCtrlTimeConst, 'String', num2str(handles.temp.mMotion(3)),...
        'Enable','on');
end
    
%Decoy Update
if handles.temp.chkbox(4) %存在干扰
    set(handles.decoyChkBox,'Value',get(handles.decoyChkBox,'Max'));
    set(handles.decoyType_popupmenu,'Value',handles.temp.decoy(1),...
        'Enable','on');
    set(handles.decoyStart, 'String', num2str(handles.temp.decoy(2)),...
        'Enable','on');
    set(handles.decoySojournTime, 'String', num2str(handles.temp.decoy(3)),...
        'Enable','on');  
else
    set(handles.decoyChkBox,'Value',get(handles.decoyChkBox,'Min'));
    set(handles.decoyType_popupmenu,'Value',handles.temp.decoy(1),...
        'Enable','off');
    set(handles.decoyStart, 'String', num2str(handles.temp.decoy(2)),...
        'Enable','off');
    set(handles.decoySojournTime, 'String', num2str(handles.temp.decoy(3)),...
        'Enable','off');
end


%---
function myQuit(hObject, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
choice = questdlg('Are you sure to quit?', ...
	'QuitDialog','Save&Quit', 'Quit','Cancel','Quit');
switch choice
    case 'Save&Quit'
        handles.curVal = handles.temp;        
        handles.curVal.tPos = 1000*handles.curVal.tPos; %km转换为m
        handles.curVal.tVel(2:3) = deg2rad(handles.curVal.tVel(2:3));%转为rad
        handles.curVal.tManu(1) =  handles.curVal.tManu(1)*9.8; %转为m/s^2 
        guidata(hObject,handles);
        uiresume(handles.figure1);
    case 'Quit'
        uiresume(handles.figure1);
    case 'Cancel'
        return;
end
