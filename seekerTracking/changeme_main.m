function varargout = changeme_main(varargin)
%CHANGEME_MAIN M-file for changeme_main.fig
%      CHANGEME_MAIN, by itself, creates a new CHANGEME_MAIN or raises the existing
%      singleton*.
%
%      H = CHANGEME_MAIN returns the handle to a new CHANGEME_MAIN or the handle to
%      the existing singleton*.
%
%      CHANGEME_MAIN('Property','Value',...) creates a new CHANGEME_MAIN using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to changeme_main_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CHANGEME_MAIN('CALLBACK') and CHANGEME_MAIN('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CHANGEME_MAIN.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help changeme_main

% Last Modified by GUIDE v2.5 07-Dec-2013 16:03:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @changeme_main_OpeningFcn, ...
                   'gui_OutputFcn',  @changeme_main_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
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


% --- Executes just before changeme_main is made visible.
function changeme_main_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for changeme_main
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% Set waiting flag in appdata
setappdata(handles.figure,'waiting',1)
% UIWAIT makes changeme_main wait for user response (see UIRESUME)
 uiwait(handles.figure);


% --- Outputs from this function are returned to the command line.
function varargout = changeme_main_OutputFcn...
                    (hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get pushbutton string from handles structure and output it
varargout{1} = get(handles.buttonChangeMe,'String');
% Now destroy yourself
delete(hObject);


% --- Executes on button press in buttonChangeMe.
function buttonChangeMe_Callback(hObject, eventdata, handles)
% hObject    handle to buttonChangeMe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Call the dialog to change button name giving this figure's handle
changeme_dialog('changeme_main', handles.figure);


% --- Executes when user attempts to close figure.
function figure_CloseRequestFcn(hObject,eventdata,handles)
% hObject    handle to figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Check appdata flag to see if the main GUI is in a wait state
if getappdata(handles.figure,'waiting')
    % The GUI is still in UIWAIT, so call UIRESUME and return
    uiresume(hObject);
    setappdata(handles.figure,'waiting',0)
else
    % The GUI is no longer waiting, so destroy it now.
    delete(hObject);
end
