function varargout = changeme_dialog(varargin)
%CHANGEME_DIALOG M-file for changeme_dialog.fig
%      CHANGEME_DIALOG, by itself, creates a new CHANGEME_DIALOG or raises the existing
%      singleton*.
%
%      H = CHANGEME_DIALOG returns the handle to a new CHANGEME_DIALOG or the handle to
%      the existing singleton*.
%
%      CHANGEME_DIALOG('Property','Value',...) creates a new CHANGEME_DIALOG using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to changeme_dialog_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      CHANGEME_DIALOG('CALLBACK') and CHANGEME_DIALOG('CALLBACK',hObject,...) call the
%      local function named CALLBACK in CHANGEME_DIALOG.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help changeme_dialog

% Last Modified by GUIDE v2.5 07-Dec-2013 16:03:58

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @changeme_dialog_OpeningFcn, ...
                   'gui_OutputFcn',  @changeme_dialog_OutputFcn, ...
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


% --- Executes just before changeme_dialog is made visible.
function changeme_dialog_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Is the changeme_main gui's handle is passed in varargin?
% if the name 'changeme_main' is found, and the next argument
% varargin{mainGuiInput+1} is a handle, assume we can open it.

dontOpen = false;
mainGuiInput = find(strcmp(varargin, 'changeme_main'));
if (isempty(mainGuiInput)) ...
    || (length(varargin) <= mainGuiInput) ...
    || (~ishandle(varargin{mainGuiInput+1}))
    dontOpen = true;
else
    % Remember the handle, and adjust our position
    handles.changeMeMain = varargin{mainGuiInput+1};
    
    % Obtain handles using GUIDATA with the caller's handle 
    mainHandles = guidata(handles.changeMeMain);
    % Set the edit text to the String of the main GUI's button
    set(handles.editChangeMe, 'String', ...
        get(mainHandles.buttonChangeMe, 'String'));
    
    % Position to be relative to parent:
    parentPosition = getpixelposition(handles.changeMeMain);
    currentPosition = get(hObject, 'Position');  
    % Set x to be directly in the middle, and y so that their tops align.
    newX = parentPosition(1) + (parentPosition(3)/2 - currentPosition(3)/2);
    newY = parentPosition(2) + (parentPosition(4)/2 - currentPosition(4)/2);
    %newY = parentPosition(2) + (parentPosition(4) - currentPosition(4));
    newW = currentPosition(3);
    newH = currentPosition(4);
    
    set(hObject, 'Position', [newX, newY, newW, newH]);
end

% Update handles structure
guidata(hObject, handles);

if dontOpen
   disp('-----------------------------------------------------');
   disp('Improper input arguments. Pass a property value pair') 
   disp('whose name is "changeme_main" and value is the handle')
   disp('to the changeme_main figure, e.g:');
   disp('   x = changeme_main()');
   disp('   changeme_dialog(''changeme_main'', x)');
   disp('-----------------------------------------------------');
else
   uiwait(hObject);
end


% --- Outputs from this function are returned to the command line.
function varargout = changeme_dialog_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

varargout{1} = [];
delete(hObject);


% --- Executes during object creation, after setting all properties.
function editChangeMe_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editChangeMe (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in buttonOK.
function buttonOK_Callback(hObject, eventdata, handles)
% hObject    handle to buttonOK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
text = get(handles.editChangeMe, 'String');
main = handles.changeMeMain;
% Obtain handles using GUIDATA with the caller's handle 
if(ishandle(main))
    mainHandles = guidata(main);
    changeMeButton = mainHandles.buttonChangeMe;
    set(changeMeButton, 'String', text);
end
uiresume(handles.figure);

% --- Executes on button press in buttonCancel.
function buttonCancel_Callback(hObject, eventdata, handles)
% hObject    handle to buttonCancel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume(handles.figure);

% --- Executes when user attempts to close figure.
function figure_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
uiresume(hObject);
