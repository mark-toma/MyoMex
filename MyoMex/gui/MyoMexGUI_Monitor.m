function varargout = MyoMexGUI_Monitor(varargin)
% MYOMEXGUI_MONITOR MATLAB code for MyoMexGUI_Monitor.fig
%      MYOMEXGUI_MONITOR, by itself, creates a new MYOMEXGUI_MONITOR or raises the existing
%      singleton*.
%
%      H = MYOMEXGUI_MONITOR returns the handle to a new MYOMEXGUI_MONITOR or the handle to
%      the existing singleton*.
%
%      MYOMEXGUI_MONITOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYOMEXGUI_MONITOR.M with the given input arguments.
%
%      MYOMEXGUI_MONITOR('Property','Value',...) creates a new MYOMEXGUI_MONITOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MyoMexGUI_Monitor_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MyoMexGUI_Monitor_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MyoMexGUI_Monitor

% Last Modified by GUIDE v2.5 30-Mar-2016 19:19:12

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MyoMexGUI_Monitor_OpeningFcn, ...
                   'gui_OutputFcn',  @MyoMexGUI_Monitor_OutputFcn, ...
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


% --- Executes just before MyoMexGUI_Monitor is made visible.
function MyoMexGUI_Monitor_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MyoMexGUI_Monitor (see VARARGIN)

handles.myoMex = [];
handles.myoDataFig = nan(1,2);

% Choose default command line output for MyoMexGUI_Monitor
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MyoMexGUI_Monitor wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MyoMexGUI_Monitor_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function myoDataCloseCallback(hObject,id,handles)

if id==1
  set(handles.cb_myo_data_1,'value',0);
elseif id==2
  set(handles.cb_myo_data_2,'value',0);
end

handles.myoDataFig(id) = nan;

guidata(hObject,handles);

function cb_myo_data_Callback(hObject, eventdata, id, handles)

if get(hObject,'value')
  % spawn figure
  handles.myoDataFig(id) = MyoDataGUI_Monitor(...
    handles.myoMex.myoData(id),...
    @()myoDataCloseCallback(hObject,id,handles));
else
  % close figure
  close(handles.myoDataFig(id));
  handles.myoDataFig(id) = nan;
end

guidata(hObject,handles);



% --- Executes on button press in cb_myo_data_1.
function cb_myo_data_1_Callback(hObject, eventdata, handles)
% hObject    handle to cb_myo_data_1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cb_myo_data_1
cb_myo_data_Callback(hObject, eventdata, 1, handles);


% --- Executes on button press in cb_myo_data_2.
function cb_myo_data_2_Callback(hObject, eventdata, handles)
% hObject    handle to cb_myo_data_2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of cb_myo_data_2
cb_myo_data_Callback(hObject, eventdata, 2, handles);


function et_count_myos_Callback(hObject, eventdata, handles)
% hObject    handle to et_count_myos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of et_count_myos as text
%        str2double(get(hObject,'String')) returns contents of et_count_myos as a double
str = get(hObject,'string');
val = str2double(str);
if isnan(val) || ~any(val==[1,2])
  set(hObject,'string','1');
  wstr = sprintf('Number of Myos must be a numeric scalar in [1,2].\n\nYou entered ''%s''.\n\nReverting to default (1).',str),...
  warndlg(wstr,'Bad value','modal');
end




% --- Executes during object creation, after setting all properties.
function et_count_myos_CreateFcn(hObject, eventdata, handles)
% hObject    handle to et_count_myos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pb_init.
function pb_init_Callback(hObject, eventdata, handles)
% hObject    handle to pb_init (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

countMyos = str2double(get(handles.et_count_myos,'string'));

switch get(hObject,'string')
  case 'Init MyoMex'
    try
      handles.myoMex = MyoMex(countMyos);
    catch err
      warndlg(...
        sprintf('MyoMex(%d) failed with message:\n\n''''\n',countMyos,err.message),...
        'MyoMex Initialization Error','modal');
      return;
    end
    
    % here everything initialized fine
    set(handles.cb_myo_data_1,'enable','on');
    if countMyos>1
      set(handles.cb_myo_data_2,'enable','on');
    end
    
    set(handles.et_count_myos,'enable','off');
    set(hObject,'string','Delete MyoMex');
    
  case 'Delete MyoMex'
    
    handles.myoMex.delete();
    handles.myoMex = [];
    
    % close figures if they exist
    % kill myoData monitor figures
    for ii=1:length(handles.myoDataFig)
      if isnan(handles.myoDataFig(ii)), continue; end
      close(handles.myoDataFig(ii));
      handles.myoDataFig(ii) = nan;
    end
    
    % disable checkboxes
    set(handles.cb_myo_data_1,'value',0,'enable','off');
    set(handles.cb_myo_data_2,'value',0,'enable','off');
    
    % enable count myos et
    set(handles.et_count_myos,'enable','on');
    
    set(hObject,'string','Init MyoMex');
    
end

guidata(hObject,handles);

% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
if ~isempty(handles.myoMex)
  handles.myoMex.delete();
end

% kill myoData monitor figures
for ii=1:length(handles.myoDataFig)
  if isnan(handles.myoDataFig(ii)), continue; end
  close(handles.myoDataFig(ii));
  delete(handles.myoDataFig(ii));
end

% Hint: delete(hObject) closes the figure
delete(hObject);
