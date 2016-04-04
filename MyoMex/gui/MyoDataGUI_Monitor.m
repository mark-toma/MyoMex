function varargout = MyoDataGUI_Monitor(varargin)
% MYODATAGUI_MONITOR MATLAB code for MyoDataGUI_Monitor.fig
%      MYODATAGUI_MONITOR, by itself, creates a new MYODATAGUI_MONITOR or raises the existing
%      singleton*.
%
%      H = MYODATAGUI_MONITOR returns the handle to a new MYODATAGUI_MONITOR or the handle to
%      the existing singleton*.
%
%      MYODATAGUI_MONITOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MYODATAGUI_MONITOR.M with the given input arguments.
%
%      MYODATAGUI_MONITOR('Property','Value',...) creates a new MYODATAGUI_MONITOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MyoDataGUI_Monitor_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MyoDataGUI_Monitor_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MyoDataGUI_Monitor

% Last Modified by GUIDE v2.5 30-Mar-2016 16:51:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 0;
gui_State = struct('gui_Name',       mfilename, ...
  'gui_Singleton',  gui_Singleton, ...
  'gui_OpeningFcn', @MyoDataGUI_Monitor_OpeningFcn, ...
  'gui_OutputFcn',  @MyoDataGUI_Monitor_OutputFcn, ...
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


% --- Executes just before MyoDataGUI_Monitor is made visible.
function MyoDataGUI_Monitor_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MyoDataGUI_Monitor (see VARARGIN)

% constants
handles.const.STRIP_TIME = 2;
handles.const.UPDATE_RATE = 25; % Hz
handles.const.UPDATE_START_DELAY = 1; % s
handles.const.STREAMING_DATA_TIME = 0.020; % [s] (50 Hz)
handles.const.STREAMING_FRAME_TIME = 0.040; % [s] (25 Hz)

handles.const.QUAT_STRIP_YLIM = [-1,1];
handles.const.GYRO_STRIP_YLIM = 2000*[-1,1];
handles.const.ACCEL_STRIP_YLIM = 2*[-1,1];
handles.const.EMG_STRIP_YLIM = [-1,1.2];

handles.const.POSE_YDATA_VALUE = 1.1;
handles.const.NUM_POSES = 5;

handles.const.ROTATE_OFFSET = -135;

% process inputs
argv = varargin;
handles.myoData = [];
handles.closeFunc = [];

assert(length(argv)>0 && isa(argv{1},'MyoData') && isscalar(argv{1}),...
  'Input (1) must be a scalar MyoData object');
handles.myoData = argv{1};
if length(argv)>1 && isa(argv{2},'function_handle')
  handles.closeFunc = argv{2};
end

% load image resources
handles.image_pose_enabled = false;

res_path = fullfile(fileparts(mfilename('fullpath')),'..','resources');
if exist(res_path,'dir')
  handles.image_pose_enabled = true;
  handles.image_pose_fist             = imread(fullfile(res_path,'image_pose_fist.png'));
  handles.image_pose_wave_in          = imread(fullfile(res_path,'image_pose_wave_in.png'));
  handles.image_pose_wave_out         = imread(fullfile(res_path,'image_pose_wave_out.png'));
  handles.image_pose_fingers_spread   = imread(fullfile(res_path,'image_pose_fingers_spread.png'));
  handles.image_pose_double_tap       = imread(fullfile(res_path,'image_pose_double_tap.png'));
end

% set up quaternion orientation
set(handles.ax_quat_orientation,...
  'nextplot','add',...
  'xlim',[-1,1],'ylim',[-1,1],'zlim',[-1,1],...
  'xtick',[],'ytick',[],'ztick',[],...
  'xcolor',get(hObject,'color'),...
  'ycolor',get(hObject,'color'),...
  'zcolor',get(hObject,'color'),...
  'dataaspectratio',[1,1,1],...
  'color',get(hObject,'color'));
view(handles.ax_quat_orientation,...
  -handles.const.ROTATE_OFFSET,30);

hp.hx_global = hgtransform('parent',handles.ax_quat_orientation);
hp.hx_body   = hgtransform('parent',hp.hx_global);

% plot body object into hx_body
% [xb,yb,zb] = sphere(50);
[xb,yb,zb] = cylinder(0.2,20);
hbody = surf(-zb,yb,xb,'parent',hp.hx_body,...
  'facecolor','w',...
  'facealpha',0.8,...
  'edgecolor','k');

% global coordinate axes
plot3(handles.ax_quat_orientation,...
  [0,1],[0,0],[0,0],'r','linewidth',2,'parent',hp.hx_global);
plot3(handles.ax_quat_orientation,...
  [0,0],[0,1],[0,0],'g','linewidth',2,'parent',hp.hx_global);
plot3(handles.ax_quat_orientation,...
  [0,0],[0,0],[0,1],'b','linewidth',2,'parent',hp.hx_global);
% body coordinate axes
plot3(handles.ax_quat_orientation,...
  [0,1],[0,0],[0,0],'r','linewidth',2,'parent',hp.hx_body);
plot3(handles.ax_quat_orientation,...
  [0,0],[0,1],[0,0],'g','linewidth',2,'parent',hp.hx_body);
plot3(handles.ax_quat_orientation,...
  [0,0],[0,0],[0,1],'b','linewidth',2,'parent',hp.hx_body);
% TODO plot other body representation

% set up strip charts
set(handles.ax_quat_strip,...
  'nextplot','add',...
  'xtick',[],...
  'xlim',[0,handles.const.STRIP_TIME],...
  'ylim',handles.const.QUAT_STRIP_YLIM);
hp.quat(1) = plot(handles.ax_quat_strip,0,0,'k');
hp.quat(2) = plot(handles.ax_quat_strip,0,0,'r');
hp.quat(3) = plot(handles.ax_quat_strip,0,0,'g');
hp.quat(4) = plot(handles.ax_quat_strip,0,0,'b');
ylabel(handles.ax_quat_strip,'quat');
legend(handles.ax_quat_strip,...
  'q_0 = s','q_1 = v_1','q_2 = v_2','q_3 = v_3',...
  'location','eastoutside');

set(handles.ax_gyro_strip,...
  'nextplot','add',...
  'xtick',[],...
  'xlim',[0,handles.const.STRIP_TIME],...
  'ylim',handles.const.GYRO_STRIP_YLIM);
hp.gyro(1) = plot(handles.ax_gyro_strip,0,0,'r');
hp.gyro(2) = plot(handles.ax_gyro_strip,0,0,'g');
hp.gyro(3) = plot(handles.ax_gyro_strip,0,0,'b');
hp.gyro(4) = plot(handles.ax_gyro_strip,0,0,'k','linewidth',2);
ylabel(handles.ax_gyro_strip,['gyro [',char(176),'/s]']);
legend(handles.ax_gyro_strip,...
  'g_x','g_y','g_z','|g|',...
  'location','eastoutside');

set(handles.ax_accel_strip,...
  'nextplot','add',...
  'xtick',[],...
  'xlim',[0,handles.const.STRIP_TIME],...
  'ylim',handles.const.ACCEL_STRIP_YLIM);
hp.accel(1) = plot(handles.ax_accel_strip,0,0,'r');
hp.accel(2) = plot(handles.ax_accel_strip,0,0,'g');
hp.accel(3) = plot(handles.ax_accel_strip,0,0,'b');
hp.accel(4) = plot(handles.ax_accel_strip,0,0,'k','linewidth',2);
ylabel(handles.ax_accel_strip,'accel [g]');
legend(handles.ax_accel_strip,...
  'a_x','a_y','a_z','|a|-g',...
  'location','eastoutside');

set(handles.ax_emg_strip,...
  'nextplot','add',...
  'xlim',[0,handles.const.STRIP_TIME],...
  'ylim',handles.const.EMG_STRIP_YLIM);
hp.emg = plot(handles.ax_emg_strip,0,zeros(1,8));
hp.emg(end+1) = plot(handles.ax_emg_strip,0,0,'-k','linewidth',2);
ylabel(handles.ax_emg_strip,'emg [%]');
legend(handles.ax_emg_strip,...
  'emg_1','emg_2','emg_3','emg_4','emg_5','emg_6','emg_7','emg_8','|emg|',...
  'location','eastoutside');
hp.pose = plot(handles.ax_emg_strip,...
  -1,...
  zeros(1,handles.const.NUM_POSES),...
  'o',...
  'markersize',5,'linewidth',2);
for ii = 1:length(hp.pose)
  set(hp.pose(ii),'markerfacecolor',get(hp.pose(ii),'markeredgecolor'));
end

% adjust all strip axes to width of least wide axes
tmp = [...
  handles.ax_quat_strip,...
  handles.ax_gyro_strip,...
  handles.ax_accel_strip,...
  handles.ax_emg_strip];
p = cell2mat(get(tmp,'position'));
w = min(p(:,3));
for ii = 1:length(tmp)
  % includes static legend hacks from undocumented matlab...
  % http://undocumentedmatlab.com/blog/plot-performance
  set(tmp(ii),...
    'position',[p(ii,1:2),w,p(ii,4)],...
    'drawmode','fast',...
    'xlimmode','manual','ylimmode','manual');%,...
  %'LegendColorbarListeners',[]);
  setappdata(tmp(ii),'LegendColorbarManualSpace',1);
  setappdata(tmp(ii),'LegendColorbarReclaimSpace',1);
end

hp.image_pose = imshow(zeros(225,225,3),...
  'parent',handles.ax_image_pose);
set(hp.image_pose,'visible','off');

handles.hp = hp;

handles.update_timer = timer(...
  'name','MyoMexGUI_Monitor_update_timer',...
  'busymode','drop',...
  'executionmode','fixedrate',...
  'period',1/handles.const.UPDATE_RATE,...
  'startdelay',handles.const.UPDATE_START_DELAY,...
  'timerfcn',@(src,evt)updateFigureCallback(src,evt,handles));
start(handles.update_timer);
handles.myoData.startStreaming();

% Choose default command line output for MyoDataGUI_Monitor
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);



% UIWAIT makes MyoDataGUI_Monitor wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MyoDataGUI_Monitor_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function updateFigureCallback(~,~,handles)

m = handles.myoData;
hp = handles.hp;

% get datas to plot
timeIMU = m.timeIMU_log;
iiIMU = find(timeIMU>=(timeIMU(end)-handles.const.STRIP_TIME));
tIMU = timeIMU(iiIMU);

q = m.quat_log(iiIMU,:);
p = m.pose_log(iiIMU,:);

if get(handles.rb_fixed,'value')
  g = m.gyro_fixed_log(iiIMU,:);
  a = m.accel_fixed_log(iiIMU,:);
else
  g = m.gyro_log(iiIMU,:);
  a = m.accel_log(iiIMU,:);
end

set(hp.quat(1),'xdata',tIMU,'ydata',q(:,1));
set(hp.quat(2),'xdata',tIMU,'ydata',q(:,2));
set(hp.quat(3),'xdata',tIMU,'ydata',q(:,3));
set(hp.quat(4),'xdata',tIMU,'ydata',q(:,4));

set(hp.gyro(1),'xdata',tIMU,'ydata',g(:,1));
set(hp.gyro(2),'xdata',tIMU,'ydata',g(:,2));
set(hp.gyro(3),'xdata',tIMU,'ydata',g(:,3));
set(hp.gyro(4),'xdata',tIMU,'ydata',sqrt(sum(g'.^2))');

set(hp.accel(1),'xdata',tIMU,'ydata',a(:,1));
set(hp.accel(2),'xdata',tIMU,'ydata',a(:,2));
set(hp.accel(3),'xdata',tIMU,'ydata',a(:,3));
set(hp.accel(4),'xdata',tIMU,'ydata',sqrt(sum(a'.^2))'-1);

tpose = tIMU(m.pose_fist_log(iiIMU));
set(hp.pose(1),'xdata',tpose,...
  'ydata',ones(1,length(tpose))*handles.const.POSE_YDATA_VALUE);
tpose = tIMU(m.pose_wave_in_log(iiIMU));
set(hp.pose(2),'xdata',tpose,...
  'ydata',ones(1,length(tpose))*handles.const.POSE_YDATA_VALUE);
tpose = tIMU(m.pose_wave_out_log(iiIMU));
set(hp.pose(3),'xdata',tpose,...
  'ydata',ones(1,length(tpose))*handles.const.POSE_YDATA_VALUE);
tpose = tIMU(m.pose_fingers_spread_log(iiIMU));
set(hp.pose(4),'xdata',tpose,...
  'ydata',ones(1,length(tpose))*handles.const.POSE_YDATA_VALUE);
tpose = tIMU(m.pose_double_tap_log(iiIMU));
set(hp.pose(5),'xdata',tpose,...
  'ydata',ones(1,length(tpose))*handles.const.POSE_YDATA_VALUE);

% pose image
if handles.image_pose_enabled
  im = [];
  switch true
    case m.pose_fist
      im = handles.image_pose_fist;
    case m.pose_wave_in
      im = handles.image_pose_wave_in;
    case m.pose_wave_out
      im = handles.image_pose_wave_out;
    case m.pose_fingers_spread
      im = handles.image_pose_fingers_spread;
    case m.pose_double_tap
      im = handles.image_pose_double_tap;
  end
  
  % update image_pose
  if ~isempty(im)
    [M,N,~] = size(im);
    [X,Y] = meshgrid(1:M,1:N);
    A = sqrt( (X-floor(M/2)).^2 + (Y-floor(N/2)).^2 )<(floor(min([M,N])/2)-5);
    set(hp.image_pose,'cdata',im,'alphadata',A,'visible','on');
  else
    set(hp.image_pose,'visible','off');
  end
end

% update emg data
timeEMG = m.timeEMG_log;
if ~isempty(timeEMG)
  iiEMG = find(timeEMG>=(timeEMG(end)-handles.const.STRIP_TIME));
  tEMG = timeEMG(iiEMG);
  e = m.emg_log(iiEMG,:);
  
  for ii = 1:size(e,2)-1
    set(hp.emg(ii),'xdata',tEMG,'ydata',e(:,ii));
  end
  set(hp.emg(end),'xdata',tEMG,'ydata',sqrt(sum(e'.^2)/8)');
end

% update axes time limits
tmp = [...
  handles.ax_quat_strip,...
  handles.ax_gyro_strip,...
  handles.ax_accel_strip,...
  handles.ax_emg_strip];
set(tmp,'xlim',m.timeIMU - [handles.const.STRIP_TIME,0]);


% update orientation view
Rcurr = m.rot;

R = Rcurr;

H = [R,[0;0;0];0,0,0,1];
set(hp.hx_body,'matrix',H);

set(handles.st_imu_rate,'string',sprintf('%7.3f',m.rateIMU));
set(handles.st_emg_rate,'string',sprintf('%7.3f',m.rateEMG));

drawnow;%('EXPOSE');


% --- Executes on button press in pb_stream.
function pb_stream_Callback(hObject, eventdata, handles)
% hObject    handle to pb_stream (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
str = get(hObject,'string');

switch str
  case 'Start Streaming'
    start(handles.update_timer);
    handles.myoData.startStreaming();
    set(handles.pb_clear,'enable','off');
    set(hObject,'string','Stop Streaming');
  case 'Stop Streaming'
    handles.myoData.stopStreaming();
    stop(handles.update_timer);
    set(handles.pb_clear,'enable','on');
    set(hObject,'string','Start Streaming');
  otherwise
    disp('bad value');
end


% --- Executes on button press in pb_clear.
function pb_clear_Callback(hObject, eventdata, handles)
% hObject    handle to pb_clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if ~isempty(handles.myoData)
  handles.myoData.clearLogs();
end
hpstrip = [...
  handles.hp.quat,...
  handles.hp.gyro,...
  handles.hp.accel,...
  handles.hp.emg',...
  handles.hp.pose'];
for ii = 1:length(hpstrip)
  set(hpstrip(ii),'xdata',[],'ydata',[]);
end
set(hObject,'enable','off');
drawnow;


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% force deletion if not performed manually
stop(handles.update_timer);
delete(handles.update_timer);
if ~isempty(handles.closeFunc)
  handles.closeFunc();
end

% Hint: delete(hObject) closes the figure
delete(hObject);


% --- Executes on slider movement.
function sl_azimuth_Callback(hObject, eventdata, handles)
% hObject    handle to sl_azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

az = round(get(hObject,'value'));
set(hObject,'value',az);

el = round(get(handles.sl_elevation,'value'));
set(handles.sl_elevation,'value',el);

view(handles.ax_quat_orientation,...
  -az-handles.const.ROTATE_OFFSET,-el);


% --- Executes during object creation, after setting all properties.
function sl_azimuth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_azimuth (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor',[.9 .9 .9]);
end
set(hObject,'sliderstep',1/359*[1,10],'value',0);


% --- Executes when selected object is changed in bg_frame.
function bg_frame_SelectionChangeFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in bg_frame
% eventdata  structure with the following fields (see UIBUTTONGROUP)
%	EventName: string 'SelectionChanged' (read only)
%	OldValue: handle of the previously selected object or empty if none was selected
%	NewValue: handle of the currently selected object
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on slider movement.
function sl_elevation_Callback(hObject, eventdata, handles)
% hObject    handle to sl_elevation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

az = round(get(handles.sl_azimuth,'value'));
set(handles.sl_azimuth,'value',az);

el = round(get(hObject,'value'));
set(hObject,'value',el);

view(handles.ax_quat_orientation,...
  -az-handles.const.ROTATE_OFFSET,-el);


% --- Executes during object creation, after setting all properties.
function sl_elevation_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sl_elevation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
  set(hObject,'BackgroundColor',[.9 .9 .9]);
end
