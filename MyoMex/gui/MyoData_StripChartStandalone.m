% MyoData_StripChartStandalone  Displays strip chart of MyoData
% Inputs:
%   
function MyoData_StripChartStandalone(myoData,tspan)

if nargin < 1 || ~isa(myoData,'MyoData')
  error();
elseif nargin < 2 || isempty(tspan)
  tspan = 10;
end

if ~isnumeric(tspan) || ~isscalar(tspan) || (tspan<2) || (tspan>10)
  error('Input tspan must be a numeric scalar in [2,10]s.');
end

h.myo = myoData;
h.tspan = tspan;

% init graphics
h.hfig = figure();
titleCell = {'gyro','accel','emg'};
for ii=1:3, h.hax(ii,1) = subplot(3,1,ii,'parent',h.hfig); end
for ii=1:3, title(h.hax(ii,1),titleCell{ii}); end
h.gyro = plot(0,zeros(1,3),'parent',h.hax(1));
h.accel = plot(0,zeros(1,3),'parent',h.hax(2));
h.emg = plot(0,zeros(1,8),'parent',h.hax(3));
drawnow;

% init timer
h.tmr = timer('executionmode','fixedrate','busymode','drop','period',0.04,...
  'timerfcn',@(src,evt)updateFigure(src,evt,h));

set(h.hfig,'closerequestfcn',@(src,evt)closeRequestFcn(src,evt,h));
start(h.tmr);

end

function updateFigure(src,evt,h)

g = h.myo.gyro_log;
a = h.myo.accel_log;
e = h.myo.emg_log;

nIMU = size(g,1);
nEMG = size(e,1);

tIMU = 1/50  * (1:1:nIMU);
tEMG = 1/200 * (1:1:nEMG);

iIMU = tIMU > (tIMU(end)-h.tspan);
iEMG = tEMG > (tEMG(end)-h.tspan);

for ii=1:3
  set(h.gyro(ii),'xdata',tIMU(iIMU),'ydata',g(iIMU,ii));
  set(h.accel(ii),'xdata',tIMU(iIMU),'ydata',a(iIMU,ii));
end
for ii=1:8
  set(h.emg(ii),'xdata',tEMG(iEMG),'ydata',e(iEMG,ii));
end

ha = findobj(h,'type','axes');
set(ha,'xlim',max([tIMU(end),tEMG(end)])+[-h.tspan,1]);

drawnow;

end

function closeRequestFcn(src,evt,h)
stop(h.tmr);
delete(h.tmr);
delete(src);
end
