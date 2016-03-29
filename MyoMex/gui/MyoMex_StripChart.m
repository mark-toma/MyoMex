% MyoMex_StripChart
%
function myoMex = MyoMex_StripChart(countMyos,tspan)

if nargout < 1
  error('Assign output myoMex to clean up manually when done!');
end

if nargin < 2, tspan = []; end

myoMex = MyoMex(2);
MyoData_StripChartStandalone(myoMex.myo_data(1),tspan);
MyoData_StripChartStandalone(myoMex.myo_data(2),tspan);

end