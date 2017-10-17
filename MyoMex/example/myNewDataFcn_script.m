%% myNewDataFcn_script.m

TOTAL_TIME = 10; % s
RATE = 5; % Hz

% Instantiate MyoMex
mm = MyoMex(1);

% Register the newDataFcn callback with MyoMex
mm.newDataFcn = @(src,evt)myNewDataFcn(src,evt);

% Inspect the userData property of MyoMex for its contents
while toc < TOTAL_TIME
  pause(1/RATE);
  ud = mm.userData;
  ud.features
end

mm.delete();