function RadarDemo(varargin)
% RADARDEMO provides basic realtime signal processing and visualization
% of a single PulsON RADAR:
%   0. reconfigure radar
%   1. collect a single scan
%   2. motion filter, 
%   3. matched filter, 
%   4. envelope filter,
%   5. detection filter,
%   6. display as [pulse plot | waterfall | "wedge" plot]
%   repeat 1-6 until stop button

%% This clears the persistent variables in the filter functions without clearing breakpoints
saved = dbstatus;
clear functions
close all
dbstop(saved)

%% This deletes any lingering serial objects (just in case.)
delete(instrfindall)

% Optionally ask for portStr (eg 'COM8' in MSWin, '/dev/tty.usbmodem101' in osx or linux
portStr =  '/dev/tty.usbmodem101';
ASK_FOR_PORT = 0;
if ASK_FOR_PORT
  portStr = inputdlg('Enter portID','PortID',1,{portStr});
  portStr = portStr{:};  % Convert from cell to string
end

%% Set the GUI type
PULSE_RESPONSE_PLOT = 1;  WATERFALL_PLOT = 2;  WEDGE_PLOT = 3;
guiType = PULSE_RESPONSE_PLOT;
guiType = WATERFALL_PLOT;
%guiType = WEDGE_PLOT;

%% Initialize the plot with stop button
figure('Units','normalized','Position',[0.01 0.01 0.975 0.9],'Color',[.5 .5 .5])
%setappdata(0,'goFlag',goFlag)

rawEnable = logical(1);
hraw = uicontrol('Style','pushbutton','string','raw', ...
  'Units','normalized','Position',[0.01,0.21,0.09,0.04],...
  'fontsize',16,'BackgroundColor','g', ...
  'callback',@cbFunction);

mofEnable = logical(1);
hmof = uicontrol('Style','pushbutton','string','motionF', ...
  'Units','normalized','Position',[0.01,0.17,0.09,0.04],...
  'fontsize',16,'BackgroundColor','g', ...
  'callback',@cbFunction);

mafEnable = logical(1);
hmaf = uicontrol('Style','pushbutton','string','matchedF', ...
  'Units','normalized','Position',[0.01,0.13,0.09,0.04],...
  'fontsize',16,'BackgroundColor','g', ...
  'callback',@cbFunction);

envEnable = logical(1);
henv = uicontrol('Style','pushbutton','string','envelope', ...
  'Units','normalized','Position',[0.01,0.09,0.09,0.04],...
  'fontsize',16,'BackgroundColor','g', ...
  'callback',@cbFunction);

detEnable = logical(1);
uicontrol('Style','pushbutton','string','detections', ...
  'Units','normalized','Position',[0.01,0.05,0.09,0.04],...
  'fontsize',16,'BackgroundColor','g', ...
  'callback',@cbFunction);

goEnable = logical(1);
uicontrol('Style','pushbutton','string','stop', ...
  'Units','normalized','Position',[0.01,0.01,0.09,0.04],...
  'fontsize',16,'BackgroundColor','g', ...
  'callback',@cbFunction);

%% Surround everything by try/catch to delete the serial object 's' 
% before exiting if error in any called functions.  Helps with debugging.
try

%% Connect to the radio, return serial object as 's'
s = connecttoradio(portStr);

%% stop the radar (Nscans=0, dT=0) if it's still going then it will fill the serial buffer
fprintf('mrmcontrolreq stop the radar\n');
Nscans = 0; dT = 0; msgID = 1;
mrmcontrolreq(s,Nscans,dT,msgID)

%% Retrieve the current MRM configuration
fprintf('mrmgetcfg: get current configuration\n');
msgID = msgID + 1;
cfgStruct = mrmgetcfg(s,msgID);
fprintf('MRM Configuration:\n  nodeId: %d, scanStartPs: %d, scanStopPs: %d, pii: %d, txGain: %d, channel: %d\n',...
  cfgStruct.nodeId, cfgStruct.scanStartPs, cfgStruct.scanStopPs, ...
  cfgStruct.pulseIntegrationIndex, cfgStruct.transmitGain, cfgStruct.codeChannel);
  
%% Configure desired scan parameters, then update the radar.
Tzero_ns = 11.25; % This sets the time at which the pulse leaves the tx antenna
Rstart_m = 0;  % This is the distance from the tx antenna at which to start scanning
Rstop_m = 6;  % This is the distance to stop scanning
fprintf('mrmscansetup set: Tzero = %g ns, Rstart = %g m, Rstop = %g m\n',...
  Tzero_ns, Rstart_m, Rstop_m);
% mrmscansetup is just a helper function (doesn't connect to radio)
[Tstart_ns,Tstop_ns,Rstart_m,Rstop_m,Rbin_m,Nbin] = mrmscansetup(Tzero_ns,Rstart_m,Rstop_m);
fprintf('             get: Tstart = %gns, Tstop = %gns, Rstart = %.5gm, Rstop = %.5gm\n',...
  Tstart_ns, Tstop_ns, Rstart_m, Rstop_m);

%% Update radar configuration with any changes
cfgStruct.scanStartPs = uint32(Tstart_ns*1000);
cfgStruct.scanStopPs = uint32(Tstop_ns*1000);
%cfgStruct.transmitGain = uint8(0);  % for P400 (has 13dB amp. Set to 0 for FCC power.)
cfgStruct.transmitGain = uint8(63); % for P410 (no amp - set to max gain.)
cfgStruct.pulseIntegrationIndex = uint16(14);
cfgStruct.codeChannel = uint8(3);

%% Now send the config struct to the radar
msgID = msgID + 1;
fprintf('mrmsetcfg\n');
mrmsetcfg(s,cfgStruct,msgID);

%% Loop, reading scan, applying filters, and displaying results
%    until stop button
i = 0;
while goEnable %getappdata(0,'goFlag')
  tic
  i = i + 1;
  
  % Command a scan
  Nscans = 1; dT = 0; msgID = msgID + 1;
  mrmcontrolreq(s,Nscans,dT,msgID);
  
  % Receive and parse the scan data
  scan_raw = double(mrmgetscaninfo(s,Nbin));
  
  % Normalize
  scan_raw = scan_raw/max(abs(scan_raw));
    
  % Motion Filter
  memDepth = 4;
  scan_motion = motionfilter(scan_raw,memDepth);
  
  % Matched Filter
  scan_matched = matchedfilter(scan_motion);
  %scan_matched = scan_matched/max(abs(scan_matched));
  
  % Envelope Filter
  scan_env = envelope(scan_matched);
  
  % Detect strong reflections
  detectMode = 1;
  minThreshold = 0.2;
  detectList = detect(Rbin_m,scan_env,detectMode,minThreshold);
   
  % Pick a plot
  if i > memDepth
    % The button toggles the enable. If ~enabled then replace the scan with NotANumber vec (Matlab will not plot this.)
    if rawEnable; rawDisplay = scan_raw; else rawDisplay = NaN(size(scan_raw)); end
    if mofEnable; mofDisplay = scan_motion; else mofDisplay = NaN(size(scan_motion)); end
    if mafEnable; mafDisplay = scan_matched; else mafDisplay = NaN(size(scan_matched)); end
    if envEnable; envDisplay = scan_env; else envDisplay = NaN(size(scan_env)); end
    if detEnable; detDisplay = detectList; else detDisplay = NaN(size(detectList)); end
    switch guiType
      case PULSE_RESPONSE_PLOT
        ylimVec = [-2 2];
        plotpulseresponse(gca, Rbin_m, rawDisplay, mofDisplay, mafDisplay, ...
          envDisplay, detDisplay, ylimVec);
      case WATERFALL_PLOT
        slowTimeDepth = 50;
        % Select plot mode and convert to deciBels because they make better waterfalls
        if envEnable; scanDisp_dB = 20*log10(scan_env); % Display envelope scan if button is good
        elseif mafEnable; scanDisp_dB = 20*log10(abs(mafDisplay)); % display matched filtered scan
        elseif mofEnable; scanDisp_dB = 20*log10(abs(mofDisplay)); % display motion filtered scan
        else scanDisp_dB = 20*log10(abs(rawDisplay)); % display raw
        end
        plotwaterfall(gca, Rbin_m, scanDisp_dB, detectList, slowTimeDepth);
      case WEDGE_PLOT
        plotwedge(gca, Rbin_m, scan_env, detectList);
    end
  end

  Ymax = max(abs(scan_env));
  fprintf('dT=%3.2gs maxY=%g\n',toc,Ymax);

end % while stop button not clicked (on GUI)

%% This is for saving the last scan for creating H_target
SAVE_SCAN = 0;
if SAVE_SCAN
  keyboard
  scanSample = scan_motion;
  save('scanSample','scanSample');
end


%% Make sure serial port is gone before exiting during normal operation
fclose(s); 
delete(s);

%% Catch any error and get rid of serial object otherwise it will be buried inside Matlab
% (although you can get it back and delete it with delete(instrfindall) at the command line)
catch err
  fclose(s); 
  delete(s);
  rethrow(err);  % Pass the error to Matlab for display
end

  %% callback function for stop button
  function cbFunction(obj,event)
    objStr = get(obj,'String');
    objVal = get(obj,'Value');

    switch objStr
      case 'raw'
        if rawEnable; set(obj,'BackgroundColor',[.5 .5 .5]); 
        else set(obj,'BackgroundColor','g'); end
        rawEnable = ~rawEnable;
      case 'motionF'
        if mofEnable; set(obj,'BackgroundColor',[.5 .5 .5]); 
        else set(obj,'BackgroundColor','g'); end
        mofEnable = ~mofEnable;
      case 'matchedF'
        if mafEnable; set(obj,'BackgroundColor',[.5 .5 .5]); 
        else set(obj,'BackgroundColor','g'); end
        mafEnable = ~mafEnable;
      case 'envelope'
        if envEnable; set(obj,'BackgroundColor',[.5 .5 .5]); 
        else set(obj,'BackgroundColor','g'); end
        envEnable = ~envEnable;
      case 'detections'
        if detEnable; set(obj,'BackgroundColor',[.5 .5 .5]); 
        else set(obj,'BackgroundColor','g'); end
        detEnable = ~detEnable;
      case 'stop'
        set(obj,'String','STOPPED','BackgroundColor','r'); 
        goEnable = ~goEnable;
        fprintf('Stopped!\n');
    end

  end % cbFunction

end % RadarDemo







