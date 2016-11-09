function RespDemo(varargin)
%=== RAFI - This function analyse the slow time Doppler for respiration

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
%portStr =  '/dev/tty.usbmodem101';
portStr =  'COM10';
ASK_FOR_PORT = 1;
if ASK_FOR_PORT
  portStr = inputdlg('Enter portID','PortID',1,{portStr});
  portStr = portStr{:};  % Convert from cell to string
end

%% Set the GUI type
PULSE_RESPONSE_PLOT = 1;  WATERFALL_PLOT = 2;  WEDGE_PLOT = 3;
% --- Select the proper GUI you need -----------------
guiType = PULSE_RESPONSE_PLOT;
%guiType = WATERFALL_PLOT;
%guiType = WEDGE_PLOT;

%% Initialize the display
figure('Units','normalized','Position',[0 0 1 1],'Color',[.5 .5 .5])

uicontrol('Style','pushbutton','string','snapscan','Units','normalized',...
  'Position',[0.01,0.25,0.09,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);

rawEnable = logical(1);
hraw = uicontrol('Style','pushbutton','string','raw', 'Units','normalized','Position',[0.01,0.21,0.09,0.04],'fontsize',16,'BackgroundColor','g', 'callback',@cbFunction);

mofEnable = logical(1);
hmof = uicontrol('Style','pushbutton','string','motionF','Units','normalized','Position',[0.01,0.17,0.09,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);

mafEnable = logical(1);
hmaf = uicontrol('Style','pushbutton','string','matchedF','Units','normalized','Position',[0.01,0.13,0.09,0.04],'fontsize',16,'BackgroundColor','g', 'callback',@cbFunction);

envEnable = logical(1);
henv = uicontrol('Style','pushbutton','string','envelope', 'Units','normalized','Position',[0.01,0.09,0.09,0.04],'fontsize',16,'BackgroundColor','g', 'callback',@cbFunction);

detEnable = logical(1);
uicontrol('Style','pushbutton','string','detections','Units','normalized','Position',[0.01,0.05,0.09,0.04],'fontsize',16,'BackgroundColor','g', 'callback',@cbFunction);

goEnable = logical(1);
uicontrol('Style','pushbutton','string','stop','Units','normalized','Position',[0.01,0.01,0.09,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);

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
Tzero_ns = 10; % This sets the time at which the pulse leaves the tx antenna
Rstart_m = 0.5;  % This is the distance from the tx antenna at which to start the scan window
Rstop_m = 2;  % This is the distance to end the scan window
fprintf('mrmscansetup set: Tzero = %g ns, Rstart = %g m, Rstop = %g m\n',...
  Tzero_ns, Rstart_m, Rstop_m);

% Note: mrmscansetup is just a helper function (doesn't connect to radio)
[Tstart_ns,Tstop_ns,Rstart_m,Rstop_m,Rbin_m,Nbin] = mrmscansetup(Tzero_ns,Rstart_m,Rstop_m);
fprintf('             get: Tstart = %gns, Tstop = %gns, Rstart = %.5gm, Rstop = %.5gm\n',...
  Tstart_ns, Tstop_ns, Rstart_m, Rstop_m);

%% Update radar configuration with any changes
cfgStruct.scanStartPs = uint32(Tstart_ns*1000);
cfgStruct.scanStopPs = uint32(Tstop_ns*1000);
%cfgStruct.transmitGain = uint8(0);  % for P400 (has 13dB amp. Set to 0 for FCC power.)
%%%cfgStruct.transmitGain = uint8(63); % for P410 (no amp - set to max gain.)
cfgStruct.transmitGain = uint8(63); % for P410 (no amp - set to max gain.)
cfgStruct.pulseIntegrationIndex = uint16(14);
cfgStruct.codeChannel = uint8(3);
cfgStruct.persistFlag = uint8(0);

%% Now send the config struct to the radar
msgID = msgID + 1;
fprintf('mrmsetcfg\n');
mrmsetcfg(s,cfgStruct,msgID);     % RAFI: configures the MRM major params

%% Loop, reading scan, applying filters, and displaying results
%    until stop button
figure(2);
i = 0;
while goEnable
  tic
  i = i + 1;
  
  % =========== Command a scan ===========
  % s - serial object
  % Nscans - number of scans
  % dT - interval between start of multiple scans (us)
  % msgID - integer message ID
  Nscans = 100; dT = 250000; msgID = msgID + 1;
  mrmcontrolreq(s,Nscans,dT,msgID);
  
  %% Receive and parse the scan data
  for j=1:Nscans
        scan_raw = double(mrmgetscaninfo(s,Nbin));
        %% Normalize 
        %scan_raw = scan_raw/max(abs(scan_raw));
        Mscan (j,:) = scan_raw;
  end
    
  %plot(Mscan');
  Mscan_fft = abs(fft(Mscan));
  Mscan_fft_db = 10.*log10(Mscan_fft);
  mesh(Mscan_fft_db(4:Nscans/2,:));
  pause;
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
      case 'snapscan'
        set(obj,'BackgroundColor',[.5 .5 .5]);
        scanSample = scan_motion;  % change which one you want to save
        save('scanSample','scanSample');
        drawnow
        tic
        while toc < 0.25; end
        set(obj,'BackgroundColor','g');
        fprintf('motion scan saved in scanSample.mat\n')
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







