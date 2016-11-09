function MrmRadarDemoIP(varargin)
% MRMRADARDEMO provides basic realtime signal processing and visualization
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

%% Initialize Constants
mrmIpAddr = '192.168.0.118';
%mrmIpAddr = '192.168.0.125';
scanStartPs = 5000; % Adjust this to match antenna delay
C_mps = 299792458;
maxDistance_m = 6;  % MRM will quantize to closest above this number
scanStopPs = scanStartPs + (2*maxDistance_m/C_mps)*1e12; % 1e12 ps in one sec
pulseIntegrationIndex = 13; % The number of pulses per scan point
transmitGain = 63; % Tx power (0 for FCC legal)
scanIntervalTime_ms = 0; % Time between start of each scan in millisecs
scanRes_ps = 61; % 61 picoseconds between each data point (from API.) Used for plotting.

%% Set the GUI type
guiType = 'PULSE_RESPONSE';
guiType = 'WATERFALL'; waterfallSlowTimeDepth = 50;
%guiType = 'WEDGE';
%guiType = 'FFT';

memDepth = 7;
caxisLim = [-30 0];

%% Open a socket for communicating with the MRM
sckt = sckt_mgr('get');
if isempty(sckt)
  sckt_mgr('open');
end

%% Get the configuration of the MRM
get_cfg_rqst(mrmIpAddr,1)
[msg,msgType,msgID,mrmIpAddr] = read_pckt;
if isempty(msgID)
  error('Unable to communicate with the MRM')
end
[CFG,msgType,msgID] = parse_msg(msg);


%% Update the config structure & send to the MRM
% The MRM API specifies the variable types
CFG.scanStartPs = uint32(scanStartPs);
CFG.scanStopPs = uint32(scanStopPs);
CFG.pulseIntegrationIndex = uint16(pulseIntegrationIndex); % The higher this is the higher the snr but longer it takes to scan
CFG.transmitGain = uint8(transmitGain);
set_cfg_rqst(mrmIpAddr,2,CFG);
fprintf('ScanStart: %5.2f ns, ScanStop: %5.2f ns, PII: %d, TxGain: %d\n', ...
    scanStartPs/1000, scanStopPs/1000, pulseIntegrationIndex, transmitGain);


%% Read the confirm from the MRM
[msg,msgType,msgID,mrmIpAddr] = read_pckt;
if ~strcmp(msgType,'1101') % MRM_SET_CONFIG_CONFIRM
    error(fprintf('Invalid message type: %s, should have been 1101',msgType));
end


%% Command radar to scan designated number of scans (-1 continuous)
scanCount = 1;
CTL.scanCount = uint16(scanCount); % 2^16-1 for continuous
CTL.reserved = uint16(0); % Aligns to word
CTL.scanIntervalTime = uint32(scanIntervalTime_ms*1000); % Microsecs between scan starts


%% Initialize the display
figure('Units','normalized','Position',[0 0 1 1],'Color',[.5 .5 .5])

% uicontrol('Style','pushbutton','string','snapscan','Units','normalized',...
%   'Position',[0.01,0.25,0.09,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);

rawEnable = logical(1);
hraw = uicontrol('Style','pushbutton','string','raw', 'Units','normalized', ...
    'Position',[0.01,0.21,0.09,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

mofEnable = logical(1);
hmof = uicontrol('Style','pushbutton','string','motionF','Units','normalized',...
    'Position',[0.01,0.17,0.09,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);
 
mafEnable = logical(1);
hmaf = uicontrol('Style','pushbutton','string','matchedF','Units','normalized', ...
    'Position',[0.01,0.13,0.09,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

envEnable = logical(1);
henv = uicontrol('Style','pushbutton','string','envelope', 'Units','normalized', ...
    'Position',[0.01,0.09,0.09,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

detEnable = logical(1);
uicontrol('Style','pushbutton','string','detections','Units','normalized', ...
    'Position',[0.01,0.05,0.09,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

goEnable = logical(1);
uicontrol('Style','pushbutton','string','stop','Units','normalized', ...
    'Position',[0.01,0.01,0.09,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);


%% Loop, reading scan, applying filters, and displaying results
%    until stop button
i = 0;
while goEnable
  tic
  i = i + 1;
  
  %% Request a scan and read it back
  % Request a scan
  ctl_rqst(mrmIpAddr,msgID,CTL) 

  % Read the confirm
  [msg,msgType,msgID,mrmIpAddr] = read_pckt;
  
  % Read the first scan msg.  Analyze the hdr for how many follow
  [msg,msgType,msgID,mrmIpAddr] = read_pckt;
  [scanInfo,msgType,msgID] = parse_msg(msg);
  s_raw = double(scanInfo.scanData);  % Save the good stuff. Append to this later

  % Loop, reading the entire waveform scan into scanDataSaved    
  for j = 1:scanInfo.numberMessages-1
      [msg,msgType,msgID,mrmIpAddr] = read_pckt;
      [scanInfo,msgType,msgID] = parse_msg(msg);
      s_raw = [s_raw, double(scanInfo.scanData)];
  end

  toc1 = toc;
  fprintf('Single scan %g ms\n',toc1*1000);

  %% Normalize raw scan
  s_raw = 1.1*s_raw/max(abs(s_raw));
  Rbin_m = ((0:length(s_raw)-1)*scanRes_ps/1e12)*C_mps/2;  % scanIndex*(61ps/step)/(ps/sec)*(meters/sec)/2 (round trip)
  
  %% Fast Motion Filter
  if ~exist('s_fast','var'); s_fast = zeros(size(s_raw)); end
  s_fast_prev = s_fast;
  s_fast = MotionFilter(s_raw,memDepth);
  
  %% Slow Motion Filter
  if ~exist('s_lpf','var')
    s_lpf = zeros(1,length(s_raw));
    alpha_lpf = 0.05;
  end
  s_lpf = alpha_lpf*s_raw + (1-alpha_lpf)*s_lpf;
  s_slow = s_raw - s_lpf;
  
  %% Detect Motion
  fast_res = sum(abs(s_fast-s_fast_prev));
  slow_res = sum(abs(s_raw-s_lpf));
%  motion_metric = fast_res + slow_res;
  motion_metric = fast_res;
  fprintf('  fast_res = %f, slow_res = %f, motion_metric = %f\n', ...
    fast_res,slow_res,motion_metric);
  
  %% Envelope Filter
%  s_fast_env = envelope(s_fast);
  s_fast_env = abs(hilbert(s_fast));
  s_fast_env = fir_lpf_ord5(s_fast_env);

%  s_slow_env = envelope(s_slow);
  s_slow_env = abs(hilbert(s_slow));

  %% TODO: update the template when there's no motion
  if ~exist('s_template','var')
    s_template = s_raw;
  end
  
  s_template_prev = s_template;
  if motion_metric > 3
%    MOTION_DETECTED = true;
    s_template = s_template_prev;
  else
%    MOTION_DETECTED = false;
    s_template = s_lpf;
  end
  s_test = abs(hilbert(s_raw-s_template));
%  s_test = s_raw-s_template;
  
  %% Matched Filter (helps remove close-in chatter from RF bouncing inside the radar
%  scanMatched = matchedfilter(scanMotion);
%  scanMatched = scanMotion;
  
    
  % Update scan memory
%   if ~exist('scanBuf','var')
%       scanBuf = NaN(50,length(scanEnv));
%   end
%   scanBuf(2:end,:) = scanBuf(1:end-1,:);
%   scanBuf(1,:) = scanEnv;  
%   scanSTD = nanstd(scanBuf(1:5,:),1);
  
%   %% Detect strong reflections
   detectMode = 'None';
   minThreshold = 0.2;
%   minThreshold = 3*scanSTD;
   [detectList,threshold] = detect(Rbin_m,s_fast,detectMode,minThreshold);
   
  % Pick a plot
  if i > memDepth
    % The button toggles the enable. If ~enabled then replace the scan with NotANumber vec (Matlab will not plot this.)
    if rawEnable; rawDisplay = s_raw; else rawDisplay = NaN(size(s_raw)); end
    if mofEnable; mofDisplay = s_fast; else mofDisplay = NaN(size(s_raw)); end
    if mafEnable; mafDisplay = s_slow; else mafDisplay = NaN(size(s_raw)); end
    if envEnable; envDisplay = s_slow_env; else envDisplay = NaN(size(s_raw)); end
%     if detEnable; 
%       detDisplay = detectList;
%       threshDisplay = threshold;
%     else
%       detDisplay = NaN(size(detectList)); 
%       threshDisplay = NaN(size(threshold));
%     end
    
    switch guiType
      case 'PULSE_RESPONSE'
        ylimVec = [-2 2];
%         plotpulseresponse(gca, Rbin_m, rawDisplay, mofDisplay, mafDisplay, ...
%           envDisplay, detDisplay, threshDisplay, ylimVec);
        line1 = s_raw;
        line2 = s_fast;
        line3 = s_fast_env;
        line4 = s_slow;
        line5 = s_slow_env;
        line6 = s_test;
        PlotStuff(Rbin_m,line1,line2,line3,line4,line5,line6,ylimVec);
      case 'WATERFALL'
        % Select plot mode and convert to deciBels because they make better waterfalls
        if envEnable; 
          scanDisp_dB = 20*log10(s_fast_env); % Display envelope scan if button is good
%         elseif mafEnable; scanDisp_dB = 20*log10(abs(mafDisplay)); % display matched filtered scan
%         elseif mofEnable; scanDisp_dB = 20*log10(abs(mofDisplay)); % display motion filtered scan
%         else scanDisp_dB = 20*log10(abs(rawDisplay)); % display raw
        end
        plotwaterfall(gca,Rbin_m,scanDisp_dB,detectList,waterfallSlowTimeDepth,caxisLim);
      case 'WEDGE'
        plotwedge(gca, Rbin_m, scanEnv, detectList);
      case 'FFT'
        PlotFFT(s_fast);
    end
  end

%  Ymax = max(abs(scanEnv));
%   firstD = detectList(1,1);
%   firstV = detectList(2,1);
%   fprintf('dT=%3.2gs firstD=%g, firstV=%g\n',toc,firstD,firstV);
  
  drawnow

end % while stop button not clicked (on GUI)

%% This is for saving the last scan for creating H_target
SAVE_SCAN = 0;
if SAVE_SCAN
  keyboard
  scanSample = scanMotion;
  save('scanSample','scanSample');
end
fprintf('Quit\n')

  %% callback function for stop button
  function cbFunction(obj,event)
    objStr = get(obj,'String');
%    objVal = get(obj,'Value');

    switch objStr
      case 'snapscan'
        set(obj,'BackgroundColor',[.5 .5 .5]);
        %scanSample = scan_motion;  % change which one you want to save
        save('snapscan','s_raw','scanMotion','scanMatched');
        drawnow
        tic
        while toc < 0.25; end
        set(obj,'BackgroundColor','g');
        fprintf('motion scan saved in scanSample.mat\n')
      case 'raw'
        rawEnable = ~rawEnable;
        if rawEnable
            set(obj,'BackgroundColor',[.7 .7 .7]); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
      case 'motionF'
        mofEnable = ~mofEnable;
        if mofEnable
            set(obj,'BackgroundColor',rgb('DarkOrange')); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
      case 'matchedF'
        mafEnable = ~mafEnable;
        if mafEnable
            set(obj,'BackgroundColor',rgb('DeepSkyBlue')); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
      case 'envelope'
        envEnable = ~envEnable;
        if envEnable
            set(obj,'BackgroundColor',rgb('LightGreen')); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
      case 'detections'
        detEnable = ~detEnable;
        if detEnable
            set(obj,'BackgroundColor',rgb('Red')); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
      case 'stop'
        goEnable = ~goEnable;
        set(obj,'String','STOPPED','BackgroundColor',[.5 .5 .5]); 
        fprintf('Stopped!\n');
    end

  end % cbFunction

end % RadarDemo








