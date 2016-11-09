function RadarDemoIP(varargin)
% MRMRADARDEMO provides basic realtime signal processing and visualization
% of a single PulsON RADAR:
%   0. reconfigure radar
%   1. collect a single scan
%   2. band pass filter
%   3. motion filter, 
%   4. envelope filter,
%   6. display as [pulse plot | waterfall | "wedge" plot]
%   repeat 1-6 until stop button

%% This clears the persistent variables in the filter functions without clearing breakpoints
saved = dbstatus;
clear functions
close all
dbstop(saved)

%% Initialize Constants
ip_addr = '192.168.0.118';
%mrmIpAddr = '192.168.0.125';
%NOTE: TPLINK 192.168.0.254
%mrmIpAddr = '192.168.0.203';
scan_start_ps = 0000; % Adjust this to match antenna delay
c_mps = 299792458;
max_distance_m = 6;  % MRM will quantize to closest above this number
scan_stop_ps = scan_start_ps + (2*max_distance_m/c_mps)*1e12; % 1e12 ps in one sec
pulse_integration_index = 13; % The number of pulses per scan point
transmit_gain = 63; % Tx power (0 for FCC legal)
scan_interval_ms = 0; % Time between start of each scan in millisecs
scan_resolution_ps = 61; % 61 picoseconds between each data point (from API.) Used for plotting.

motion_depth = 4;
waterfall_depth = 50;
caxis_lim = [-30 0];

%% Open a socket for communicating with the MRM
sckt = sckt_mgr('get');
if isempty(sckt)
  sckt_mgr('open');
end

%% Get the configuration of the MRM
get_cfg_rqst(ip_addr,1)
[msg,msg_type,msg_id,ip_addr] = read_pckt;
if isempty(msg_id)
  error('Unable to communicate with the radar.')
end
[CFG,msg_type,msg_id] = parse_msg(msg);


%% Update the config structure & send to the MRM
% The MRM API specifies the variable types
CFG.scan_start_ps = uint32(scan_start_ps);
CFG.scan_stop_ps = uint32(scan_stop_ps);
CFG.pulse_integration_index = uint16(pulse_integration_index); % The higher this is the higher the snr but longer it takes to scan
CFG.transmit_gain = uint8(transmit_gain);
set_cfg_rqst(ip_addr,2,CFG);
fprintf('ScanStart: %5.2f ns, ScanStop: %5.2f ns, PII: %d, TxGain: %d\n', ...
    scan_start_ps/1000, scan_stop_ps/1000, pulse_integration_index, transmit_gain);

%% Read the confirm from the MRM
[msg,msg_type,msg_id,ip_addr] = read_pckt;
if ~strcmp(msg_type,'1101') % MRM_SET_CONFIG_CONFIRM
    error(fprintf('Invalid message type: %s, should have been 1101',msg_type));
end

%% Command radar to scan designated number of scans (-1 continuous)
scan_count = 1;
CTL.scan_count = uint16(scan_count); % 2^16-1 for continuous
CTL.reserved = uint16(0); % Aligns to word
CTL.scan_interval_ms = uint32(scan_interval_ms*1000); % Microsecs between scan starts

%% Initialize the display
figure('Units','normalized','Position',[0 0 1 1],'Color',[.5 .5 .5])

gui_type = 'pulse';
%plotType = 'waterfall';
h_plot_btn = uicontrol('Style','pushbutton','string',gui_type, 'Units','normalized', ...
    'Position',[0.01,0.29,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

raw_enable = logical(1);
h_raw_btn = uicontrol('Style','pushbutton','string','raw', 'Units','normalized', ...
    'Position',[0.01,0.25,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

bpf_enable = logical(1);
h_bpf_btn = uicontrol('Style','pushbutton','string','bpf', 'Units','normalized', ...
    'Position',[0.01,0.21,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

motion_enable = logical(1);
h_motion_btn = uicontrol('Style','pushbutton','string','motion','Units','normalized',...
    'Position',[0.01,0.17,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);
 
persist_enable = logical(1);
h_persist_btn = uicontrol('Style','pushbutton','string','persist','Units','normalized', ...
    'Position',[0.01,0.13,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

env_enable = logical(1);
h_env_btn = uicontrol('Style','pushbutton','string','envelope', 'Units','normalized', ...
    'Position',[0.01,0.09,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

snap_enable = logical(0);
h_snap_btn = uicontrol('Style','pushbutton','string','snap','Units','normalized', ...
    'Position',[0.01,0.05,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

go_enable = logical(1);
h_go_btn = uicontrol('Style','pushbutton','string','stop','Units','normalized', ...
    'Position',[0.01,0.01,0.08,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);

%% Loop, reading scan, applying filters, and displaying results
%    until stop button
i = 0;
while go_enable
  tic
  i = i + 1;
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%%%% Read Raw
  % Request a scan
  ctl_rqst(ip_addr,msg_id,CTL) 
  % Read the confirm
  [msg,msg_type,msg_id,ip_addr] = read_pckt;

  % Read the initial scan msg.  Analyze the header for how many follow
  [msg,msg_type,msg_id,ip_addr] = read_pckt;
  [scan_info,msg_type,msg_id] = parse_msg(msg);
  s_raw = double(scan_info.scan_data);  % Save the good stuff. Append to this later

  % Reading the entire waveform scan into scanDataSaved    
  for j = 1:scan_info.num_msgs-1
      [msg,msg_type,msg_id,ip_addr] = read_pckt;
      [scan_info,msg_type,msg_id] = parse_msg(msg);
      s_raw = [s_raw, double(scan_info.scan_data)];
  end
  
  toc1 = toc;
  fprintf('Single scan %g ms\n',toc1*1000);
  scan_len = length(s_raw);

  %% %%%%%%%%%%%%%%%%%%%%%%%%%%%  Normalize raw scan
  s_raw = 1.1*s_raw/max(abs(s_raw));
  if 1 == i
    Rbin_m = ((0:scan_len-1)*scan_resolution_ps/1e12)*c_mps/2;  % scanIndex*(61ps/step)/(ps/sec)*(meters/sec)/2 (round trip)
  end
  s = s_raw;
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%% Band Pass Filter
  if bpf_enable
    %s_bpf = fir_lpf_ord5(s);
    if ~exist('b_bpf','var')
      [b_bpf,a_bpf] = butter(3,0.6,'low');
    end
    s_bpf = filter(b_bpf,a_bpf,s);
  end
  s = s_bpf;
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%% Persist Filter
  if persist_enable
    load('s_snap.mat');
    s_persist = s - s_snap;
  end
  s = s_persist;

  %% %%%%%%%%%%%%%%%%%%%%%%%%% Motion Filter
  mem_depth = 4;
  s_motion = motion_filter(s,mem_depth);
  s = s_motion;
    
  %% %%%%%%%%%%%%%%%%%%%%%%%%% Envelope Filter
  if env_enable
    %s_env = envelope(s_fast);
    s_env = abs(hilbert(s));
  end
  s = s_env;

  %% %%%%%%%%%%%%%%%%%%%%%%%%% Snap a scan  
%   if snap_enable
%     s_snap = s_raw;
%     save('s_snap.mat','s_snap')
%     fprintf('scan saved to s_snap.mat')
%   end
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%%% Update scan memory
  scanbuf_depth = 10;
  if ~exist('scanbuf','var')
      scanbuf = NaN(scanbuf_depth,scan_len);
  end
  scanbuf(2:end,:) = scanbuf(1:end-1,:);
  scanbuf(1,:) = s;  
  scan_std = nanstd(scanbuf,1);
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%%% Detect Motion
%   fast_res = sum(abs(s_fast-s_fast_prev));
%   slow_res = sum(abs(s_raw-s_lpf));
% %  motion_metric = fast_res + slow_res;
%   motion_metric = fast_res;
%   fprintf('  fast_res = %f, slow_res = %f, motion_metric = %f\n', ...
%     fast_res,slow_res,motion_metric);
  %   %% Detect strong reflections
%   detectMode = 'None';
%   minThreshold = 0.2;
%   minThreshold = 3*scanSTD;
%   [detectList,threshold] = detect(Rbin_m,s_fast,detectMode,minThreshold);
   
  %% %%%%%%%%%%%%%%%%%%%%%%%%%%  Plot
  if i > mem_depth

    % The button toggles the enable. If ~enabled then replace the scan with NotANumber vec (Matlab will not plot this.)
    if raw_enable; raw_plot = s_raw; else raw_plot = NaN(1,scan_len); end
    if bpf_enable; bpf_plot = s_bpf; else bpf_plot = NaN(1,scan_len); end
    if motion_enable; motion_plot = s_motion; else motion_plot = NaN(1,scan_len); end
    if persist_enable; persist_plot = s_persist; else persist_plot = NaN(1,scan_len); end
    if env_enable; env_plot = s_env; else env_plot = NaN(1,scan_len); end

    switch gui_type
      case 'pulse'
  %         plotpulseresponse(gca, Rbin_m, rawDisplay, mofDisplay, mafDisplay, ...
  %           envDisplay, detDisplay, threshDisplay, ylimVec);
  %        PlotStuff(Rbin_m,line1,line2,line3,line4,line5,line6,ylimVec);
        plot_pulse(gca,Rbin_m,raw_plot,bpf_plot,motion_plot,persist_plot,env_plot)
      case 'WATERFALL'
        % Select plot mode and convert to deciBels because they make better waterfalls
        if envEnable; 
          scanDisp_dB = 20*log10(s_fast_env); % Display envelope scan if button is good
  %         elseif mafEnable; scanDisp_dB = 20*log10(abs(mafDisplay)); % display matched filtered scan
  %         elseif mofEnable; scanDisp_dB = 20*log10(abs(mofDisplay)); % display motion filtered scan
  %         else scanDisp_dB = 20*log10(abs(rawDisplay)); % display raw
        end
        plotwaterfall(gca,Rbin_m,scanDisp_dB,detectList,waterfallSlowTimeDepth,caxis_lim);
      case 'WEDGE'
        plotwedge(gca, Rbin_m, scanEnv, detectList);
      case 'FFT'
        PlotFFT(s_fast);
      case 'PULSE_AND_WATERFALL'
        if ~exist('ax1','var')
          ax1 = subplot(3,1,1);
          ax2 = subplot(3,1,2:3);
        end
        plot_pulse(ax1,Rbin_m,s_raw,s_motion,s_mo_env)
        s_db = 20*log10(s_mo_env); % Display envelope scan if button is good
        plot_waterfall(ax2,Rbin_m,s_db,waterfall_depth,caxis_lim)

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
  function cbFunction(obj,eventdata)
    btn_label = get(obj,'String');
%    objVal = get(obj,'Value');

    switch btn_label

      case 'raw'
        raw_enable = ~raw_enable;
        if raw_enable
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end

      case 'bpf'
        bpf_enable = ~bpf_enable;
        if bpf_enable
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end

      case 'motion'
        motion_enable = ~motion_enable;
        if motion_enable
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
        
      case 'persist'
        persist_enable = ~persist_enable;
        if persist_enable
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
        
      case 'envelope'
        env_enable = ~env_enable;
        if env_enable
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
        
      case 'snap'
        set(obj,'BackgroundColor',[.5 .5 .5]);
        s_snap = s_raw;
        save('s_snap.mat','s_snap');
        drawnow
        pause(0.1)
        set(obj,'BackgroundColor','g');
        fprintf('saved in s_snap.mat\n')

      case 'stop'
        go_enable = ~go_enable;
        set(obj,'String','STOPPED','BackgroundColor',[.5 .5 .5]); 
%        fprintf('Stopped!\n');
    end

  end % cbFunction

end % RadarDemo








