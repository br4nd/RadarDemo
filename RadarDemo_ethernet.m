function RadarDemo_eth(varargin)
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
ip_addr = '192.168.1.105';
%ip_addr = '10.3.1.103';
%NOTE: TPLINK 192.168.0.254
%ip_addr = '192.168.0.203';
scan_start_m = 1.45;
c_mps = 299792458;
scan_start_ps = (2*scan_start_m/c_mps)*1e12;
%scan_start_ps = 5000; % Adjust this to match antenna delay
max_distance_m = 13.0;  % MRM will quantize to closest above this number
scan_stop_ps = scan_start_ps + (2*max_distance_m/c_mps)*1e12; % 1e12 ps in one sec
pulse_integration_index = 10; % The number of pulses per scan point
transmit_gain = 127; % Tx power (0 for FCC legal)
scan_interval_ms = 0; % Time between start of each scan in millisecs (0 is as fast as possible)
scan_resolution_ps = 61; % 61 picoseconds between each data point (from API.) Used for plotting.

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
figure('Units','normalized','Position',[1 0 1 1],'Color',[.5 .5 .5])

plot_raw = true;
h_raw_btn = uicontrol('Style','pushbutton','string','raw', 'Units','normalized', ...
    'Position',[0.01,0.36,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

%bpf_enable = true;
plot_bpf = true;
h_bpf_btn = uicontrol('Style','pushbutton','string','bpf', 'Units','normalized', ...
    'Position',[0.01,0.31,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

plot_motion = true;
h_motion_btn = uicontrol('Style','pushbutton','string','motion','Units','normalized',...
    'Position',[0.01,0.26,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);
 
plot_env = true;
h_env_btn = uicontrol('Style','pushbutton','string','envelope', 'Units','normalized', ...
    'Position',[0.01,0.21,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

plot_persist = false;
h_persist_btn = uicontrol('Style','pushbutton','string','persist','Units','normalized', ...
    'Position',[0.01,0.16,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('Gray'),'callback',@cbFunction);

snap_enable = false;
h_snap_btn = uicontrol('Style','pushbutton','string','snap','Units','normalized', ...
    'Position',[0.01,0.11,0.08,0.04],'fontsize',16,'BackgroundColor',rgb('LimeGreen'),'callback',@cbFunction);

%gui_type = 'pulse'; btn_color = 'lightblue';
gui_type = 'waterfall'; btn_color = 'orange';
h_plot_btn = uicontrol('Style','pushbutton','string',gui_type, 'Units','normalized', ...
    'Position',[0.01,0.06,0.08,0.04],'fontsize',16,'BackgroundColor',rgb(btn_color),'callback',@cbFunction);

go_enable = true;
h_go_btn = uicontrol('Style','pushbutton','string','stop','Units','normalized', ...
    'Position',[0.01,0.01,0.08,0.04],'fontsize',16,'BackgroundColor','g','callback',@cbFunction);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%% Read Raw
% Request a scan
ctl_rqst(ip_addr,msg_id,CTL) 
% Read the confirm
[msg,msg_type,msg_id,ip_addr] = read_pckt;

% Read the initial scan msg.  Analyze the header for how many follow
[msg,msg_type,msg_id,ip_addr] = read_pckt;
[scan_info,msg_type,msg_id] = parse_msg(msg);
s_raw = double(scan_info.scan_data);  % Save the good stuff. Append to this later

% Reading the entire waveform scan   
for j = 1:scan_info.num_msgs-1
    [msg,msg_type,msg_id,ip_addr] = read_pckt;
    [scan_info,msg_type,msg_id] = parse_msg(msg);
    s_raw = [s_raw, double(scan_info.scan_data)];
end
s = s_raw;

  %% Loop, reading scan, applying filters, and displaying results
%    until stop button
i = 0;
t_start = tic;
% SA_saved = [];
datetime = regexprep(datestr(now),' ','_');
while go_enable
  t0 = tic;
  i = i + 1;
    
  % Request next scan
  ctl_rqst(ip_addr,msg_id,CTL) 
  % Read the confirm
  [msg,msg_type,msg_id,ip_addr] = read_pckt;

  scan_len = length(s);
  
  s_norm = s/max(abs(s));
  s = s_norm;

  %% %%%%%%%%%%%%%%%%%%%%%%%%% Band Pass Filter
  if ~exist('b_bpf','var')
    [b_bpf,a_bpf] = butter(5,0.7,'low'); % taps, cutoff (Nyquist = 1)
  end
  s_bpf = filter(b_bpf,a_bpf,s);
  s = s_bpf;
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%%%%  Normalize raw and apply 1/r^alpha 
  if 1 == i
    rbin = ((0:scan_len-1)*scan_resolution_ps/1e12)*c_mps/2;
    alpha = 1.0;
    r_gain = rbin.^alpha;
    %r_gain(r_gain<.5) = .5;
  end
  s = s.*r_gain;
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%% Persist Filter
  if plot_persist
    load('s_snap.mat')
    s_persist = s - s_snap;
    s = s_persist;
  end
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%% Motion Filter
  mem_depth = 1;
  s_motion = motion_filter(s,mem_depth);
  s = s_motion;
    
  %% %%%%%%%%%%%%%%%%%%%%%%%%% Envelope Filter
  s_env = abs(hilbert(s));
  if plot_persist
    s_env = s_env + abs(hilbert(s_persist));
  end
  s = s_env;
  t_stamp = toc(t_start);
  
%   %% %%%%%%%%%%%%%%%%%%%%%%%%%% Update scan memory
%   scanbuf_enable = true;
%   if scanbuf_enable
%     scanbuf_depth = 10;
%     if ~exist('scanbuf','var')
%         scanbuf = NaN(scanbuf_depth,scan_len);
%     end
%     scanbuf(2:end,:) = scanbuf(1:end-1,:);
%     scanbuf(1,:) = s;  
%     scan_std = nanstd(scanbuf,1);
%   end
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%%% Detect Motion
  %   %% Detect strong reflections
%   detectMode = 'None';
%   minThreshold = 0.2;
%   minThreshold = 3*scanSTD;
%   [detectList,threshold] = detect(Rbin_m,s_fast,detectMode,minThreshold);

  % Read the initial scan msg.  Analyze the header for how many follow
  [msg,msg_type,msg_id,ip_addr] = read_pckt;
  [scan_info,msg_type,msg_id] = parse_msg(msg);
  s_raw = double(scan_info.scan_data);  % Save the good stuff. Append to this later

  % Reading the entire waveform scan   
  for j = 1:scan_info.num_msgs-1
      [msg,msg_type,msg_id,ip_addr] = read_pckt;
      [scan_info,msg_type,msg_id] = parse_msg(msg);
      s_raw = [s_raw, double(scan_info.scan_data)];
  end
  s = s_raw;
%   SA_saved = [SA_saved; s];

  t_loop = toc(t0);
  t1 = tic;
  
  %% %%%%%%%%%%%%%%%%%%%%%%%%%%  Plot
  if i > mem_depth

    % The button toggles the enable. If ~enabled then replace the scan with NotANumber vec (Matlab will not plot this.)
    if plot_raw; raw_plot = s_norm; else raw_plot = NaN(1,scan_len); end
    if plot_bpf; bpf_plot = s_bpf; else bpf_plot = NaN(1,scan_len); end
    if plot_motion; motion_plot = s_motion; else motion_plot = NaN(1,scan_len); end
    if plot_persist && plot_persist; persist_plot = s_persist; else persist_plot = NaN(1,scan_len); end
    if plot_env; env_plot = s_env; else env_plot = NaN(1,scan_len); end

    switch gui_type
      case 'waterfall'  % the label is opposite what's being displayed
  %         plotpulseresponse(gca, Rbin_m, rawDisplay, mofDisplay, mafDisplay, ...
  %           envDisplay, detDisplay, threshDisplay, ylimVec);
  %        PlotStuff(Rbin_m,line1,line2,line3,line4,line5,line6,ylimVec);
        plot_pulse(rbin,raw_plot,bpf_plot,motion_plot,persist_plot,env_plot)
      case 'pulse'  % the label is opposite what's being displayed
        % Select plot mode and convert to deciBels because they make better waterfalls
        if plot_env; s_db = 20*log10(s_env); % Display envelope scan if button is good
  %         elseif mafEnable; scanDisp_dB = 20*log10(abs(mafDisplay)); % display matched filtered scan
  %         elseif mofEnable; scanDisp_dB = 20*log10(abs(mofDisplay)); % display motion filtered scan
  %         else scanDisp_dB = 20*log10(abs(rawDisplay)); % display raw
        end
        wf_depth = 30;
        caxis_lim = [-11 5];
        plot_waterfall(rbin, s_db, wf_depth, caxis_lim);
      case 'WEDGE'
        plotwedge(gca, rangebin_m, scanEnv, detectList);
      case 'FFT'
        PlotFFT(s_fast);
      case 'PULSE_AND_WATERFALL'
        if ~exist('ax1','var')
          ax1 = subplot(3,1,1);
          ax2 = subplot(3,1,2:3);
        end
        plot_pulse(ax1,rangebin_m,s_raw,s_motion,s_mo_env)
        s_db = 20*log10(s_mo_env); % Display envelope scan if button is good
        plot_waterfall(ax2,rangebin_m,s_db,wf_depth,caxis_lim)

    end
  end

  drawnow

  t_plot = toc(t1);
  t_total = toc(t0);
  fprintf('t_loop=%3.2f, t_plot=%3.2f, t_total=%3.2f (%3.2fHz)\n',...
          t_loop*1000,t_plot*1000,t_total*1000,1/t_total);

end % while stop button not clicked (on GUI)

% logfn = sprintf('SA_%s.mat',datetime);
% save(logfn,'SA_saved')  % uncomment to save scans to a .mat file

  %% callback function for stop button
  function cbFunction(obj,eventdata)
    btn_label = get(obj,'String');
%    objVal = get(obj,'Value');

    switch btn_label

      case 'raw'
        plot_raw = ~plot_raw;
        if plot_raw
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end

      case 'bpf'
        plot_bpf = ~plot_bpf;
        if plot_bpf
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end

      case 'motion'
        plot_motion = ~plot_motion;
        if plot_motion
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
        
      case 'persist'
        plot_persist = ~plot_persist;
        if plot_persist
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
        
      case 'envelope'
        plot_env = ~plot_env;
        if plot_env
            set(obj,'BackgroundColor','g'); 
        else
            set(obj,'BackgroundColor',[.5 .5 .5]); 
        end
        
      case 'pulse' % set gui type to waterfall
        cla
        gui_type = 'waterfall';
        set(obj,'string',gui_type,'BackgroundColor',rgb('orange')); 
        
      case 'waterfall' % set gui type to pulse
        cla
        gui_type = 'pulse';
        set(obj,'string',gui_type,'BackgroundColor',rgb('lightblue')); 

      case 'snap'
        new_snap = true;
        set(obj,'BackgroundColor',[.5 .5 .5]);
        s_snap = s_bpf;
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








