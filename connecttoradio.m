function s = connecttoradios(varargin)
%CONNECTRADIOS Summary of this function goes here
%   Detailed explanation goes here

% OS X
path = '/dev';
mask = 'tty.usbmodem*';
portListSA = dir(fullfile(path,mask));

switch length(portListSA)
  case 0  % No SPP usb device found
    error(sprintf('No USB device found at %s',fullfile(path,mask)));
  case 1
    portName = fullfile(path,portListSA(1).name);
  otherwise
    portNameList = {portListSA.name};
    [selection,ok] = listdlg('PromptString','Select one:',...
                    'SelectionMode','single',...
                    'ListString',portNameList)
    portName = fullfile(path,selection);
end
  
  %% Configure and open serial object s.  
  s = serial(portName, ...
    'BaudRate',115200,'DataBits',8,'Parity','none','StopBits',1, ...
    'InputBufferSize',16000, ...
    'BytesAvailableFcnCount', 1, ...
    'BytesAvailableFcnMode', 'byte', ...
    'ReadAsyncMode','continuous' ...
    );
  fopen(s)
  fprintf('opened %s -> s\n',portName);

