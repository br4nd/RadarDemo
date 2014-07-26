function scanData = mrmgetscaninfo(s,Nbin)
% MRMGETSCANINFO 

%% Read the first message to find out how many more
Ktry = 0;
bytesPerMsg = 1456;
while s.BytesAvailable < bytesPerMsg && Ktry < 1000
  Ktry = Ktry + 1;
  pause(0.0001)
end
%Ktry

if Ktry == 1000
  error('** no response from radar');
end

%s.BytesAvailable
msg = uint8(fread(s,bytesPerMsg,'uint8'));
[scanStruct,msgType,msgID] = parsemsg(msg);
%scanStruct

%% Read in the first chunk of data
scanData = scanStruct.scanData;

%% Read the rest of the data
for i = 2:scanStruct.numberMessages
  msg = uint8(fread(s,bytesPerMsg,'uint8'));
  [scanStruct,msgType,msgID] = parsemsg(msg);
  scanData = [scanData scanStruct.scanData];
end

% if s.BytesAvailable > 0
%   msg = fread(s,s.BytesAvailable,'uint8');
%   warning('Flushed buffer')
% end



  
% SCNmsgNbin = 350;  % number of bins in each message (see API)
% USBpfxNbyt = 4;
% %CFRMmsgNbyt = 8;
% SCNmsgNbyt = 1452;
% Nscn = 1;
% Nmsg = ceil(Nbin/SCNmsgNbin);
% totNbyt = USBpfxNbyt + Nscn*Nmsg*(USBpfxNbyt + SCNmsgNbyt);
% 
% Ktry = 0;
% while s.BytesAvailable < totNbyt && Ktry <= 1000
%   Ktry = Ktry + 1;
%   pause(0.0001)
% end
% Ktry
% 
% if Ktry <= 1000  
%   msg = uint8(fread(s,s.BytesAvailable,'uint8'));
%   
%   Ibyt = 1;
%   Ibyt = Ibyt + USBpfxNbyt;
%   [str,msgType,msgID] = parsemsg(msg(Ibyt:Ibyt+CFRMmsgNbyt-1));
%   Ibyt = Ibyt + CFRMmsgNbyt;
% 
%   scanData = [];
%   for n = 1:Nmsg
%     Ibyt = Ibyt + USBpfxNbyt;
%     [str,msgType,msgID] = parsemsg(msg(Ibyt:Ibyt+SCNmsgNbyt-1));
%     Ibyt = Ibyt + SCNmsgNbyt;
%     
%     scanData = [scanData str.scanData(1:str.messageSamples)];
%   end
%   
% else
%   error('** no response from radio.')
% end


%{
Here is a list of messages and bytes that result from the control request:

     NUM       FIRST        LAST
   BYTES        BYTE        BYTE
                                    ----- MRM_CONTROL_CONFIRM -----
       4           1           4    USB prefix (0xA5A5 & number of bytes)
       8           5          12    message data

                                    ----- MRM_SCAN_INFO -----
       4          13          16    USB prefix (0xA5A5 & number of bytes)
      52          17          68    message header data
    1400          69        1468    message scan data (message 1/4) [350 bins]
       4        1469        1472    USB prefix (0xA5A5 & number of bytes)
      52        1473        1524    message header data
    1400        1525        2924    message scan data (message 2/4) [350 bins]
       4        2925        2928    USB prefix (0xA5A5 & number of bytes)
      52        2929        2980    message header data
    1400        2981        4380    message scan data (message 3/4) [350 bins]
       4        4381        4384    USB prefix (0xA5A5 & number of bytes)
      52        4385        4436    message header data
    1400        4437        5836    message scan data (message 4/4) [350 bins]

Note that the last MRM_SCAN_INFO message only contains 6 bins with valid
scan values.

Note that the very last byte value (5836), which is the total number of
bytes, is the BytesAvailable number that triggers fread.
%}
