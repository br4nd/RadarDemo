function mrmsetcfg(s,cfgStruct,msgID)
% VIEW_CFG Function to view configuration.
%
% Syntax
% chng_cfg(srl,Tscn,Gtx,PII)
%
% Input
% srl - serial object
% Tscn - two element array with scan start and stop times in ns
% Gtx - transmit gain [0 (low) - 63 (high)]
% PII - pulse integration index (integration = 2^PII)
%
% Output
% NONE
%
% Usage Notes
% Input srl can be an array of objects.
%
% See also OPEN_COM_PORT, VIEW_CFG.
%
% Copyright © 2014 Time Domain, Huntsville, AL

%% Setup the message and send
DAT = str2dat(rmfield(cfgStruct,'timeStamp'));
SYNC_PAT = uint16(hex2dec('A5A5'));
PCKT_LEN = uint16(length(DAT)+4);
MSG_TYPE = uint16(hex2dec('1001')); 
MSG_ID = uint16(msgID);
SET_CFG_RQST = [typecast(swapbytes([SYNC_PAT PCKT_LEN MSG_TYPE MSG_ID]),'int8') DAT];
fwrite(s,SET_CFG_RQST,'int8')

%% Catch the RCM_SET_SLEEPMODE_CONFIRM response: F105, 12 bytes (with serial prefix)
msgLen_confirm = 12;
Ktry = 0;
while s.BytesAvailable < msgLen_confirm && Ktry < 1000
  Ktry = Ktry + 1;
  pause(0.0001)
end
%fprintf('Ktry:%d\n',Ktry);

if Ktry < 1000  % Success!
  msg = uint8(fread(s,s.BytesAvailable,'uint8'));
  if length(msg) > msgLen_confirm
    msg = msg(1:msgLen_confirm);
    warning('** extra bytes in serial buffer removed.')
  end
else
  error('** no response from radio.');
end

% Check msgType returned from radio.
msgType_confirm = '1101'; % MRM_S_CONFIRM message
msgType_returned = dec2hex(typecast(msg([6 5]),'uint16'),4);
if ~strcmp(msgType_returned,msgType_confirm)
  error('** wrong msgType from radio: %s',cfgStructOut.msgType);
end

% Check msgID returned.  Should match sent.
msgID_returned = typecast(msg([8 7]),'uint16');
if msgID_returned ~= msgID
  error('** wrong msgID returned from radio: %s',msgID_returned);
end

