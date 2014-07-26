function cfgStruct = mrmgetcfg(s,msgID)
% VIEW_CFG Function to view configuration.
%
% Syntax
% view_cfg(srl)
%
% Input
% srl - serial object
%
% Output
% NONE
%
% Usage Notes
% Input srl can be an array of objects.
%
% See also OPEN_COM_PORT.
%
% Copyright © 2014 Time Domain, Huntsville, AL

%% Setup and write the config request
SYNC_PAT = uint16(hex2dec('A5A5'));
PCKT_LEN = uint16(4);
MSG_TYPE = uint16(hex2dec('1002')); 
MSG_ID = uint16(msgID);
GET_CFG_RQST = typecast(swapbytes([SYNC_PAT PCKT_LEN MSG_TYPE MSG_ID]),'uint8');
fwrite(s,GET_CFG_RQST,'uint8');

%% Catch the RCM_SET_SLEEPMODE_CONFIRM response: F105, 12 bytes (with serial prefix)
msgLen_confirm = 48;
Ktry = 0;
while s.BytesAvailable < msgLen_confirm && Ktry < 1000
  Ktry = Ktry + 1;
  pause(0.0001)
end
%Ktry

if Ktry < 1000  % Success!
  msg = uint8(fread(s,s.BytesAvailable,'uint8'));
  if length(msg) > msgLen_confirm
    msg = msg(1:msgLen_confirm);
    warning('** extra bytes in serial buffer removed.')
  end
  [cfgStruct,msgType,msgID] = parsemsg(msg);
else
  error('** no response from radio.');
end

