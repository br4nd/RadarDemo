function mrmcontrolreq(s,Nscans,dT,msgID)
% CTL_RQST Control request function.
%
% Syntax
% ctl_rqst(srl,Nscn,dTscn,msgID)
% CTL_RQST = ctl_rqst(srl,Nscn,dTscn,msgID)
%
% Input
% srl - serial object
% Nscn - number of scans
% dTscn - interval between start of multiple scans (us)
% msgID - integer message ID
%
% Output
% CTL_RQST - control request message bytes array
%
% Usage Notes
% Creates control request message. If no output argument is specified, the
% message is written to the serial port. If the output argment is
% specified, the message bytes are returned as an array and can be written
% manually using FWRITE.
%
% Copyright © 2014 Time Domain, Huntsville, AL

SYNC_PAT = uint16(hex2dec('A5A5'));
PCKT_LEN = uint16(12);
MSG_TYPE = uint16(hex2dec('1003')); 
MSG_ID = uint16(msgID);  % message ID
SCN_CNT = uint16(Nscans);  % number of scans
RSRV = uint16(0);
SCN_INT_TM = uint32(dT);  % time in us between scans
CTL_RQST = [typecast(swapbytes([SYNC_PAT PCKT_LEN MSG_TYPE MSG_ID SCN_CNT RSRV]),'uint8') typecast(swapbytes(SCN_INT_TM),'uint8')];
fwrite(s,CTL_RQST,'uint8');

%% Catch the RCM_SET_SLEEPMODE_CONFIRM response: F105, 12 bytes (with serial prefix)
msgLen_confirm = 12;

Ktry = 0;
while s.BytesAvailable < msgLen_confirm && Ktry < 1000
  Ktry = Ktry + 1;
  pause(0.0001)
end
%fprintf('Ktry:%d\n',Ktry);

if Ktry < 1000  % Success!
  msg = uint8(fread(s,msgLen_confirm,'uint8'));
else
  error('** no response from radio.');
end

% Check msgType returned from radio.
msgType_confirm = '1103'; % MRM_CONTROL_CONFIRM message
msgType_returned = dec2hex(typecast(msg([6 5]),'uint16'),4);
if ~strcmp(msgType_returned,msgType_confirm)
  error('** wrong msgType from radio: %s',msgType_returned);
end

% Check msgID returned.  Should match sent.
msgID_returned = typecast(msg([8 7]),'uint16');
if msgID_returned ~= msgID
  error('** wrong msgID returned from radio: %s',msgID_returned);
end

