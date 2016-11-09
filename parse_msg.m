function [DS,msg_type,msgID] = parse_msg(msg)

msg_type = dec2hex(typecast([msg(2) msg(1)],'uint16'));
msgID = typecast([msg(4) msg(3)],'uint16');

switch msg_type
  case '1102'
    % MRM_GET_CONFIG_CONFIRM
    DS.node_id = typecast([msg(8) msg(7) msg(6) msg(5)],'uint32');
    DS.scan_start_ps = typecast([msg(12) msg(11) msg(10) msg(9)],'int32');
    DS.scan_stop_ps = typecast([msg(16) msg(15) msg(14) msg(13)],'int32');
    DS.scan_step_bins = typecast([msg(18) msg(17)],'uint16');
    DS.pulse_integration_index = typecast([msg(20) msg(19)],'uint16');
    DS.seg1_num_samples = typecast([msg(22) msg(21)],'uint16');
    DS.seg2_num_samples = typecast([msg(24) msg(23)],'uint16');
    DS.seg3_num_samples = typecast([msg(26) msg(25)],'uint16');
    DS.seg4_num_samples = typecast([msg(28) msg(27)],'uint16');
    DS.seg1_multiple = uint8(msg(29));
    DS.seg2_multiple = uint8(msg(30));
    DS.seg3_multiple = uint8(msg(31));
    DS.seg4_multiple = uint8(msg(32));
    DS.antenna_mode = uint8(msg(33));
    DS.transmit_gain = uint8(msg(34));
    DS.code_channel = uint8(msg(35));
    DS.persist_flag = uint8(msg(36));
    DS.timestamp = typecast([msg(40) msg(39) msg(38) msg(37)],'uint32');
    
  case '1103'
    % MRM_CONTROL_CONFIRM
    DS.stat = typecast([msg(8) msg(7) msg(6) msg(5)],'uint32');
    
  case 'F201'
    % MRM_SCAN_INFO
    DS.node_id = typecast([msg(8) msg(7) msg(6) msg(5)],'uint32');
    DS.timestamp = typecast([msg(12) msg(11) msg(10) msg(9)],'uint32');
    DS.scan_start_ps = typecast([msg(32) msg(31) msg(30) msg(29)],'uint32');
    DS.scan_stop_ps = typecast([msg(36) msg(35) msg(34) msg(33)],'uint32');
    DS.scan_step_bins =  typecast([msg(38) msg(37)],'uint16');
    DS.antenna_id = uint8(msg(39));
    DS.scan_type = uint8(msg(41));
    DS.op_mode = uint8(msg(42));
    DS.msg_samples = typecast([msg(44) msg(43)],'uint16');
    DS.total_samples = typecast([msg(48) msg(47) msg(46) msg(45)],'uint32');
    DS.msg_index = typecast([msg(50) msg(49)],'uint16');
    DS.num_msgs = typecast([msg(52) msg(51)],'uint16');
    DS.scan_data = repmat(int32(0),1,single(DS.msg_samples));
    for i = 1:DS.msg_samples
      j = 4*(i - 1);
      k = ((j + 3):-1:j) + 53;
      DS.scan_data(i) = typecast(msg(k),'int32');
    end
    
  otherwise
    DS = [];
    
end
