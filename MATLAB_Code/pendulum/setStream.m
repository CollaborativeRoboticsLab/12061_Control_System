function measurement = setStream(device,enable)
%SETSTREAM Turn the binary DataScope waveform stream on or off.
%
%   The firmware boots with streaming ON, so an untouched board still
%   works with the WHEELTEC upper-computer. The first ASCII command it
%   receives switches streaming OFF automatically, so MATLAB normally
%   never needs this function.
%
%   Call setStream(device,true) only if you want to hand the board back
%   to the WHEELTEC PC software. While streaming is on, readline() will
%   see binary frame bytes and every read will fail.

arguments
    device
    enable (1,1) logical
end

command  = "STREAM," + string(double(enable));
expected = "STREAM=" + string(double(enable));

[reply,measurement] = pendulumCommand(device,command,0.50,2,["STREAM="]);

if reply ~= expected
    error("STM32:MalformedStream","Unexpected STREAM response: %s",reply);
end
end
