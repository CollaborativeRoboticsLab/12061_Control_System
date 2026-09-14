function [gains,measurement] = readGains(device)
%READGAINS Read the controller gains currently live in the firmware.
%
%   Since R4-5 the angle loop is PID, not PD, so K returns FIVE numbers:
%       Balance_KP, Balance_KD, Balance_KI, Position_KP, Position_KD
%   A four-value reply means the board is still running R4-4 or earlier and
%   has no integral term at all, which Lab 4 needs. That is reported as a
%   firmware-version problem rather than as a parse failure, because the
%   fix is to reflash, not to debug MATLAB.

[reply,measurement] = pendulumCommand(device,"K",0.50,2,["K="]);

num = "(-?\d+\.\d)";
five = regexp(reply,"^K=" + join(repmat(num,1,5),",") + "$","tokens","once");

if ~isempty(five)
    v = str2double(five);
    gains = struct("Balance_KP",v(1),"Balance_KD",v(2),"Balance_KI",v(3), ...
                   "Position_KP",v(4),"Position_KD",v(5));
    return;
end

four = regexp(reply,"^K=" + join(repmat(num,1,4),",") + "$","tokens","once");
if ~isempty(four)
    error("STM32:FirmwareTooOld", ...
        ['The board returned four gains, so it is running R4-4 or older ' ...
         'and its angle loop is pure PD. Lab 4 needs the integral term ' ...
         'added in R4-5. Rebuild and upload Control_System_lab_1.']);
end

error("STM32:MalformedGains","Malformed K response: %s",reply);
end
