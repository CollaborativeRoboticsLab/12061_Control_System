function [angleRaw,measurement] = readAngleWheeltec(device)
%READANGLEWHEELTEC Pendulum angle, raw ADC counts (0..4095).
%
%   Same job as the older readAngle.m, but goes through pendulumCommand,
%   which frames the reply itself instead of relying on readline. Hanging
%   should read about 1020, upright about 3100.

[reply,measurement] = pendulumCommand(device,"A",0.50,3,"ANGLE=");

tokens = regexp(reply,"^ANGLE=(\d+)$","tokens","once");
if isempty(tokens)
    error("STM32:MalformedAngle","Malformed ANGLE response: %s",reply);
end
angleRaw = str2double(tokens{1});

if angleRaw < 0 || angleRaw > 4095
    warning("STM32:AngleOutOfRange", ...
        "Raw ADC value outside the 12-bit range: %g",angleRaw);
end
end
