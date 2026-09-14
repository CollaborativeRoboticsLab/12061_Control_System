function [angleRaw,measurement] = readAngle(device)

[reply,measurement] = sendSTM32Command( ...
    device,"A",0.50,2,["ANGLE="]);

tokens = regexp(reply,"^ANGLE=(\d+)$","tokens","once");

if isempty(tokens)
    error("STM32:MalformedAngle", ...
        "Malformed ANGLE response: %s",reply);
end

angleRaw = str2double(tokens{1});

if angleRaw < 0 || angleRaw > 4095
    warning("STM32:AngleOutOfRange", ...
        "Raw ADC value is outside the expected 12-bit range: %g", ...
        angleRaw);
end
end