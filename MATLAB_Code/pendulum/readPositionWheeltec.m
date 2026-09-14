function [position,measurement] = readPositionWheeltec(device)
%READPOSITIONWHEELTEC Cart position, TIM4 encoder counts.
%
%   Rail centre is about 7925; the firmware's own end stops are 5900 and
%   9900. Goes through pendulumCommand rather than readline.

[reply,measurement] = pendulumCommand(device,"P",0.50,3,"POS=");

tokens = regexp(reply,"^POS=(-?\d+)$","tokens","once");
if isempty(tokens)
    error("STM32:MalformedPosition","Malformed POS response: %s",reply);
end
position = str2double(tokens{1});
end
