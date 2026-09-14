function [zeroCounts,measurement] = setZero(device)
%SETZERO Take the current cart position as Position_Zero.

[reply,measurement] = pendulumCommand(device,"ZERO",0.50,2,["ZERO="]);

tokens = regexp(reply,"^ZERO=(-?\d+)$","tokens","once");
if isempty(tokens)
    error("STM32:MalformedZero","Malformed ZERO response: %s",reply);
end
zeroCounts = str2double(tokens{1});
end
