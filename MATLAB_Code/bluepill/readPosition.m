function [position,measurement] = readPosition(device)

[reply,measurement] = sendSTM32Command( ...
    device,"P",0.50,2,["POS="]);

tokens = regexp(reply,"^POS=(-?\d+)$","tokens","once");

if isempty(tokens)
    error("STM32:MalformedPosition", ...
        "Malformed POS response: %s",reply);
end

position = str2double(tokens{1});
end