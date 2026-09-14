function [appliedPWM,measurement] = setMotorPWM(device,requestedPWM)

arguments
    device
    requestedPWM (1,1) double {mustBeFinite}
end

requestedPWM = round(requestedPWM);
requestedPWM = max(-999,min(999,requestedPWM));

command = "M," + string(requestedPWM);

[reply,measurement] = sendSTM32Command( ...
    device,command,0.50,2,["PWM="]);

tokens = regexp(reply,"^PWM=(-?\d+)$","tokens","once");

if isempty(tokens)
    error("STM32:MalformedPWM", ...
        "Malformed PWM acknowledgement: %s",reply);
end

appliedPWM = str2double(tokens{1});

if appliedPWM ~= requestedPWM
    warning("STM32:PWMMismatch", ...
        "Requested PWM=%d but STM32 acknowledged PWM=%d.", ...
        requestedPWM,appliedPWM);
end
end