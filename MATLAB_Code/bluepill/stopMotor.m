function confirmed = stopMotor(device)

confirmed = false;

if isempty(device)
    return;
end

try
    [reply,~] = sendSTM32Command( ...
        device,"STOP",0.50,3,["STOPPED"]);

    confirmed = reply == "STOPPED";

    if confirmed
        fprintf("STM32 confirmed motor stop.\n");
    else
        warning("STM32:StopNotConfirmed", ...
            "Expected STOPPED but received %s.",reply);
    end

catch ME
    warning("STM32:StopFailed", ...
        "STOP could not be confirmed: %s",ME.message);
end
end