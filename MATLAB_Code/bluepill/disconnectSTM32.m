function report = disconnectSTM32(device)

report = struct( ...
    "StopConfirmed",false, ...
    "CallbacksDisabled",false, ...
    "ControlLinesDisabled",false);

if isempty(device)
    return;
end

try
    report.StopConfirmed = stopMotor(device);
catch
end

try
    configureCallback(device,"off");
    report.CallbacksDisabled = true;
catch
end

try
    setDTR(device,false);
    setRTS(device,false);
    report.ControlLinesDisabled = true;
catch
end

try
    flush(device);
catch
end

fprintf("Disconnect preparation complete.\n");
fprintf("Now clear the serial object in the caller using:\n");
fprintf("  clear device\n");
end