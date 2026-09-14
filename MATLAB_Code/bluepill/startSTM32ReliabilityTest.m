clear;
clc;

device = [];

try
    device = connectSTM32("COM4",9600);

    % Explicitly establish the safe state before the long test.
    stopConfirmed = stopMotor(device);

    if ~stopConfirmed
        error("STM32:InitialStopUnconfirmed", ...
            "The initial STOP command was not confirmed.");
    end

    [results,summary] = runSTM32ReliabilityTest( ...
        device,10,0.10);

    timestamp = string(datetime( ...
        "now","Format","yyyyMMdd_HHmmss"));

    csvFile = "STM32_Reliability_" + timestamp + ".csv";
    matFile = "STM32_Reliability_" + timestamp + ".mat";

    writetable(results,csvFile);
    save(matFile,"results","summary");

    fprintf("Saved CSV: %s\n",csvFile);
    fprintf("Saved MAT: %s\n",matFile);

catch ME
    fprintf(2,"\nRELIABILITY TEST FAILED\n");
    fprintf(2,"Identifier: %s\n",ME.identifier);
    fprintf(2,"Message: %s\n",ME.message);
end

if ~isempty(device)
    try
        stopMotor(device);
    catch
    end

    try
        disconnectSTM32(device);
    catch
    end
end

clear device;

pause(0.25);

fprintf("\nAvailable ports after test:\n");
disp(serialportlist("available"));