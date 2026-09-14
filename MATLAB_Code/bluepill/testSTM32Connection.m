clear;
clc;

device = [];

try
    device = connectSTM32("COM4",9600);

    fprintf("\nTesting STATUS...\n");
    status = readStatus(device);
    disp(status);

    fprintf("\nTesting angle request...\n");
    [angleRaw,angleTiming] = readAngle(device);
    fprintf("Angle raw: %d\n",angleRaw);
    fprintf("Angle RTT: %.3f ms\n", ...
        1000 * angleTiming.RoundTripSeconds);

    fprintf("\nTesting encoder-position request...\n");
    [position,positionTiming] = readPosition(device);
    fprintf("Position: %d\n",position);
    fprintf("Position RTT: %.3f ms\n", ...
        1000 * positionTiming.RoundTripSeconds);

    fprintf("\nTesting combined DATA request...\n");
    data = readSTM32Data(device);
    disp(data);

    fprintf("\nTesting safe STOP...\n");
    stopConfirmed = stopMotor(device);
    fprintf("STOP confirmed: %d\n",stopConfirmed);

catch ME
    fprintf(2,"\nTEST FAILED\n");
    fprintf(2,"Identifier: %s\n",ME.identifier);
    fprintf(2,"Message: %s\n",ME.message);

    if ~isempty(device)
        try
            stopMotor(device);
        catch
        end
    end
end

if ~isempty(device)
    disconnectSTM32(device);
end

clear device;

pause(0.25);

fprintf("\nAvailable ports after disconnect:\n");
disp(serialportlist("available"));