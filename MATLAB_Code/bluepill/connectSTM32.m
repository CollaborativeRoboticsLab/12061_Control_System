function device = connectSTM32(port,baudRate)
%CONNECTSTM32 Open and validate the STM32 serial connection.
%
% device = connectSTM32("COM4",9600)
%
% The function:
%   1. verifies that the port exists;
%   2. verifies that it is available;
%   3. opens it only once;
%   4. immediately disables DTR and RTS;
%   5. configures the CR terminator;
%   6. removes stale input;
%   7. requires a valid PONG response.

arguments
    port (1,1) string = "COM4"
    baudRate (1,1) double {mustBePositive,mustBeInteger} = 9600
end

allPorts = string(serialportlist("all"));
availablePorts = string(serialportlist("available"));

if ~any(strcmpi(allPorts,port))
    error("STM32:PortNotDetected", ...
        "%s is not detected by Windows/MATLAB.",port);
end

if ~any(strcmpi(availablePorts,port))
    error("STM32:PortUnavailable", ...
        "%s exists but is unavailable. Close PlatformIO Monitor, " + ...
        "other terminal programs, Simulink models, and other MATLAB sessions.", ...
        port);
end

device = [];

try
    device = serialport(port,baudRate, ...
        "DataBits",8, ...
        "StopBits",1, ...
        "Parity","none", ...
        "FlowControl","none", ...
        "Timeout",0.75);

    % Critical CH9102 configuration.
    setDTR(device,false);
    setRTS(device,false);

    % Preserve the proven outgoing command terminator.
    configureTerminator(device,"CR/LF","CR");

    % Ensure no callback from an earlier experiment is active.
    configureCallback(device,"off");

    % Allow the USB/serial path to settle without reopening the port.
    pause(0.30);

    % Remove startup messages and genuinely stale data.
    startupLines = drainSTM32Input(device,0.50);

    if ~isempty(startupLines)
        fprintf("Discarded startup/stale lines:\n");
        for k = 1:numel(startupLines)
            fprintf("  %s\n",startupLines(k));
        end
    end

    % Validate end-to-end transmission and reception.
    reply = sendSTM32Command(device,"PING",0.50,3,["PONG"]);

    if reply ~= "PONG"
        error("STM32:UnexpectedPingReply", ...
            "Expected PONG but received: %s",reply);
    end

    fprintf("STM32 connection established successfully.\n");
    fprintf("  Port: %s\n",device.Port);
    fprintf("  Baud rate: %d\n",device.BaudRate);
    fprintf("  DTR: false\n");
    fprintf("  RTS: false\n");
    fprintf("  Verification: PING -> PONG\n");

catch ME
    if ~isempty(device)
        try
            configureCallback(device,"off");
        catch
        end

        try
            setDTR(device,false);
            setRTS(device,false);
        catch
        end
    end

    clear device
    rethrow(ME);
end
end