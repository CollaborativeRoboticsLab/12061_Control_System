function [reply,measurement] = sendSTM32Command( ...
    device,command,timeout,retries,expectedPrefixes)
%SENDSTM32COMMAND Send one command and wait for one matching response.
%
% Unsolicited ALIVE lines are counted and ignored.
%
% Example:
%   reply = sendSTM32Command(s,"PING",0.5,3,"PONG")
%
% Outputs:
%   reply       - Valid response received from the STM32
%   measurement - Timing and parser information

arguments
    device
    command (1,1) string
    timeout (1,1) double {mustBePositive} = 0.50
    retries (1,1) double {mustBePositive,mustBeInteger} = 3
    expectedPrefixes string = strings(0,1)
end

command = upper(strtrim(command));
expectedPrefixes = string(expectedPrefixes(:));

if strlength(command) == 0
    error("STM32:EmptyCommand", ...
        "The command cannot be empty.");
end

reply = "";

measurement = struct( ...
    "Command",command, ...
    "Reply","", ...
    "RoundTripSeconds",NaN, ...
    "Attempts",0, ...
    "IgnoredAlive",0, ...
    "IgnoredLines",strings(0,1));

for attempt = 1:retries

    measurement.Attempts = attempt;

    transactionTimer = tic;

    writeline(device,command);

    while toc(transactionTimer) < timeout

        if device.NumBytesAvailable == 0
            pause(0.001);
            continue;
        end

        try
            candidate = strtrim(string(readline(device)));
        catch readError
            measurement.IgnoredLines(end+1,1) = ...
                "READ_ERROR:" + string(readError.identifier);
            pause(0.001);
            continue;
        end

        % Ensure the parser handles only one scalar line.
        if isempty(candidate)
            continue;
        end

        candidate = candidate(1);

        if strlength(candidate) == 0
            continue;
        end

        % Ignore asynchronous diagnostic heartbeat messages.
        if strcmp(candidate,"ALIVE")
            measurement.IgnoredAlive = ...
                measurement.IgnoredAlive + 1;
            continue;
        end

        % Firmware explicitly rejected the command.
        if startsWith(candidate,"ERR=")
            measurement.RoundTripSeconds = ...
                toc(transactionTimer);

            measurement.Reply = candidate;

            error("STM32:FirmwareError", ...
                "STM32 rejected command '%s' with response '%s'.", ...
                command,candidate);
        end

        if isExpectedResponse(candidate,expectedPrefixes)

            reply = candidate;

            measurement.RoundTripSeconds = ...
                toc(transactionTimer);

            measurement.Reply = reply;

            return;
        end

        measurement.IgnoredLines(end+1,1) = candidate;
    end
end

error("STM32:ResponseTimeout", ...
    "No expected response to '%s' after %d attempts. " + ...
    "Port=%s, timeout=%.3f seconds.", ...
    command,retries,device.Port,timeout);

end


function tf = isExpectedResponse(candidate,expectedPrefixes)
%ISEXPECTEDRESPONSE Return one scalar logical result.

candidate = string(candidate);
candidate = candidate(1);

expectedPrefixes = string(expectedPrefixes(:));

if isempty(expectedPrefixes)
    tf = true;
    return;
end

exactMatches = strcmp(candidate,expectedPrefixes);
prefixMatches = startsWith(candidate,expectedPrefixes);

tf = any(exactMatches(:)) || any(prefixMatches(:));

end