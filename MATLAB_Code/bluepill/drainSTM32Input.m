function lines = drainSTM32Input(device,drainDuration)
%DRAINSTM32INPUT Remove currently waiting startup or stale input safely.
%
% This function reads available bytes directly. It does not call readline,
% because NumBytesAvailable may represent only a partial line.

arguments
    device
    drainDuration (1,1) double {mustBeNonnegative} = 0.25
end

receivedBytes = uint8([]);
drainTimer = tic;

% Continue briefly so that an already arriving startup sequence can finish.
while toc(drainTimer) < drainDuration

    numberOfBytes = device.NumBytesAvailable;

    if numberOfBytes > 0
        newBytes = read(device,numberOfBytes,"uint8");
        receivedBytes = [receivedBytes; newBytes(:)]; %#ok<AGROW>
    else
        pause(0.005);
    end
end

if isempty(receivedBytes)
    lines = strings(0,1);
    return;
end

% Convert the received ASCII bytes into text.
receivedText = string(char(receivedBytes.'));

% Normalise CR/LF, isolated CR, and isolated LF into newline characters.
receivedText = replace(receivedText, ...
    [sprintf("\r\n"), sprintf("\r")], ...
    [sprintf("\n"),   sprintf("\n")]);

splitLines = split(receivedText,newline);
splitLines = strtrim(splitLines);

% Remove empty entries caused by a trailing terminator.
lines = splitLines(strlength(splitLines) > 0);
lines = lines(:);

end