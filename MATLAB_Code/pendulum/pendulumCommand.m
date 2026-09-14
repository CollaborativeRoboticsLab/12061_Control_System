function [reply,measurement] = pendulumCommand(device,command,timeout,retries,expectedPrefixes)
%PENDULUMCOMMAND Send one command, get one reply. Raw bytes, no readline.
%
%   Drop-in replacement for sendSTM32Command on the pendulum rig. Same
%   arguments, same two outputs, so swapping it in is one line per file.
%
%   WHY THIS EXISTS
%   probeCommandRx proved the board answers PING with PONG when the bytes
%   are written and read directly. The same exchange through writeline /
%   readline times out with "unable to read any data", i.e. bytes arrive
%   but MATLAB never finds the line terminator it was configured with.
%   Rather than keep guessing at terminator semantics, this function does
%   its own framing: write the command plus CR, then read raw bytes and
%   split them on CR or LF itself.
%
%   It also has to share the wire with the binary DataScope stream, whose
%   frames contain 0x0D and therefore look like line breaks. Any segment
%   containing a non-printable byte is discarded, so binary frames cannot
%   be mistaken for a reply.

arguments
    device
    command          (1,1) string
    timeout          (1,1) double {mustBePositive} = 0.50
    retries          (1,1) double {mustBePositive,mustBeInteger} = 3
    expectedPrefixes string = strings(0,1)
end

command          = upper(strtrim(command));
expectedPrefixes = string(expectedPrefixes(:));

if strlength(command) == 0
    error("STM32:EmptyCommand","The command cannot be empty.");
end

reply = "";
measurement = struct("Command",command,"Reply","","RoundTripSeconds",NaN, ...
    "Attempts",0,"IgnoredAlive",0,"IgnoredLines",strings(0,1));

txBytes = uint8([char(command) 13]);        % command + CR

for attempt = 1:retries
    measurement.Attempts = attempt;

    flush(device);                          % drop stale binary / old replies
    t0  = tic;
    write(device,txBytes,"uint8");

    buf = uint8([]);
    while toc(t0) < timeout
        n = device.NumBytesAvailable;
        if n == 0
            pause(0.002);
            continue;
        end
        buf = [buf; uint8(read(device,n,"uint8")')];        %#ok<AGROW>

        [lines,buf] = takeLines(buf);
        for k = 1:numel(lines)
            candidate = lines(k);

            if candidate == "ALIVE"
                measurement.IgnoredAlive = measurement.IgnoredAlive + 1;
                continue;
            end

            if startsWith(candidate,"ERR=")
                measurement.RoundTripSeconds = toc(t0);
                measurement.Reply = candidate;
                error("STM32:FirmwareError", ...
                    "STM32 rejected command '%s' with response '%s'.",command,candidate);
            end

            if isExpected(candidate,expectedPrefixes)
                reply = candidate;
                measurement.RoundTripSeconds = toc(t0);
                measurement.Reply = reply;
                return;
            end

            measurement.IgnoredLines(end+1,1) = candidate;   %#ok<AGROW>
        end
    end
end

error("STM32:ResponseTimeout", ...
    "No expected response to '%s' after %d attempts. Port=%s, timeout=%.3f s.", ...
    command,retries,device.Port,timeout);
end


function [lines,rest] = takeLines(buf)
%TAKELINES Split on CR or LF. Keep only all-printable segments, so binary
%   DataScope frames (which contain 0x0D) cannot pose as replies.
lines = strings(0,1);
brk   = find(buf == 13 | buf == 10);

if isempty(brk)
    rest = buf;
    return;
end

start = 1;
for k = 1:numel(brk)
    seg   = buf(start:brk(k)-1);
    start = brk(k) + 1;
    if isempty(seg), continue; end
    if all(seg >= 32 & seg <= 126)
        lines(end+1,1) = string(char(seg'));   %#ok<AGROW>
    end
end
rest = buf(start:end);
end


function tf = isExpected(candidate,expectedPrefixes)
if isempty(expectedPrefixes)
    tf = true;
    return;
end
tf = any(strcmp(candidate,expectedPrefixes)) || any(startsWith(candidate,expectedPrefixes));
end
