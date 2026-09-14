function result = probeCommandRx(port,baudRate,opts)
%PROBECOMMANDRX Find out whether the board RECEIVES commands at all.
%
%   probeCommandRx("COM4")
%
%   PING times out, but that single fact cannot tell you which half of the
%   link is broken. This test can, because the firmware gives us a second,
%   independent channel that needs only the board's TRANSMITTER:
%
%       stream_enable starts at 1, and the FIRST valid ASCII command sets
%       it to 0. So a command that is received and parsed makes the binary
%       DataScope stream STOP -- whether or not its reply ever gets back.
%
%   That splits the problem cleanly:
%
%     stream stops   -> RX works, the parser ran. The fault is on the way
%                       back: reply framing, terminator, or MATLAB's read.
%     stream continues -> the board never saw the command. The fault is on
%                       the way in: the RX pin, RXNEIE, the NVIC, or the
%                       USB adapter not driving the STM32's RX line.
%
%   Three line endings are tried in turn (CR, LF, CRLF) so a terminator
%   mismatch cannot be mistaken for a dead receiver.
%
%   The board is only sent the harmless PING command. Nothing moves.

arguments
    port     (1,1) string = "COM4"
    baudRate (1,1) double {mustBePositive,mustBeInteger} = 128000
    opts.SettleSeconds (1,1) double {mustBeNonnegative} = 2.6
end

FRAME_LEN = 14;

device = serialport(port,baudRate, ...
    "DataBits",8,"StopBits",1,"Parity","none", ...
    "FlowControl","none","Timeout",1.0);
cleaner = onCleanup(@() closePort(device));   %#ok<NASGU>

setDTR(device,false);
setRTS(device,false);
pause(opts.SettleSeconds);
flush(device);

%% Phase 0 — baseline: is the board streaming right now?
fprintf("Baseline: listening 2 s with nothing sent...\n");
base   = listenFor(device,2.0);
nBase  = countFrames(base,FRAME_LEN);
rBase  = nBase/2.0;
fprintf("  %d frames  (%.1f Hz)\n\n",nBase,rBase);

if nBase == 0
    fprintf(2,"The board is not streaming, so this test cannot run.\n");
    fprintf(2,"Either it is not powered, or a previous session already sent a\n");
    fprintf(2,"command and muted the stream. Power-cycle the board and retry.\n");
    result = struct("Conclusion","no baseline stream");
    return;
end

%% Phase 1 — send PING with three different line endings
variants = { ...
    "CR",    uint8([80 73 78 71 13]);        ... % PING\r
    "LF",    uint8([80 73 78 71 10]);        ... % PING\n
    "CRLF",  uint8([80 73 78 71 13 10])};    ... % PING\r\n

muted   = false;
sawText = "";
usedEnding = "";

for v = 1:size(variants,1)
    ending = variants{v,1};
    bytes  = variants{v,2};

    flush(device);
    fprintf("Sending PING with %s ending ...\n",ending);
    write(device,bytes,"uint8");

    LISTEN  = 1.5;
    after   = listenFor(device,LISTEN);
    nAfter  = countFrames(after,FRAME_LEN);
    txt     = printableRun(after);
    expect  = rBase*LISTEN;                 % frames if nothing had changed

    fprintf("  frames after : %d  (%.0f expected if unchanged)\n",nAfter,expect);
    if strlength(txt) > 0
        fprintf("  ASCII seen   : %s\n",txt);
        sawText = txt;
    end

    % A reply, or the stream collapsing, both mean the command got through.
    % Do not demand exactly zero frames: one frame is usually already in
    % flight when stream_enable is cleared.
    gotReply  = contains(txt,"PONG");
    collapsed = nAfter < 0.25*max(expect,1);

    if gotReply || collapsed
        muted = true;
        usedEnding = ending;
        if gotReply
            fprintf("  -> PONG received. This ending WORKS.\n\n");
        else
            fprintf("  -> stream collapsed. The board parsed this command.\n\n");
        end
        break;
    else
        fprintf("  -> no reply, stream unchanged. This ending did nothing.\n\n");
    end
end

%% Verdict
fprintf("--------------------------------------------------------\n");
if muted && contains(sawText,"PONG")
    conclusion = "all good";
    fprintf("EVERYTHING WORKS. The board received PING with the %s ending and\n",usedEnding);
    fprintf("replied PONG. The Route 4 command layer is proven on real hardware.\n\n");
    fprintf("Note: testing stops at the first ending that works, because the\n");
    fprintf("stream is now muted and later endings could not be told apart.\n");
    fprintf("CR is what MATLAB's writeline sends, so CR is the one that matters.\n\n");
    fprintf("If connectSTM32 still times out, it is NOT the command layer. Its\n");
    fprintf("three PING attempts land 0.8, 1.3 and 1.8 s after the port opens,\n");
    fprintf("while dropping DTR/RTS resets the board into a 2.0 s startup delay --\n");
    fprintf("so every attempt is spent before the board is listening.\n");
    fprintf("Use connectPendulum(""%s"") instead: same thing, but it waits first.\n",port);

elseif muted
    conclusion = "rx ok, tx reply missing";
    fprintf("RX WORKS, THE REPLY DOES NOT COME BACK.\n");
    fprintf("The stream stopped, so USART1 receive, the NVIC and ProcessCommand\n");
    fprintf("all ran. What did not arrive is the ""PONG\\r\\n"" reply.\n");
    fprintf("Look at cmd_putc in usart_cmd.c: it waits on the TC flag (SR bit 6)\n");
    fprintf("before writing DR, the same way DataScope does. If TC is never set\n");
    fprintf("the first reply byte blocks forever and the firmware hangs there --\n");
    fprintf("which would ALSO explain the stream never coming back.\n");
    fprintf("Check whether the board is still alive: re-run probeStreamTiming.\n");

else
    conclusion = "rx dead";
    fprintf("THE BOARD IS NOT RECEIVING. None of the three line endings had any\n");
    fprintf("effect, so the bytes are not reaching ProcessCommand.\n\n");
    fprintf("Ruled out already (verified in the built firmware):\n");
    fprintf("  - the USART1 vector does point at usart_cmd.c's handler\n");
    fprintf("  - MY_NVIC_Init handles IRQ 37 correctly (ISER[1] bit 5)\n\n");
    fprintf("Still to check, cheapest first:\n");
    fprintf("  1. Is PA10 (USART1_RX) actually wired to the USB adapter on this\n");
    fprintf("     board? The stock firmware never receives, so this path has\n");
    fprintf("     never been proven on this hardware.\n");
    fprintf("  2. uart_init leaves PA10 as input pull-DOWN (it sets CRH but not\n");
    fprintf("     ODR bit 10). A weakly driven RX line would fail this way.\n");
    fprintf("  3. Confirm UartCmd_Init() really runs: it must be called AFTER\n");
    fprintf("     uart_init(72,128000) in Minibalance.c.\n");
end
fprintf("--------------------------------------------------------\n");

result = struct("Conclusion",conclusion,"BaselineFrames",nBase, ...
    "BaselineHz",rBase,"Muted",muted,"Ending",usedEnding,"AsciiSeen",sawText);
end


function raw = listenFor(device,secs)
raw = uint8([]);
t0 = tic;
while toc(t0) < secs
    n = device.NumBytesAvailable;
    if n > 0
        raw = [raw; uint8(read(device,n,"uint8")')];   %#ok<AGROW>
    else
        pause(0.002);
    end
end
end


function n = countFrames(raw,FRAME_LEN)
n = 0;
if numel(raw) < FRAME_LEN, return; end
for k = 1:(numel(raw)-FRAME_LEN+1)
    if raw(k) == uint8('$') && raw(k+FRAME_LEN-1) == uint8(13)
        n = n + 1;
    end
end
end


function s = printableRun(raw)
%PRINTABLERUN Longest run of printable ASCII, to spot a reply hiding in
%   the binary stream. Binary float bytes rarely form long readable runs.
s = "";
best = 0; cur = 0; curStart = 1;
for k = 1:numel(raw)
    b = raw(k);
    if b >= 32 && b <= 126
        if cur == 0, curStart = k; end
        cur = cur + 1;
        if cur > best
            best = cur;
            s = string(char(raw(curStart:k)'));
        end
    else
        cur = 0;
    end
end
if best < 3, s = ""; end
end


function closePort(device)
try
    configureCallback(device,"off");
catch
end
end
