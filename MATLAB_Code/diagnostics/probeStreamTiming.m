function result = probeStreamTiming(port,baudRate,seconds,opts)
%PROBESTREAMTIMING Diagnose WHY the DataScope frame rate is what it is. READ ONLY.
%
%   probeStreamTiming("COM4")          % 128000 baud, 10 s
%   probeStreamTiming("COM4",128000,20)
%
%   checkDataScopeLink proved the LINK is clean, but measured only 1.5 Hz
%   where the firmware's 50 ms main loop should give about 20 Hz. That is
%   a 13x discrepancy, and there are three quite different explanations.
%   This function tells them apart by time-stamping every chunk of bytes
%   as it arrives instead of only counting them at the end.
%
%   A  Board streams steadily but slowly
%        -> frames spread evenly across the whole capture, gap ~= 1/rate
%        -> the board's main loop really is slower than 50 ms
%
%   B  Board streamed briefly then stopped
%        -> all frames bunched at the start, then silence
%        -> the board left the main loop: stuck in the calibration/Tips
%           page (a jammed button), a crash, or streaming was switched off
%
%   C  Board is fine, MATLAB is not keeping up
%        -> chunks arrive in big lumps at long intervals
%        -> the polling loop or pause() granularity is the bottleneck,
%           not the board
%
%   Sends nothing. Cannot move the cart.
%
%   DTR / RTS ARE FORCED LOW, THEN THE BOARD IS GIVEN TIME TO BOOT.
%   Dropping DTR and RTS is what makes the CH340/CH9102 deliver data
%   reliably on this rig -- the same fix already used in connectSTM32.m.
%   The side effect is that moving those lines resets the STM32, which
%   then spends 2 s in the two delay_ms(1000) calls at the top of main().
%   That is the 1.8 s of silence seen in the first measurements.
%
%   So both things are done, in the right order: force the lines low,
%   wait SettleSeconds (default 2.6 s) for the board to come back, THEN
%   flush and start measuring. Pass "SettleSeconds",0.3 to skip the wait
%   when you know the board is already up and the lines are already low.

arguments
    port     (1,1) string = "COM4"
    baudRate (1,1) double {mustBePositive,mustBeInteger} = 128000
    seconds  (1,1) double {mustBePositive} = 10
    opts.SettleSeconds (1,1) double {mustBeNonnegative} = 2.6
end

FRAME_LEN = 14;
HEADER    = uint8('$');
TAIL      = uint8(13);
EXPECTED  = 20;             % Hz, from the 50 ms main loop
DELAY_MS  = 50;             % the delay_flag window: 10 ticks x 5 ms TIM1

device = serialport(port,baudRate, ...
    "DataBits",8,"StopBits",1,"Parity","none", ...
    "FlowControl","none","Timeout",1.0);
cleaner = onCleanup(@() closePort(device));   %#ok<NASGU>

% Proven configuration for this rig: hold both control lines low.
setDTR(device,false);
setRTS(device,false);

% Moving those lines resets the board, and main() then waits 2 s before
% it says anything. Sit out that startup delay instead of measuring it.
pause(opts.SettleSeconds);

flush(device);              % drop whatever arrived during startup

fprintf("Listening on %s at %d baud for %.0f s (nothing is sent)...\n", ...
    port,baudRate,seconds);

raw     = uint8([]);
tChunk  = zeros(0,1);       % arrival time of each chunk
nChunk  = zeros(0,1);       % bytes in each chunk
t0 = tic;
while toc(t0) < seconds
    n = device.NumBytesAvailable;
    if n > 0
        tNow = toc(t0);
        raw  = [raw; uint8(read(device,n,"uint8")')];   %#ok<AGROW>
        tChunk(end+1,1) = tNow;                         %#ok<AGROW>
        nChunk(end+1,1) = n;                            %#ok<AGROW>
    else
        pause(0.002);
    end
end
elapsed = toc(t0);

nBytes = numel(raw);
fprintf("\n  bytes      : %d in %.1f s  (%.0f bytes/s)\n",nBytes,elapsed,nBytes/elapsed);
fprintf("  chunks     : %d\n",numel(tChunk));

if nBytes < FRAME_LEN
    fprintf(2,"\nNothing usable arrived. Is the board powered and streaming?\n");
    result = struct("Pass",false);
    return;
end

%% Frames
last  = nBytes - FRAME_LEN + 1;
isHdr = false(last,1);
for k = 1:last
    isHdr(k) = (raw(k) == HEADER) && (raw(k+FRAME_LEN-1) == TAIL);
end
idx      = find(isHdr);
nFrames  = numel(idx);
rateMeas = nFrames/elapsed;

fprintf("  frames     : %d  ->  %.1f Hz   (firmware should give ~%d Hz)\n", ...
    nFrames,rateMeas,EXPECTED);

%% Arrival pattern — this is what separates A / B / C
firstT = tChunk(1);
lastT  = tChunk(end);
span   = lastT - firstT;
fprintf("  first byte : %.2f s\n",firstT);
fprintf("  last byte  : %.2f s   (capture was %.1f s)\n",lastT,elapsed);

if firstT > 1.2
    fprintf(2,"\n  UNEXPECTED: %.1f s of silence even though the board was given\n",firstT);
    fprintf(2,"  %.1f s to settle before measuring started. Either the board is\n",opts.SettleSeconds);
    fprintf(2,"  resetting again mid-capture, or it is slow to resume streaming.\n");
    fprintf(2,"  Re-run with a longer settle: probeStreamTiming(port,baud,secs,""SettleSeconds"",5)\n");
end

if numel(tChunk) > 1
    gaps = diff(tChunk);
    fprintf("  chunk gaps : min %.0f ms, median %.0f ms, max %.0f ms\n", ...
        1000*min(gaps),1000*median(gaps),1000*max(gaps));
    fprintf("  chunk size : min %d, median %d, max %d bytes\n", ...
        min(nChunk),median(nChunk),max(nChunk));
else
    gaps = [];
end

%% Verdict
stoppedEarly = lastT < 0.6*elapsed;
bigLumps     = ~isempty(gaps) && median(nChunk) > 3*FRAME_LEN;
periodMs     = 1000*median(gaps);
blindMs      = max(periodMs - DELAY_MS, 0);

fprintf("\n");
if stoppedEarly
    verdict = "B";
    fprintf(2,"VERDICT B — the board STOPPED streaming.\n");
    fprintf(2,"  Data arrived for the first %.1f s of a %.1f s capture, then nothing.\n",span,elapsed);
    fprintf(2,"  Most likely the board left its main loop. Check in this order:\n");
    fprintf(2,"    1. Look at the OLED. If it shows a calibration/instruction page,\n");
    fprintf(2,"       the Menu button is stuck or was held -- press PID- to back out.\n");
    fprintf(2,"    2. Power-cycle the board and re-run. If it streams briefly after\n");
    fprintf(2,"       every reset and then stops, the firmware is hanging.\n");
elseif bigLumps
    verdict = "C";
    fprintf(2,"VERDICT C — MATLAB is the bottleneck, not the board.\n");
    fprintf(2,"  Bytes arrive in lumps of %d, so the board is filling the OS buffer\n",round(median(nChunk)));
    fprintf(2,"  faster than this loop drains it. Nothing to fix in the firmware.\n");
else
    % Always show the breakdown -- the loop period is the number that
    % actually decides both command latency and capture resolution.
    fprintf("  MAIN LOOP BREAKDOWN\n");
    fprintf("    measured period : %.0f ms   (%.1f Hz)\n",periodMs,rateMeas);
    fprintf("    delay_flag wait : %d ms   (10 TIM1 ticks x 5 ms)\n",DELAY_MS);
    fprintf("    work per loop   : %.0f ms   (DataScope + Tips/OLED refresh)\n",blindMs);

    fprintf("\n  COMMAND LATENCY\n");
    fprintf("    UartCmd_Poll runs throughout the %d ms wait and not during the\n",DELAY_MS);
    fprintf("    %.0f ms of work, so a command waits at most %.0f ms to be seen.\n",blindMs,blindMs);
    fprintf("    sendSTM32Command allows 500 ms -> %.0fx margin.\n",500/max(blindMs,1));

    fprintf("\n  CAPTURE RESOLUTION\n");
    fprintf("    %.1f Hz gives %.1f samples across the 190 ms right-half-plane\n",rateMeas,0.190*rateMeas);
    fprintf("    undershoot. Enough to suspect it, not enough to prove it.\n");

    if periodMs <= DELAY_MS*1.2
        verdict = "OK";
        fprintf("\nVERDICT OK — loop is running at its designed %d ms.\n",DELAY_MS);
    else
        verdict = "A";
        fprintf("\nVERDICT A — loop is HEALTHY but %.0f%% slower than designed.\n", ...
            100*(periodMs/DELAY_MS - 1));
        fprintf("  Nothing is broken and command latency is fine. The cost is only\n");
        fprintf("  data resolution. To find out what the %.0f ms of work actually is,\n",blindMs);
        fprintf("  comment out Tips(); in the Minibalance.c main loop and re-measure:\n");
        fprintf("  if the rate jumps to ~20 Hz it is the OLED refresh.\n");
    end
end

%% Decoded values, as a sanity check on the sensors
if nFrames >= 1
    ch = zeros(nFrames,3);
    for k = 1:nFrames
        ch(k,:) = double(typecast(raw(idx(k)+1 : idx(k)+12),"single"));
    end
    fprintf("\n  angle ADC  : %.0f .. %.0f", min(ch(:,1)),max(ch(:,1)));
    if median(ch(:,1)) < 1010 || median(ch(:,1)) > 1030
        fprintf(2,"   <-- OUTSIDE 1010-1030, this rig needs calibration\n");
    else
        fprintf("   (hanging target 1010-1030, OK)\n");
    end
    fprintf("  encoder    : %.0f .. %.0f\n",min(ch(:,2)),max(ch(:,2)));
    fprintf("  motor PWM  : %.0f .. %.0f\n",min(ch(:,3)),max(ch(:,3)));
else
    ch = [];
end

result = struct("Verdict",verdict,"FrameRateHz",rateMeas,"Frames",nFrames, ...
    "BytesReceived",nBytes,"Seconds",elapsed, ...
    "FirstByteSec",firstT,"LastByteSec",lastT, ...
    "ChunkTimes",tChunk,"ChunkSizes",nChunk,"Channels",ch);
end


function closePort(device)
try
    configureCallback(device,"off");
catch
end
end
