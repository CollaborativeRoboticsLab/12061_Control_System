function result = checkDataScopeLink(port,baudRate,seconds,opts)
%CHECKDATASCOPELINK Pre-flight check of the 128000-baud link. READ ONLY.
%
%   result = checkDataScopeLink("COM4")            % 128000 baud, 2 s
%   result = checkDataScopeLink("COM4",128000,3)
%
%   RUN THIS BEFORE FLASHING ANYTHING.
%
%   The MATLAB toolkit is already proven at 9600 against the bluepill
%   board, so the serial engine, the CH9102 driver, the DTR/RTS handling
%   and the terminator settings are all known good. The one thing that
%   has never been exercised is 128000 baud, which is the rate the
%   WHEELTEC pendulum firmware uses. This function settles that question
%   without touching a single line of firmware.
%
%   It works on the board EXACTLY AS IT IS RIGHT NOW, before any of the
%   Route 4 changes: the stock firmware transmits a DataScope frame
%   roughly every 50 ms whether anyone is listening or not. This function
%   only listens. It never writes to the port, so it cannot move the cart
%   and cannot disturb the board.
%
%   Frame format, read out of HARDWARE/DataScope_DP/DataScope_DP.C:
%
%       byte  0      '$'  (0x24)          frame header
%       bytes 1..12  3 x float32, little endian
%                        ch1 = Angle_Balance  (ADC counts, ~1020..3100)
%                        ch2 = Encoder        (cart position, ~5900..9900)
%                        ch3 = Moto           (PWM, -6900..6900)
%       byte  13     13   (0x0D)          length marker, NOT a newline
%
%   PASS means: frames arrive, headers land exactly 14 bytes apart, and
%   the decoded channels are physically sensible. That is proof the whole
%   USB path carries 128000 baud cleanly on this PC, this cable and this
%   board. If it passes, the only untested thing left in Route 4 is the
%   new C code, which already passes its own unit tests.
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
    seconds  (1,1) double {mustBePositive} = 2
    opts.SettleSeconds (1,1) double {mustBeNonnegative} = 2.6
end

FRAME_LEN = 14;
HEADER    = uint8('$');     % 0x24
TAIL      = uint8(13);

allPorts = string(serialportlist("all"));
if ~any(strcmpi(allPorts,port))
    error("STM32:PortNotDetected","%s is not detected by Windows/MATLAB.",port);
end

availablePorts = string(serialportlist("available"));
if ~any(strcmpi(availablePorts,port))
    error("STM32:PortUnavailable", ...
        "%s exists but is busy. Close PlatformIO Monitor, the WHEELTEC " + ...
        "upper-computer, other terminals and other MATLAB sessions.",port);
end

device = serialport(port,baudRate, ...
    "DataBits",8,"StopBits",1,"Parity","none", ...
    "FlowControl","none","Timeout",1.0);

% 出错或 Ctrl-C 时也要把回调关掉,免得端口被占住下次打不开。
cleaner = onCleanup(@() closePort(device));   %#ok<NASGU>

% Proven configuration for this rig: hold both control lines low.
setDTR(device,false);
setRTS(device,false);

% Moving those lines resets the board, and main() then waits 2 s before
% it says anything. Sit out that startup delay instead of measuring it.
pause(opts.SettleSeconds);

flush(device);              % drop whatever arrived during startup

fprintf("Listening on %s at %d baud for %.1f s (no data is sent)...\n", ...
    port,baudRate,seconds);

raw = uint8([]);
t0  = tic;
while toc(t0) < seconds
    n = device.NumBytesAvailable;
    if n > 0
        raw = [raw; uint8(read(device,n,"uint8")')];   %#ok<AGROW>
    else
        pause(0.01);
    end
end
elapsed = toc(t0);

result = struct("Port",port,"BaudRate",baudRate,"Seconds",elapsed, ...
    "BytesReceived",numel(raw),"Frames",0,"FrameRateHz",NaN, ...
    "AlignedFraction",NaN,"Pass",false,"RateLooksNormal",false,"Channels",[]);

fprintf("  bytes received : %d  (%.0f bytes/s)\n",numel(raw),numel(raw)/elapsed);

if numel(raw) < 3*FRAME_LEN
    fprintf(2,"\nFAIL: almost nothing arrived.\n");
    explainFailure(baudRate);
    return;
end

%% Locate frames: header AND correct tail, so a 0x24 inside a float
%  cannot be mistaken for a header.
last  = numel(raw) - FRAME_LEN + 1;
isHdr = false(last,1);
for k = 1:last
    isHdr(k) = (raw(k) == HEADER) && (raw(k+FRAME_LEN-1) == TAIL);
end
idx = find(isHdr);

if numel(idx) < 3
    fprintf(2,"\nFAIL: %d frame headers found. The bytes are arriving " + ...
        "but they are not DataScope frames.\n",numel(idx));
    fprintf(2,"First 28 bytes: %s\n",sprintf("%02X ",raw(1:min(28,end))));
    explainFailure(baudRate);
    return;
end

gaps    = diff(idx);
aligned = sum(gaps == FRAME_LEN) / numel(gaps);

result.Frames          = numel(idx);
result.FrameRateHz     = numel(idx)/elapsed;
result.AlignedFraction = aligned;

fprintf("  frames found   : %d\n",numel(idx));
fprintf("  frame rate     : %.1f Hz\n",result.FrameRateHz);
fprintf("  header spacing : %.1f %% exactly 14 bytes apart\n",100*aligned);

% The firmware's 50 ms main loop should give about 20 Hz. A clean link at
% the wrong rate still means something is wrong, so say so rather than
% printing PASS and moving on.
EXPECTED_HZ = 20;
result.RateLooksNormal = result.FrameRateHz > 0.5*EXPECTED_HZ;
if ~result.RateLooksNormal
    fprintf(2,"\n  WARNING: %.1f Hz, but the 50 ms main loop should give ~%d Hz.\n", ...
        result.FrameRateHz,EXPECTED_HZ);
    fprintf(2,"  The link is clean; the RATE is not. Run probeStreamTiming(port)\n");
    fprintf(2,"  to find out whether the board is slow, stopped, or MATLAB is behind.\n");
end

%% Decode the channels of the aligned frames
good = idx(gaps == FRAME_LEN);
ch   = zeros(numel(good),3);
for k = 1:numel(good)
    ch(k,:) = double(typecast(raw(good(k)+1 : good(k)+12),"single"));
end
result.Channels = ch;

fprintf("\n  channel 1 (Angle_Balance, ADC) : %7.1f .. %7.1f\n",min(ch(:,1)),max(ch(:,1)));
fprintf("  channel 2 (Encoder, counts)    : %7.1f .. %7.1f\n",min(ch(:,2)),max(ch(:,2)));
fprintf("  channel 3 (Moto, PWM)          : %7.1f .. %7.1f\n",min(ch(:,3)),max(ch(:,3)));

%% Verdict
sane = all(ch(:,1) >= 0 & ch(:,1) <= 4095) && ...
       all(abs(ch(:,3)) <= 7200) && ...
       all(isfinite(ch(:)));

result.Pass = (aligned > 0.98) && sane;

fprintf("\n");
if result.Pass
    fprintf("PASS (link). %d baud carries clean frames on this PC and cable.\n",baudRate);
    if ~result.RateLooksNormal
        fprintf(2,"       Note: this says nothing about the RATE -- see the warning above.\n");
    end
    fprintf("Keep uart_init(72,%d) in Minibalance.c and connect with\n",baudRate);
    fprintf("    s = connectSTM32(""%s"",%d);\n",port,baudRate);
    a = median(ch(:,1));
    if a < 1010 || a > 1030
        fprintf(2,"\nNote: hanging angle reads %.0f, outside the 1010-1030 window.\n",a);
        fprintf(2,"This rig needs the angle-sensor calibration before it will balance.\n");
    end
    fprintf("\nSide benefit: the DataScope rate is now MEASURED at %.1f Hz " + ...
        "rather than assumed.\n",result.FrameRateHz);
else
    if ~sane
        fprintf(2,"FAIL: frames decode to impossible values -> bit errors at %d baud.\n",baudRate);
    else
        fprintf(2,"FAIL: only %.0f %% of headers are 14 bytes apart -> dropped bytes.\n",100*aligned);
    end
    explainFailure(baudRate);
end
end


function closePort(device)
try
    configureCallback(device,"off");
catch
end
end


function explainFailure(baudRate)
fprintf(2,"\nWhat to do\n");
if baudRate ~= 9600
    fprintf(2,"  1. Re-run at the proven rate to prove the cable is fine:\n");
    fprintf(2,"       checkDataScopeLink(""COM4"",9600)\n");
    fprintf(2,"     (it will find no frames, because the board is transmitting\n");
    fprintf(2,"      at %d, but you should still see a steady byte stream)\n",baudRate);
    fprintf(2,"  2. If %d is genuinely unreliable on this machine, drop the\n",baudRate);
    fprintf(2,"     firmware to 9600: change ONE line in USER/Minibalance.c,\n");
    fprintf(2,"       uart_init(72,128000)  ->  uart_init(72,9600)\n");
    fprintf(2,"     and open the port with connectSTM32(port,9600).\n");
    fprintf(2,"     Cost: the MATLAB polling loop caps out near 20 Hz instead\n");
    fprintf(2,"     of 35 Hz, and DataScope cannot be raised past ~60 Hz.\n");
else
    fprintf(2,"  Check the USB cable, the CH9102/CH340 driver, and that the\n");
    fprintf(2,"  board is powered and running (OLED lit).\n");
end
end
