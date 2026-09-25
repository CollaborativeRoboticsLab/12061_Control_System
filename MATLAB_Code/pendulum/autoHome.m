function info = autoHome(s,opts)
%AUTOHOME Find the rail by touch, park the cart in the middle, send HOME.
%
%   info = autoHome(s)                      rod hanging, board just connected
%   info = autoHome(s,"Confirm",false)      skip the "press Enter" prompt
%   info = autoHome(s,"MeasureRail",true)   touch BOTH ends and report the
%                                           travel (occasional calibration)
%
%   The automatic replacement for the hand step in homeCart. Nothing to
%   slide, nothing to judge by eye: the motor pushes the cart gently to one
%   end of the rail until the encoder stops moving, drives the known half
%   travel back, and declares that point the middle with HOME.
%
%   IF THE BOARD IS ALREADY HOMED it does not re-home (it cannot -- see
%   below); it simply drives the cart back to the middle. That is the
%   between-trials use: rod hanging, balance off, cart wherever it ended up.
%
%   WHAT IT NEEDS
%     - the rod hanging down and the balance controller OFF. Every pulse here is an M command, and M replaces the balance loop: sent while
%       the rod is balancing it would simply drop it.- hands clear of the rail. The cart WILL touch one end stop.
%
%   WHY THIS IS SAFE NOW, WHEN THE FIRST MOTOR VERSION WAS NOT
%   homeCart's notes give two reasons the motor approach was abandoned:
%     1. the PWM sign had to be assumed. Here it is MEASURED: the first
%        pulse is a short probe, and whichever way the encoder moves is
%        taken as the answer. A rig wired the other way still homes; it
%        just gets a warning, because the firmware's end-stop guard
%        assumes the usual sign.
%     2. the deadband (PWM 900..1500) made small pulses unreliable. Here
%        every search pulse is 2500 for 300 ms -- the most the firmware
%        allows before HOME -- which is well clear of the deadband, and
%        still too short for the cart to build up any speed.
%   Before HOME the firmware also skips its end-stop test, so the cart can
%   always reach the stop it is looking for.
%
%   HOW AN END IS RECOGNISED
%   Two pulses in a row that move the encoder less than StallCounts in the
%   pushing direction. One is not enough: a sticky patch of rail can eat a
%   single pulse.
%
%   THE RAIL LENGTH
%   RailCounts defaults to 4110, measured on this rig on 24 Sep 2026: the
%   OLED read P:10000 at the left stop and P:5890 at the right, i.e. 447 mm
%   at 9.196 counts/mm. Run with "MeasureRail",true now and then; if the
%   travel it reports drifts, the belt is slipping or a stop has moved.
%
%   WHY IT CANNOT RE-HOME AN ALREADY HOMED BOARD
%   After HOME the firmware refuses any M that drives further past its
%   guard lines (9900 / 5900), so the cart can no longer reach a stop to
%   measure it. Re-homing needs a fresh frame: reconnect (connecting resets
%   the board) and run autoHome again.
%
%   Needs firmware R4-4 or later (HOME, HOMED, and the capped M).

arguments
    s
    opts.Confirm     (1,1) logical = true
    opts.MeasureRail (1,1) logical = false
    opts.Force       (1,1) logical = false
    opts.Verbose     (1,1) logical = true
    opts.RailCounts  (1,1) double {mustBePositive} = 4110
    opts.Target      (1,1) double = 7925    % UARTCMD_ENC_HOME
    opts.Tol         (1,1) double {mustBePositive} = 25     % ~2.7 mm
    opts.PWM         (1,1) double {mustBePositive} = 2500
    opts.PulseMs     (1,1) double {mustBePositive} = 300
    opts.StallCounts (1,1) double {mustBePositive} = 15
    opts.MaxPulses   (1,1) double {mustBePositive,mustBeInteger} = 60
end

NUDGE_PWM = 2500;            % UARTCMD_NUDGE_PWM: M cap before HOME
NUDGE_MS  = 300;             % UARTCMD_NUDGE_MS
COUNTS_PER_MM = 1040/(pi*36);
FIRMWARE_UP_SIGN = -1;       % firmware assumes +PWM makes the count FALL

pwm = min(round(opts.PWM),NUDGE_PWM);
opts.PulseMs = min(round(opts.PulseMs),NUDGE_MS);
v = opts.Verbose;
t0 = tic;

info = struct("Mode","","HighEndRaw",NaN,"LowEndRaw",NaN, ...
    "RailCounts",NaN,"RailMm",NaN,"MiddleOffsetCounts",NaN, ...
    "UpSign",NaN,"Pulses",0,"Seconds",NaN);

% ---------------------------------------------------------------- checks
r = pendulumCommand(s,"HOMED",0.50,3,["HOMED="]);
homed = strtrim(r) == "HOMED=1";

st = readStatusWheeltec(s);
if ~st.FlagStop && ~st.ManualMode
    error("autoHome:Balancing", ...
        ['The balance controller (or swing-up) is running. Every pulse ' ...
         'here is an M command, which would drop the rod.\nLet the rod ' ...
         'hang, call setBalance(s,false), then run autoHome again.']);
end
if st.AngleRaw > 2000
    error("autoHome:RodUp", ...
        ['The angle reads %d, so the rod is being held up. Let it hang ' ...
         'straight down first.'],st.AngleRaw);
end
if st.VoltageVolts < 7.4
    warning("autoHome:LowBattery", ...
        "Battery %.2f V. Pulses may be too weak to move the cart reliably.", ...
        st.VoltageVolts);
end
if homed && opts.Force
    error("autoHome:AlreadyHomed", ...
        ['The board is already homed, and a homed board will not drive ' ...
         'past its guard lines, so the stops cannot be found again.\n' ...
         'Reconnect (that resets the board), then run autoHome.']);
end

if opts.Confirm
    if homed, what = "drive back to the middle";
    else,     what = "drive to one end of the rail and back (about 20 s)";
    end
    fprintf("\n===== autoHome =====\n");
    fprintf("Rod hanging, hands clear of the rail. The cart will %s.\n",what);
    fprintf("KEY5 stops it at any time.\n");
    a = input("Press Enter to start, or type n to cancel: ","s");
    if strcmpi(strtrim(string(a)),"n")
        fprintf("Cancelled. Nothing moved.\n");
        info.Mode = "cancelled";
        return;
    end
end

cleanup = onCleanup(@() stopQuietly(s));   %#ok<NASGU> also on Ctrl+C

% ------------------------------------------- already homed: just recentre
if homed
    msg(v,"Already homed; driving back to %d.\n",opts.Target);
    [eF,n] = driveTo(s,opts.Target,FIRMWARE_UP_SIGN,pwm,0.5,opts);
    info.Mode = "recentred";
    info.UpSign = FIRMWARE_UP_SIGN;
    info.Pulses = n;
    info.Seconds = toc(t0);
    msg(v,"Cart at %d (%+d from the middle). Done in %.1f s.\n", ...
        eF,eF-opts.Target,info.Seconds);
    return;
end

% ------------------------------------------ 1. probe: which way is "up"?
msg(v,"Probing motor direction...\n");
[d,st] = pulse(s,+pwm,150);
nP = 1;
if abs(d) >= opts.StallCounts
    upSign = sign(d);                 % count change produced by +PWM
    rate0  = abs(d)/150;
else
    [d,st] = pulse(s,-pwm,150);       % maybe it was sitting on a stop
    nP = 2;
    if abs(d) < opts.StallCounts
        error("autoHome:NoMotion", ...
            ['Neither +%d nor -%d PWM moved the cart (%d counts). Check ' ...
             'the motor power switch and the battery, and that nothing ' ...
             'is jamming the cart.'],pwm,pwm,d);
    end
    upSign = -sign(d);
    rate0  = abs(d)/150;
end
checkStop(st);
if upSign ~= FIRMWARE_UP_SIGN
    warning("autoHome:MotorReversed", ...
        ['+PWM makes the encoder count RISE on this rig. Homing still ' ...
         'works, but the firmware''s end-stop guard assumes the opposite ' ...
         'and will protect the wrong end.']);
end
pwmUp = upSign*pwm;                   % drives the count up

% -------------------------------------------------- 2. find the high end
msg(v,"Looking for the end stop...\n");
[eHigh,n,rate] = findEnd(s,pwmUp,+1,rate0,opts);
nP = nP + n;
msg(v,"  end stop at %d (raw count).\n",eHigh);

if opts.MeasureRail
    msg(v,"Looking for the other end stop...\n");
    [eLow,n] = findEnd(s,-pwmUp,-1,rate,opts);
    nP = nP + n;
    rail = eHigh - eLow;
    mid  = round((eHigh + eLow)/2);
    msg(v,"  other end at %d. Travel %d counts = %.0f mm.\n", ...
        eLow,rail,rail/COUNTS_PER_MM);
    if abs(rail - opts.RailCounts) > 60
        warning("autoHome:RailChanged", ...
            ['Measured travel %d counts, expected %d. If this repeats, ' ...
             'the belt is slipping or an end stop has moved; update ' ...
             'RailCounts once you trust the new value.'],rail,opts.RailCounts);
    end
    info.LowEndRaw  = eLow;
    info.RailCounts = rail;
    info.RailMm     = rail/COUNTS_PER_MM;
else
    rail = opts.RailCounts;
    mid  = eHigh - round(rail/2);
end

% ------------------------------------------------- 3. drive to the middle
msg(v,"Driving to the middle...\n");
[eF,n] = driveTo(s,mid,upSign,pwm,rate,opts);
nP = nP + n;
resid = eF - mid;                     % how far short / past the middle

% ------------------------------------------------------------ 4. declare
reply = pendulumCommand(s,"HOME",0.50,3,["HOME="]);
tok   = regexp(strtrim(reply),"^HOME=(-?\d+)$","tokens","once");
if isempty(tok)
    error("autoHome:HomeFailed","Board answered HOME with: %s",reply);
end
pause(0.05);
pos = readPositionWheeltec(s);
if abs(pos - str2double(tok{1})) > 10
    error("autoHome:HomeNotApplied", ...
        "HOME acknowledged %s but the cart now reads %d.",tok{1},pos);
end

info.Mode = "homed";
info.HighEndRaw = eHigh;
info.UpSign = upSign;
info.MiddleOffsetCounts = -resid;     % true middle, in the new frame, minus Target
info.Pulses = nP;
info.Seconds = toc(t0);

hiEnd = opts.Target + (eHigh - eF);   % the stops, in the new frame
loEnd = hiEnd - rail;
msg(v,"\nHomed in %.1f s (%d pulses).\n",info.Seconds,nP);
msg(v,"  cart now          : %d counts\n",pos);
msg(v,"  true middle       : %+d counts (%.1f mm) from %d\n", ...
    -resid,-resid/COUNTS_PER_MM,opts.Target);
msg(v,"  end stops         : about %d and %d\n",loEnd,hiEnd);
msg(v,"  firmware guards   : 5900 and 9900 (%d and %d counts inside the stops)\n", ...
    5900-loEnd,hiEnd-9900);
end


% ======================================================================
function [eEnd,n,rate] = findEnd(s,pwmDir,dirSign,rate,opts)
%FINDEND Push in one direction until two pulses in a row barely move.
stalls = 0; travel = 0; moving = [];
for n = 1:opts.MaxPulses
    [d,st] = pulse(s,pwmDir,opts.PulseMs);
    checkStop(st);
    travel = travel + abs(d);
    if d*dirSign < opts.StallCounts
        stalls = stalls + 1;
        if stalls >= 2
            eEnd = st.Encoder;
            if ~isempty(moving), rate = median(moving)/opts.PulseMs; end
            return;
        end
    else
        stalls = 0;
        moving(end+1) = abs(d);                        %#ok<AGROW>
    end
    if travel > opts.RailCounts + 400
        error("autoHome:NoEndFound", ...
            ['Travelled %d counts, more than the whole rail, without ' ...
             'stopping. The encoder is turning but the cart is not: ' ...
             'check the belt for slip.'],travel);
    end
end
error("autoHome:NoEndFound", ...
    "No end stop after %d pulses (%d counts).",opts.MaxPulses,travel);
end


function [eF,n] = driveTo(s,target,upSign,pwm,rate,opts)
%DRIVETO Short open-loop pulses, sized from the measured speed, until the
%   cart is within Tol. Pulses too short to beat static friction get
%   longer each time they fail.
boost = 1;
for n = 1:40
    st = readStatusWheeltec(s);
    e  = target - st.Encoder;
    if abs(e) <= opts.Tol
        eF = st.Encoder;
        return;
    end
    ms = round(0.7*abs(e)/max(rate,0.05)*boost);
    ms = min(opts.PulseMs,max(30,ms));
    [d,st] = pulse(s,sign(e)*upSign*pwm,ms);
    checkStop(st);
    if abs(d) < 3
        boost = boost*1.6;                % did not break away: push longer
    else
        boost = 1;
        if ms >= 100, rate = 0.5*rate + 0.5*abs(d)/ms; end
    end
end
st = readStatusWheeltec(s);
eF = st.Encoder;
if abs(target - eF) > 2*opts.Tol
    error("autoHome:CouldNotPark", ...
        "Could not get within %d counts of %d; stopped at %d.", ...
        opts.Tol,target,eF);
end
warning("autoHome:LooseParking", ...
    "Parked %d counts from %d (tolerance %d).",eF-target,target,opts.Tol);
end


function [d,st] = pulse(s,pwm,ms)
%PULSE One timed M command; returns the encoder change once it is still.
st0 = readStatusWheeltec(s);
setMotorPWM_wheeltec(s,pwm,ms);
pause(ms/1000 + 0.10);
st = waitStill(s);
d  = st.Encoder - st0.Encoder;
end


function st = waitStill(s)
%WAITSTILL Let the cart coast to a stop (at most 1 s).
st = readStatusWheeltec(s);
t  = tic;
while toc(t) < 1.0
    pause(0.05);
    st2 = readStatusWheeltec(s);
    if abs(st2.Encoder - st.Encoder) <= 2
        st = st2;
        return;
    end
    st = st2;
end
end


function checkStop(st)
%CHECKSTOP Every M clears Flag_Stop, so a set flag after a pulse means
%   someone pressed KEY5. Respect it instead of sending the next pulse.
if st.FlagStop
    error("autoHome:Stopped","Stopped with KEY5. Nothing was homed.");
end
end


function stopQuietly(s)
try
    pendulumCommand(s,"STOP",0.50,2,["STOPPED"]);
catch
end
end


function msg(v,varargin)
if v, fprintf(varargin{:}); end
end
