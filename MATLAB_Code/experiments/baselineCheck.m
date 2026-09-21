%% baselineCheck.m
% BASELINE HEALTH CHECK  -  staff tool, not a student lab
% WHEELTEC linear inverted pendulum, firmware R4-6 or later
%
% Run this ONCE on the rig before writing or running Lab 5. Everything Lab 5
% needs to be decided -- what the baseline controller is, how big the
% disturbance should be, how far the derivative gain can be pushed, and
% whether the angle sensor needs trimming -- is a measurement, and none of
% those measurements exist yet. Guessing them and then writing the lab
% around the guess is how a session gets lost.
%
% HOW TO RUN IT
%   Run ONE SECTION AT A TIME with Ctrl+Enter. Never Run All: several
%   sections need you to be holding the rod, or to have taken it off.
%
% WHAT IT PRODUCES
%   A summary block at the end, in plain text, designed to be copied
%   straight out of the Command Window.
%
% SAFETY
%   Sections 3 and 4 drive the motor with the rod up. Hands clear of the
%   rail, KEY5 within reach. The firmware cuts the motor once the rod
%   passes about 43 degrees, so a fall stops the cart rather than sending
%   it into the end of the rail -- but it will move sharply first.

%% ===================== SECTION 0 - CONNECT ==============================
clear; clc;

% >> EDIT THIS
PORT = "COM11";

% Locate this script through the MATLAB path, not through mfilename:
% Run Section executes a temporary copy, so mfilename points into %TEMP%.
here = '';
onPath = which('baselineCheck.m');
if ~isempty(onPath) && ~startsWith(onPath,tempdir,'IgnoreCase',true)
    here = fileparts(onPath);
end
if isempty(here), here = pwd; end
DATA_FOLDER = fullfile(here,"baseline_data");
if ~exist(DATA_FOLDER,"dir"), mkdir(DATA_FOLDER); end

if ~exist("s","var") || isempty(s) || ~isvalid(s)
    s = connectPendulum(PORT);
end

g0 = readGains(s);
fprintf("\n=== BASELINE AS COMPILED INTO Minibalance.c ===\n");
fprintf("  Balance_KP  = %g\n",  g0.Balance_KP);
fprintf("  Balance_KD  = %g\n",  g0.Balance_KD);
fprintf("  Balance_KI  = %g\n",  g0.Balance_KI);
fprintf("  Position_KP = %g\n",  g0.Position_KP);
fprintf("  Position_KD = %g\n",  g0.Position_KD);
fprintf("  Position_KI = %g\n",  g0.Position_KI);
fprintf("  Angle_Zero  = %g counts\n", g0.Angle_Zero);

st = readStatusWheeltec(s);
fprintf("  battery     : %.2f V", st.VoltageVolts);
if st.VoltageVolts < 7.4, fprintf("   <-- TOO LOW, charge it first"); end
fprintf("\n  STATUS fields: %d", st.NumFields);
if st.NumFields < 10
    error("Baseline:FirmwareTooOld", ...
        "STATUS returned %d fields. This check needs R4-6 (10 fields).", ...
        st.NumFields);
end
fprintf("   (R4-6 confirmed)\n");

homeCart(s);

%% ===================== SECTION 1 - WHERE IS VERTICAL? ===================
% The firmware aims at Angle_Zero. If that is not the ADC reading at TRUE
% vertical, the rod settles at a permanent lean and the cart sits
% permanently off-centre to balance it -- and no gain can remove either.
% Lab 5 scores both as separate criteria, so this has to be right first.
%
% The rod hangs 180 degrees from upright, and the sensor spans about 2080
% counts over 180 degrees, so:   upright = hanging + 2080.
%
% >> STUDENT ACTION (well, tutor action)
%    Let the rod hang completely still. Do not touch it.

input("Rod hanging free and still. Press Enter: ","s");
hang = zeros(60,1);
for k = 1:60, hang(k) = readStatusWheeltec(s).AngleRaw; pause(0.02); end
HANG = median(hang);

fprintf("\n  hanging      : %.0f counts  (spread %.0f over 60 samples)\n", ...
        HANG, max(hang)-min(hang));

% >> TUTOR ACTION
%    Now hold the rod as close to TRUE vertical as you can judge by eye.
%    Use a spirit level or a phone level app if one is to hand; 1 degree
%    of eyeball error is 11 counts, which is worth caring about here.
input("Now hold the rod at TRUE vertical and keep it there. Press Enter: ","s");
up = zeros(60,1);
for k = 1:60, up(k) = readStatusWheeltec(s).AngleRaw; pause(0.02); end
UP = median(up);

PREDICTED = HANG + 2080;
fprintf("  held upright : %.0f counts  (spread %.0f)\n", UP, max(up)-min(up));
fprintf("  hanging+2080 : %.0f counts  (what the geometry predicts)\n",PREDICTED);
fprintf("  firmware aims: %g counts  (Angle_Zero)\n", g0.Angle_Zero);

AZ_SUGGESTED = round(mean([UP PREDICTED]));
errDeg = (g0.Angle_Zero - AZ_SUGGESTED)*(180/2080);
fprintf("\n  >> suggested Angle_Zero : %d counts\n", AZ_SUGGESTED);
fprintf("  >> current setpoint is off by %.0f counts = %.2f degrees\n", ...
        g0.Angle_Zero - AZ_SUGGESTED, errDeg);
if abs(errDeg) > 0.5
    fprintf(2,"  >> worth trimming. Section 3 runs twice to show the difference.\n");
else
    fprintf("  >> close enough; no trim needed.\n");
end

%% ===================== SECTION 2 - HOW NOISY IS THE SENSOR? =============
% This sets the CEILING on Balance_KD, and nobody has ever measured it.
%
% The firmware's derivative term is a raw per-tick DIFFERENCE, not divided
% by Ts:
%       D_Bias  = Bias - Last_Bias;
%       balance = -Kp*Bias - Kd*D_Bias - ...
% so Kd multiplies ADC counts directly onto a PWM scale that clamps at
% 6900. One count of tick-to-tick noise at Kd=400 is 400 PWM of pure noise.
%
% >> TUTOR ACTION
%    Hold the rod still near vertical, or clamp it. Motor stays off.

input("Hold the rod still (motor is off). Press Enter: ","s");
N = 300;
raw = zeros(N,1); tn = zeros(N,1); t0 = tic;
for k = 1:N
    raw(k) = readStatusWheeltec(s).AngleRaw;
    tn(k)  = toc(t0);
end
d = diff(raw);
NOISE = std(d);
fprintf("\n  sample rate      : %.1f Hz\n", N/tn(end));
fprintf("  sample-to-sample : std %.2f counts, max jump %d counts\n", ...
        NOISE, max(abs(d)));
fprintf("\n  CAUTION: polling samples every 5th control tick (25 ms), not\n");
fprintf("  every tick (5 ms), so this UNDERSTATES the noise the D term\n");
fprintf("  actually sees. Treat it as a lower bound.\n");
fprintf("\n  PWM that pure noise would command:\n");
for kd = [400 800 1200 2000]
    p = kd*NOISE;
    flag = "";
    if p > 2000, flag = "   <-- noise alone would dominate"; end
    fprintf("    Kd = %4d  ->  %5.0f PWM  (%4.1f%% of full scale)%s\n", ...
            kd, p, 100*p/6900, flag);
end

%% ===================== SECTION 3 - BASELINE, NO DISTURBANCE =============
% Sixty seconds of the baseline controller simply holding the rod up. This
% is the "is the baseline good enough to build a lab on" test.
%
% >> SAFETY - the rig is live from here on. Hands clear of the rail.

DURATION = 60;
fprintf("\nLift the rod to VERTICAL and hold it steady.\n");
waitUpright(s,g0.Angle_Zero);
setBalance(s,true);
fprintf("Controller is LIVE. Release on the prompt and stand clear.\n");
pause(1.5);
disp(">>> RELEASE NOW <<<");

base = sampleRun(s,DURATION,g0.Angle_Zero);
setBalance(s,false);
reportRun("BASELINE, no disturbance", base);
save(fullfile(DATA_FOLDER,"baseline_quiet.mat"),"base","g0");

%% ===================== SECTION 4 - HOW BIG SHOULD THE KICK BE? ==========
% Lab 5 compares controllers by how each recovers from the SAME
% disturbance. This finds the size that produces a clear, measurable
% recovery without throwing the cart at the end of the rail.
%
% Each trial: balance, settle 3 s, kick, record 7 s.

KICKS = [600 1000 1400 1800];
sweep = struct([]);

for k = 1:numel(KICKS)
    fprintf("\n--- KICK = %d ---\n", KICKS(k));
    fprintf("Lift the rod to VERTICAL and hold it steady.\n");
    waitUpright(s,g0.Angle_Zero);

    st = readStatusWheeltec(s);
    if abs(st.Encoder - 7925) > 400
        fprintf("Cart at %d, re-homing.\n", st.Encoder);
        homeCart(s,"Force",true);
    end

    setBalance(s,true);
    pause(1.5);
    disp(">>> RELEASE NOW <<<");
    pause(3);                                  % let it settle before the kick

    r = sampleRun(s,7,g0.Angle_Zero,@() disturb(s,KICKS(k),100));
    setBalance(s,false);

    r.kick = KICKS(k);
    sweep = [sweep, r];                        %#ok<AGROW>
    reportRun(sprintf("KICK = %d",KICKS(k)), r);
end

save(fullfile(DATA_FOLDER,"kick_sweep.mat"),"sweep","g0");

figure("Name","Kick sweep","NumberTitle","off"); hold on;
for k = 1:numel(sweep)
    plot(sweep(k).t - sweep(k).tKick, sweep(k).theta_deg, "LineWidth",1.3);
end
grid on; yline(0,"k--"); xline(0,"k:");
xlabel("Time from the kick (s)"); ylabel("\theta (deg from setpoint)");
title("Which disturbance size gives a clear recovery?");
legend(compose("KICK = %d",[sweep.kick]),"Location","best");

%% ===================== SECTION 5 - SUMMARY TO COPY =====================
fprintf("\n\n");
fprintf("========= COPY EVERYTHING BELOW THIS LINE =========\n");
fprintf("rig            : %s\n", PORT);
fprintf("firmware       : R4-6, STATUS fields %d\n", st.NumFields);
fprintf("battery        : %.2f V\n", base.volts);
fprintf("\n-- sensor zero --\n");
fprintf("hanging        : %.0f counts\n", HANG);
fprintf("held upright   : %.0f counts\n", UP);
fprintf("hanging+2080   : %.0f counts\n", PREDICTED);
fprintf("Angle_Zero now : %g counts\n", g0.Angle_Zero);
fprintf("suggested AZ   : %d counts  (off by %.2f deg)\n", AZ_SUGGESTED, errDeg);
fprintf("\n-- sensor noise --\n");
fprintf("tick-to-tick   : std %.2f counts, max jump %d\n", NOISE, max(abs(d)));
fprintf("\n-- baseline, 60 s quiet --\n");
fprintf("gains          : Kp=%g Kd=%g Ki=%g | PKp=%g PKd=%g PKi=%g\n", ...
        g0.Balance_KP,g0.Balance_KD,g0.Balance_KI, ...
        g0.Position_KP,g0.Position_KD,g0.Position_KI);
fprintf("outcome        : %s\n", base.outcome);
fprintf("angle RMS      : %.3f deg\n", base.angleRms);
fprintf("angle peak     : %.3f deg\n", base.anglePeak);
fprintf("PWM RMS        : %.0f\n", base.effortRms);
fprintf("PWM saturated  : %.1f %% of samples at +/-6900\n", base.satPct);
fprintf("cart drift     : %+.0f mm\n", base.driftMm);
fprintf("cart range     : %.0f mm\n", base.cartRangeMm);
fprintf("sample rate    : %.1f Hz\n", base.rateHz);
fprintf("\n-- kick sweep --\n");
for k = 1:numel(sweep)
    fprintf("KICK=%4d  peak %5.2f deg  recovery %5.2f s  maxCart %4.0f mm  %s\n", ...
        sweep(k).kick, sweep(k).anglePeak, sweep(k).recoverS, ...
        sweep(k).cartRangeMm, sweep(k).outcome);
end
fprintf("========= COPY EVERYTHING ABOVE THIS LINE =========\n\n");


%% ======================== helpers ======================================
function waitUpright(s,az)
%WAITUPRIGHT Block until the rod is inside the firmware's enable window.
%   Turn_Off() latches Flag_Stop the moment the angle leaves az +/- 500 and
%   nothing clears it again except a fresh BAL,1 -- so enabling the
%   controller before the rod is up kills it silently.
    tw = tic; last = -1;
    while true
        st = readStatusWheeltec(s);
        if abs(st.AngleRaw - az) <= 450, break; end
        if toc(tw) > 45
            error("Baseline:RodNotUpright", ...
                "Gave up after 45 s; angle reads %d, window is %g..%g.", ...
                st.AngleRaw, az-500, az+500);
        end
        if floor(toc(tw)) ~= last
            last = floor(toc(tw));
            fprintf("  waiting...  angle = %4d\n", st.AngleRaw);
        end
        pause(0.2);
    end
    fprintf("Rod is up (angle = %d).\n", st.AngleRaw);
end

function r = sampleRun(s,seconds,az,kickFcn)
%SAMPLERUN Poll STATUS for `seconds`, optionally firing a disturbance.
%   The kick instant is taken from STATUS field 10 (KickTicks going
%   non-zero), not from a host-side timestamp, so recovery time does not
%   carry a serial round trip in it.
    if nargin < 4, kickFcn = []; end
    cap = 4000;
    t = zeros(cap,1); a = zeros(cap,1); u = zeros(cap,1);
    x = zeros(cap,1); kt = zeros(cap,1); fs = false(cap,1);
    fired = isempty(kickFcn);
    n = 0; t0 = tic; volts = NaN;
    while toc(t0) < seconds
        st = readStatusWheeltec(s);
        n = n + 1;
        t(n)  = toc(t0);
        a(n)  = (st.AngleRaw - az)*(180/2080);
        u(n)  = st.Moto;
        x(n)  = (st.Encoder - 7925)/(1040/(pi*0.036))*1000;
        kt(n) = st.KickTicks;
        fs(n) = st.FlagStop;
        if isnan(volts), volts = st.VoltageVolts; end
        if ~fired
            kickFcn(); fired = true;
        end
        if n >= cap, break; end
    end
    idx = 1:n;
    r = struct("t",t(idx),"theta_deg",a(idx),"u_pwm",u(idx), ...
               "cart_mm",x(idx),"kickTicks",kt(idx),"flagStop",fs(idx), ...
               "volts",volts,"rateHz",n/t(n));

    % Kick instant: first sample where the board reported ticks remaining.
    j = find(r.kickTicks > 0, 1, "first");
    if isempty(j), r.tKick = 0; else, r.tKick = r.t(j); end
    r = addMetrics(r);
end

function r = addMetrics(r)
    th = r.theta_deg;  after = r.t >= r.tKick;
    r.angleRms    = sqrt(mean(th(after).^2));
    r.anglePeak   = max(abs(th(after)));
    r.effortRms   = sqrt(mean(r.u_pwm.^2));
    r.satPct      = 100*mean(abs(r.u_pwm) >= 6890);
    r.driftMm     = r.cart_mm(end) - r.cart_mm(1);
    r.cartRangeMm = max(r.cart_mm) - min(r.cart_mm);

    % Settling: the last moment |theta| leaves a band of 20 % of the peak
    % (floored at 0.3 deg so sensor noise cannot define the band).
    band = max(0.3, 0.2*r.anglePeak);
    out  = find(abs(th) > band & after, 1, "last");
    if isempty(out) || out == numel(th)
        r.recoverS = NaN;                      % never settled, or never left
    else
        r.recoverS = r.t(out) - r.tKick;
    end

    if any(r.flagStop)
        r.outcome = "ROD FELL (firmware cut the motor)";
    elseif max(abs(r.cart_mm)) > 200
        r.outcome = "rod up, CART REACHED THE RAIL END";
    else
        r.outcome = "stable";
    end
end

function reportRun(label,r)
    fprintf("\n  --- %s ---\n",label);
    fprintf("  outcome       : %s\n", r.outcome);
    fprintf("  angle RMS     : %.3f deg   peak %.3f deg\n", r.angleRms, r.anglePeak);
    fprintf("  recovery      : %.2f s\n", r.recoverS);
    fprintf("  control effort: %.0f PWM rms,  %.1f%% saturated\n", r.effortRms, r.satPct);
    fprintf("  cart          : drift %+.0f mm,  range %.0f mm\n", r.driftMm, r.cartRangeMm);
    fprintf("  sample rate   : %.1f Hz\n", r.rateHz);
end
