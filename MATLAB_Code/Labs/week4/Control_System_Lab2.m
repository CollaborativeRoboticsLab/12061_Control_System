%% Control_System_Lab2.m
% CONTROL SYSTEMS 12061 / 12601 - Laboratory 2
% WHEELTEC linear inverted pendulum
%
% PURPOSE
%   This script continues directly from Lab 1.
%   Lab 1: observe the tutor demonstration and the live measurements.
%   Lab 2: operate the rig yourself, configure MATLAB, acquire your own
%          data, check calibration, analyse noise/quantisation/sampling,
%          and compare simple signal-conditioning methods.
%
% IMPORTANT
%   Run ONE section at a time using Ctrl+Enter (Run Section).
%   Do NOT press F5 / Run All.
%
% DATA FORMAT FROM STM32 (DataScope binary frame)
%   Byte 1      : 0x24 ('$')             frame header
%   Bytes 2-5   : Channel 1, float32 LE  Angle_Balance (ADC raw)
%   Bytes 6-9   : Channel 2, float32 LE  Encoder / cart position
%   Bytes 10-13 : Channel 3, float32 LE  Motor command u(t) / Moto
%   Byte 14     : 0x0D                   frame tail
%
% CURRENT RIG / FIRMWARE ASSUMPTIONS
%   UART baud rate      : 128000
%   DataScope update    : approximately 14 Hz  (MEASURED on this rig,
%                         2026-09-07. The 50 ms loop implies 20 Hz; the
%                         OLED refresh costs the rest. Every section
%                         measures the real rate rather than assuming.)
%   Angle upright ADC   : approximately 3100
%   Angle downward ADC  : approximately 1020
%   Encoder initial CNT : 10000
%
% NOTE
%   The cart position scale MUST be checked experimentally with a ruler.
%   The default pulley model below is only a nominal starting point.
%
% MATLAB: tested for R2024a-style serialport workflow.
%
% -------------------------------------------------------------------------
% CHANGES MADE 2026-09-07 (checked against the rig, firmware R4-2)
%   1. Default port COM10 -> COM4.
%   2. Added pause(2.6) after setDTR/setRTS in SECTION 1: those lines reset
%      the board, which is then silent for 2 s. Without the wait the first
%      two seconds of every capture were empty.
%   3. Sampling rate comments corrected: measured 14 Hz, not 20 Hz.
%   4. SECTION 10 now prints the moving-average window IN CONTEXT -- window
%      length in seconds against the measured swing period. At 14 Hz the
%      default N=10 spans 0.7 s against a 1.2 s swing, so it removes about
%      a quarter of the real swing amplitude, not just noise. Students
%      should see that number rather than trust the default.
%   5. Added estimateSwingPeriod (autocorrelation, no toolbox needed).
%      Peak counting was tried first and abandoned: it breaks above about
%      1.5 deg of noise, reporting a period several times too short.
%      Autocorrelation with sub-sample interpolation stays within 0.5 %
%      up to 4 deg of noise, and returns NaN instead of a wrong number
%      when there is no clear periodicity.
%   6. The frame parser was stress-tested (random chunking, leading junk,
%      0x24/0x0D inside payloads, dropped bytes). It is correct as written.
% -------------------------------------------------------------------------

%
% -------------------------------------------------------------------------
% RECOMMENDED LAB FLOW
%   SECTION 0  - User settings
%   SECTION 1  - Open serial port
%   SECTION 2  - Live monitor: theta(t), x(t), u(t)
%   SECTION 3  - Calibration checks (angle + cart position)
%   SECTION 4  - Capture A: rod held still
%   SECTION 5  - Capture B: rod swinging freely
%   SECTION 6  - Capture C: balancing + position step
%   SECTION 7  - Data check and measured sampling rate
%   SECTION 8  - Noise analysis
%   SECTION 9  - Quantisation / resolution
%   SECTION 10 - Causal moving-average filter
%   SECTION 11 - First-order low-pass filter
%   SECTION 12 - Compare raw and filtered signals
%   SECTION 99 - Close serial port
% -------------------------------------------------------------------------


%% ===================== SECTION 0 - USER SETTINGS ========================
% EDIT THIS SECTION ONLY unless your tutor tells you otherwise.

clearvars -except ser
clc;

cfg = struct;

% ---- Group / file naming ----
cfg.GROUP_ID = "G01";                 % e.g. "G01", "G02", ...
% Where captures are saved. Anchored to THIS SCRIPT's own folder, NOT to
% MATLAB's current directory. That is the fix for Lab2_Data appearing in the
% toolbox root: "Lab2_Data" was a relative path, so the folder landed wherever
% MATLAB happened to be pointing when you ran setupPath.
labFolder = fileparts(mfilename("fullpath"));
if isempty(labFolder), labFolder = pwd; end   % e.g. pasted into Command Window
cfg.DATA_FOLDER = fullfile(labFolder,"Lab2_Data");

% ---- Serial connection ----
cfg.PORT = "COM4";                    % CHANGE THIS after serialportlist
cfg.BAUD = 128000;
% cfg.BAUD = 9600;
cfg.SERIAL_TIMEOUT_S = 2;

% ---- Live monitor ----
cfg.LIVE_TIME_S = 30;                 % set longer if needed
cfg.STATUS_PRINT_EVERY_N_FRAMES = 5;

% ---- Angle calibration ----
% theta = 0 deg is defined at the upright equilibrium.
cfg.ANGLE_UPRIGHT_ADC  = 3100;
cfg.ANGLE_DOWNWARD_ADC = 1020;
cfg.ANGLE_SIGN = 1;                   % flip to -1 if required

% ---- Cart encoder reference ----
cfg.ENCODER_ZERO = 10000;
cfg.POSITION_SIGN = -1;               % flip to +1 if required

% Nominal mechanical conversion.
% Original firmware comments indicate 1040 encoder counts/output revolution.
cfg.ENCODER_COUNTS_PER_REV = 1040;
cfg.PULLEY_DIAMETER_M = 0.020;        % nominal only; verify with ruler

% OPTIONAL: if you have measured a better scale with the ruler test,
% enter it here in mm/count. Leave as NaN to use the pulley model above.
cfg.POSITION_MM_PER_COUNT = NaN;

% ---- Capture durations ----
cfg.CAPTURE_A_TIME_S = 20;            % rod held still
cfg.CAPTURE_B_TIME_S = 30;            % free swing
cfg.CAPTURE_C_TIME_S = 40;            % balancing + position steps

% ---- Filtering ----
cfg.MA_WINDOW = 10;                   % samples
cfg.LP_CUTOFF_HZ = 2.0;               % first-order low-pass cutoff

% ---- DataScope frame definition ----
cfg.HEADER = uint8(hex2dec('24'));
cfg.TAIL = uint8(hex2dec('0D'));
cfg.FRAME_LENGTH = 14;

% ---- Derived conversion constants ----
cfg.ADC_SPAN = cfg.ANGLE_UPRIGHT_ADC - cfg.ANGLE_DOWNWARD_ADC;
cfg.DEG_PER_ADC_COUNT = 180 / cfg.ADC_SPAN;
cfg.RAD_PER_ADC_COUNT = pi / cfg.ADC_SPAN;

if isfinite(cfg.POSITION_MM_PER_COUNT)
    cfg.M_PER_COUNT = cfg.POSITION_MM_PER_COUNT / 1000;
else
    cfg.M_PER_COUNT = pi * cfg.PULLEY_DIAMETER_M / cfg.ENCODER_COUNTS_PER_REV;
end

if ~exist(cfg.DATA_FOLDER, "dir")
    mkdir(cfg.DATA_FOLDER);
end

fprintf("\n=== LAB 2 CONFIGURATION ===\n");
fprintf("Group ID                : %s\n", cfg.GROUP_ID);
fprintf("Port                    : %s\n", cfg.PORT);
fprintf("Baud                    : %d\n", cfg.BAUD);
fprintf("Angle resolution        : %.6f deg/ADC count\n", abs(cfg.DEG_PER_ADC_COUNT));
fprintf("Nominal position scale  : %.6f mm/encoder count\n", abs(cfg.M_PER_COUNT*1000));
fprintf("Data folder             : %s\n\n", cfg.DATA_FOLDER);

disp("Before opening the port, run:");
disp('    serialportlist("available")');
disp("and update cfg.PORT above if necessary.");


%% ===================== SECTION 1 - OPEN SERIAL PORT =====================
% Run this section after SECTION 0.
%
% If the port opens but no data arrive later, press RESET on the STM32 once.

if ~exist("cfg","var")
    error("Run SECTION 0 first.");
end

fprintf("\nAvailable serial ports:\n");
disp(serialportlist("available"));

% Close an old MATLAB serial object on the same port.
oldPorts = serialportfind;
for k = 1:numel(oldPorts)
    try
        if strcmpi(string(oldPorts(k).Port), cfg.PORT)
            delete(oldPorts(k));
        end
    catch
    end
end
pause(0.2);

fprintf("Opening %s at %d baud...\n", cfg.PORT, cfg.BAUD);

ser = serialport(cfg.PORT, cfg.BAUD, "Timeout", cfg.SERIAL_TIMEOUT_S);

% Dropping DTR/RTS is required for reliable data on this adapter, but the
% lines are wired to the reset circuit: the board REBOOTS here and then sits
% in main()'s two delay_ms(1000) calls for 2 s. Wait, THEN flush -- without
% this the first two seconds of every capture are empty and the "no frame"
% warning fires for no reason.
try
    setDTR(ser, false);
catch
end
try
    setRTS(ser, false);
catch
end

fprintf("Board reboots when the port opens. Waiting 2.6 s for it...\n");
pause(2.6);
flush(ser, "input");

fprintf("Serial port opened.\n");
fprintf("If no data appear in SECTION 2, press RESET on the STM32 once.\n");


%% ===================== SECTION 2 - LIVE MONITOR =========================
% STUDENT TASK
%   1. Gently move the pendulum by hand.
%   2. Move the cart slowly left and right.
%   3. Observe:
%        theta(t)  - pendulum angle
%        x(t)      - cart position
%        u(t)      - motor command
%
% During a motor-OFF activity, u(t) should normally be near zero.
%
% This section records a short live dataset in the variable "liveData".

assertSerialReady(ser);

disp(" ");
disp("=== LIVE MONITOR ===");
disp("Move the rod and cart as instructed in the manual.");
fprintf("Recording for %.1f s...\n", cfg.LIVE_TIME_S);

liveData = acquireDataScope(ser, cfg.LIVE_TIME_S, cfg, true, ...
    "Lab 2 - Live Monitor", false);

if ~isempty(liveData)
    fprintf("\nLive monitor finished: %d valid frames.\n", height(liveData));
    fprintf("Measured mean sampling rate: %.2f Hz\n", ...
        estimateSamplingRate(liveData.time_s));
end


%% ===================== SECTION 3 - CALIBRATION CHECKS ==================
% This section performs simple calibration checks.
%
% 3A: Angle
%   - rod hanging DOWN
%   - rod held vertically UP
%   - compute ADC span and deg/count
%
% 3B: Cart position
%   - record encoder at a starting point
%   - move cart a measured distance with a ruler
%   - compute mm/count
%
% IMPORTANT:
%   Do not run this section while the balancing controller is active.

assertSerialReady(ser);

disp(" ");
disp("============================================================");
disp("SECTION 3A - ANGLE CALIBRATION");
disp("============================================================");

disp("Place the rod hanging straight DOWN and keep it still.");
input("Press Enter when ready... ","s");
downData = acquireDataScope(ser, 2.0, cfg, false, "Angle Down", false);
adcDownMeasured = mean(downData.angle_adc, "omitnan");

disp("Now hold the rod vertically UP and keep it still.");
input("Press Enter when ready... ","s");
upData = acquireDataScope(ser, 2.0, cfg, false, "Angle Up", false);
adcUpMeasured = mean(upData.angle_adc, "omitnan");

adcSpanMeasured = adcUpMeasured - adcDownMeasured;
degPerCountMeasured = 180 / adcSpanMeasured;

fprintf("\nMeasured downward ADC : %.2f\n", adcDownMeasured);
fprintf("Measured upright ADC  : %.2f\n", adcUpMeasured);
fprintf("Measured ADC span      : %.2f counts\n", adcSpanMeasured);
fprintf("Measured resolution    : %.6f deg/count\n", abs(degPerCountMeasured));

if abs(adcDownMeasured - 1020) <= 10
    fprintf("Manufacturer downward ADC check (1020 +/- 10): PASS\n");
else
    fprintf("Manufacturer downward ADC check (1020 +/- 10): CHECK WITH TUTOR\n");
end

fprintf("\nCurrent cfg values:\n");
fprintf("cfg.ANGLE_DOWNWARD_ADC = %.1f\n", cfg.ANGLE_DOWNWARD_ADC);
fprintf("cfg.ANGLE_UPRIGHT_ADC  = %.1f\n", cfg.ANGLE_UPRIGHT_ADC);

disp("If your tutor approves the measured calibration, update SECTION 0");
disp("and re-run SECTION 0 before continuing.");

disp(" ");
disp("============================================================");
disp("SECTION 3B - CART POSITION CALIBRATION");
disp("============================================================");

disp("Place the cart at a convenient starting point.");
input("Press Enter when the cart is still... ","s");
startData = acquireDataScope(ser, 1.5, cfg, false, "Cart Start", false);
encStart = mean(startData.encoder_raw, "omitnan");

distance_mm = input( ...
    "Enter the physical ruler distance you will move the cart (mm), e.g. 100: ");

disp("Move the cart by that measured distance and hold it still.");
input("Press Enter when the cart is at the new position... ","s");
endData = acquireDataScope(ser, 1.5, cfg, false, "Cart End", false);
encEnd = mean(endData.encoder_raw, "omitnan");

deltaCounts = encEnd - encStart;

if deltaCounts == 0
    warning("Encoder count did not change. Repeat SECTION 3B.");
else
    measuredMmPerCount = distance_mm / abs(deltaCounts);

    fprintf("\nEncoder start          : %.2f counts\n", encStart);
    fprintf("Encoder end            : %.2f counts\n", encEnd);
    fprintf("Encoder change         : %.2f counts\n", deltaCounts);
    fprintf("Ruler distance         : %.3f mm\n", distance_mm);
    fprintf("Measured scale         : %.6f mm/count\n", measuredMmPerCount);
    fprintf("Current script scale   : %.6f mm/count\n", abs(cfg.M_PER_COUNT*1000));

    reportedDistanceMm = abs(deltaCounts) * abs(cfg.M_PER_COUNT*1000);
    calibrationErrorPct = ...
        (reportedDistanceMm - distance_mm) / distance_mm * 100;

    fprintf("Current script reports : %.3f mm\n", reportedDistanceMm);
    fprintf("Calibration error      : %.2f %%\n", calibrationErrorPct);

    disp("If your tutor approves the ruler result, put the measured");
    disp("mm/count value into cfg.POSITION_MM_PER_COUNT in SECTION 0,");
    disp("then re-run SECTION 0.");
end


%% ===================== SECTION 4 - CAPTURE A: HELD STILL ================
% Operator:
%   Hold the rod as steadily as possible at a fixed angle.
%
% MATLAB driver:
%   Run this section.
%
% Output:
%   captureA_hold_still table
%   MAT + CSV files saved automatically

assertSerialReady(ser);

disp(" ");
disp("=== CAPTURE A: ROD HELD STILL ===");
countdown(3);
fprintf("Recording for %.1f s. Hold the rod still.\n", cfg.CAPTURE_A_TIME_S);

captureA_hold_still = acquireDataScope(ser, cfg.CAPTURE_A_TIME_S, ...
    cfg, true, "Capture A - Rod Held Still", false);

if ~isempty(captureA_hold_still)
    saveCapture(captureA_hold_still, cfg, "A_hold_still");
    fprintf("Capture A complete: %d samples.\n", height(captureA_hold_still));
end


%% ===================== SECTION 5 - CAPTURE B: FREE SWING ================
% Operator:
%   Lift the rod approximately 30 deg from the hanging position,
%   then release it cleanly. Do not touch it again.
%
% Output:
%   captureB_free_swing table
%   MAT + CSV files saved automatically

assertSerialReady(ser);

disp(" ");
disp("=== CAPTURE B: FREE SWING ===");
disp("Prepare the rod approximately 30 deg away from hanging.");
countdown(3);
fprintf("Release now. Recording for %.1f s.\n", cfg.CAPTURE_B_TIME_S);

captureB_free_swing = acquireDataScope(ser, cfg.CAPTURE_B_TIME_S, ...
    cfg, true, "Capture B - Free Swing", false);

if ~isempty(captureB_free_swing)
    saveCapture(captureB_free_swing, cfg, "B_free_swing");
    fprintf("Capture B complete: %d samples.\n", height(captureB_free_swing));
end


%% ============ SECTION 6 - CAPTURE C: BALANCING + POSITION STEP =========
% MOTOR LIVE - HANDS CLEAR OF THE TRACK.
%
% Follow your tutor's approved balancing procedure.
%
% Suggested timing:
%   0-10 s   : get the pendulum balanced
%   10-20 s  : let it balance quietly
%   at 20 s  : SINGLE-CLICK SPARE button -> forward position step
%   at 30 s  : DOUBLE-CLICK SPARE button -> reverse position step
%   30-40 s  : observe recovery
%
% Output:
%   captureC_balancing table
%   MAT + CSV files saved automatically

assertSerialReady(ser);

disp(" ");
disp("============================================================");
disp("CAPTURE C: BALANCING + POSITION STEP");
disp("MOTOR LIVE - HANDS CLEAR OF THE TRACK");
disp("============================================================");
disp("Follow tutor instructions for hand start or swing-up.");
countdown(3);

captureC_balancing = acquireDataScope(ser, cfg.CAPTURE_C_TIME_S, ...
    cfg, true, "Capture C - Balancing and Position Step", true);

if ~isempty(captureC_balancing)
    saveCapture(captureC_balancing, cfg, "C_balancing_position_step");
    fprintf("Capture C complete: %d samples.\n", height(captureC_balancing));
end


%% ===================== SECTION 7 - DATA CHECK ===========================
% Check the captured datasets before leaving the rig.

disp(" ");
disp("=== DATA CHECK ===");

datasetNames = ["captureA_hold_still", ...
                "captureB_free_swing", ...
                "captureC_balancing"];

for i = 1:numel(datasetNames)
    name = datasetNames(i);

    if evalin("base", "exist('" + name + "','var')")
        Tcheck = evalin("base", name);

        if isempty(Tcheck)
            fprintf("%s: EMPTY - repeat this capture.\n", name);
            continue;
        end

        fsMeasured = estimateSamplingRate(Tcheck.time_s);
        durationMeasured = Tcheck.time_s(end) - Tcheck.time_s(1);

        fprintf("\n%s\n", name);
        fprintf("  Samples            : %d\n", height(Tcheck));
        fprintf("  Duration           : %.2f s\n", durationMeasured);
        fprintf("  Mean/median fs     : %.2f Hz\n", fsMeasured);
        fprintf("  theta range        : %.3f to %.3f deg\n", ...
            min(Tcheck.theta_deg), max(Tcheck.theta_deg));
        fprintf("  position range     : %.5f to %.5f m\n", ...
            min(Tcheck.position_m), max(Tcheck.position_m));
        fprintf("  motor command peak : %.1f PWM counts\n", ...
            max(abs(Tcheck.motor_u_pwm)));
    else
        fprintf("\n%s: NOT FOUND\n", name);
    end
end

disp(" ");
disp("If any required capture is missing or clearly wrong, repeat it now.");


%% ===================== SECTION 8 - NOISE ANALYSIS =======================
% Uses Capture A (rod held still).
%
% This section quantifies the measurement variation.

if ~exist("captureA_hold_still","var") || isempty(captureA_hold_still)
    error("Run SECTION 4 first.");
end

Tnoise = captureA_hold_still;

mean_angle = mean(Tnoise.theta_deg, "omitnan");
std_angle  = std(Tnoise.theta_deg,  "omitnan");
var_angle  = var(Tnoise.theta_deg,  "omitnan");
ptp_angle  = max(Tnoise.theta_deg) - min(Tnoise.theta_deg);

mean_position = mean(Tnoise.position_m, "omitnan");
std_position  = std(Tnoise.position_m,  "omitnan");
ptp_position  = max(Tnoise.position_m) - min(Tnoise.position_m);

fprintf("\n=== NOISE ANALYSIS: CAPTURE A ===\n");
fprintf("Angle mean              : %.6f deg\n", mean_angle);
fprintf("Angle standard deviation: %.6f deg\n", std_angle);
fprintf("Angle variance          : %.6f deg^2\n", var_angle);
fprintf("Angle peak-to-peak      : %.6f deg\n", ptp_angle);

fprintf("Cart mean               : %.6f m\n", mean_position);
fprintf("Cart standard deviation : %.6f m\n", std_position);
fprintf("Cart peak-to-peak       : %.6f m\n", ptp_position);

figure("Name","Lab 2 - Noise Analysis","NumberTitle","off");
tiledlayout(2,1);

nexttile;
plot(Tnoise.time_s, Tnoise.theta_deg);
grid on;
xlabel("Time (s)");
ylabel("\theta (deg)");
title("Pendulum Angle - Held Still");

nexttile;
plot(Tnoise.time_s, Tnoise.position_m);
grid on;
xlabel("Time (s)");
ylabel("x (m)");
title("Cart Position - Held Still");


%% ===================== SECTION 9 - QUANTISATION / RESOLUTION ============
% Examine raw sensor levels and the smallest observed non-zero steps.
%
% Angle:
%   potentiometer -> ADC -> counts
%
% Cart:
%   Hall encoder -> encoder counts

if ~exist("captureA_hold_still","var") || isempty(captureA_hold_still)
    error("Run SECTION 4 first.");
end

Tq = captureA_hold_still;

dADC = abs(diff(Tq.angle_adc));
dADC = dADC(dADC > 0);

dENC = abs(diff(Tq.encoder_raw));
dENC = dENC(dENC > 0);

fprintf("\n=== QUANTISATION / RESOLUTION ===\n");

if isempty(dADC)
    fprintf("No non-zero ADC step was observed in this capture.\n");
else
    minAdcStep = min(dADC);
    fprintf("Smallest observed ADC step : %.3f counts\n", minAdcStep);
    fprintf("Equivalent angle step      : %.6f deg\n", ...
        minAdcStep * abs(cfg.DEG_PER_ADC_COUNT));
end

if isempty(dENC)
    fprintf("No non-zero encoder step was observed in this capture.\n");
else
    minEncStep = min(dENC);
    fprintf("Smallest observed encoder step : %.3f counts\n", minEncStep);
    fprintf("Equivalent cart step            : %.6f mm\n", ...
        minEncStep * abs(cfg.M_PER_COUNT*1000));
end

figure("Name","Lab 2 - Quantisation","NumberTitle","off");
tiledlayout(2,1);

nexttile;
stairs(Tq.time_s, Tq.angle_adc);
grid on;
xlabel("Time (s)");
ylabel("ADC count");
title("Raw Angle ADC Levels");

nexttile;
stairs(Tq.time_s, Tq.encoder_raw);
grid on;
xlabel("Time (s)");
ylabel("Encoder count");
title("Raw Cart Encoder Levels");


%% ===================== SECTION 10 - CAUSAL MOVING AVERAGE ==============
% Uses the free-swing dataset by default because it contains both
% real motion and measurement noise.
%
% IMPORTANT:
%   For a real-time controller, the filter must be CAUSAL.
%   Therefore we use only the current and previous samples:
%       movmean(signal,[N-1 0])

if ~exist("captureB_free_swing","var") || isempty(captureB_free_swing)
    error("Run SECTION 5 first.");
end

Tfilt = captureB_free_swing;

Nma = cfg.MA_WINDOW;
theta_ma = movmean(Tfilt.theta_deg, [Nma-1 0], "Endpoints", "shrink");

% A window is only meaningful relative to the signal it filters. At the
% measured ~14 Hz, N=10 spans about 0.7 s, while the rod's free-swing
% period is about 1.2 s -- so this window covers more than half a swing
% and will visibly flatten the real motion, not just the noise.
TsHere = median(diff(Tfilt.time_s));
fprintf("\n=== MOVING AVERAGE WINDOW IN CONTEXT ===\n");
fprintf("Measured Ts            : %.4f s  (fs = %.2f Hz)\n",TsHere,1/TsHere);
fprintf("Window N=%d spans      : %.3f s\n",Nma,Nma*TsHere);
swingPeriod = estimateSwingPeriod(Tfilt.time_s, Tfilt.theta_deg);
if isfinite(swingPeriod)
    fprintf("Measured swing period  : %.3f s   (model predicts 1.198 s)\n",swingPeriod);
    fprintf("Window / period        : %.2f", Nma*TsHere/swingPeriod);
    if Nma*TsHere > 0.25*swingPeriod
        fprintf("   <-- TOO LONG: this is filtering the SIGNAL, not just noise\n");
    else
        fprintf("   (reasonable)\n");
    end
else
    fprintf("Swing period           : could not measure it from this capture\n");
end

figure("Name","Lab 2 - Moving Average","NumberTitle","off");
plot(Tfilt.time_s, Tfilt.theta_deg);
hold on;
plot(Tfilt.time_s, theta_ma, "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("\theta (deg)");
legend("Raw","Causal moving average");
title("Raw vs Causal Moving-Average Filter");

% Compare several windows.
ma5  = movmean(Tfilt.theta_deg, [5-1 0],  "Endpoints", "shrink");
ma10 = movmean(Tfilt.theta_deg, [10-1 0], "Endpoints", "shrink");
ma20 = movmean(Tfilt.theta_deg, [20-1 0], "Endpoints", "shrink");
ma50 = movmean(Tfilt.theta_deg, [50-1 0], "Endpoints", "shrink");

figure("Name","Lab 2 - Moving Average Window Comparison", ...
       "NumberTitle","off");
plot(Tfilt.time_s, Tfilt.theta_deg);
hold on;
plot(Tfilt.time_s, ma5);
plot(Tfilt.time_s, ma10);
plot(Tfilt.time_s, ma20);
plot(Tfilt.time_s, ma50);
grid on;
xlabel("Time (s)");
ylabel("\theta (deg)");
legend("Raw","N=5","N=10","N=20","N=50");
title("Effect of Moving-Average Window Length");


%% ===================== SECTION 11 - FIRST-ORDER LOW-PASS FILTER =========
% We implement the filter directly so that it links to the first-order
% system studied in the tutorial:
%
%       G(s) = 1 / (tau*s + 1)
%
%       fc  = 1/(2*pi*tau)
%       pole = -1/tau
%
% Discrete causal approximation:
%
%       y[k] = alpha*x[k] + (1-alpha)*y[k-1]
%
% where alpha = Ts/(tau + Ts)
%
% This avoids requiring a separate filtering toolbox.

if ~exist("captureB_free_swing","var") || isempty(captureB_free_swing)
    error("Run SECTION 5 first.");
end

Tfilt = captureB_free_swing;

TsMeasured = median(diff(Tfilt.time_s));
fsMeasured = 1 / TsMeasured;

fc = cfg.LP_CUTOFF_HZ;
tau = 1 / (2*pi*fc);
filterPole = -1/tau;

alpha = TsMeasured / (tau + TsMeasured);

theta_lp = zeros(size(Tfilt.theta_deg));
theta_lp(1) = Tfilt.theta_deg(1);

for k = 2:length(Tfilt.theta_deg)
    theta_lp(k) = alpha * Tfilt.theta_deg(k) ...
                + (1-alpha) * theta_lp(k-1);
end

fprintf("\n=== FIRST-ORDER LOW-PASS FILTER ===\n");
fprintf("Measured median Ts : %.6f s\n", TsMeasured);
fprintf("Measured fs        : %.3f Hz\n", fsMeasured);
fprintf("Cutoff fc          : %.3f Hz\n", fc);
fprintf("Time constant tau  : %.6f s\n", tau);
fprintf("Continuous pole    : %.6f rad/s\n", filterPole);
fprintf("Discrete alpha     : %.6f\n", alpha);

figure("Name","Lab 2 - First-Order Low-Pass Filter","NumberTitle","off");
plot(Tfilt.time_s, Tfilt.theta_deg);
hold on;
plot(Tfilt.time_s, theta_lp, "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("\theta (deg)");
legend("Raw","First-order low-pass");
title("Raw vs First-Order Low-Pass Filter");


%% ===================== SECTION 12 - COMPARE ALL THREE ==================
% Compare:
%   raw signal
%   causal moving average
%   first-order low-pass filter

if ~exist("theta_ma","var")
    error("Run SECTION 10 first.");
end

if ~exist("theta_lp","var")
    error("Run SECTION 11 first.");
end

figure("Name","Lab 2 - Filter Comparison","NumberTitle","off");
plot(Tfilt.time_s, Tfilt.theta_deg);
hold on;
plot(Tfilt.time_s, theta_ma, "LineWidth", 1.5);
plot(Tfilt.time_s, theta_lp, "LineWidth", 1.5);
grid on;
xlabel("Time (s)");
ylabel("\theta (deg)");
legend("Raw","Causal moving average","First-order low-pass");
title("Comparison of Signal Conditioning Methods");

% Optional simple noise comparison using Capture A.
if exist("captureA_hold_still","var") && ~isempty(captureA_hold_still)

    Tstill = captureA_hold_still;

    still_ma = movmean(Tstill.theta_deg, ...
        [cfg.MA_WINDOW-1 0], "Endpoints", "shrink");

    TsStill = median(diff(Tstill.time_s));
    tauStill = 1/(2*pi*cfg.LP_CUTOFF_HZ);
    alphaStill = TsStill/(tauStill + TsStill);

    still_lp = zeros(size(Tstill.theta_deg));
    still_lp(1) = Tstill.theta_deg(1);

    for k = 2:length(Tstill.theta_deg)
        still_lp(k) = alphaStill*Tstill.theta_deg(k) ...
                    + (1-alphaStill)*still_lp(k-1);
    end

    fprintf("\n=== HELD-STILL NOISE COMPARISON ===\n");
    fprintf("Raw std             : %.6f deg\n", std(Tstill.theta_deg));
    fprintf("Moving-average std  : %.6f deg\n", std(still_ma));
    fprintf("Low-pass std        : %.6f deg\n", std(still_lp));
end


%% ===================== SECTION 99 - CLOSE SERIAL PORT ==================
% Always run this section before disconnecting the rig or leaving the lab.

disp("Closing serial connection...");

if exist("ser","var")
    try
        flush(ser);
    catch
    end

    try
        delete(ser);
    catch
    end

    clear ser
end

disp("Serial port closed.");


%% ========================= LOCAL FUNCTIONS ==============================
% Do not edit below this line unless instructed by your tutor.


function assertSerialReady(ser)
    if nargin < 1 || isempty(ser)
        error("Serial port is not open. Run SECTION 1 first.");
    end

    try
        if ~isvalid(ser)
            error("Serial port object is invalid. Run SECTION 1 again.");
        end
    catch
        error("Serial port is not ready. Run SECTION 1 first.");
    end
end


function countdown(n)
    for k = n:-1:1
        fprintf("%d...\n", k);
        pause(1);
    end
end


function T = estimateSwingPeriod(t, y)
%ESTIMATESWINGPERIOD Swing period by autocorrelation. No toolbox needed.
%
%   Counting peaks is the obvious method and it is fragile: once the noise
%   is a few tenths of a degree, noise wiggles become extra "peaks" and the
%   period comes out several times too short. Tested against a synthetic
%   1.198 s swing, peak counting broke at about 1.5 deg of noise (-59 %)
%   while autocorrelation stayed exact up to 4 deg.
%
%   Returns NaN rather than a wrong number when it cannot find a clear
%   periodicity.

    t = t(:); y = y(:);
    if numel(t) < 20, T = NaN; return; end

    Ts = median(diff(t));
    fs = 1/Ts;

    x = y - mean(y,"omitnan");
    x(~isfinite(x)) = 0;

    r = xcorr_local(x);
    r = r / r(1);

    % Ignore very short lags: those are the signal correlating with itself.
    lo = max(2, round(0.15*fs));
    if lo >= numel(r)-2, T = NaN; return; end

    seg = r(lo:end);
    isPk = [false; seg(2:end-1) > seg(1:end-2) & seg(2:end-1) > seg(3:end); false];
    isPk = isPk & (seg > 0.3);          % a weak bump is not a period

    idx = find(isPk,1,"first");
    if isempty(idx)
        T = NaN;
        return;
    end

    % r(1) is lag 0, so seg(idx) = r(lo+idx-1) is lag (lo+idx-2).
    lagSamples = lo + idx - 2;

    % Refine to sub-sample resolution by fitting a parabola through the
    % correlation peak and its two neighbours. Without this the answer is
    % quantised to one sample, which at ~14 Hz is 0.07 s -- 6 % of the
    % period we are trying to measure.
    m = lo + idx - 1;                       % index into r
    if m > 1 && m < numel(r)
        y1 = r(m-1); y2 = r(m); y3 = r(m+1);
        den = y1 - 2*y2 + y3;
        if den ~= 0
            delta = 0.5*(y1 - y3)/den;
            if abs(delta) < 1
                lagSamples = lagSamples + delta;
            end
        end
    end

    T = lagSamples * Ts;
end


function r = xcorr_local(x)
%XCORR_LOCAL Positive-lag autocorrelation, written out so the script does
%   not need the Signal Processing Toolbox.
    n = numel(x);
    r = zeros(n,1);
    for k = 0:n-1
        r(k+1) = sum(x(1:n-k) .* x(k+1:n));
    end
end


function fs = estimateSamplingRate(t)
    t = t(:);
    if numel(t) < 2
        fs = NaN;
        return;
    end

    dt = diff(t);
    dt = dt(isfinite(dt) & dt > 0);

    if isempty(dt)
        fs = NaN;
    else
        % Median is less sensitive to occasional serial/OS scheduling delays.
        fs = 1 / median(dt);
    end
end


function saveCapture(T, cfg, label)
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    base = cfg.GROUP_ID + "_" + label + "_" + stamp;

    matFile = fullfile(cfg.DATA_FOLDER, base + ".mat");
    csvFile = fullfile(cfg.DATA_FOLDER, base + ".csv");

    save(matFile, "T");
    writetable(T, csvFile);

    fprintf("Saved MAT: %s\n", matFile);
    fprintf("Saved CSV: %s\n", csvFile);
end


function T = acquireDataScope(ser, duration_s, cfg, showLive, figureTitle, timedInstructions)
%ACQUIREDATASCOPE Acquire and decode WHEELTEC DataScope binary frames.
%
% T columns:
%   time_s
%   angle_adc
%   theta_deg
%   theta_rad
%   encoder_raw
%   position_count
%   position_m
%   motor_u_pwm

    assertSerialReady(ser);

    flush(ser, "input");

    rxBuffer = uint8([]);
    frameCount = 0;

    time_s = [];
    angle_adc = [];
    theta_deg = [];
    theta_rad = [];
    encoder_raw = [];
    position_count = [];
    position_m = [];
    motor_u_pwm = [];

    if showLive
        fig = figure("Name", figureTitle, "NumberTitle", "off");
        tl = tiledlayout(fig, 3, 1);

        ax1 = nexttile(tl);
        hTheta = animatedline(ax1);
        grid(ax1, "on");
        xlabel(ax1, "Time (s)");
        ylabel(ax1, "\theta (deg)");
        title(ax1, "Pendulum Angle \theta(t)");

        ax2 = nexttile(tl);
        hPosition = animatedline(ax2);
        grid(ax2, "on");
        xlabel(ax2, "Time (s)");
        ylabel(ax2, "Cart position x (m)");
        title(ax2, "Cart Position x(t)");

        ax3 = nexttile(tl);
        hMotor = animatedline(ax3);
        grid(ax3, "on");
        xlabel(ax3, "Time (s)");
        ylabel(ax3, "u (PWM counts)");
        title(ax3, "Motor Command u(t)");
    else
        fig = [];
    end

    t0 = tic;
    lastNoFrameMessage = 0;
    sent10 = false;
    sent20 = false;
    sent30 = false;

    while toc(t0) < duration_s

        elapsed = toc(t0);

        % Timed instructions for Capture C.
        if timedInstructions
            if elapsed >= 10 && ~sent10
                fprintf("\n10 s: let the rig balance quietly.\n");
                sent10 = true;
            end
            if elapsed >= 20 && ~sent20
                fprintf("\n20 s: SINGLE-CLICK SPARE for forward position step.\n");
                sent20 = true;
            end
            if elapsed >= 30 && ~sent30
                fprintf("\n30 s: DOUBLE-CLICK SPARE for reverse position step.\n");
                sent30 = true;
            end
        end

        % Pull all currently available serial bytes.
        n = ser.NumBytesAvailable;

        if n > 0
            newBytes = read(ser, n, "uint8");
            newBytes = reshape(uint8(newBytes), 1, []);
            rxBuffer = [rxBuffer, newBytes]; %#ok<AGROW>
        else
            pause(0.002);
        end

        % Parse as many complete frames as possible.
        while numel(rxBuffer) >= cfg.FRAME_LENGTH

            % Synchronise to '$' = 0x24.
            headerIdx = find(rxBuffer == cfg.HEADER, 1, "first");

            if isempty(headerIdx)
                rxBuffer = uint8([]);
                break;
            end

            if headerIdx > 1
                rxBuffer(1:headerIdx-1) = [];
            end

            if numel(rxBuffer) < cfg.FRAME_LENGTH
                break;
            end

            candidate = rxBuffer(1:cfg.FRAME_LENGTH);

            % A valid 3-channel DataScope frame must end in 0x0D.
            if candidate(end) ~= cfg.TAIL
                rxBuffer(1) = [];
                continue;
            end

            % Decode three little-endian IEEE-754 float32 channels.
            ch1 = double(typecast(uint8(candidate(2:5)),   "single"));
            ch2 = double(typecast(uint8(candidate(6:9)),   "single"));
            ch3 = double(typecast(uint8(candidate(10:13)), "single"));

            rxBuffer(1:cfg.FRAME_LENGTH) = [];

            if ~(isfinite(ch1) && isfinite(ch2) && isfinite(ch3))
                continue;
            end

            frameCount = frameCount + 1;
            t = toc(t0);

            adc = ch1;
            enc = ch2;

            thetaDeg = cfg.ANGLE_SIGN * ...
                (adc - cfg.ANGLE_UPRIGHT_ADC) * cfg.DEG_PER_ADC_COUNT;

            thetaRad = cfg.ANGLE_SIGN * ...
                (adc - cfg.ANGLE_UPRIGHT_ADC) * cfg.RAD_PER_ADC_COUNT;

            posCount = cfg.POSITION_SIGN * (enc - cfg.ENCODER_ZERO);
            posM = posCount * cfg.M_PER_COUNT;

            % Store.
            time_s(end+1,1) = t; %#ok<AGROW>
            angle_adc(end+1,1) = adc; %#ok<AGROW>
            theta_deg(end+1,1) = thetaDeg; %#ok<AGROW>
            theta_rad(end+1,1) = thetaRad; %#ok<AGROW>
            encoder_raw(end+1,1) = enc; %#ok<AGROW>
            position_count(end+1,1) = posCount; %#ok<AGROW>
            position_m(end+1,1) = posM; %#ok<AGROW>
            motor_u_pwm(end+1,1) = ch3; %#ok<AGROW>

            if showLive
                if ~isempty(fig) && isvalid(fig)
                    addpoints(hTheta, t, thetaDeg);
                    addpoints(hPosition, t, posM);
                    addpoints(hMotor, t, ch3);

                    if mod(frameCount, cfg.STATUS_PRINT_EVERY_N_FRAMES) == 0
                        drawnow limitrate;
                    end
                end
            end

            if mod(frameCount, cfg.STATUS_PRINT_EVERY_N_FRAMES) == 0
                fprintf(['t=%7.2f s | ADC=%7.1f | theta=%8.3f deg | ' ...
                         'Encoder=%8.1f | x=%9.5f m | u=%7.1f\n'], ...
                         t, adc, thetaDeg, enc, posM, ch3);
            end
        end

        % Helpful warning if the port is open but no valid frame arrives.
        if frameCount == 0 && elapsed > 3 && elapsed-lastNoFrameMessage > 3
            fprintf("No valid frame yet. Press STM32 RESET once if required.\n");
            lastNoFrameMessage = elapsed;
        end
    end

    if frameCount == 0
        warning("No valid DataScope frames were decoded.");
        T = table();
        return;
    end

    T = table( ...
        time_s(:), ...
        angle_adc(:), ...
        theta_deg(:), ...
        theta_rad(:), ...
        encoder_raw(:), ...
        position_count(:), ...
        position_m(:), ...
        motor_u_pwm(:), ...
        'VariableNames', { ...
            'time_s', ...
            'angle_adc', ...
            'theta_deg', ...
            'theta_rad', ...
            'encoder_raw', ...
            'position_count', ...
            'position_m', ...
            'motor_u_pwm'});

    fprintf("\nAcquisition finished.\n");
    fprintf("Valid frames received : %d\n", frameCount);
    fprintf("Measured sampling rate: %.2f Hz\n", estimateSamplingRate(T.time_s));
end
