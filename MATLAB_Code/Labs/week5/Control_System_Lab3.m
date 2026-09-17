%% Control_System_Lab3.m
% CONTROL SYSTEMS 12061 / 12601 - Laboratory 3, PART B
% WHEELTEC linear inverted pendulum
%
% SESSION 3 - OPEN-LOOP BEHAVIOUR: EXPERIENCING INSTABILITY
%
%   "What happens if the system is left alone - without control?"
%
% In Labs 1 and 2 you observed the rig and measured its signals. Here you
% make it fall, on purpose, and measure HOW it falls. The point is that
% instability is not an abstract property of a pole location: it is a rate,
% and you can measure that rate from your own data.
%
% =========================================================================
% READ THIS FIRST
% =========================================================================
%   1. Run ONE SECTION AT A TIME. Click inside a section and press
%      Ctrl+Enter. Do NOT press F5 or Run All -- in three of these sections
%      you have to be holding the rod when the section starts.
%
%   2. Everything YOU have to do is boxed, like this:
%
%          % ------------------------------------------------------------
%          % >> STUDENT ACTION
%          %    ...
%          % ------------------------------------------------------------
%
%      There are five labels, and they mean different things:
%
%        EDIT THIS        change this before you run anything
%        PREDICT FIRST    write your answer down BEFORE you run the
%                         section -- the point of the experiment is lost
%                         if you read the result first
%        STUDENT ACTION   do something with your hands
%        SAFETY           the motor is live; read it before you run.
%                         Its box is drawn in !!!! instead of ---- so you
%                         cannot scroll past it by accident.
%        FOR YOUR REPORT  numbers or figures to record as you go
%
%      A dashed box means YOU do something. Plain comments explain how the
%      code works -- read them if you are curious, but you do not have to
%      act on them.
%
%   3. THIS SCRIPT DOES NOT MATCH THE HANDOUT LISTING, on purpose.
%
%      The handout shows serialport("COMX",115200) with readline and
%      sscanf('%f,%f'). That does not work on this rig: it runs at 128000
%      baud and does not send comma-separated text. Everything here goes
%      through the tested toolbox instead (connectPendulum,
%      readStatusWheeltec). The physics you are asked to observe is
%      identical; only the plumbing differs.
%
%      The handout also writes motorPWM(0) and motorPWM(0.3). Here the
%      serial object is an explicit argument:
%
%          motorPWM(s, 0)        and      motorPWM(s, 0.3)
%
%      u is normalised: -1 .. +1 maps to the firmware's -6900 .. +6900, so
%      u = 0.3 is 2070 counts -- comfortably above this rig's measured
%      static friction deadband of roughly 700-900, which matters, because
%      a smaller u would move nothing at all and you would conclude the
%      motor was broken.
%
%   4. SECTIONS. Run them in order.
%
%        0   Settings and connection
%        1   Angle reference (measure YOUR rig's upright and hanging)
%        2   Experiment 1: small disturbance, zero motor input
%        3   Experiment 2: repeatability of instability (3 trials)
%        4   Experiment 3: does a constant motor input fix it?  MOTOR LIVE
%        5   Experiment 4: annotated response + measure the pole
%        6   Connect to the tutorial: why settling time has no meaning here
%        99  Close -- always run this before you leave
% =========================================================================


%% ===================== SECTION 0 - SETTINGS =============================
clearvars -except s
clc;

cfg = struct;

% -------------------------------------------------------------------------
% >> EDIT THIS
%    Two lines to change before your first run:
%
%      cfg.PORT      the COM port your rig is on. Find it in Windows
%                    Device Manager, or type  serialportlist  in the
%                    Command Window. It is almost never COM1.
%      cfg.GROUP_ID  your group number. Every file this script saves is
%                    named with it, so if you leave it as G01 your data
%                    will overwrite somebody else's.
% -------------------------------------------------------------------------
cfg.PORT        = "COM4";
cfg.GROUP_ID    = "G01";
% Where does this script live? Sections are run one at a time with
% Ctrl+Enter, and MATLAB executes Run Section (and Evaluate Selection) from
% a TEMPORARY COPY of the text -- so mfilename("fullpath") returns
% %TEMP%\Editor_xxxxx and CANNOT be used to locate this file. mfilename is
% reliable inside a FUNCTION, which is why setupPath.m still uses it, but
% not in a script you run section by section, which is how this lab works.
%
% Resolve through the MATLAB search path instead: setupPath puts every
% Labs\week* folder on it, and the temp folder is never on it.
labFolder = '';

onPath = which('Control_System_Lab3.m');
if ~isempty(onPath) && ~startsWith(onPath,tempdir,'IgnoreCase',true)
    labFolder = fileparts(onPath);
end

if isempty(labFolder)                 % not on the path -- try mfilename
    here = fileparts(mfilename('fullpath'));
    if ~isempty(here) && ~startsWith(here,tempdir,'IgnoreCase',true)
        labFolder = here;
    end
end

if isempty(labFolder)                 % last resort, and say so out loud
    labFolder = pwd;
    warning("Lab3:FolderGuess", ...
        ['Could not locate Control_System_Lab3.m on the MATLAB path, so captures will ' ...
         'go to\n  %s\nRun setupPath from MATLAB_Code if that is not ' ...
         'where you want them.'],labFolder);
end
cfg.DATA_FOLDER = fullfile(labFolder,"data");

% Capture durations (s)
cfg.FALL_TIME_S   = 4.0;    % Experiment 1 and 2: enough to see the whole fall
cfg.DRIVEN_TIME_S = 6.0;    % Experiment 3

% Cart travel limits, in encoder counts. This is the softer band the
% script enforces; the firmware independently refuses to drive outside
% 5900..9900. Both are ABSOLUTE numbers, which only mean anything once
% homeCart has run -- see the STUDENT ACTION box at the end of Section 0.
cfg.CART_MIN    = 6600;
cfg.CART_MAX    = 9300;
cfg.CART_TARGET = 7925;   % POSITION_MIDDLE in control.h
cfg.CART_TOL    = 200;    % close enough to the middle to start

% Normalised motor input for Experiment 3 (handout uses 0.3)
cfg.U_CONSTANT = 0.3;

cfg.PWM_FULL_SCALE = 6900;  % matches UARTCMD_PWM_LIMIT in the firmware

if ~exist(cfg.DATA_FOLDER,"dir"), mkdir(cfg.DATA_FOLDER); end

if ~exist("s","var") || isempty(s) || ~isvalid(s)
    s = connectPendulum(cfg.PORT);
end

st = readStatusWheeltec(s);
fprintf("\n=== LAB 3 READY ===\n");
fprintf("  cart position : %d counts\n",st.Encoder);
fprintf("  battery       : %.2f V\n",st.VoltageVolts);
fprintf("  rx errors     : %d\n",st.RxErrors);
if st.VoltageVolts < 10.5
    warning("Lab3:LowBattery","Battery %.2f V. Experiment 3 needs the motor.",st.VoltageVolts);
end

% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    Slide the cart by hand to the MIDDLE of the rail, then answer the
%    prompt. A centimetre either way is fine.
%
%    Why: the cart encoder counts how far the cart has moved, but it has
%    no idea where the cart started. Every reset sets the count to 10000
%    wherever the cart happens to be sitting, and opening the serial port
%    resets the board. So until you tell the firmware where the cart is,
%    the rail-end protection is measuring from an origin nobody chose.
%    Homing declares the origin: the count becomes 7925 and 5900 / 9900
%    become the real ends of the rail.
% -------------------------------------------------------------------------
homeCart(s,"Target",cfg.CART_TARGET,"Tol",cfg.CART_TOL);
st = readStatusWheeltec(s);
fprintf("  cart position : %d counts (homed)\n",st.Encoder);


%% ===================== SECTION 1 - ANGLE REFERENCE ======================
% Every angle in this lab is measured relative to two numbers that are
% specific to YOUR rig. The firmware assumes hanging = 1020 and upright =
% 3100, but an uncalibrated rig differs, and if you use the firmware's
% numbers instead of your own, every angle you report is wrong by a fixed
% offset and a fixed scale factor.

% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    This section asks you for two positions. Do them properly -- the
%    whole lab is calibrated against them.
%
%      1. Let the rod hang straight down and STOP MOVING. Take your hands
%         right off it; a hand resting on the rail is enough to bias the
%         reading. Then press Enter.
%      2. Hold the rod as close to vertically UP as you can judge, and hold
%         it STILL for the two seconds after you press Enter. Do not let it
%         drift while the script is averaging.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Record all four numbers this section prints: hanging ADC, upright
%    ADC, span, and degrees per count. You need the last one to convert
%    any raw ADC value into an angle, and the demonstrator will ask why
%    yours is not the same as the group next to you.
% -------------------------------------------------------------------------

motorPWM(s,0);

disp(" ");
disp("Let the rod hang straight down and come to rest. Hands off.");
input("Press Enter when it is still... ","s");
adcDown = meanAngle(s,2.0);

disp("Now hold the rod vertically UP, as close to balanced as you can.");
input("Press Enter while holding it there... ","s");
adcUp = meanAngle(s,2.0);

cfg.ADC_DOWN = adcDown;
cfg.ADC_UP   = adcUp;
cfg.ADC_SPAN = adcUp - adcDown;
cfg.DEG_PER_COUNT = 180 / cfg.ADC_SPAN;

fprintf("\nHanging ADC   : %.1f   (firmware assumes 1020)\n",adcDown);
fprintf("Upright ADC   : %.1f   (firmware assumes 3100)\n",adcUp);
fprintf("Span          : %.1f counts over 180 deg\n",cfg.ADC_SPAN);
fprintf("Resolution    : %.4f deg per count\n",cfg.DEG_PER_COUNT);

if abs(adcDown-1020) > 10
    fprintf(2,"\nHanging value is %.0f counts from the firmware's 1020.\n",adcDown-1020);
    fprintf(2,"The rig needs mechanical calibration before it can BALANCE,\n");
    fprintf(2,"but today's experiments only make it fall, so carry on.\n");
end


%% ============ SECTION 2 - EXPERIMENT 1: FALL WITH ZERO INPUT ============
% Objective: watch what the pendulum does near upright when nothing is
% controlling it.

% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Write your answers down NOW, before you run this section:
%
%      a) Can the upright pendulum stabilise itself?
%      b) After a very small disturbance, what will the angle do?
%      c) Will the error stay small?
%
%    You will be asked to compare these predictions with the measured
%    result, and "I predicted what the plot showed" is not worth marks
%    if you wrote it after looking at the plot.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% >> STUDENT ACTION
%      1. Run this section (Ctrl+Enter).
%      2. During the 3-second countdown, hold the rod 5-10 deg from
%         vertical. Not 45 deg -- the analysis assumes you started near
%         upright.
%      3. When it prints RELEASE NOW, let go COMPLETELY. Do not touch the
%         rod or the cart again until the capture finishes.
% -------------------------------------------------------------------------

requireRef(cfg);
motorPWM(s,0);

disp(" ");
disp("=== EXPERIMENT 1: ZERO MOTOR INPUT ===");
disp("Hold the rod 5-10 deg from vertical. Release on the prompt.");
countdown(3);
disp(">>> RELEASE NOW <<<");

exp1_free = captureRun(s,cfg.FALL_TIME_S,cfg);
saveCapture(exp1_free,cfg,"exp1_free_response");
plotFall(exp1_free,"Experiment 1 - Free response, u = 0");

reportFall(exp1_free,cfg);


%% ========= SECTION 3 - EXPERIMENT 2: REPEATABILITY (3 TRIALS) ===========
% The same release, three times, from slightly different starting angles.

% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    Three releases, one after another. The script prompts you each time.
%
%    Use a DIFFERENT small angle for each trial -- say roughly 4, 8 and
%    12 degrees. If you release from the same angle three times you have
%    measured your own consistency, not the pendulum's.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    The handout asks three questions. Answer them from your own table:
%
%      a) Were all three responses identical?
%      b) Did a smaller disturbance always give the same fall time?
%      c) What physical effects could cause the differences?
%
%    Then look at the last column. Fall time and doubling time do NOT
%    behave the same way across your three trials, and explaining why is
%    the point of this experiment.
% -------------------------------------------------------------------------

requireRef(cfg);

trials = struct("angle0",{},"tFall",{},"lambda",{},"Td",{},"data",{});

for k = 1:3  %3
    motorPWM(s,0);
    fprintf("\n--- Trial %d of 3 ---\n",k);
    fprintf("Hold the rod at a DIFFERENT small angle than last time.\n");
    input("Press Enter when you are holding it steady... ","s");
    countdown(3);
    disp(">>> RELEASE NOW <<<");

    T = captureRun(s,cfg.FALL_TIME_S,cfg);
    r = reportFall(T,cfg);

    trials(k).angle0 = r.theta0_deg;
    trials(k).tFall  = r.tFall_s;
    trials(k).lambda = r.lambda;
    trials(k).Td     = r.Td_ms;
    trials(k).data   = T;

    saveCapture(T,cfg,sprintf("exp2_trial%d",k));
end

fprintf("\n=== EXPERIMENT 2 SUMMARY ===\n");
Tsum = table((1:3)',[trials.angle0]',[trials.tFall]',[trials.Td]', ...
    'VariableNames',{'Trial','InitialAngleDeg','TimeToFallSec','DoublingTimeMs'});
disp(Tsum);

fprintf("Initial angles differ by %.1f deg across trials.\n", ...
    max([trials.angle0])-min([trials.angle0]));
fprintf("Fall times differ by %.2f s.\n", ...
    max([trials.tFall])-min([trials.tFall]));
fprintf("\nBut look at the doubling times: %.0f, %.0f, %.0f ms.\n",trials.Td);
fprintf("The fall TIME depends on where you started. The GROWTH RATE does\n");
fprintf("not -- it is a property of the system, not of your release.\n");

figure("Name","Lab 3 - Repeatability","NumberTitle","off");
hold on;
for k = 1:3
    plot(trials(k).data.t_s, trials(k).data.theta_deg, "LineWidth",1.2);
end
grid on; xlabel("Time (s)"); ylabel("\theta (deg from upright)");
title("Three releases from slightly different angles");
legend(compose("Trial %d (\\theta_0 = %.1f deg)",(1:3)',[trials.angle0]'), ...
    "Location","southwest");


%% ====== SECTION 4 - EXPERIMENT 3: CONSTANT MOTOR INPUT ==================
% This is the only section that drives the motor.

% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% >> SAFETY - THE MOTOR RUNS IN THIS SECTION
%    Read this before you press Ctrl+Enter.
%
%      * HANDS AND SLEEVES CLEAR OF THE RAIL. The cart moves fast and it
%        does not stop for fingers.
%      * Keep the KEY5 stop button within reach. It is wired into the
%        firmware and always outranks anything MATLAB sends.
%      * Nothing loose on the rail: pens, phones, the handout.
%
%    There are three layers of protection, and you should know what each
%    one does and does not cover:
%
%      1. This script watches the cart and stops it at
%         cfg.CART_MIN / cfg.CART_MAX, well before the physical ends.
%      2. The firmware independently refuses to drive past 5900 / 9900,
%         and every motor command carries its own timeout, so the motor
%         stops even if MATLAB crashes mid-run.
%      3. KEY5, which is hardware.
%
%    None of them protect a hand that is already on the rail.
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Before you run it: will a constant motorPWM(s,0.3) hold the
%    pendulum up? Write down yes or no, and your reason.
%
%    The reason matters more than the answer here.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% >> STUDENT ACTION
%      1. Run the section. It will ask you to type y and Enter to confirm
%         you are ready for the motor -- that is deliberate friction, not
%         a formality.
%      2. Hold the rod near vertical during the countdown.
%      3. Release on the prompt and keep clear.
% -------------------------------------------------------------------------

requireRef(cfg);

st = readStatusWheeltec(s);
if st.Encoder < cfg.CART_MIN || st.Encoder > cfg.CART_MAX
    % Do not just refuse -- drifting off centre during Sections 1-3 is
    % normal, and Experiment 3 needs room on both sides.
    fprintf("Cart has drifted to %d, outside %d..%d.\n", ...
        st.Encoder,cfg.CART_MIN,cfg.CART_MAX);
    fprintf("Slide it back to the middle by hand, then re-home.\n");
    homeCart(s,"Force",true,"Target",cfg.CART_TARGET,"Tol",cfg.CART_TOL);
    st = readStatusWheeltec(s);
end
if st.VoltageVolts < 10.5
    error("Lab3:LowBattery","Battery %.2f V is too low to drive the motor.",st.VoltageVolts);
end

disp(" ");
disp("=== EXPERIMENT 3: CONSTANT MOTOR INPUT ===");
fprintf("u = %.2f  ->  PWM %d counts\n",cfg.U_CONSTANT,round(cfg.U_CONSTANT*cfg.PWM_FULL_SCALE));
disp("MOTOR LIVE - hands clear of the rail.");
if ~strcmpi(strtrim(string(input("Type y and Enter to continue: ","s"))),"y")
    disp("Cancelled."); return;
end

disp("Hold the rod near vertical. Release on the prompt.");
countdown(3);
disp(">>> RELEASE NOW <<<");

exp3_driven = captureRun(s,cfg.DRIVEN_TIME_S,cfg,cfg.U_CONSTANT);
motorPWM(s,0);
pendulumCommand(s,"STOP",0.5,3,"STOPPED");

saveCapture(exp3_driven,cfg,"exp3_constant_input");
plotFall(exp3_driven,sprintf("Experiment 3 - Constant input u = %.2f",cfg.U_CONSTANT));

r3 = reportFall(exp3_driven,cfg);

fprintf("\n=== THE IMPORTANT QUESTION ===\n");
fprintf("The motor was clearly doing something: the cart travelled %.0f counts.\n", ...
    max(exp3_driven.enc)-min(exp3_driven.enc));
fprintf("So why is this not control?\n\n");
fprintf("  Does the motor know the current pendulum angle?      no\n");
fprintf("  Does it know whether the rod is falling left or right? no\n");
fprintf("  Does the command change when the error changes?       no\n");
fprintf("  Does the input try to reduce the measured error?      no\n\n");
fprintf("Actuation is not control. A motor can push hard without knowing\n");
fprintf("whether it is helping.\n");

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    This is the central idea of the whole session, so put it in your own
%    words rather than copying the four lines above.
%
%    The motor was clearly doing work -- the cart travelled hundreds of
%    counts. The pendulum still fell. Explain what is missing, using the
%    word FEEDBACK, and say what the controller in Lab 4 will do that this
%    constant input cannot.
% -------------------------------------------------------------------------


%% ===== SECTION 5 - EXPERIMENT 4: ANNOTATE AND MEASURE THE POLE =========
% The handout asks you to mark five things on your figure: the release,
% the initial disturbance, where the angle stays small, where it starts
% growing rapidly, and where it is clearly not upright. This section
% annotates them for you, and then goes one step further -- it MEASURES
% the unstable pole from your own data.
%
% This section re-analyses the Experiment 1 capture. Nothing moves.

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Record these four numbers, and the annotated figure:
%
%      release angle theta0, growth rate lambda, doubling time, and R^2
%
%    Then compare your lambda with the model's 5.246 1/s. A difference of
%    a few per cent is a good result. If yours is off by tens of per cent,
%    say so and give a reason -- a bad angle reference in Section 1, a
%    release that was not from rest, or a hand still touching the rod are
%    all more likely than the model being wrong.
%
%    The doubling time is the number to remember: it is how long your
%    controller has before the error is twice as big.
% -------------------------------------------------------------------------

if ~exist("exp1_free","var") || isempty(exp1_free)
    error("Run SECTION 2 first.");
end

r = reportFall(exp1_free,cfg,true);

fprintf("\n=== MEASURED FROM YOUR OWN DATA ===\n");
fprintf("  release angle theta0   : %.2f deg\n",r.theta0_deg);
fprintf("  unstable pole lambda   : %.3f 1/s\n",r.lambda);
fprintf("  doubling time          : %.0f ms\n",r.Td_ms);
fprintf("  fit quality R^2        : %.4f  (%d points used)\n",r.R2,r.nPoints);
fprintf("\n  MODEL PREDICTION       : lambda = 5.246 1/s, doubling = 132 ms\n");
fprintf("  YOUR MEASUREMENT       : lambda = %.3f 1/s, doubling = %.0f ms\n",r.lambda,r.Td_ms);
fprintf("  difference             : %+.1f %%\n",100*(r.lambda/5.246 - 1));

fprintf("\nHow this was measured, so you can check it yourself:\n");
fprintf("  For a pendulum released from rest at theta0, energy conservation\n");
fprintf("  gives      thetadot^2 = 2*lambda^2*(cos(theta0) - cos(theta))\n");
fprintf("  Rearranged, thetadot^2 = A - B*cos(theta), which is a straight\n");
fprintf("  line in cos(theta). Fitting it gives B = 2*lambda^2 and\n");
fprintf("  theta0 = acos(A/B). This uses the WHOLE fall, including large\n");
fprintf("  angles where a small-angle exponential fit would be wrong.\n");


%% ===== SECTION 6 - CONNECT TO THE TUTORIAL =============================
% In Part A you characterised stable second-order responses with rise
% time, peak time, overshoot and settling time. Now try to apply the same
% specifications here, and watch most of them fall apart.

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    For each of the four specifications, say whether it can be measured
%    on your Experiment 1 data, and if not, WHY not. "Undefined" on its
%    own is not an answer -- the reason is the marks.
%
%      steady-state value      overshoot
%      2% settling time          rise time
%
%    Then state what does describe this response, with the number from
%    Section 5, and finish the sentence: a supervisory link running at
%    10 Hz cannot close this loop because ...
% -------------------------------------------------------------------------

if ~exist("exp1_free","var") || isempty(exp1_free)
    error("Run SECTION 2 first.");
end

th = exp1_free.theta_deg;

fprintf("\n=== APPLYING STABLE-SYSTEM SPECIFICATIONS TO AN UNSTABLE ONE ===\n\n");

fprintf("Steady-state value : ");
tailSpread = max(th(round(0.7*end):end)) - min(th(round(0.7*end):end));
if tailSpread > 20
    fprintf("none. The last 30%% of the record still spans %.0f deg.\n",tailSpread);
else
    fprintf("%.1f deg -- but that is the rod HANGING, not the upright\n",mean(th(round(0.7*end):end)));
    fprintf("                     equilibrium we were asking about.\n");
end

fprintf("Overshoot          : undefined. Overshoot is measured relative to a\n");
fprintf("                     final value, and there is no final value near\n");
fprintf("                     upright to overshoot.\n");
fprintf("2%% settling time    : undefined. Settling time asks when the response\n");
fprintf("                     ENTERS and STAYS inside a band around its final\n");
fprintf("                     value. This response leaves every band around\n");
fprintf("                     upright and never returns.\n");
fprintf("Rise time          : measurable as a number, but it describes the\n");
fprintf("                     rod falling away, not rising to a target.\n\n");

fprintf("What DOES describe this response: the growth rate.\n");
fprintf("  lambda = %.3f 1/s, doubling time %.0f ms.\n",r.lambda,r.Td_ms);
fprintf("  Every %.0f ms, whatever error you have becomes twice as large.\n",r.Td_ms);
fprintf("  That is why the controller has to run at 200 Hz, and why a\n");
fprintf("  supervisory link at %.0f Hz could never close this loop.\n", ...
    1/median(diff(exp1_free.t_s)));

figure("Name","Lab 3 - Why settling time fails","NumberTitle","off");
plot(exp1_free.t_s,th,"LineWidth",1.3); hold on;
yline(0,"k--","upright equilibrium");
yline(2,"r:"); yline(-2,"r:","+/- 2 deg band");
grid on; xlabel("Time (s)"); ylabel("\theta (deg from upright)");
title("The response leaves every band and never comes back");


%% ===================== SECTION 99 - CLOSE ==============================
% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    Run this before you leave, every time. It stops the motor and frees
%    the COM port. If you skip it, the port stays locked and the next
%    group gets a timeout they will spend twenty minutes debugging.
%
%    Your data is already saved in this week's data folder as .mat and
%    one pair per capture. Copy that folder before you leave the lab.
% -------------------------------------------------------------------------
try, pendulumCommand(s,"STOP",0.5,3,"STOPPED"); catch, end
try, clear s; catch, end
disp("Motor stopped, serial closed.");


%% ========================= LOCAL FUNCTIONS =============================
% Nothing below this line needs to be run or edited. It is here so you can
% see exactly how every number in this lab was computed -- particularly
% estimateUnstablePole, which is where the doubling time comes from.


function motorPWM(s,u,durationMs)
%MOTORPWM Normalised open-loop motor command, -1 .. +1.
%
%   motorPWM(s,0)     no drive
%   motorPWM(s,0.3)   30 % of full scale = 2070 counts
%
%   The handout writes motorPWM(0.3); the serial object is explicit here.
%   Full scale is 6900, which is the firmware's own clamp (Xianfu_Pwm),
%   not 7200 -- the extra 4 % is deliberate headroom in the stock code.
%
%   NOTE: u = 0 is not the same as "motor disconnected". With PWM low the
%   TB6612 short-brakes the motor, so the cart resists being pushed. That
%   is what makes the fixed-pivot pendulum model a fair approximation for
%   the fall you are about to measure.
    arguments
        s
        u (1,1) double {mustBeGreaterThanOrEqual(u,-1),mustBeLessThanOrEqual(u,1)}
        durationMs (1,1) double {mustBePositive} = 10000
    end
    setMotorPWM_wheeltec(s, round(u*6900), durationMs);
end


function a = meanAngle(s,seconds)
%MEANANGLE Average raw angle ADC over a short window.
    v = [];
    t0 = tic;
    while toc(t0) < seconds
        v(end+1) = readAngleWheeltec(s); %#ok<AGROW>
    end
    a = mean(v,"omitnan");
end


function requireRef(cfg)
    if ~isfield(cfg,"ADC_UP")
        error("Lab3:NoReference","Run SECTION 1 first to set the angle reference.");
    end
end


function countdown(n)
    for k = n:-1:1
        fprintf("%d...\n",k); pause(1);
    end
end


function T = captureRun(s,seconds,cfg,uConstant)
%CAPTURERUN Poll STATUS as fast as the link allows and build a table.
%
%   Polling beats the 14 Hz binary DataScope stream here: one STATUS reply
%   carries angle, cart position and motor command together, and the round
%   trip runs at roughly 25-40 Hz.
%
%   If uConstant is given, that drive is applied for the whole run and the
%   cart is watched against cfg.CART_MIN/MAX.

    if nargin < 4, uConstant = []; end

    if ~isempty(uConstant)
        setMotorPWM_wheeltec(s, round(uConstant*cfg.PWM_FULL_SCALE), ...
            min(10000, round(seconds*1000)+500));
    end

    n = 0; cap = 4000;
    t_s = zeros(cap,1); adc = zeros(cap,1); enc = zeros(cap,1); u = zeros(cap,1);

    t0 = tic;
    stoppedEarly = false;
    while toc(t0) < seconds
        st = readStatusWheeltec(s);
        n = n + 1;
        t_s(n) = toc(t0);
        adc(n) = st.AngleRaw;
        enc(n) = st.Encoder;
        u(n)   = st.Moto;

        if ~isempty(uConstant) && (st.Encoder < cfg.CART_MIN || st.Encoder > cfg.CART_MAX)
            fprintf(2,"\nCart reached %d counts - stopping before the end of the rail.\n",st.Encoder);
            pendulumCommand(s,"STOP",0.5,3,"STOPPED");
            stoppedEarly = true;
            break;
        end
        if n >= cap, break; end
    end

    t_s = t_s(1:n); adc = adc(1:n); enc = enc(1:n); u = u(1:n);

    theta_deg = (adc - cfg.ADC_UP) * cfg.DEG_PER_COUNT;

    T = table(t_s,adc,theta_deg,enc,u, ...
        'VariableNames',{'t_s','adc','theta_deg','enc','u_pwm'});

    fprintf("Captured %d samples in %.2f s (%.1f Hz)%s\n", ...
        n, t_s(end), n/t_s(end), ternary(stoppedEarly," - stopped early",""));
end


function out = ternary(c,a,b)
    if c, out = a; else, out = b; end
end


function plotFall(T,titleText)
    figure("Name",titleText,"NumberTitle","off");
    tiledlayout(3,1);

    nexttile;
    plot(T.t_s,T.theta_deg,"LineWidth",1.3); grid on;
    ylabel("\theta (deg)"); title(titleText);
    yline(0,"k--");

    nexttile;
    plot(T.t_s,T.enc,"LineWidth",1.2); grid on;
    ylabel("cart (counts)");

    nexttile;
    stairs(T.t_s,T.u_pwm,"LineWidth",1.2); grid on;
    ylabel("u (PWM)"); xlabel("Time (s)");
end


function r = reportFall(T,cfg,doAnnotate)
%REPORTFALL Measure the fall: growth rate, doubling time, release angle.

    if nargin < 3, doAnnotate = false; end

    r = struct("theta0_deg",NaN,"lambda",NaN,"Td_ms",NaN, ...
               "R2",NaN,"nPoints",0,"tFall_s",NaN);

    if isempty(T) || height(T) < 10
        fprintf(2,"Not enough samples to analyse this run.\n");
        return;
    end

    t  = T.t_s;
    th = T.theta_deg;

    % Time to leave a +/-15 deg band around upright: a simple, honest
    % "how long did it stay up" number that does not need any model.
    idx = find(abs(th) > 15, 1, "first");
    if ~isempty(idx), r.tFall_s = t(idx) - t(1); end

    e = estimateUnstablePole(t,th);
    r.theta0_deg = e.theta0_deg;
    r.lambda     = e.lambda;
    r.Td_ms      = 1000*e.Td;
    r.R2         = e.R2;
    r.nPoints    = e.nPoints;

    fprintf("  release angle   : %.2f deg\n",r.theta0_deg);
    fprintf("  left +/-15 deg  : %.2f s after capture start\n",r.tFall_s);
    fprintf("  growth rate     : %.3f 1/s  ->  doubling every %.0f ms  (R^2 %.3f)\n", ...
        r.lambda,r.Td_ms,r.R2);

    if doAnnotate
        figure("Name","Experiment 4 - Annotated open-loop response","NumberTitle","off");
        plot(t,th,"LineWidth",1.4); hold on; grid on;
        yline(0,"k--","upright");
        yline(15,"r:"); yline(-15,"r:");
        if ~isnan(r.tFall_s)
            xline(t(1)+r.tFall_s,"m-",sprintf("leaves 15 deg at %.2f s",r.tFall_s));
        end
        small = abs(th) <= 5;
        if any(small)
            area(t(small),th(small),"FaceAlpha",0.15,"EdgeColor","none");
        end
        xlabel("Time (s)"); ylabel("\theta (deg from upright)");
        title(sprintf("Open-loop fall: doubling time %.0f ms (model says 132 ms)",r.Td_ms));
        legend("\theta(t)","upright","","","","angle still small","Location","southwest");
    end
end


function e = estimateUnstablePole(t,theta_deg)
%ESTIMATEUNSTABLEPOLE Growth rate of the inverted pendulum, from one fall.
%
%   METHOD
%   For theta'' = lambda^2*sin(theta) released from rest at theta0, energy
%   conservation gives exactly
%
%       thetadot^2 = 2*lambda^2*(cos(theta0) - cos(theta))
%
%   which rearranges to a straight line in cos(theta):
%
%       thetadot^2 = A - B*cos(theta),   B = 2*lambda^2,  A = B*cos(theta0)
%
%   Fitting that line gives BOTH lambda and the release angle, and needs no
%   detection of the release instant.
%
%   WHY NOT FIT AN EXPONENTIAL
%   Released from rest the solution is theta0*cosh(lambda*t), which is flat
%   at first and only later looks exponential; above about 25 deg, sin(theta)
%   falls below theta and the growth slows again. Fitting log|theta| over
%   the visible part of the fall was tried against simulated data and came
%   out 37-78 % too slow. The energy fit stayed within 3 % over release
%   angles of 4-15 deg, sampling rates of 14-40 Hz and 0.25 deg of noise
%   (24 simulated cases, worst error 3.1 %).
%
%   lambda is the reliable output. The recovered theta0 is a by-product and
%   gets soft when the release angle is small and the sampling slow -- it
%   comes from an intercept, so it is far more sensitive than the slope.
%   Judge it with the R^2 that is reported next to it.

    e = struct("lambda",NaN,"Td",NaN,"theta0_deg",NaN,"R2",NaN,"nPoints",0);

    t  = t(:);
    th = deg2rad(theta_deg(:));
    if numel(t) < 8, return; end

    w = gradient(th,t);                    % angular velocity, rad/s

    % Use the part of the fall that is genuinely moving and still within
    % the range where the rod has not swung past the horizontal.
    m = abs(theta_deg(:)) <= 60 & abs(w) > 0.15;
    if nnz(m) < 6, return; end

    X = [ones(nnz(m),1), -cos(th(m))];
    y = w(m).^2;
    coef = X\y;
    A = coef(1); B = coef(2);

    if B <= 0, return; end

    e.lambda     = sqrt(B/2);
    e.Td         = log(2)/e.lambda;
    e.theta0_deg = rad2deg(acos(max(-1,min(1,A/B))));
    e.R2         = 1 - var(y - X*coef)/var(y);
    e.nPoints    = nnz(m);
end


function saveCapture(T,cfg,label)
    if isempty(T), return; end
    stamp = string(datetime("now","Format","yyyyMMdd_HHmmss"));
    base = cfg.GROUP_ID + "_" + label + "_" + stamp;
    save(fullfile(cfg.DATA_FOLDER,base+".mat"),"T");
    writetable(T,fullfile(cfg.DATA_FOLDER,base+".csv"));
    fprintf("Saved: %s.(mat|csv)\n",base);
end
