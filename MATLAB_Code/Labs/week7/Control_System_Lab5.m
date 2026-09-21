%% Control_System_Lab5.m
% CONTROL SYSTEMS 12061 / 12601 - Laboratory 5, PART B
% WHEELTEC linear inverted pendulum        firmware R4-6 or later
%
% SESSION 5 - PID DESIGN CHALLENGE
%
%   "You are given a working controller. Make it better, and prove it."
%
% Part A of the handout (PID structure, step-response diagnosis, Routh, choosing P/PI/PD/PID) is pen and paper. This script is Part B only.
%
% =========================================================================
% READ THIS FIRST
% =========================================================================
%   1. Run ONE SECTION AT A TIME with Ctrl+Enter. Never F5 / Run All.Several sections need you to be holding the rod when they start.
%
%   2. THIS LAB IS RIG ONLY. There is no simulation mode. Lab 4 had one because it was about understanding P, I and D. This session is an
%      engineering design exercise, and the things it asks you to trade off -- motor saturation, sensor noise, friction, 434 mm of rail --
%      are exactly the things a model leaves out.
%
%   3. HOW THIS SESSION DIFFERS FROM LAB 4
%      Lab 4: prescribed gains, one at a time, "what does each term do?"
%      Lab 5: a working baseline, YOUR diagnosis, YOUR changes, and an argument for why your final controller is better.
%      
%
%   4. THE DISTURBANCE IS THE INSTRUMENT.
%      Every comparison here - baseline against final, trial against
%      trial, and the three-run repeatability check - is only meaningful
%      because the disturbance is IDENTICAL every time. That is what the
%      KICK command is for. A hand push is not repeatable: the spread
%      between two pushes is wider than the spread between two
%      controllers, so a hand-pushed experiment measures the person doing
%      the pushing.
%
%   5. WHAT YOU may FIND, AND IT IS NOT A FAULT
%      The baseline controller keeps the rod up but never really settles: it rings continuously at roughly 3 Hz. So "recovery time" may come
%      back as DID NOT SETTLE. That is a real measurement, not a broken script, and turning it into an actual number is the most valuable thing you can do this session.
% =========================================================================


%% ===================== SECTION 0 - SETUP ===============================
clc;

cfg = struct;

% -------------------------------------------------------------------------
% >> EDIT THIS
%    cfg.PORT      your COM port. serialportlist("available") shows them.
%    cfg.GROUP_ID  your group number, or your files overwrite someone's.
% -------------------------------------------------------------------------
cfg.PORT     = "COM11";
cfg.GROUP_ID = "G01";

% -------------------------------------------------------------------------
cfg.STANDARD_KICK    = 1000;     % PWM
cfg.STANDARD_KICK_MS = 30;      % ms


cfg.TRIAL_SECONDS    = 8;        % recording window, from the kick
cfg.SETTLE_SECONDS   = 10;       % calm-down allowance before each kick
cfg.SETTLED_RMS_DEG  = 1.2;      % must be quieter than this to start
cfg.UPRIGHT_WINDOW   = 150;      % counts; tighter than the firmware's 500
cfg.RAIL_HALF_MM     = 217;      % half the usable rail
cfg.RECENTRE_COUNTS  = 500;      % re-home if the cart drifts beyond this

% Where does this script live? Run Section executes a TEMPORARY COPY of the
% text, so mfilename("fullpath") points into %TEMP% and cannot be used.
% setupPath puts every Labs/week* folder on the MATLAB path; the temp
% folder is never on it.
labFolder = '';
onPath = which('Control_System_Lab5.m');

if ~isempty(onPath) && ~startsWith(onPath,tempdir,'IgnoreCase',true)
    labFolder = fileparts(onPath);
end
if isempty(labFolder)
    here = fileparts(mfilename('fullpath'));
    if ~isempty(here) && ~startsWith(here,tempdir,'IgnoreCase',true)
        labFolder = here;
    end
end
if isempty(labFolder)
    labFolder = pwd;
    warning("Lab5:FolderGuess", ...
        ['Could not locate Control_System_Lab5.m on the MATLAB path, so ' ...
         'captures will go to\n  %s\nRun setupPath from MATLAB_Code if ' ...
         'that is not where you want them.'],labFolder);
end
cfg.DATA_FOLDER = fullfile(labFolder,"data");
if ~exist(cfg.DATA_FOLDER,"dir"), mkdir(cfg.DATA_FOLDER); end

cfg.results = struct([]);

fprintf("\n=== LAB 5 READY ===\n");
fprintf("  data folder   : %s\n",cfg.DATA_FOLDER);

if ~exist("s","var") || isempty(s) || ~isvalid(s)
    s = connectPendulum(cfg.PORT);
end

st = readStatusWheeltec(s);
if st.NumFields < 10
    error("Lab5:FirmwareTooOld", ...
        ['STATUS returned %d fields; this lab needs the 10 that R4-6 ' ...
         'sends.\nThe KICK disturbance command does not exist on older ' ...
         'firmware, and without it nothing in this session can be\n' ...
         'compared. Open source_code/ in VS Code and upload.'],st.NumFields);
end

% THE BASELINE IS WHATEVER THE FIRMWARE BOOTS WITH.
% Opening the serial port resets the board, so the gains always come back to the values compiled into Minibalance.c. Reading them here makes that
% file the single source of truth: the tutor changes the baseline by editing four numbers and reflashing, and this script needs no edit.

cfg.BASELINE = readGains(s);
printGains("BASELINE, as compiled into the firmware", cfg.BASELINE);

fprintf("  battery       : %.2f V\n", st.VoltageVolts);
if st.VoltageVolts < 7.4
    warning("Lab5:LowBattery", ...
        ['Battery %.2f V. Below about 7 V the firmware refuses to drive ' ...
         'the motor at all, and a sagging battery changes the results ' ...
         'between your first trial and your last.'], st.VoltageVolts);
end

homeCart(s);


%% ===================== SECTION 1 - THINK BEFORE YOU START ==============
% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Fill in the handout's five answers NOW, before touching the rig.
%    Edit the strings so they are saved with every capture you take.
% -------------------------------------------------------------------------
cfg.think = struct( ...
    "P_responds_to",        "...", ...
    "I_responds_to",        "...", ...
    "D_responds_to",        "...", ...
    "why_stable_is_not_good", "...", ...   % a controller can hold the rod
       "why_one_gain_at_a_time", "...");   % up and still be a poor design
 
disp("Predictions recorded. They are saved with every capture from here on.");


%% ===================== SECTION 2 - STEP 1: THE BASELINE ================
% Thirty seconds of the baseline simply holding the rod up, with no disturbance at all. This is the reference every later number is compared
% against, and it fills the handout's first observation table.
%
% -------------------------------------------------------------------------
% >> SAFETY !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%    !! The rig is LIVE from this section onward.                      !!
%    !! Hands and sleeves clear of the rail once you release the rod.  !!
%    !! KEY5 on the board is a hardware stop and always outranks       !!
%    !! MATLAB. Keep it within reach.                                  !!
%    !! If the rod falls the firmware cuts the motor by itself, but    !!
%    !! the cart will move sharply first.                              !!
% -------------------------------------------------------------------------
%
% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Before you run it: will the rod sit still, or keep moving? Will the
%    cart stay put? Write it down.
% -------------------------------------------------------------------------

applyGains(s,cfg.BASELINE);
armBalance(s,cfg);

fprintf("Letting it settle for %g s before the measurement starts...\n", ...
        cfg.SETTLE_SECONDS);
pause(cfg.SETTLE_SECONDS);

quiet = sampleRun(s,cfg,30,[]);
quiet.label = "Baseline, undisturbed";
quiet.gains = cfg.BASELINE;
% setGain(s,"BKP",60); setGain(s,"BKD",150);

quiet.think = cfg.think;
reportTrial(quiet);
cfg.results = [cfg.results, quiet];

plotTrial(quiet,cfg);

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT - the handout's baseline observation table
%       Does the pendulum remain upright?      -> outcome
%       Approximate peak angular deviation     -> angle peak
%       Does the angle oscillate noticeably?   -> angle RMS, and the plot
%       Does the cart drift from the centre?   -> cart drift and range
%       Does the motor action look aggressive? -> PWM RMS and % saturated
% -------------------------------------------------------------------------


%% ===================== SECTION 3 - STEP 2: THE DISTURBANCE =============
% The same controller, now given the standard disturbance. Everything from here on uses this identical kick.
%
% The rod is NOT re-caught between trials. Once it is balancing, the script waits until it is calm, clears the integrator, and kicks. That is
% deliberate: re-catching by hand would add a release transient far larger than the disturbance, and you would be measuring the release.

baseKick = disturbanceTrial(s,cfg,"Baseline + standard kick",cfg.BASELINE);
cfg.results = [cfg.results, baseKick];
plotTrial(baseKick,cfg);

fprintf("exit the inverted pendulum's balancing state")

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT - the handout's performance-indicator tablea 
% Note what the handout says: for an inverted pendulum, rise time and percentage overshoot are NOT the useful measures. Peak angle, recovery, residual offset and cart travel are.
% -------------------------------------------------------------------------


%% ===================== SECTION 4 - STEP 3: DIAGNOSE ====================
% Look at the two plots and the two printouts you already have. Decide what is actually wrong BEFORE you change anything.
%
% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    Edit the four logicals below to record your diagnosis, then edit
%    cfg.prediction to say what you expect the FIRST gain change to do.
% -------------------------------------------------------------------------
cfg.diagnosis = struct( ...
    "recovery_too_weak",      false, ...
    "persistent_offset",      false, ...
    "excessive_oscillation",  false, ...
    "cart_or_motor_too_hard", false);

cfg.prediction = "...";     % e.g. "raising Kd will reduce the ringing but raise the control effort"

fprintf("\n=== YOUR DIAGNOSIS ===\n");
d = fieldnames(cfg.diagnosis);
for k = 1:numel(d)
    fprintf("  %-24s : %s\n",d{k},string(cfg.diagnosis.(d{k})));
end
fprintf("  prediction: %s\n",cfg.prediction);
if ~any(struct2array(cfg.diagnosis))
    fprintf(2,"  Nothing selected. Look at the plots again -- an angle\n");
    fprintf(2,"  trace that never stops moving IS a problem worth naming.\n");
end


%% ===================== SECTION 5 - STEP 4: GAIN TRIALS =================
% Three changes, ONE GAIN AT A TIME, each relative to the baseline, each
% given exactly the same disturbance.
%
% These three are prescribed rather than free so that a two-hour session
% produces comparable data from every group. The engineering judgement the
% handout is after is in Section 4 (what is wrong) and Section 7 (which
% controller you choose and why), not in guessing which number to type.
%
% WHY THESE THREE
%   Kd = 800   twice the baseline damping. The closed-loop damping term
%              is (0.30 + 0.00585*Kd), so this is the one gain that can
%              change how quickly the ringing dies away.
%   Kd = 1200  three times. Does twice as much Kd keep helping? The
%              derivative term multiplies a raw tick-to-tick ADC
%              difference, so it amplifies sensor noise in direct
%              proportion -- there is a ceiling, and this probes it.
%   Kp = 600   a different axis: stiffer correction, damping unchanged.
%              Expect a faster, harder reaction. Whether that is an
%              improvement is the question.

trialSpecs = { ...
    "Kd x2   (Kd=800)",   struct("Balance_KD", 800), ; ...   %replace these values as needed
    "Kd x3   (Kd=1200)",  struct("Balance_KD",1200), ; ...   %replace these values as needed
    "Kp x1.5 (Kp=600)",   struct("Balance_KP", 600)  };      %replace these values as needed

trials = struct([]);
for k = 1:size(trialSpecs,1)
    g = applyDelta(cfg.BASELINE, trialSpecs{k,2});
    t = disturbanceTrial(s,cfg,trialSpecs{k,1},g);
    trials = [trials, t];                                   %#ok<AGROW>
    cfg.results = [cfg.results, t];
end

plotOverlay([baseKick trials],"Step 4 - one gain at a time",cfg);
trialTable([baseKick trials]);

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    The handout wants, for every trial: what you changed, what you
%    expected, and what actually happened. The table above gives you the
%    third column. The first two are yours.
% -------------------------------------------------------------------------


%% ===================== SECTION 6 - STEP 5: WHAT THE DATA SAYS ==========
% Answer the handout's questions from the numbers, not from memory.

fprintf("\n=== STEP 5 - READ THIS OFF THE TABLE ABOVE ===\n\n");
fprintf("  Kd changed  (800, 1200 vs 400):\n");
fprintf("    - did peak angle fall?\n");
fprintf("    - did the recovery time become a NUMBER rather than\n");
fprintf("      DID NOT SETTLE? That is the single clearest sign that\n");
fprintf("      damping was what the baseline was short of.\n");
fprintf("    - what happened to control effort and %% saturated?\n\n");
fprintf("  Kp changed  (600 vs 400):\n");
fprintf("    - a stiffer loop reacts harder. Did that help the peak,\n");
fprintf("      and what did it cost in effort and cart travel?\n\n");
fprintf("  Ki:\n");
fprintf("    The handout says do NOT force integral action into the\n");
fprintf("    angle loop of this rig, and the baseline ships with Ki = 0.\n");
fprintf("    Section 9 is an optional experiment that shows why.\n\n");
fprintf("  Which change gave the clearest IMPROVEMENT?\n");
fprintf("  Which gave the clearest SIDE EFFECT?\n");
fprintf("  They may well be the same change.\n");


%% ===================== SECTION 7 - STEP 6: YOUR FINAL CONTROLLER =======
% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    Choose your final gains and put them here. They do not have to be one
%    of the three you tried -- you may combine, or go part way.
% -------------------------------------------------------------------------
cfg.FINAL = applyDelta(cfg.BASELINE, struct( ...
    "Balance_KP",  400, ...
    "Balance_KD",  800, ...
    "Balance_KI",    0, ...
    "Position_KP",  20, ...
    "Position_KD", 300));

printGains("YOUR FINAL CONTROLLER", cfg.FINAL);

finalTrial = disturbanceTrial(s,cfg,"FINAL",cfg.FINAL);
cfg.results = [cfg.results, finalTrial];

compareTable(baseKick, finalTrial);

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Three to five sentences on why this controller is better than the
%    baseline. "Better" has six columns in the table above and they do not
%    all move the same way. Name the trade-off you accepted and say why it
%    was the right one for a rig with 434 mm of rail and a motor that
%    clamps at 6900.
% -------------------------------------------------------------------------


%% ===================== SECTION 8 - STEP 7: IS IT REPEATABLE? ===========
% Three identical runs with the final controller. The point is not to get
% a better number -- it is to find out whether the improvement you just
% claimed survives being done again.

robust = struct([]);
for k = 1:3
    t = disturbanceTrial(s,cfg,sprintf("FINAL, run %d",k),cfg.FINAL);
    robust = [robust, t];                                   %#ok<AGROW>
    cfg.results = [cfg.results, t];
end

robustnessTable(robust);

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Was the behaviour consistent? If not, what physical causes could
%    explain the spread: initial angle, friction varying along the rail,
%    sensor noise, battery sag over the session, where the cart started?
%    A controller whose three runs disagree more than the baseline-to-final
%    difference has not actually been shown to be better.
% -------------------------------------------------------------------------


%% ===================== SECTION 9 - OPTIONAL: THE OFFSET ================
% Only if you have time. This is the experiment behind the handout's
% instruction not to force integral action into the angle loop.
%
% The rig's angle sensor does not read exactly 2080 counts between hanging
% and upright, so the setpoint the firmware aims at is a degree or two away
% from true vertical. The loop holds the rod exactly where it was told to,
% and the cart then has to sit permanently off-centre to balance it. Both
% of those are scored in Step 6.
%
% A. Try to remove the offset with integral action on the angle loop.
% B. Remove it by correcting the setpoint instead.
% Compare what each costs.

offA = disturbanceTrial(s,cfg,"A: angle-loop Ki = 60", ...
        applyDelta(cfg.FINAL,struct("Balance_KI",60)));

% >> EDIT THIS. From baselineCheck.m Section 1: the ADC count at TRUE
%    vertical, which is the hanging reading plus 2080.
AZ_TRUE = 3076;

setGain(s,"AZ",AZ_TRUE);
offB = disturbanceTrial(s,cfg,sprintf("B: setpoint AZ = %d",AZ_TRUE),cfg.FINAL);
setGain(s,"AZ",cfg.BASELINE.Angle_Zero);        % put it back

cfg.results = [cfg.results, offA, offB];
compareTable(offA, offB);

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Which one actually removed the offset, and which one paid for it
%    somewhere else? An integrator cannot move a mechanical zero: it can
%    only lean the rod further from true vertical until the cart comes
%    back. Trading cart travel for angle error is not a fix.
% -------------------------------------------------------------------------


%% ===================== SECTION 10 - SAVE AND PLOT ======================
setBalance(s,false);

plotOverlay([baseKick finalTrial],"Step 8 - baseline vs final",cfg);

base = cfg.GROUP_ID + "_Lab5_" + ...
       string(datetime("now","Format","yyyyMMdd_HHmmss"));
results = cfg.results;                                       %#ok<NASGU>
save(fullfile(cfg.DATA_FOLDER,base+".mat"),"results","cfg");
writetable(summaryTable(cfg.results), ...
           fullfile(cfg.DATA_FOLDER,base+".csv"));

fprintf("\nSaved:\n  %s.mat\n  %s.csv\nin %s\n", base, base, cfg.DATA_FOLDER);

fprintf("\n=== WHAT YOU MUST PRODUCE ===\n");
fprintf("  [ ] Baseline and final angle traces on the same axes\n");
fprintf("  [ ] Completed controller trial table\n");
fprintf("  [ ] Completed baseline-versus-final performance table\n");
fprintf("  [ ] Your final Kp, Ki and Kd\n");
fprintf("  [ ] Three-run robustness check\n");
fprintf("  [ ] 3-5 sentences justifying the final controller\n");



















%% ========================== helpers ====================================

function printGains(label,g)
    fprintf("\n=== %s ===\n",label);
    fprintf("  angle loop : Kp=%g  Kd=%g  Ki=%g\n", ...
            g.Balance_KP,g.Balance_KD,g.Balance_KI);
    fprintf("  cart loop  : Kp=%g  Kd=%g  Ki=%g\n", ...
            g.Position_KP,g.Position_KD,g.Position_KI);
    fprintf("  setpoint   : %g counts\n",g.Angle_Zero);
end

function g = applyDelta(baseGains,delta)
%APPLYDELTA Copy the baseline and override only the named fields.
    g = baseGains;
    f = fieldnames(delta);
    for k = 1:numel(f), g.(f{k}) = delta.(f{k}); end
end

function applyGains(s,g)
    setGain(s,"BKP",g.Balance_KP);   setGain(s,"BKD",g.Balance_KD);
    setGain(s,"BKI",g.Balance_KI);   setGain(s,"PKP",g.Position_KP);
    setGain(s,"PKD",g.Position_KD);
    if ~isnan(g.Position_KI), setGain(s,"PKI",g.Position_KI); end
    resetIntegrator(s);
end

function armBalance(s,cfg)
%ARMBALANCE Get the rod up, hand the motor over, and CHECK it took.
%
%   Turn_Off() latches Flag_Stop the instant the angle leaves the
%   setpoint +/- 500 counts, and nothing in the firmware clears it again:
%   only a fresh BAL,1 does. So enabling the controller while the rod is
%   still on its way up kills it silently -- MATLAB gets BAL=1 back,
%   nothing looks wrong, and the motor stays dead for the whole run.
%
%   Hence two things here: wait for the rod first, and then READ THE FLAG
%   BACK. A failure is reported at the moment it happens rather than four
%   seconds later as a confusing error somewhere else.
    az = cfg.BASELINE.Angle_Zero;

    % Re-centre BEFORE asking for the rod. Sliding the cart takes both
    % hands, so anything done after the rod is up will put it down again.
    st = readStatusWheeltec(s);
    if abs(st.Encoder - 7925) > cfg.RECENTRE_COUNTS
        fprintf("Cart at %d counts. Re-centring before this run.\n",st.Encoder);
        setBalance(s,false);
        homeCart(s,"Force",true);
    end

    fprintf("Lift the rod to VERTICAL and hold it steady.\n");
    tw = tic; last = -1;
    while true
        st = readStatusWheeltec(s);
        if abs(st.AngleRaw - az) <= cfg.UPRIGHT_WINDOW, break; end
        if toc(tw) > 60
            error("Lab5:RodNotUpright", ...
                ['Gave up after 60 s. The angle reads %d and this script ' ...
                 'wants it within %d counts of %g.\nIf the rod IS upright ' ...
                 'and the reading is wrong, the sensor needs calibrating ' ...
                 '-- that is mechanical, not a gain.'], ...
                 st.AngleRaw,cfg.UPRIGHT_WINDOW,az);
        end
        if floor(toc(tw)) ~= last
            last = floor(toc(tw));
            fprintf("  waiting...  angle = %4d  (want %g +/- %d)\n", ...
                    st.AngleRaw, az, cfg.UPRIGHT_WINDOW);
        end
        pause(0.2);
    end
    fprintf("Rod is up (angle = %d).\n",st.AngleRaw);

    setBalance(s,true);
    fprintf("Controller is LIVE - you will feel it push against your hand.\n");
    pause(1.5);
    disp(">>> RELEASE NOW <<<");

    pause(0.5);
    st = readStatusWheeltec(s);
    if st.FlagStop
        setBalance(s,false);
        error("Lab5:BalanceDidNotArm", ...
            ['The firmware stopped the motor immediately after BAL,1 ' ...
             '(angle %d, setpoint %g).\nThat means the rod left the ' ...
             '+/-500 count window during the hand-over. Try again and ' ...
             'hold it steadier.'],st.AngleRaw,cfg.BASELINE.Angle_Zero);
    end
end

function ok = waitSettled(s,cfg)
%WAITSETTLED Hold until the rod is calm enough for a comparable trial.
%   Returns false, with a warning, rather than erroring: a rig that never
%   fully settles is a finding, and the trial is still worth taking as
%   long as the printout says the run started agitated.
    az = cfg.BASELINE.Angle_Zero;
    tw = tic; ok = false;
    while toc(tw) < cfg.SETTLE_SECONDS
        win = zeros(30,1);
        for k = 1:30
            win(k) = (readStatusWheeltec(s).AngleRaw - az)*(180/2080);
        end
        if sqrt(mean(win.^2)) < cfg.SETTLED_RMS_DEG, ok = true; break; end
    end
    if ~ok
        warning("Lab5:NeverSettled", ...
            ['The rod was still moving %.2f deg RMS after %g s, so this ' ...
             'trial starts agitated.\nThat is itself worth reporting: a ' ...
             'controller that never settles has no recovery time to ' ...
             'measure.'], sqrt(mean(win.^2)), cfg.SETTLE_SECONDS);
    end
end

function r = disturbanceTrial(s,cfg,label,gains)
%DISTURBANCETRIAL One controller, one standard kick, one recording.
    fprintf("\n--- %s ---\n",label);
    applyGains(s,gains);

    st = readStatusWheeltec(s);
    if st.FlagStop
        armBalance(s,cfg);          % not running yet, or it fell
    else
        fprintf("Already balancing; not re-catching the rod.\n");
    end

    r = readStatusWheeltec(s);
    if abs(r.Encoder - 7925) > cfg.RECENTRE_COUNTS
        setBalance(s,false);
        armBalance(s,cfg);          % drifted far enough to need re-centring
    end

    settled = waitSettled(s,cfg);
    resetIntegrator(s);

    r = sampleRun(s,cfg,cfg.TRIAL_SECONDS, ...
                  @() disturb(s,cfg.STANDARD_KICK,cfg.STANDARD_KICK_MS));
    r.label   = string(label);
    r.gains   = gains;
    r.think   = cfg.think;
    r.settled = settled;
    r.group   = cfg.GROUP_ID;
    r.stamp   = string(datetime("now","Format","yyyyMMdd_HHmmss"));
    reportTrial(r);
end

function r = sampleRun(s,cfg,seconds,kickFcn)
%SAMPLERUN Poll STATUS, optionally firing the disturbance on sample 1.
%   The kick instant comes from STATUS field 10 (KickTicks going non-zero),
%   not from a host-side timestamp, so recovery time does not carry a
%   serial round trip inside it.
    az = cfg.BASELINE.Angle_Zero;
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
        if ~fired, kickFcn(); fired = true; end
        if n >= cap, break; end
    end

    i = 1:n;
    r = struct("t",t(i),"theta_deg",a(i),"u_pwm",u(i),"cart_mm",x(i), ...
               "kickTicks",kt(i),"flagStop",fs(i),"volts",volts, ...
               "rateHz",n/t(n),"label","","gains",struct(),"think",struct(), ...
               "settled",true,"group","","stamp","");

    j = find(r.kickTicks > 0, 1, "first");
    if isempty(j), r.tKick = 0; else, r.tKick = r.t(j); end
    r = addMetrics(r);
end

function r = addMetrics(r)
%ADDMETRICS The six numbers the handout's tables ask for.
%   Everything is measured RELATIVE TO THE PRE-KICK MEAN, not to zero. The
%   rod usually sits at a small steady offset before the kick, and calling
%   that offset part of the disturbance response would flatter every
%   controller equally and tell you nothing.
    th = r.theta_deg;
    pre = r.t < r.tKick;
    if nnz(pre) >= 5, r.preMeanDeg = mean(th(pre)); else, r.preMeanDeg = 0; end
    dev = th - r.preMeanDeg;

    post = r.t >= r.tKick;
    r.anglePeak  = max(abs(dev(post)));
    r.angleRms   = sqrt(mean(dev(post).^2));
    r.effortRms  = sqrt(mean(r.u_pwm.^2));
    r.satPct     = 100*mean(abs(r.u_pwm) >= 6890);
    r.driftMm    = r.cart_mm(end) - r.cart_mm(1);
    r.cartMaxMm  = max(abs(r.cart_mm));
    r.cartRange  = max(r.cart_mm) - min(r.cart_mm);

    % Residual: where the rod ends up, in absolute terms. This is the
    % handout's "residual angle offset", and no gain can remove the part
    % of it that comes from a mis-set sensor zero.
    tail = r.t > r.t(end) - 2;
    r.residualDeg = mean(th(tail));

    % Recovery: the last moment the deviation leaves a band of 20 % of the
    % peak, floored at 0.4 deg so sensor noise cannot define the band.
    band = max(0.4, 0.2*r.anglePeak);
    out  = find(abs(dev) > band & post, 1, "last");
    if isempty(out) || out >= numel(th) - 2
        r.recoverS  = NaN;
        r.settledOk = false;      % still outside the band when time ran out
    else
        r.recoverS  = r.t(out) - r.tKick;
        r.settledOk = true;
    end

    if any(r.flagStop)
        r.outcome = "ROD FELL - firmware cut the motor";
    elseif r.cartMaxMm > 200
        r.outcome = "rod up, CART REACHED THE RAIL END";
    elseif ~r.settledOk
        r.outcome = "upright, but DID NOT SETTLE in the window";
    else
        r.outcome = "recovered";
    end
end

function reportTrial(r)
    fprintf("  outcome        : %s\n", r.outcome);
    fprintf("  peak |angle|   : %.2f deg   (from the pre-kick mean)\n", r.anglePeak);
    if isnan(r.recoverS)
        fprintf("  recovery       : DID NOT SETTLE within %.0f s\n", r.t(end)-r.tKick);
    else
        fprintf("  recovery       : %.2f s\n", r.recoverS);
    end
    fprintf("  post-kick RMS  : %.3f deg\n", r.angleRms);
    fprintf("  residual angle : %+.2f deg  (last 2 s, absolute)\n", r.residualDeg);
    fprintf("  control effort : %.0f PWM rms,  %.1f%% saturated\n", ...
            r.effortRms, r.satPct);
    fprintf("  cart           : max |x| %.0f mm,  range %.0f mm\n", ...
            r.cartMaxMm, r.cartRange);
    fprintf("  sample rate    : %.1f Hz\n", r.rateHz);
end

function trialTable(trials)
    fprintf("\n%-22s %5s %5s %5s | %6s %7s %7s %7s %6s\n", ...
        "trial","Kp","Kd","Ki","peak","recov","RMS","effort","maxX");
    fprintf("%s\n",repmat('-',1,82));
    for k = 1:numel(trials)
        r = trials(k);
        if isnan(r.recoverS), rec = "  n/a"; else, rec = sprintf("%5.2f",r.recoverS); end
        fprintf("%-22s %5g %5g %5g | %6.2f %7s %7.3f %7.0f %6.0f\n", ...
            r.label, r.gains.Balance_KP, r.gains.Balance_KD, ...
            r.gains.Balance_KI, r.anglePeak, rec, r.angleRms, ...
            r.effortRms, r.cartMaxMm);
    end
    fprintf("%s\n",repmat('-',1,82));
    fprintf("recov = n/a means the rod never returned inside the band --\n");
    fprintf("a real result, and usually the one worth fixing first.\n");
end

function compareTable(a,b)
    fprintf("\n%-22s %12s %12s   %s\n","criterion",a.label,b.label,"better?");
    fprintf("%s\n",repmat('-',1,66));
    row("peak |angle| (deg)", a.anglePeak,   b.anglePeak,   true);
    row("recovery (s)",       a.recoverS,    b.recoverS,    true);
    row("post-kick RMS (deg)",a.angleRms,    b.angleRms,    true);
    row("residual (deg)",     abs(a.residualDeg), abs(b.residualDeg), true);
    row("control effort",     a.effortRms,   b.effortRms,   true);
    row("max cart (mm)",      a.cartMaxMm,   b.cartMaxMm,   true);
    fprintf("%s\n",repmat('-',1,66));
    fprintf("They will not all improve together. Naming the trade-off you\n");
    fprintf("accepted is the point of the exercise.\n");

    function row(name,va,vb,lowerIsBetter)
        if isnan(va) || isnan(vb)
            verdict = "n/a";
        elseif (vb < va) == lowerIsBetter
            verdict = "yes";
        else
            verdict = "no";
        end
        fprintf("%-22s %12.3f %12.3f   %s\n",name,va,vb,verdict);
    end
end

function robustnessTable(rs)
    p = [rs.anglePeak];  c = [rs.cartMaxMm];  e = [rs.effortRms];
    fprintf("\n%-12s %8s %8s %8s  %s\n","run","peak","maxX","effort","outcome");
    fprintf("%s\n",repmat('-',1,64));
    for k = 1:numel(rs)
        fprintf("%-12d %8.2f %8.0f %8.0f  %s\n",k,p(k),c(k),e(k),rs(k).outcome);
    end
    fprintf("%s\n",repmat('-',1,64));
    fprintf("spread       %8.2f %8.0f %8.0f\n",max(p)-min(p),max(c)-min(c), ...
            max(e)-min(e));
    fprintf("\nIf that spread is comparable to the baseline-to-final\n");
    fprintf("difference, you have not yet shown your controller is better.\n");
end

function plotTrial(r,cfg)
    figure("Name",r.label,"NumberTitle","off");
    tiledlayout(3,1);
    nexttile; plot(r.t,r.theta_deg,"LineWidth",1.3); grid on;
    ylabel("\theta (deg)"); title(r.label); yline(0,"k--");
    if r.tKick > 0, xline(r.tKick,"r:","kick"); end
    nexttile; stairs(r.t,r.u_pwm,"LineWidth",1.1); grid on;
    ylabel("u (PWM)"); yline([-6900 6900],"r--");
    nexttile; plot(r.t,r.cart_mm,"LineWidth",1.1); grid on;
    ylabel("cart (mm)"); xlabel("Time (s)");
    yline([-cfg.RAIL_HALF_MM cfg.RAIL_HALF_MM],"r--","rail");
end

function plotOverlay(trials,titleText,cfg)                  %#ok<INUSD>
    figure("Name",titleText,"NumberTitle","off"); hold on;
    for k = 1:numel(trials)
        plot(trials(k).t - trials(k).tKick, trials(k).theta_deg, ...
             "LineWidth",1.3);
    end
    grid on; yline(0,"k--"); xline(0,"k:");
    xlabel("Time from the kick (s)"); ylabel("\theta (deg)");
    title(titleText);
    legend([trials.label],"Location","best","Interpreter","none");
end

function T = summaryTable(rs)
    T = table(string({rs.label}'), [rs.anglePeak]', [rs.recoverS]', ...
              [rs.angleRms]', [rs.residualDeg]', [rs.effortRms]', ...
              [rs.satPct]', [rs.cartMaxMm]', string({rs.outcome}'), ...
        'VariableNames', {'trial','peak_deg','recovery_s','rms_deg', ...
                          'residual_deg','effort_rms','sat_pct', ...
                          'max_cart_mm','outcome'});
end
