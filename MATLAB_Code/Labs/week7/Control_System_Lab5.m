%% Control_System_Lab5.m

%Please make sure you refer to the Tutorial Sheet on Canva and change the
%code as needed to finalise your experiment!!!!!!!


% CONTROL SYSTEMS 12061 / 12601 - Lab 5 Part B: PID design challenge
% WHEELTEC linear inverted pendulum, firmware R4-6 or later.
%
% "You are given a working controller. Make it better, and prove it."
% Part A of the handout is pen and paper; this script is Part B.
%
% READ FIRST
%   1. Run ONE SECTION AT A TIME (Ctrl+Enter), never Run All. Several
%      sections need you to be holding the rod when they start.
%   2. Every comparison uses the same software KICK, because hand pushes
%      are not repeatable. It is added to the controller output for a fixed
%      time, and the script measures how the controller recovers.
%   3. Expect a ~1.3 deg angle offset, a slow sway (~1.5 s period) and
%      frequent PWM saturation. These are findings to diagnose, not faults.


%% ===================== SECTION 0 - SETUP ===============================
clc;
cfg = struct;

% >> EDIT THIS: your COM port and group number.
cfg.PORT     = "COM11";
cfg.GROUP_ID = "G01";

% The standard disturbance, identical for every trial. Firmware limits are
% 6900 PWM and 1000 ms; bigger kicks push the cart further, so start centred.
cfg.STANDARD_KICK    = 6000;    % PWM
cfg.STANDARD_KICK_MS = 1000;    % ms

cfg.PREROLL_SECONDS  = 1.5;     % recorded before the kick (the reference)
cfg.TRIAL_SECONDS    = 8;       % recorded after the kick
cfg.SETTLE_SECONDS   = 10;      % longest wait for the rod to calm down
cfg.SETTLED_RMS_DEG  = 1.0;     % "calm" means sway below this
cfg.RECOVERY_BAND_DEG = 1.5;    % "recovered" band; Section 2 replaces it with 3x the measured sway
cfg.UPRIGHT_WINDOW   = 150;     % counts from the setpoint that count as upright
cfg.RAIL_HALF_MM     = 223;     % measured half travel of the cart
cfg.RECENTRE_COUNTS  = 500;     % re-centre the cart if it drifts further

KICK_PWM_MAX = 6900;  KICK_MAX_MS = 1000;     % must match usart_cmd.h
if abs(cfg.STANDARD_KICK) > KICK_PWM_MAX || cfg.STANDARD_KICK_MS > KICK_MAX_MS
    warning("Lab5:KickCapped","Kick capped to the firmware limits: %d PWM, %d ms.",KICK_PWM_MAX,KICK_MAX_MS);
    cfg.STANDARD_KICK    = sign(cfg.STANDARD_KICK)*min(abs(cfg.STANDARD_KICK),KICK_PWM_MAX);
    cfg.STANDARD_KICK_MS = min(cfg.STANDARD_KICK_MS,KICK_MAX_MS);
end

% Captures go to data/ next to this script. Run Section executes a temporary
% copy, so the script is found with which(), not mfilename.
here = fileparts(which("Control_System_Lab5.m"));
if isempty(here) || startsWith(here,tempdir,"IgnoreCase",true)
    here = pwd;
    warning("Lab5:FolderGuess","Script not on the path; saving to %s. Run setupPath first.",here);
end
cfg.DATA_FOLDER = fullfile(here,"data");
if ~exist(cfg.DATA_FOLDER,"dir"), mkdir(cfg.DATA_FOLDER); end
cfg.results = struct([]);

if ~exist("s","var") || isempty(s) || ~isvalid(s)
    s = connectPendulum(cfg.PORT);
end
st = readStatusWheeltec(s);
if st.NumFields < 10
    error("Lab5:FirmwareTooOld","This lab needs firmware R4-6 (the KICK command). Upload source_code/ first.");
end

% The baseline is whatever the firmware boots with (set in Minibalance.c).
cfg.BASELINE = readGains(s);
printGains("BASELINE (from firmware)",cfg.BASELINE);
fprintf("  battery    : %.2f V\n  data       : %s\n",st.VoltageVolts,cfg.DATA_FOLDER);
if st.VoltageVolts < 7.4
    warning("Lab5:LowBattery","Battery %.2f V is low; results will drift during the session.",st.VoltageVolts);
end
% homeCart(s);
autoHome(s);




%% ===================== SECTION 1 - PREDICT FIRST =======================
% >> Answer the handout's five questions here before touching the rig.
%    They are saved with every capture.
cfg.think = struct( ...
    "P_responds_to",          "...", ...
    "I_responds_to",          "...", ...
    "D_responds_to",          "...", ...
    "why_stable_is_not_good", "...", ...
    "why_one_gain_at_a_time", "...");
disp("Predictions recorded.");


%% ===================== SECTION 2 - STEP 1: BASELINE, NO KICK ===========
% SAFETY: the rig is live from here on, so keep hands clear once you let
% go. KEY5 on the board always stops the motor.
% Ten seconds of the baseline holding the rod up: the reference for everything later.
applyGains(s,cfg.BASELINE);
armBalance(s,cfg);
fprintf("Settling for %g s...\n",cfg.SETTLE_SECONDS);
pause(cfg.SETTLE_SECONDS);

quiet = sampleRun(s,cfg,3,false);
quiet.label = "Baseline, no kick";  quiet.gains = cfg.BASELINE;  quiet.think = cfg.think;
reportTrial(quiet);
plotTrial(quiet,cfg);
cfg.results = [cfg.results, quiet];

% One recovery band for every later trial, so their recovery times compare fairly.
cfg.RECOVERY_BAND_DEG = max(0.4, 3*quiet.preSwayDeg);
fprintf("Recovery band for all trials: +/-%.2f deg\n",cfg.RECOVERY_BAND_DEG);
% REPORT: the handout's baseline table comes straight from this printout and plot.


%% ===================== SECTION 3 - STEP 2: BASELINE + KICK =============
% The same controller, now kicked. Each recording starts 1.5 s before the
% kick, and every number is measured from that pre-kick mean.

baseKick = disturbanceTrial(s,cfg,"Baseline + kick",cfg.BASELINE);
plotTrial(baseKick,cfg);
cfg.results = [cfg.results, baseKick];
% REPORT: peak angle, recovery, residual offset and cart travel matter here,
% not rise time or overshoot.


%% ===================== SECTION 4 - STEP 3: DIAGNOSE ====================
% >> Tick what you saw, and predict what your first change will do.
cfg.diagnosis = struct( ...
    "recovery_too_weak",      false, ...
    "persistent_offset",      false, ...
    "excessive_oscillation",  false, ...
    "cart_or_motor_too_hard", false);
cfg.prediction = "...";    % e.g. "raising Kd will reduce the sway but raise the effort"

disp(cfg.diagnosis);
fprintf("prediction: %s\n",cfg.prediction);
if ~any(cell2mat(struct2cell(cfg.diagnosis)))
    warning("Lab5:NoDiagnosis","Nothing ticked. A trace that never stops moving is worth naming.");
end


%% ===================== SECTION 5 - STEP 4: ONE GAIN AT A TIME ==========
% Three changes from the baseline, each given the same kick. Kd adds
% damping but also amplifies sensor noise; Kp stiffens the reaction.
trialSpecs = { ...
    "Kd=800",  struct("Balance_KD", 800); ...   % replace these values as needed
    "Kd=1200", struct("Balance_KD",1200); ...
    "Kp=600",  struct("Balance_KP", 600)};

trials = struct([]);
for k = 1:size(trialSpecs,1)
    g = applyDelta(cfg.BASELINE,trialSpecs{k,2});
    trials = [trials, disturbanceTrial(s,cfg,trialSpecs{k,1},g)];   %#ok<AGROW>
end
cfg.results = [cfg.results, trials];

plotOverlay([baseKick trials],"Step 4 - one gain at a time");
trialTable([baseKick trials]);
% REPORT: for each trial write what you changed, what you expected, and
% what happened. The table gives you the last column.


%% ===================== SECTION 6 - STEP 5: WHAT THE DATA SAYS ==========
% Which change gave the clearest improvement, and which the clearest side
% effect (effort, saturation, cart travel)? A difference smaller than the
% pre-kick sway is noise, not a result.
% Ki stays 0 in the angle loop, as the handout says; Section 9 shows why.


%% ===================== SECTION 7 - STEP 6: YOUR FINAL CONTROLLER =======
% >> Replace these with YOUR gains, chosen from the Section 5 table. They
%    start equal to the baseline and need not be one of the three trials.


cfg.FINAL = applyDelta(cfg.BASELINE, struct( ...
    "Balance_KP",  cfg.BASELINE.Balance_KP, ...
    "Balance_KD",  cfg.BASELINE.Balance_KD, ...
    "Balance_KI",  0, ...
    "Position_KP", cfg.BASELINE.Position_KP, ...
    "Position_KD", cfg.BASELINE.Position_KD));
printGains("YOUR FINAL CONTROLLER",cfg.FINAL);

f = ["Balance_KP","Balance_KD","Balance_KI","Position_KP","Position_KD"];
if all(arrayfun(@(k) cfg.FINAL.(k) == cfg.BASELINE.(k), f))
    warning("Lab5:FinalIsBaseline","FINAL still equals the baseline. Enter your own gains first.");
end

finalTrial = disturbanceTrial(s,cfg,"FINAL",cfg.FINAL);
cfg.results = [cfg.results, finalTrial];
compareTable(baseKick,finalTrial);
% REPORT: 3-5 sentences on why this controller is better. Name the
% trade-off you accepted.


%% ===================== SECTION 8 - STEP 7: IS IT REPEATABLE? ===========
% Three identical runs of the final controller. If their spread is as big
% as the baseline-to-final difference, the improvement is not proven.
robust = struct([]);
for k = 1:3
    robust = [robust, disturbanceTrial(s,cfg,sprintf("FINAL run %d",k),cfg.FINAL)];   %#ok<AGROW>
end
cfg.results = [cfg.results, robust];
trialTable(robust,true);


%% ===================== SECTION 9 - OPTIONAL: THE ANGLE OFFSET ==========
% The sensor zero is a degree or two off true vertical, so the rod leans
% and the cart sits off-centre. Compare fixing it with angle-loop Ki (A) against correcting the setpoint AZ (B).
offA = disturbanceTrial(s,cfg,"A: Ki=60",applyDelta(cfg.FINAL,struct("Balance_KI",60)));

AZ_TRUE = 3076;    % >> EDIT: ADC count at true vertical (hanging reading + 2080)
setGain(s,"AZ",AZ_TRUE);
offB = disturbanceTrial(s,cfg,sprintf("B: AZ=%d",AZ_TRUE),cfg.FINAL);
setGain(s,"AZ",cfg.BASELINE.Angle_Zero);    % put it back

cfg.results = [cfg.results, offA, offB];
compareTable(offA,offB);
% REPORT: which one removed the offset, and which one paid for it elsewhere?


%% ===================== SECTION 10 - SAVE ===============================
setBalance(s,false);
plotOverlay([baseKick finalTrial],"Baseline vs final");

base = cfg.GROUP_ID + "_Lab5_" + string(datetime("now","Format","yyyyMMdd_HHmmss"));
results = cfg.results;
save(fullfile(cfg.DATA_FOLDER,base+".mat"),"results","cfg");
writetable(summaryTable(results),fullfile(cfg.DATA_FOLDER,base+".csv"));
fprintf("Saved %s.mat and %s.csv in %s\n",base,base,cfg.DATA_FOLDER);


% HAND IN: baseline and final traces together, the trial table, the
% baseline-vs-final table, your final gains, the 3-run check and your justification.








%% ========================== helpers ====================================

function printGains(label,g)
    fprintf("\n=== %s ===\n",label);
    fprintf("  angle loop : Kp=%g  Kd=%g  Ki=%g\n",g.Balance_KP,g.Balance_KD,g.Balance_KI);
    fprintf("  cart loop  : Kp=%g  Kd=%g  Ki=%g\n",g.Position_KP,g.Position_KD,g.Position_KI);
    fprintf("  setpoint   : %g counts\n",g.Angle_Zero);
end

function g = applyDelta(g,delta)
% Copy the gains and override only the named fields.
    for f = string(fieldnames(delta))', g.(f) = delta.(f); end
end

function applyGains(s,g)
    setGain(s,"BKP",g.Balance_KP);   setGain(s,"BKD",g.Balance_KD);
    setGain(s,"BKI",g.Balance_KI);   setGain(s,"PKP",g.Position_KP);
    setGain(s,"PKD",g.Position_KD);
    if ~isnan(g.Position_KI), setGain(s,"PKI",g.Position_KI); end
    resetIntegrator(s);
end

function armBalance(s,cfg)
% Enable balance only once the rod is upright, then check it took. Enabled
% too early, the firmware stops the motor and stays stopped.
    az = cfg.BASELINE.Angle_Zero;
    st = readStatusWheeltec(s);
    if abs(st.Encoder - 7925) > cfg.RECENTRE_COUNTS    % re-centre first: it needs both hands
        fprintf("Cart at %d counts; re-centring.\n",st.Encoder);
        setBalance(s,false);
        homeCart(s,"Force",true);
    end

    fprintf("Lift the rod to VERTICAL and hold it steady.\n");
    tw = tic; k = 0;
    while abs(st.AngleRaw - az) > cfg.UPRIGHT_WINDOW
        if toc(tw) > 60
            error("Lab5:RodNotUpright", ...
                "Angle reads %d, want %g +/- %d. If the rod IS upright, the sensor needs calibrating.", ...
                st.AngleRaw,az,cfg.UPRIGHT_WINDOW);
        end
        k = k + 1;
        if mod(k,5) == 0, fprintf("  angle %d (want %g +/- %d)\n",st.AngleRaw,az,cfg.UPRIGHT_WINDOW); end
        pause(0.2);
        st = readStatusWheeltec(s);
    end

    setBalance(s,true);
    fprintf("Controller LIVE (angle %d).\n",st.AngleRaw);
    pause(1.5);  disp(">>> RELEASE NOW <<<");  pause(0.5);
    if readStatusWheeltec(s).FlagStop
        setBalance(s,false);
        error("Lab5:BalanceDidNotArm","The firmware stopped the motor during the hand-over. Hold the rod steadier and retry.");
    end
end

function ok = waitSettled(s,cfg)
% Wait until the rod's sway about its own mean is below SETTLED_RMS_DEG.
% A rig that never settles is a finding, so this only warns.
    tw = tic; ok = false; sway = NaN;
    while ~ok && toc(tw) < cfg.SETTLE_SECONDS
        w = zeros(30,1);
        for k = 1:30, w(k) = readStatusWheeltec(s).AngleRaw; end
        sway = std(movmedian(w,3)*(180/2080),1);
        ok = sway < cfg.SETTLED_RMS_DEG;
    end
    if ~ok
        warning("Lab5:NeverSettled","Still swaying %.2f deg RMS after %g s; this trial starts agitated.", ...
            sway,cfg.SETTLE_SECONDS);
    end
end

function r = disturbanceTrial(s,cfg,label,gains)
% One controller, one standard kick, one recording.
    fprintf("\n--- %s ---\n",label);
    applyGains(s,gains);
    st = readStatusWheeltec(s);
    if st.FlagStop || abs(st.Encoder - 7925) > cfg.RECENTRE_COUNTS
        setBalance(s,false);
        armBalance(s,cfg);          % not balancing, fell, or drifted too far
    end
    settled = waitSettled(s,cfg);
    resetIntegrator(s);

    r = sampleRun(s,cfg,cfg.TRIAL_SECONDS,true);
    r.label = string(label);  r.gains = gains;  r.think = cfg.think;  r.settled = settled;
    reportTrial(r);
end

function r = sampleRun(s,cfg,seconds,withKick)
% Record PREROLL + seconds and kick after the pre-roll, or just seconds with
% no kick. The kick start is read from STATUS field 10 (5 ms ticks still to run).
    az  = cfg.BASELINE.Angle_Zero;
    pre = withKick*cfg.PREROLL_SECONDS;
    N = ceil(80*(pre + seconds)) + 50;
    D = zeros(N,6);  n = 0;  tFire = NaN;  t0 = tic;
    while toc(t0) < pre + seconds && n < N
        st = readStatusWheeltec(s);
        n = n + 1;
        D(n,:) = [toc(t0), (st.AngleRaw - az)*180/2080, st.Moto, ...
                  (st.Encoder - 7925)/(1040/(pi*36)), st.KickTicks, st.FlagStop];
        if withKick && isnan(tFire) && D(n,1) >= pre
            tFire = toc(t0);
            disturb(s,cfg.STANDARD_KICK,cfg.STANDARD_KICK_MS);
        end
    end
    D = D(1:n,:);

    r = struct("label","","gains",struct(),"think",struct(),"settled",true, ...
        "t",D(:,1),"theta_deg",D(:,2),"u_pwm",D(:,3),"cart_mm",D(:,4), ...
        "kickTicks",D(:,5),"flagStop",D(:,6) > 0,"hasKick",withKick, ...
        "kickPwm",NaN,"kickMs",NaN,"tKick",NaN,"bandDeg",cfg.RECOVERY_BAND_DEG);
    if withKick
        r.kickPwm = cfg.STANDARD_KICK;  r.kickMs = cfg.STANDARD_KICK_MS;
        j = find(r.kickTicks > 0,1);
        if isempty(j)
            r.tKick = tFire;
        else
            r.tKick = max(tFire, r.t(j) - (r.kickMs - 5*r.kickTicks(j))/1000);
        end
    end
    r = addMetrics(r);
end

function r = addMetrics(r)
% Angles are median-filtered (removes sensor spikes) and measured from the
% pre-kick mean. Recovery = first moment after the kick from which the rod
% stays inside the fixed band for a full second.
    r.theta_filt = movmedian(r.theta_deg,3);
    if r.hasKick, pre = r.t < r.tKick; else, pre = true(size(r.t)); end
    post = ~pre | ~r.hasKick;

    r.preMeanDeg = mean(r.theta_filt(pre));
    dev = r.theta_filt - r.preMeanDeg;
    x0  = mean(r.cart_mm(pre));

    r.preSwayDeg  = std(dev(pre),1);
    r.anglePeak   = max(abs(dev(post)));
    r.angleRms    = sqrt(mean(dev(post).^2));
    r.effortRms   = sqrt(mean(r.u_pwm(post).^2));
    r.peakPwm     = max(abs(r.u_pwm(post)));
    r.satPct      = 100*mean(abs(r.u_pwm(post)) >= 6890);
    r.cartKickMm  = max(abs(r.cart_mm(post) - x0));
    r.cartMaxMm   = max(abs(r.cart_mm));
    r.driftMm     = r.cart_mm(end) - x0;
    r.residualDeg = mean(r.theta_filt(r.t > r.t(end) - 2));

    r.recoverS = NaN;
    if r.hasKick
        inBand = abs(dev) <= r.bandDeg;
        for i = find(post)'
            if r.t(i) + 1 > r.t(end), break; end
            if all(inBand(r.t >= r.t(i) & r.t <= r.t(i) + 1))
                r.recoverS = r.t(i) - r.tKick;
                break;
            end
        end
    end

    if any(r.flagStop),         r.outcome = "ROD FELL";
    elseif r.cartMaxMm > 200,   r.outcome = "CART REACHED THE RAIL END";
    elseif ~r.hasKick,          r.outcome = "upright, no kick";
    elseif isnan(r.recoverS),   r.outcome = "DID NOT SETTLE";
    else,                       r.outcome = "recovered";
    end
end

function reportTrial(r)
    fprintf("  outcome      : %s\n",r.outcome);
    fprintf("  mean angle   : %+.2f deg, sway %.2f deg RMS\n",r.preMeanDeg,r.preSwayDeg);
    fprintf("  peak |angle| : %.2f deg from that mean\n",r.anglePeak);
    if r.hasKick
        fprintf("  recovery     : %s from kick start (band +/-%.2f deg)\n",fmtRec(r.recoverS),r.bandDeg);
        fprintf("  cart pushed  : %.0f mm\n",r.cartKickMm);
    else
        fprintf("  cart drift   : %+.0f mm\n",r.driftMm);
    end
    fprintf("  residual     : %+.2f deg (last 2 s)\n",r.residualDeg);
    fprintf("  effort       : %.0f PWM rms, peak %.0f, %.1f%% saturated\n",r.effortRms,r.peakPwm,r.satPct);
end

function txt = fmtRec(v)
    if isnan(v), txt = "DID NOT SETTLE"; else, txt = sprintf("%.2f s",v); end
end

function trialTable(rs,showSpread)
    if nargin < 2, showSpread = false; end
    fprintf("\n%-16s %5s %5s | %6s %15s %6s %6s %6s\n","trial","Kp","Kd","peak","recovery","RMS","effort","cartX");
    for r = rs
        fprintf("%-16s %5g %5g | %6.2f %15s %6.2f %6.0f %6.0f\n",r.label,r.gains.Balance_KP, ...
            r.gains.Balance_KD,r.anglePeak,fmtRec(r.recoverS),r.angleRms,r.effortRms,r.cartKickMm);
    end
    if showSpread
        sp = @(v) max(v) - min(v);
        fprintf("%-16s %5s %5s | %6.2f %13.2f s %6.2f %6.0f %6.0f\n","spread","","",sp([rs.anglePeak]), ...
            sp([rs.recoverS]),sp([rs.angleRms]),sp([rs.effortRms]),sp([rs.cartKickMm]));
    end
    fprintf("angles in deg from the pre-kick mean; cartX = mm the kick pushed the cart.\n");
end

function compareTable(a,b)
    fprintf("\n%-20s %12s %12s  better?\n","criterion",a.label,b.label);
% A difference inside the tolerance (angles: the larger pre-kick sway) is
% reported as noise rather than as better or worse.
    sway = max(a.preSwayDeg,b.preSwayDeg);
    rows = {"peak |angle| (deg)", a.anglePeak,        b.anglePeak,        sway;
            "recovery (s)",       a.recoverS,         b.recoverS,         0.2;
            "RMS (deg)",          a.angleRms,         b.angleRms,         sway/2;
            "|residual| (deg)",   abs(a.residualDeg), abs(b.residualDeg), sway;
            "effort (PWM rms)",   a.effortRms,        b.effortRms,        0.05*a.effortRms;
            "cart pushed (mm)",   a.cartKickMm,       b.cartKickMm,       5};
    for k = 1:size(rows,1)
        [va,vb,tol] = rows{k,2:4};
        if isnan(va) || isnan(vb),  v = "n/a";
        elseif abs(vb - va) < tol,  v = "same (noise)";
        elseif vb < va,             v = "yes";
        else,                       v = "no";
        end
        fprintf("%-20s %12.3f %12.3f  %s\n",rows{k,1},va,vb,v);
    end
    fprintf("Tolerance for angles: %.2f deg (the larger pre-kick sway).\n",sway);
end

function plotTrial(r,cfg)
    figure("Name",r.label,"NumberTitle","off");  tiledlayout(3,1);
    nexttile;  plot(r.t,r.theta_deg,"Color",[.75 .75 .75]);  hold on;
    plot(r.t,r.theta_filt,"LineWidth",1.3);  yline(r.preMeanDeg,"k--");
    ylabel("\theta (deg)");  title(r.label);  grid on;  markKick(r);
    nexttile;  stairs(r.t,r.u_pwm);  yline([-6900 6900],"r--");
    ylabel("u (PWM)");  grid on;  markKick(r);
    nexttile;  plot(r.t,r.cart_mm);  yline([-1 1]*cfg.RAIL_HALF_MM,"r--");
    ylabel("cart (mm)");  xlabel("Time (s)");  grid on;  markKick(r);
end

function markKick(r)
    if r.hasKick, xline(r.tKick + [0 r.kickMs/1000],"r:"); end
end

function plotOverlay(rs,titleText)
% Each trial as its deviation from its own pre-kick mean, aligned on the kick.
    figure("Name",titleText,"NumberTitle","off");  hold on;
    for r = rs, plot(r.t - r.tKick, r.theta_filt - r.preMeanDeg,"LineWidth",1.3); end
    xline([0 rs(1).kickMs/1000],"k:");  yline(0,"k--");  grid on;
    xlabel("Time from the kick (s)");  ylabel("\theta - pre-kick mean (deg)");  title(titleText);
    legend([rs.label],"Location","best");
end

function T = summaryTable(rs)
    T = table([rs.label]',[rs.anglePeak]',[rs.recoverS]',[rs.angleRms]',[rs.preSwayDeg]', ...
        [rs.residualDeg]',[rs.effortRms]',[rs.peakPwm]',[rs.satPct]',[rs.cartKickMm]', ...
        [rs.cartMaxMm]',[rs.outcome]', ...
        'VariableNames',{'trial','peak_deg','recovery_s','rms_deg','pre_sway_deg','residual_deg', ...
                         'effort_rms','peak_pwm','sat_pct','cart_pushed_mm','max_cart_mm','outcome'});
end
