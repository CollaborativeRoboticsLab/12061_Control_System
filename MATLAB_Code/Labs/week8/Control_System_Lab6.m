%% Control_System_Lab6.m
% CONTROL SYSTEMS 12061 / 12601 - Lab 6 Part B: PID tuning and evaluation WHEELTEC linear inverted pendulum, firmware R4-6 or later.
%
% Tune -> Test -> Measure -> Evaluate -> Refine.
% Sample code only: change it to suit your own setup, as the handout says！！！！！！！！！！！
%
% RULES
%   1. Run ONE SECTION AT A TIME (Ctrl+Enter). Several need you to hold the rod.

%   2. Change only ONE gain between consecutive trials. changeGain() does
%      exactly that, so always build a new controller with it.

%   3. Lines marked ">> TODO" are yours to write. Most stop with an error until you fill them in.
%
% Toolbox used (MATLAB_Code/pendulum): kickTrial, changeGain, printTrials, plotTrials, autoHome. Type "help kickTrial" to see what each metric means.


%% ===================== SETUP ===========================================
clc;
cfg = struct;
cfg.PORT     = "COM11";     % >> your COM port
cfg.GROUP_ID = "G01";       % >> your group, please make this the same as the tags on the inverted pendulum bag

% Standard disturbance: identical for every trial (firmware max 6900 PWM, 1000 ms).
cfg.KICK_PWM = 6000;   cfg.KICK_MS = 500;
cfg.PREROLL_S = 1.5;   cfg.POST_S = 8;        % recorded before / after the kick
cfg.SETTLE_S  = 10;    cfg.SETTLED_RMS_DEG = 1.0;
cfg.UPRIGHT_WINDOW = 150;  cfg.RECENTRE_COUNTS = 500;
cfg.BAND_DEG = 1.5;    % recovery band; Activity 1 replaces it with a measured value

% Tutor-approved gain ranges; kickTrial refuses anything outside them. You  may change the code to update this as needed
cfg.APPROVED = struct("Kp",[200 900], "Ki",[0 60], "Kd",[100 1500]);

% design specification for Activity 5. If the handout's values (5 deg, 3 s,
% 0.25 m) are already met by the baseline on this rig, you need to set
% tighter ones here.

cfg.SPEC = struct("peakDeg",5.0, "recoverS",3, "cartTravelM",0.250);

if ~exist("s","var") || isempty(s) || ~isvalid(s), s = connectPendulum(cfg.PORT); end
cfg.BOOT = readGains(s);                 % gains compiled into the firmware
cfg.AZ   = cfg.BOOT.Angle_Zero;
autoHome(s);


%% ===================== ACTIVITY 1 - BASELINE (3 runs) ==================
% >> TODO: your controller from Lab 5.
BASE = cfg.BOOT;
BASE.Balance_KP = NaN;
BASE.Balance_KI = NaN;
BASE.Balance_KD = NaN;

% 8 s with no kick: the rod's own jitter sets the recovery band for every trial.
noise = kickTrial(s,cfg,BASE,"noise","Kick",false);
cfg.BAND_DEG = max(0.4, 3*noise.preSwayDeg);
fprintf("Recovery band: +/-%.2f deg\n",cfg.BAND_DEG);

base = struct([]);
for k = 1:3
    base = [base, kickTrial(s,cfg,BASE,sprintf("Baseline %d",k))];   %#ok<AGROW>
end
printTrials(base);
plotTrials(base,"Activity 1 - baseline, 3 runs");

% >> TODO: how consistent are the 3 runs? Spread = max - min.
peakSpread = NaN;
recSpread  = NaN;
fprintf("Spread over 3 runs: peak %.2f deg, recovery %.2f s\n",peakSpread,recSpread);



%% ===================== ACTIVITY 2 - TUNE Kp ============================
% >> TODO: at least two larger Kp values, in small steps (Ki, Kd unchanged).

KP_TRIALS = [];
assert(numel(KP_TRIALS) >= 2,"Lab6:TODO","Give at least two Kp values in KP_TRIALS.");

kp = base(1);
for k = 1:numel(KP_TRIALS)
    g  = changeGain(kp(end).gains,"Kp",KP_TRIALS(k));
    kp = [kp, kickTrial(s,cfg,g,sprintf("Kp=%g",KP_TRIALS(k)))];     %
end
printTrials(kp);
plotTrials(kp,"Activity 2 - Kp");

% Q3-Q5: which quantities improved, which got worse, and was the largest Kp best?


%% ===================== ACTIVITY 3 - TUNE Kd ============================
% >> TODO: the best Kp from Activity 2, then at least two Kd values. If
% varying Kd doesn't change the system performance, consider use the basedline Kp rather than the best Kp. 

BEST_KP   = NaN;
KD_TRIALS = [];
assert(numel(KD_TRIALS) >= 2,"Lab6:TODO","Give at least two Kd values in KD_TRIALS.");

kd = kickTrial(s,cfg,changeGain(BASE,"Kp",BEST_KP),"Before Kd");
for k = 1:numel(KD_TRIALS)
    g  = changeGain(kd(end).gains,"Kd",KD_TRIALS(k));
    kd = [kd, kickTrial(s,cfg,g,sprintf("Kd=%g",KD_TRIALS(k)))];     %#ok<AGROW>
end
printTrials(kd);
plotTrials(kd,"Activity 3 - Kd");
% Q6-Q8: oscillation, recovery time, and any noisy cart corrections (look at u).


%% ===================== ACTIVITY 4 - DO WE NEED Ki? =====================
% The residual is the mean angle over the last 2 s of each run, measured
% from the ADC zero, not from the pre-kick mean.

fprintf("\nResidual angle after recovery:\n");
for r = [base kp(2:end) kd], fprintf("  %-12s %+.2f deg\n",r.label,r.residualDeg); end
% Q9-Q11: is there a persistent error, and would Ki be the right fix for it?


%% ===================== ACTIVITY 5 - DESIGN CHALLENGE ===================
% Run this section once per trial. Each trial changes ONE gain from the
% previous one; record every trial, including the ones that fail.
if ~exist("design","var"), design = kd(end); end
meetsSpec(base(1),cfg.SPEC);             % checks your meetsSpec before the rig moves

% >> TODO: which gain, its new value, and why (your decision from the last trial).
g   = changeGain(design(end).gains,"Kd",NaN);
WHY = "...";

r = kickTrial(s,cfg,g,sprintf("Design %d",numel(design)));
r.decision = WHY;   r.pass = meetsSpec(r,cfg.SPEC);
design = [design, r];
printTrials(design);


%% ===================== ACTIVITY 6 - FINAL CONTROLLER (3 runs) ==========
% >> TODO: the gains you choose.
FINAL = cfg.BOOT;
FINAL.Balance_KP = NaN;
FINAL.Balance_KI = NaN;
FINAL.Balance_KD = NaN;

meetsSpec(base(1),cfg.SPEC);
final = struct([]);
for k = 1:3
    r = kickTrial(s,cfg,FINAL,sprintf("Final %d",k));
    r.pass = meetsSpec(r,cfg.SPEC);
    final = [final, r];                                           
end
printTrials(final);
plotTrials([base(1) final(1)],"Baseline vs final");

% >> TODO: does the final controller meet the spec in all 3 runs?
allPass = NaN;
fprintf("Final controller meets the spec in all 3 runs: %d\n",allPass);
% Q12-Q15: answer on the handout.


%% ===================== SAVE ============================================
setBalance(s,false);
name = cfg.GROUP_ID + "_Lab6_" + string(datetime("now","Format","yyyyMMdd_HHmmss"));
folder = fullfile(fileparts(which("Control_System_Lab6.m")),"data");
if ~exist(folder,"dir"), mkdir(folder); end
results = [base kp(2:end) kd design(2:end) final];
save(fullfile(folder,name+".mat"),"results","cfg");
writetable(printTrials(results),fullfile(folder,name+".csv"));
fprintf("Saved %s.mat and .csv in %s\n",name,folder);






%% ===================== FUNCTION ===================================
function ok = meetsSpec(r,spec)
% >> TODO: return true only if this run meets EVERY part of the spec:
%    peak below spec.peakDeg, recovery below spec.recoverS (NaN = fail),
%    cart travel below spec.cartTravelM (r.cartTravelMm is in mm),
%    no sustained oscillation, and a stable recovery (r.stable).
error("Lab6:TODO","Write meetsSpec at the bottom of the script first.");
end
