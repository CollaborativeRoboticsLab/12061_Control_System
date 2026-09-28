function r = kickTrial(s,cfg,gains,label,opts)
%KICKTRIAL One standard-kick trial on the balancing pendulum, with metrics.
%
%   r = kickTrial(s,cfg,gains,label)
%       Applies the gains, makes sure the rod is balancing near the rail
%       centre, records cfg.PREROLL_S of quiet, kicks (cfg.KICK_PWM for
%       cfg.KICK_MS), records cfg.POST_S more, and returns the result.

%   r = kickTrial(...,"Kick",false)
%       Records cfg.POST_S with no kick (the noise reference).
%
%   Data in r (all vectors, time in s from the start of the recording):
%       t, theta (deg), x (mm from rail centre), u (PWM)
%   Metrics in r (angles are median-filtered and taken from the pre-kick mean):
%       peakDeg       largest |theta| deviation after the kick starts
%       recoverS      time from the END of the kick until |theta| stays
%                     inside +/-cfg.BAND_DEG for 1 s (NaN = never)
%       cartTravelMm  largest cart movement away from its pre-kick position
%       nOsc          number of separate excursions outside the band
%       sustained     true if the rod is still leaving the band in the last 3 s
%       residualDeg   mean angle over the last 2 s (absolute)
%       effortRms, satPct, preSwayDeg, stable, outcome
%
%   Needs firmware R4-6 or later. See Control_System_Lab6.m for cfg.

arguments
    s
    cfg struct
    gains struct
    label (1,1) string = ""
    opts.Kick (1,1) logical = true
end

checkApproved(gains,cfg);
fprintf("\n--- %s   (Kp=%g Ki=%g Kd=%g) ---\n",label,gains.Balance_KP,gains.Balance_KI,gains.Balance_KD);
applyGains(s,gains);

st = readStatusWheeltec(s);
if st.FlagStop || st.ManualMode || st.AutoRun || abs(st.Encoder - 7925) > cfg.RECENTRE_COUNTS
    armBalance(s,cfg);
end
waitSettled(s,cfg);
resetIntegrator(s);

r = record(s,cfg,opts.Kick);
r.label = label;  r.gains = gains;
r = metrics(r,cfg);
fprintf("  %s | peak %.2f deg | recovery %s | cart %.0f mm | %d excursions\n", ...
    r.outcome,r.peakDeg,fmtS(r.recoverS),r.cartTravelMm,r.nOsc);
end


% ======================================================================
function checkApproved(g,cfg)
% Refuse unfilled (NaN) or unapproved gains before anything moves.
    names = ["Balance_KP","Balance_KI","Balance_KD"];  keys = ["Kp","Ki","Kd"];
    for k = 1:3
        v = g.(names(k));  lim = cfg.APPROVED.(keys(k));
        if isnan(v)
            error("Lab:GainNotSet","%s is not set yet (NaN). Fill in the TODO first.",keys(k));
        end
        if v < lim(1) || v > lim(2)
            error("Lab:GainNotApproved","%s = %g is outside the approved range [%g %g].",keys(k),v,lim(1),lim(2));
        end
    end
end

function applyGains(s,g)
    setGain(s,"BKP",g.Balance_KP);   setGain(s,"BKD",g.Balance_KD);
    setGain(s,"BKI",g.Balance_KI);   setGain(s,"PKP",g.Position_KP);
    setGain(s,"PKD",g.Position_KD);
    if ~isnan(g.Position_KI), setGain(s,"PKI",g.Position_KI); end
    resetIntegrator(s);
end

function armBalance(s,cfg)
% Re-centre the cart if needed, wait for the rod to be held upright, then
% enable balance. Enabled too early, the firmware stops and stays stopped.
    st = readStatusWheeltec(s);
    if ~st.FlagStop || st.ManualMode || st.AutoRun, setBalance(s,false); end
    if abs(st.Encoder - 7925) > cfg.RECENTRE_COUNTS
        input("Cart is off-centre. Let the rod hang still, then press Enter: ","s");
        autoHome(s,"Confirm",false,"Verbose",false);
    end

    fprintf("Lift the rod to VERTICAL and hold it steady.\n");
    tw = tic;  k = 0;  st = readStatusWheeltec(s);
    while abs(st.AngleRaw - cfg.AZ) > cfg.UPRIGHT_WINDOW
        if toc(tw) > 60
            error("Lab:RodNotUpright","Angle reads %d, want %g +/- %d.",st.AngleRaw,cfg.AZ,cfg.UPRIGHT_WINDOW);
        end
        k = k + 1;
        if mod(k,5) == 0, fprintf("  angle %d (want %g +/- %d)\n",st.AngleRaw,cfg.AZ,cfg.UPRIGHT_WINDOW); end
        pause(0.2);  st = readStatusWheeltec(s);
    end
    setBalance(s,true);
    fprintf("Controller LIVE.\n");  pause(1.5);  disp(">>> RELEASE NOW <<<");  pause(0.5);
    if readStatusWheeltec(s).FlagStop
        setBalance(s,false);
        error("Lab:BalanceDidNotArm","The firmware stopped the motor during the hand-over. Hold the rod steadier and retry.");
    end
end

function waitSettled(s,cfg)
% Wait until the sway about its own mean is below cfg.SETTLED_RMS_DEG.
    tw = tic;  sway = Inf;
    while sway >= cfg.SETTLED_RMS_DEG && toc(tw) < cfg.SETTLE_S
        w = zeros(30,1);
        for k = 1:30, w(k) = readStatusWheeltec(s).AngleRaw; end
        sway = std(movmedian(w,3)*(180/2080),1);
    end
    if sway >= cfg.SETTLED_RMS_DEG
        warning("Lab:NeverSettled","Still swaying %.2f deg RMS; this trial starts agitated.",sway);
    end
end

function r = record(s,cfg,withKick)
% Poll STATUS; kick once after the pre-roll. The kick start is read from
% STATUS field 10 (5 ms ticks still to run), not from the host clock.
    pre = withKick*cfg.PREROLL_S;
    N = ceil(80*(pre + cfg.POST_S)) + 50;
    D = zeros(N,6);  n = 0;  tFire = NaN;  t0 = tic;
    while toc(t0) < pre + cfg.POST_S && n < N
        st = readStatusWheeltec(s);
        n = n + 1;
        D(n,:) = [toc(t0), (st.AngleRaw - cfg.AZ)*180/2080, st.Moto, ...
                  (st.Encoder - 7925)/(1040/(pi*36)), st.KickTicks, st.FlagStop];
        if withKick && isnan(tFire) && D(n,1) >= pre
            tFire = toc(t0);
            disturb(s,cfg.KICK_PWM,cfg.KICK_MS);
        end
    end
    D = D(1:n,:);
    r = struct("label","","gains",struct(),"decision","","pass",NaN, ...
        "t",D(:,1),"theta",D(:,2),"u",D(:,3),"x",D(:,4),"kickTicks",D(:,5), ...
        "fell",any(D(:,6) > 0),"hasKick",withKick,"kickPwm",NaN,"kickMs",NaN, ...
        "tKick",NaN,"tEnd",NaN,"band",cfg.BAND_DEG);
    if withKick
        r.kickPwm = cfg.KICK_PWM;  r.kickMs = cfg.KICK_MS;
        j = find(r.kickTicks > 0,1);
        if isempty(j), r.tKick = tFire;
        else,          r.tKick = max(tFire, r.t(j) - (r.kickMs - 5*r.kickTicks(j))/1000);
        end
        r.tEnd = r.tKick + r.kickMs/1000;
    end
end

function r = metrics(r,cfg)
    thF = movmedian(r.theta,3);                     % removes single-sample spikes
    if r.hasKick, pre = r.t < r.tKick; else, pre = true(size(r.t)); end
    post = ~pre | ~r.hasKick;

    r.mean0 = mean(thF(pre));   r.x0 = mean(r.x(pre));
    r.dev   = thF - r.mean0;
    r.preSwayDeg   = std(r.dev(pre),1);
    r.peakDeg      = max(abs(r.dev(post)));
    r.cartTravelMm = max(abs(r.x(post) - r.x0));
    r.maxXmm       = max(abs(r.x));
    r.residualDeg  = mean(thF(r.t > r.t(end) - 2));
    r.effortRms    = sqrt(mean(r.u(post).^2));
    r.satPct       = 100*mean(abs(r.u(post)) >= 6890);

    % Excursions outside the band; blips shorter than 3 samples are sensor
    % spikes the median filter missed, not oscillation.
    out = abs(r.dev) > cfg.BAND_DEG & post;
    d  = diff([false; out; false]);  s0 = find(d == 1);  s1 = find(d == -1) - 1;
    keep = (s1 - s0 + 1) >= 3;       s0 = s0(keep);      s1 = s1(keep);
    out = false(size(out));
    for k = 1:numel(s0), out(s0(k):s1(k)) = true; end
    r.nOsc      = r.hasKick*numel(s0);
    r.sustained = r.hasKick && nnz(r.t(s0) > r.t(end) - 3) >= 2;   % still swinging at the end

    r.recoverS = NaN;
    if r.hasKick
        inBand = ~out;
        for i = find(r.t >= r.tEnd)'
            if r.t(i) + 1 > r.t(end), break; end
            if all(inBand(r.t >= r.t(i) & r.t <= r.t(i) + 1)), r.recoverS = r.t(i) - r.tEnd; break; end
        end
    end

    r.stable = ~r.fell && r.maxXmm < 200 && ~r.sustained && (~r.hasKick || ~isnan(r.recoverS));
    if r.fell,                   r.outcome = "ROD FELL";
    elseif r.maxXmm >= 200,      r.outcome = "CART AT RAIL END";
    elseif r.sustained,          r.outcome = "SUSTAINED OSCILLATION";
    elseif ~r.hasKick,           r.outcome = "no kick";
    elseif isnan(r.recoverS),    r.outcome = "DID NOT RECOVER";
    else,                        r.outcome = "recovered";
    end
end

function txt = fmtS(v)
    if isnan(v), txt = "--"; else, txt = sprintf("%.2f s",v); end
end
