%% Control_System_Lab4.m
% CONTROL SYSTEMS 12061 / 12601 - Laboratory 4, PART B
% WHEELTEC linear inverted pendulum
%
% SESSION 4 - PID CONTROL: FROM CHAOS TO CONTROL
%
%   "Can feedback control stabilise an unstable inverted pendulum?"
%
% In Session 3 you made the pendulum fall and measured how fast. Here you % close the loop and shape its behaviour. Part A of the handout (poles,
% Routh tables, gain ranges) is pen and paper; this script is Part B only.
%
% =========================================================================
% READ THIS FIRST
% =========================================================================
%   1. Run ONE SECTION AT A TIME with Ctrl+Enter. Never F5 / Run All.
%
%   2. Boxes mark what YOU do. Five labels, same as Lab 3:
%        EDIT THIS / PREDICT FIRST / STUDENT ACTION / SAFETY / FOR YOUR REPORT
%      The SAFETY box is drawn in !!!! so you cannot scroll past it.
%
%   3. TWO MODES. Set cfg.MODE in Section 0.
%        "rig"  drives the real pendulum
%        "sim"  runs a model of it, no hardware needed. This is not
%        required/ Just for practice.

%      Use "sim" to prepare before the session, to push gains until the
%      system fails without risking the rig, and to finish your plots
%      afterwards. Use "rig" for the numbers that go in your report.
%
%   4. THE THING THAT SURPRISES EVERYONE.
%
%      You are NOT going to write u = Kp*e in MATLAB. It would not work,
%      and it is worth understanding why before you start.
%
%      In Lab 3 you measured the unstable pole: the angle error DOUBLES
%      every ~132 ms. A control loop needs many samples per doubling to
%      act in time. MATLAB talking over the serial link manages about
%      30 Hz, i.e. 33 ms, i.e. FOUR samples per doubling. The firmware's
%      control interrupt runs at 200 Hz: 26 samples per doubling.
%
%      So the PID lives on the STM32, where it can keep up. What MATLAB
%      does is CHANGE ITS GAINS while it runs, and record what happens:
%
%          setGain(s,"BKP",400)     % Kp   angle loop
%          setGain(s,"BKD",400)     % Kd   angle loop
%          setGain(s,"BKI",0)       % Ki   angle loop  (new in R4-5)
%          setGain(s,"PKP",20)      % Kp   cart position loop
%          setGain(s,"PKD",300)     % Kd   cart position loop
%
%      That is still "varying the gains experimentally" exactly as the
%      handout asks. It is simply the only place the loop can run fast
%      enough to be a loop at all.
%
%   5. SECTIONS
%        0   Settings, mode, connection
%        1   THINK before you start
%        2   Step 1  - no control (baseline)
%        3   Step 2  - P only: small / medium / large Kp
%        4   Step 3  - add D
%        5   Step 4  - add I, and why it behaves as it does here
%        6   Step 5  - the four-way comparison
%        7   Step 6  - the two loops (angle inner, cart outer)
%        8   Steps 7 and 8 - plots and the three tables
%        9   Reflection questions
%        99  Close
% =========================================================================


%% ===================== SECTION 0 - SETTINGS =============================
clearvars -except s
clc;

cfg = struct;

% -------------------------------------------------------------------------
% >> EDIT THIS
%    cfg.MODE      "sim" to run without hardware, "rig" for the real thing.
%                  Start in "sim" if you have not used the rig yet.
%    cfg.PORT      your COM port. serialportlist shows what exists.
%    cfg.GROUP_ID  your group number, or your files overwrite someone's.
% -------------------------------------------------------------------------
% cfg.MODE     = "sim"; 
cfg.MODE     = "rig"; 
cfg.PORT     = "COM11";
cfg.GROUP_ID = "G01";

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

onPath = which('Control_System_Lab4.m');
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
    warning("Lab4:FolderGuess", ...
        ['Could not locate Control_System_Lab4.m on the MATLAB path, so captures will ' ...
         'go to\n  %s\nRun setupPath from MATLAB_Code if that is not ' ...
         'where you want them.'],labFolder);
end

cfg.DATA_FOLDER = fullfile(labFolder,"data");
if ~exist(cfg.DATA_FOLDER,"dir"), mkdir(cfg.DATA_FOLDER); end

cfg.TRIAL_SECONDS = 10;      % how long to record one trial
cfg.SETTLE_SECONDS = 1.5;    % ignore this much after a gain change

% The firmware's shipped gains. Every experiment below starts from these.
cfg.DEFAULT = struct("BKP",400,"BKD",400,"BKI",0,"PKP",20,"PKD",300);

% ---- simulation constants, all traceable to a measurement -------------
% lambda: the unstable pole YOU measured in Lab 3 (5.246 1/s -> 132 ms).
% kCart : from Lab 3's motor test -- PWM 2500 for 300 ms moved the cart 443
%         encoder counts, i.e. 48 mm, i.e. 1.07 m/s^2. Minus the ~800-count
%         friction deadband that leaves 6.3e-4 m/s^2 per PWM count.
% These are approximations of YOUR rig, not a digital twin of it. The rig
% is always the authority; the model is for intuition and for safety.
cfg.sim = struct( ...
    "lambda",        5.246, ...      % 1/s, measured in Lab 3
    "kCart",         6.30e-4, ...    % m/s^2 per PWM count, from Lab 3
    "pivotDamping",  0.3, ...        % 1/s, bearing friction
    "deadbandPWM",   0, ...          % set 800 to include stiction
    "disturb_mps2",  0.20, ...       % steady disturbance: see Section 5
    "countsPerRad",  2080/pi, ...    % 2080 ADC counts over 180 degrees
    "countsPerMetre",1040/(pi*0.036), ...
    "Ts",            0.005, ...      % firmware control period
    "iPwmLimit",     2000, ...       % matches BALANCE_I_PWM_LIMIT
    "theta0_deg",    1.2, ...        % release angle for a trial
    "railHalf_mm",   217);           % half the usable rail

cfg.results = struct([]);            % every trial accumulates here

fprintf("\n=== LAB 4 READY ===\n");
fprintf("  mode          : %s\n",cfg.MODE);
fprintf("  data folder   : %s\n",cfg.DATA_FOLDER);

if cfg.MODE == "rig"
    if ~exist("s","var") || isempty(s) || ~isvalid(s)
        s = connectPendulum(cfg.PORT);
    end
    homeCart(s);
    g  = readGains(s);
    st = readStatusWheeltec(s);
    fprintf("  gains on board: Kp=%g Kd=%g Ki=%g | PKp=%g PKd=%g\n", ...
        g.Balance_KP,g.Balance_KD,g.Balance_KI,g.Position_KP,g.Position_KD);
    fprintf("  battery       : %.2f V\n",st.VoltageVolts);
    if st.VoltageVolts < 10.5
        warning("Lab4:LowBattery", ...
            "Battery %.2f V. Balancing needs the motor at full strength.", ...
            st.VoltageVolts);
    end
else
    fprintf("  no hardware needed in sim mode\n");
end


%% ============== SECTION 1 - THINK BEFORE YOU START ======================
% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Fill in the handout's five answers NOW, before running anything.
%    Edit the strings below so they are saved with your data.
%
%      What is feedback?
%      What does P do?
%      What does I do?
%      What does D do?
%      Why are controller gains important?
% -------------------------------------------------------------------------
cfg.think = struct( ...
    "feedback",  "...", ...
    "P_does",    "...", ...
    "I_does",    "...", ...
    "D_does",    "...", ...
    "why_gains", "...");

disp("Predictions recorded. They are saved with every capture from here on.");


%% ============= SECTION 2 - STEP 1: NO CONTROL (BASELINE) ================
% The same thing you saw in Lab 3, repeated so that today's four-way
% comparison has a like-for-like first row.

% -------------------------------------------------------------------------
% >> SAFETY - the rig is live from this section onward (rig mode only)
%    Hands and sleeves clear of the rail. Keep the KEY5 stop button within
%    reach: it is wired into the firmware and always outranks MATLAB.
% -------------------------------------------------------------------------

% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    With every gain at zero, what will the angle do after you release?
%    Sketch it before you look.
% -------------------------------------------------------------------------

trial = runTrial(cfg,"No control", struct("BKP",0,"BKD",0,"BKI",0, ...
                                          "PKP",0,"PKD",0), s_or_empty(cfg));
cfg.results = [cfg.results, trial];
plotTrial(trial);
reportTrial(trial);


%% ============ SECTION 3 - STEP 2: P CONTROL, THREE SIZES ================
% u = Kp*e, with e = 0 - theta. Kd and Ki stay at zero, so this is pure
% proportional action: the correction is a fixed multiple of the error.

% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    The handout expects: small Kp = weak correction, may still fall;
%    medium = tries to stabilise, oscillation may appear; large =
%    aggressive, larger oscillation, possible loss of stability.
%    Which of those do you actually believe? Write it down first.
% -------------------------------------------------------------------------

% 20 is genuinely too weak on this rig and the rod falls; 400 is the
% firmware default; 1600 saturates the motor. Verified in simulation.
kpList = [20 400 1600];         % small, medium, large
pOnly  = struct([]);

for k = 1:numel(kpList)
    gains = struct("BKP",kpList(k),"BKD",0,"BKI",0,"PKP",0,"PKD",0);
    t = runTrial(cfg,sprintf("P only, Kp=%d",kpList(k)),gains,s_or_empty(cfg));
    pOnly = [pOnly, t];                                         %#ok<AGROW>
    cfg.results = [cfg.results, t];
    reportTrial(t);
end

plotOverlay(pOnly,"Step 2 - proportional action alone");

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Fill the handout's first table from the numbers reportTrial printed:
%
%      Kp | pendulum behaviour | oscillation | control effort
%
%    "Control effort" is the RMS PWM in that printout. Note that a bigger
%    Kp does NOT simply mean better: compare the oscillation column with
%    the control-effort column and say what you are trading away.
% -------------------------------------------------------------------------


%% ================= SECTION 4 - STEP 3: ADD D ============================
% derivative ~ (e(k) - e(k-1)) / Ts, then u = Kp*e + Kd*derivative.
%
% The firmware computes exactly that difference every 5 ms. D responds to
% how fast the error is changing, so it can start correcting before the
% error itself is large.

% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Keep Kp at 400. What will increasing Kd do to the oscillation, to the
%    smoothness, and to the control effort?
% -------------------------------------------------------------------------

kdList = [0 200 400 1600];
pdRuns = struct([]);

for k = 1:numel(kdList)
    gains = struct("BKP",400,"BKD",kdList(k),"BKI",0,"PKP",0,"PKD",0);
    t = runTrial(cfg,sprintf("PD, Kd=%d",kdList(k)),gains,s_or_empty(cfg));
    pdRuns = [pdRuns, t];                                       %#ok<AGROW>
    cfg.results = [cfg.results, t];
    reportTrial(t);
end

plotOverlay(pdRuns,"Step 3 - adding derivative action");

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    Record the effect of increasing Kd. Look at the residual-angle column
%    across the four runs: the improvement is large at first and then
%    stops mattering. Say where that knee is for your rig, and why more D
%    beyond it buys you nothing (hint: what is D amplifying?).
% -------------------------------------------------------------------------


%% ============ SECTION 5 - STEP 4: ADD I, AND WHAT IT REALLY DOES ========
% integral(k) = integral(k-1) + error(k)*Ts
% u = Kp*error + Ki*integral + Kd*derivative
%
% -------------------------------------------------------------------------
% >> PREDICT FIRST
%    Integral action is usually introduced as "it removes steady-state
%    error". Before you run this: on an inverted pendulum balanced by a
%    cart that is free to move, what has to happen to the CART for the
%    controller to hold a steady non-zero output? Write your answer down.
% -------------------------------------------------------------------------

kiList = [0 60 150 400 1000];
pidRuns = struct([]);

for k = 1:numel(kiList)
    gains = struct("BKP",400,"BKD",600,"BKI",kiList(k),"PKP",0,"PKD",0);
    t = runTrial(cfg,sprintf("PID, Ki=%d",kiList(k)),gains,s_or_empty(cfg));
    pidRuns = [pidRuns, t];                                     %#ok<AGROW>
    cfg.results = [cfg.results, t];
    reportTrial(t);
end

plotOverlay(pidRuns,"Step 4 - adding integral action");
plotCartDrift(pidRuns,"Step 4 - what the integral does to the CART");

disp(" ");
disp("=== WHY THE CART PLOT IS THE INTERESTING ONE HERE ===");
disp("Something on this rig pushes steadily one way -- a rail that is not");
disp("quite level, cable drag, a rod that is not perfectly balanced. With");
disp("PD alone the controller can hold the ANGLE near zero, but only by");
disp("leaning slightly, and a permanent lean means the cart keeps sliding.");
disp("Watch the cart plot: at Ki = 0 it walks off, and as Ki rises the");
disp("walk slows down, because the integral supplies the steady effort the");
disp("lean was providing.");
disp(" ");
disp("Then look at the largest Ki. The residual is tiny, the cart barely");
disp("moves -- and the response has started oscillating and the control");
disp("effort has tripled. Integral action is not free.");
disp(" ");
disp("Note what integral action CANNOT fix: an error in the sensor itself.");
disp("It drives the MEASURED error to zero, and if the measurement is");
disp("biased, zero measured error is the wrong angle. That is a calibration");
disp("job, not a tuning job.");

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    The handout asks four questions here. Answer them from YOUR plots:
%      does residual error decrease?  does the controller keep acting on a
%      small persistent error?  do slow oscillations appear as Ki grows?
%      can excessive integral action make the response worse?
%    Then add the one the handout does not ask: what did the cart do, and
%    what does that tell you about where integral action belongs?
% -------------------------------------------------------------------------


%% ============ SECTION 6 - STEP 5: THE FOUR-WAY COMPARISON ===============
% No control, P, PD, PID -- one figure, like for like.

fourWay = [ ...
    runTrial(cfg,"1 No control",struct("BKP",0,  "BKD",0,  "BKI",0, "PKP",0,"PKD",0),s_or_empty(cfg)), ...
    runTrial(cfg,"2 P",         struct("BKP",400,"BKD",0,  "BKI",0, "PKP",0,"PKD",0),s_or_empty(cfg)), ...
    runTrial(cfg,"3 PD",        struct("BKP",400,"BKD",600,"BKI",0, "PKP",0,"PKD",0),s_or_empty(cfg)), ...
    runTrial(cfg,"4 PID",       struct("BKP",400,"BKD",600,"BKI",150,"PKP",0,"PKD",0),s_or_empty(cfg))];

cfg.results = [cfg.results, fourWay];
plotOverlay(fourWay,"Step 5 - no control / P / PD / PID");
plotEffort(fourWay,"Step 5 - control input versus time");
comparisonTable(fourWay);


%% ============== SECTION 7 - STEP 6: THE TWO LOOPS =======================
% Inner loop  - pendulum angle. Fast. Must react within a doubling time.
% Outer loop  - cart position. Slow. Decides where the cart should be.
%
% The firmware already has both:
%     Moto = Balance_Pwm - Position_Pwm
% Balance_Pwm is the angle PID you have been tuning. 
% Position_Pwm is a PD on cart position whose gains are PKP and PKD. Subtracting it is what
% nudges the angle setpoint so the cart comes home.
%
% Everything so far ran with PKP = PKD = 0, i.e. the outer loop switched
% off, so that you were looking at the angle loop on its own. Now switch it
% back on and watch the cart.

% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    This section is RIG ONLY, on purpose.
%
%    The simulation in this script models the inner angle loop faithfully,
%    but its outer position loop is not trustworthy enough to tune against:
%    the cart-to-encoder scaling depends on the drive pulley diameter,
%    which is still an open measurement on this rig (20 mm measured by hand
%    against 36 mm in the vendor drawing - a factor of 1.8 straight
%    through the position gain). Rather than have you tune a model that may
%    be wrong by that much, this step uses the real thing.
% -------------------------------------------------------------------------
if cfg.MODE ~= "rig"
    disp(" ");
    disp("Section 7 needs the rig. In sim mode, here is what to expect and");
    disp("why, so you can predict before you get to the bench:");
    disp("  PKp = 0   the angle loop alone. The rod stays up but the cart");
    disp("            wanders, because nothing is asking it to come back.");
    disp("  PKp = 20  the shipped value. The cart returns slowly; the rod");
    disp("            leans briefly in the direction it has to travel.");
    disp("  PKp = 60  the outer loop now demands angles the inner loop has");
    disp("            to chase. Expect a fight, then loss of balance.");
    disp(" ");
    disp("That last one is the handout's second THINK question in physical");
    disp("form: a position controller demanding a large angle reference is");
    disp("asking the fast loop to track something it cannot.");
else
    cascade = struct([]);
    for pkp = [0 20 60]
        gains = struct("BKP",400,"BKD",600,"BKI",0,"PKP",pkp,"PKD",300);
        t = runTrial(cfg,sprintf("cascade, PKp=%d",pkp),gains,s_or_empty(cfg));
        cascade = [cascade, t];                                 %#ok<AGROW>
        cfg.results = [cfg.results, t];
        reportTrial(t);
    end
    plotCartDrift(cascade,"Step 6 - the outer loop brings the cart back");
end

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    The handout's two THINK questions, answered from these runs:
%      Why should the angle loop react faster than the position loop?
%      What could happen if the position controller demands a very large
%      angle reference?
%    For the second one: raise PKp until it does happen, and describe it.
% -------------------------------------------------------------------------


%% ========== SECTION 8 - STEPS 7 AND 8: PLOTS AND TABLES =================
% Everything the handout's "What You Must Produce" asks for.

summary = gainEffectTable(cfg);
disp(summary);
saveSession(cfg);

% -------------------------------------------------------------------------
% >> FOR YOUR REPORT
%    You now have: angle vs time for no control / P / PD / PID, the
%    comparison overlay, and control input vs time. Plus the three tables.
%    What is left is the short discussion, and it is the part that carries
%    the marks: explain how the behaviour changed as P, D and I were
%    introduced, in terms of stability, oscillation, residual error and
%    control effort.
% -------------------------------------------------------------------------


%% ================= SECTION 9 - REFLECTION ===============================
% The handout's eight questions. Answer them from your own data, not from
% the internet. Question 7 is the one most people get wrong.
%
%   1 Why was feedback necessary to stabilise the inverted pendulum?
%   2 Why did P alone not necessarily produce smooth stabilisation?
%   3 How did D action change the oscillatory behaviour?
%   4 Why can excessive I action degrade or destabilise the response?
%   5 Which term had the greatest effect? Support it with numbers.
%   6 How did control effort change as the controller became aggressive?
%   7 Was the controller with the smallest angular error necessarily the
%     best controller? Explain.
%   8 What trade-offs did you see between stability, speed, oscillation
%     and control effort?

disp(" ");
disp("Answer the eight reflection questions in your report.");
disp("Q5 and Q7 both need numbers from your tables, not opinions.");


%% ===================== SECTION 99 - CLOSE ===============================
% -------------------------------------------------------------------------
% >> STUDENT ACTION
%    Run this before you leave. It restores the firmware's shipped gains --
%    otherwise the next group inherits yours and cannot work out why the
%    rig behaves oddly -- then stops the motor and frees the COM port.
% -------------------------------------------------------------------------
if cfg.MODE == "rig" && exist("s","var") && ~isempty(s) && isvalid(s)
    try
        restoreDefaults(s,cfg);
        pendulumCommand(s,"STOP",0.5,3,"STOPPED");
    catch ME
        fprintf(2,"Could not restore the rig cleanly: %s\n",ME.message);
    end
end
try, clear s; catch, end
disp("Gains restored, motor stopped, serial closed.");


%% ========================= LOCAL FUNCTIONS =============================
% Nothing below needs editing. Read it if you want to see exactly how the
% numbers in your tables were produced -- particularly simulate(), which
% runs the same difference equations the firmware runs.


function sOut = s_or_empty(cfg)
%S_OR_EMPTY Fetch the serial object from the base workspace in rig mode.
%   Sections are run one at a time, so the script's variables live in the
%   base workspace; this keeps runTrial's signature honest in both modes.
    if cfg.MODE == "rig"
        sOut = evalin("base","s");
    else
        sOut = [];
    end
end


function trial = runTrial(cfg,label,gains,s)
%RUNTRIAL One experiment: set the gains, release, record angle and input.
    fprintf("\n--- %s ---\n",label);
    if cfg.MODE == "sim"
        [t,theta,u,x] = simulate(cfg,gains);
        note = "simulated";
    else
        [t,theta,u,x] = runOnRig(cfg,gains,s);
        note = "measured";
    end

    % Section 1 (the THINK table) is optional, and students skip it. Never
    % throw away a capture that has already been taken just because that
    % section was not run -- the error used to fire on the line below,
    % AFTER the ten seconds of data had been recorded.
    if isfield(cfg,"think")
        think = cfg.think;
    else
        think = struct("feedback","(Section 1 not run)","P_does","", ...
                       "I_does","","D_does","","why_gains","");
    end

    trial = struct("label",string(label),"gains",gains,"mode",cfg.MODE, ...
                   "t",t,"theta_deg",theta,"u_pwm",u,"cart_mm",x, ...
                   "note",string(note),"think",think, ...
                   "group",cfg.GROUP_ID, ...
                   "stamp",string(datetime("now","Format","yyyyMMdd_HHmmss")));
end


function [t,theta_deg,u,x_mm] = simulate(cfg,g)
%SIMULATE The firmware's control law around a cart-driven pendulum.
%
%   MODEL
%     theta'' = lambda^2 sin(theta) - (a/l) cos(theta) - c theta'
%   with l = g/lambda^2 so that the OPEN-LOOP pole is exactly the 5.246 1/s
%   you measured in Lab 3, and a the cart acceleration.
%
%   CONTROLLER
%     Exactly the firmware's difference equations, at the firmware's 5 ms
%     tick, including that D is a per-tick DIFFERENCE (not divided by Ts)
%     and that the integral contribution is clamped -- so gain values here
%     mean the same thing they mean on the rig.
%
%   WHAT IT DOES NOT MODEL
%     Sensor noise, belt compliance, or the rail ends. The deadband is
%     available (cfg.sim.deadbandPWM) but off by default, because with it
%     on the response limit-cycles and the teaching point is harder to see.
%     Turn it on once you have the idea: the jitter it produces is the
%     jitter you see on the real rig.

    p  = cfg.sim;
    Ts = p.Ts;
    L  = 9.81/p.lambda^2;
    n  = round(cfg.TRIAL_SECONDS/Ts);

    th = deg2rad(p.theta0_deg);  w = 0;  xm = 0;  v = 0;
    lastBias = [];  I = 0;  posBias = 0;  lastPos = 0;  posTerm = 0;  ptick = 0;

    t = (0:n-1)'*Ts;  theta_deg = zeros(n,1);  u = zeros(n,1);  x_mm = zeros(n,1);

    for k = 1:n
        bias = p.countsPerRad*th;
        if isempty(lastBias), lastBias = bias; end
        d = bias - lastBias;                        % per-tick difference
        if abs(g.BKI) > 1e-4
            I = I + bias*Ts;
            lim = abs(p.iPwmLimit/g.BKI);
            I = min(max(I,-lim),lim);               % anti-windup, as R4-5
        else
            I = 0;
        end
        balance = -g.BKP*bias - g.BKD*d - g.BKI*I;
        lastBias = bias;

        ptick = ptick + 1;
        if ptick > 4                                % outer loop every 25 ms
            ptick = 0;
            posCounts = -xm*p.countsPerMetre;       % +PWM lowers the count
            posBias = 0.8*posBias + 0.2*posCounts;
            posTerm = posBias*g.PKP + (posBias-lastPos)*g.PKD;
            lastPos = posBias;
        end

        moto = min(max(balance - posTerm,-6900),6900);

        eff = abs(moto) - p.deadbandPWM;
        if eff <= 0
            a = 0;
        else
            a = -sign(moto)*p.kCart*eff;            % cart catches the fall
        end
        % A steady disturbance the controller has to work against: a rail
        % that is not quite level, cable drag, a slightly off-centre rod.
        % Without it there is no persistent error, and then integral action
        % has nothing to demonstrate.
        a = a + p.disturb_mps2;

        for j = 1:5                                 % sub-step the plant
            h = Ts/5;
            thdd = p.lambda^2*sin(th) - (a/L)*cos(th) - p.pivotDamping*w;
            w  = w + thdd*h;   th = th + w*h;
            v  = v + a*h;      xm = xm + v*h;
        end

        theta_deg(k) = rad2deg(th);
        u(k) = moto;
        x_mm(k) = xm*1000;

        % Two ways a run ends early, and both are real failures.
        if abs(theta_deg(k)) > 45                   % the rod fell over
            t = t(1:k); theta_deg = theta_deg(1:k);
            u = u(1:k); x_mm = x_mm(1:k);
            return;
        end
        if abs(x_mm(k)) > p.railHalf_mm             % the cart hit the end
            fprintf(2,"  cart reached the end of the rail at t = %.1f s\n",t(k));
            fprintf(2,"  (keeping the rod up while sliding off the rail is\n");
            fprintf(2,"   not a working controller -- say so in your table)\n");
            t = t(1:k); theta_deg = theta_deg(1:k);
            u = u(1:k); x_mm = x_mm(1:k);
            return;
        end
    end
end


function [t,theta_deg,u,x_mm] = runOnRig(cfg,g,s)
%RUNONRIG Push the gains to the firmware, release, and poll STATUS.
    resetIntegrator(s);
    setGain(s,"BKP",g.BKP);  setGain(s,"BKD",g.BKD);  setGain(s,"BKI",g.BKI);
    setGain(s,"PKP",g.PKP);  setGain(s,"PKD",g.PKD);

    allZero = (g.BKP==0 && g.BKD==0 && g.BKI==0);
    if allZero
        setBalance(s,false);            % no control: the firmware lets go
    else
        st = readStatusWheeltec(s);
        if abs(st.Encoder - 7925) > 400
            fprintf("Cart at %d, re-homing before this trial.\n",st.Encoder);
            homeCart(s,"Force",true);
        end
    end

    % ---- wait for the rod to be upright; do not race a countdown --------
    % This used to be a blind "3... 2... 1...", which gave you three
    % seconds to get from the keyboard to the rod. That is not enough, and
    % missing it is expensive: the firmware's Turn_Off() LATCHES
    % Flag_Stop = 1 as soon as the angle leaves ZHONGZHI +/- 500
    % (2600..3600 counts), and nothing in the firmware ever clears it again
    % -- only a fresh BAL,1 does. So the controller was being switched on
    % during the very seconds you were still lifting the rod, and it died
    % there: MATLAB still got BAL=1 back, nothing looked wrong, but Moto
    % stayed 0 for the whole run and the cart never moved.
    %
    % Now the script waits for you. Take as long as you need.
    fprintf("Lift the rod to VERTICAL and hold it steady.\n");
    fprintf("(hanging reads about 1020 counts, upright about 3100)\n");
    tw = tic;  lastPrint = -1;
    while true
        st = readStatusWheeltec(s);
        if abs(st.AngleRaw - 3100) <= 450, break; end
        if toc(tw) > 45
            setBalance(s,false);
            error("Lab4:RodNotUpright", ...
                ['Gave up after 45 s. The angle still reads %d counts and ' ...
                 'the window is 2600..3600.\n\n' ...
                 'If the rod IS upright and the reading is still wrong, the ' ...
                 'angle sensor needs calibrating\n(see the calibration ' ...
                 'guide) -- that is a mechanical fault, not a gain you can ' ...
                 'tune.'],st.AngleRaw);
        end
        if floor(toc(tw)) ~= lastPrint
            lastPrint = floor(toc(tw));
            fprintf("  waiting...  angle = %4d\n",st.AngleRaw);
        end
        pause(0.2);
    end

    % Rod confirmed upright, so hand the motor over NOW, with no gap in
    % which Turn_Off() could latch the stop flag again.
    fprintf("Rod is up (angle = %d).\n",st.AngleRaw);
    if ~allZero
        setBalance(s,true);
        fprintf("Controller is LIVE -- you will feel it push against your hand.\n");
    end
    pause(1.5);
    disp(">>> RELEASE NOW <<<");

    cap = 4000;
    t = zeros(cap,1); theta_deg = zeros(cap,1); u = zeros(cap,1); x_mm = zeros(cap,1);
    ref = readGains(s);  %#ok<NASGU>   % one round trip so timing is warm
    n = 0;  t0 = tic;
    while toc(t0) < cfg.TRIAL_SECONDS
        st = readStatusWheeltec(s);
        n = n + 1;
        t(n) = toc(t0);
        theta_deg(n) = (st.AngleRaw - 3100)*(180/2080);
        u(n) = st.Moto;
        x_mm(n) = (st.Encoder - 7925)/(1040/(pi*0.036))*1000;
        if n >= cap, break; end
    end
    t = t(1:n); theta_deg = theta_deg(1:n); u = u(1:n); x_mm = x_mm(1:n);

    setBalance(s,false);
    fprintf("Captured %d samples in %.1f s (%.1f Hz).\n",n,t(end),n/t(end));
end


function r = reportTrial(trial)
%REPORTTRIAL The four numbers every table in the handout needs.
    th = trial.theta_deg;  u = trial.u_pwm;  t = trial.t;
    % Two distinct failures, and the difference matters in your table:
    % the rod falling, and the rod staying up while the cart runs out of
    % rail. The second one still gets a lovely angle trace.
    fell    = abs(th(end)) > 40;
    railEnd = ~fell && abs(trial.cart_mm(end)) > 200;   % 217 mm is the end
    tailIdx = t > 0.65*t(end);
    residual = mean(abs(th(tailIdx)));
    sig = th(tailIdx);
    crossings = sum(sig(1:end-1).*sig(2:end) < 0);
    effort = sqrt(mean(u.^2));
    drift = (trial.cart_mm(end) - trial.cart_mm(1));

    r = struct("fell",fell,"railEnd",railEnd,"residual_deg",residual, ...
               "crossings",crossings,"effort_rms",effort,"cart_drift_mm",drift);

    if fell
        fprintf("  outcome         : ROD FELL\n");
    elseif railEnd
        fprintf("  outcome         : rod stayed up, CART RAN OUT OF RAIL\n");
    else
        fprintf("  outcome         : stable\n");
    end
    fprintf("  residual |angle|: %.3f deg   (mean over the last third)\n",residual);
    fprintf("  oscillation     : %d zero crossings\n",crossings);
    fprintf("  control effort  : %.0f PWM rms\n",effort);
    fprintf("  cart drift      : %+.0f mm\n",drift);
end


function plotTrial(trial)
    figure("Name",trial.label,"NumberTitle","off");
    tiledlayout(3,1);
    nexttile; plot(trial.t,trial.theta_deg,"LineWidth",1.3); grid on;
    ylabel("\theta (deg)"); title(trial.label); yline(0,"k--");
    nexttile; stairs(trial.t,trial.u_pwm,"LineWidth",1.1); grid on;
    ylabel("u (PWM)");
    nexttile; plot(trial.t,trial.cart_mm,"LineWidth",1.1); grid on;
    ylabel("cart (mm)"); xlabel("Time (s)");
end


function plotOverlay(trials,titleText)
    figure("Name",titleText,"NumberTitle","off"); hold on;
    for k = 1:numel(trials)
        plot(trials(k).t,trials(k).theta_deg,"LineWidth",1.3);
    end
    grid on; yline(0,"k--");
    xlabel("Time (s)"); ylabel("\theta (deg from upright)");
    title(titleText);
    legend([trials.label],"Location","best","Interpreter","none");
end


function plotEffort(trials,titleText)
    figure("Name",titleText,"NumberTitle","off"); hold on;
    for k = 1:numel(trials)
        stairs(trials(k).t,trials(k).u_pwm,"LineWidth",1.1);
    end
    grid on; xlabel("Time (s)"); ylabel("u (PWM counts)");
    title(titleText);
    legend([trials.label],"Location","best","Interpreter","none");
    yline(6900,"r:","saturation"); yline(-6900,"r:");
end


function plotCartDrift(trials,titleText)
    figure("Name",titleText,"NumberTitle","off"); hold on;
    for k = 1:numel(trials)
        plot(trials(k).t,trials(k).cart_mm,"LineWidth",1.3);
    end
    grid on; yline(0,"k--");
    xlabel("Time (s)"); ylabel("cart position (mm from centre)");
    title(titleText);
    legend([trials.label],"Location","best","Interpreter","none");
end


function T = comparisonTable(trials)
%COMPARISONTABLE The handout's four-controller table, filled from the data.
    n = numel(trials);
    name = strings(n,1); stable = strings(n,1); osc = zeros(n,1);
    res = zeros(n,1); eff = zeros(n,1); drift = zeros(n,1);
    for k = 1:n
        r = quietReport(trials(k));
        name(k)   = trials(k).label;
        if r.fell,         stable(k) = "rod fell";
        elseif r.railEnd,  stable(k) = "cart ran out";
        else,              stable(k) = "yes";
        end
        osc(k)    = r.crossings;
        res(k)    = r.residual_deg;
        eff(k)    = r.effort_rms;
        drift(k)  = r.cart_drift_mm;
    end
    T = table(name,stable,osc,res,eff,drift,'VariableNames', ...
        {'Controller','Stable','Crossings','ResidualDeg','EffortRMS','CartDriftMm'});
    fprintf("\n=== STEP 5 COMPARISON ===\n"); disp(T);
end


function T = gainEffectTable(cfg)
%GAINEFFECTTABLE Every trial in one table, for the "effect of increasing
%   Kp / Ki / Kd" summary the handout asks for.
    tr = cfg.results;
    n = numel(tr);
    label = strings(n,1); kp = zeros(n,1); kd = zeros(n,1); ki = zeros(n,1);
    pkp = zeros(n,1); res = zeros(n,1); osc = zeros(n,1);
    eff = zeros(n,1); drift = zeros(n,1); stable = strings(n,1);
    for k = 1:n
        r = quietReport(tr(k));
        label(k) = tr(k).label;
        kp(k) = tr(k).gains.BKP;  kd(k) = tr(k).gains.BKD;
        ki(k) = tr(k).gains.BKI;  pkp(k) = tr(k).gains.PKP;
        res(k) = r.residual_deg;  osc(k) = r.crossings;
        eff(k) = r.effort_rms;    drift(k) = r.cart_drift_mm;
        if r.fell,         stable(k) = "rod fell";
        elseif r.railEnd,  stable(k) = "cart ran out";
        else,              stable(k) = "yes";
        end
    end
    T = table(label,kp,kd,ki,pkp,stable,res,osc,eff,drift,'VariableNames', ...
        {'Trial','Kp','Kd','Ki','PKp','Stable','ResidualDeg','Crossings', ...
         'EffortRMS','CartDriftMm'});
    fprintf("\n=== EVERY TRIAL THIS SESSION ===\n");
end


function r = quietReport(trial)
    th = trial.theta_deg; u = trial.u_pwm; t = trial.t;
    fell = abs(th(end)) > 40;
    m = t > 0.65*t(end);
    sig = th(m);
    r = struct("fell",fell, ...
               "railEnd",~fell && abs(trial.cart_mm(end)) > 200, ...
               "residual_deg",mean(abs(th(m))), ...
               "crossings",sum(sig(1:end-1).*sig(2:end) < 0), ...
               "effort_rms",sqrt(mean(u.^2)), ...
               "cart_drift_mm",trial.cart_mm(end)-trial.cart_mm(1));
end


function saveSession(cfg)
%SAVESESSION One .mat with every trial, plus a flat .csv of the summary.
    if isempty(cfg.results)
        fprintf(2,"Nothing to save -- run some sections first.\n"); return;
    end
    stamp = string(datetime("now","Format","yyyyMMdd_HHmmss"));
    base  = cfg.GROUP_ID + "_lab4_" + cfg.MODE + "_" + stamp;
    results = cfg.results;                                      %#ok<NASGU>
    save(fullfile(cfg.DATA_FOLDER,base+".mat"),"results","cfg");
    writetable(gainEffectTable(cfg),fullfile(cfg.DATA_FOLDER,base+".csv"));
    fprintf("Saved: %s.(mat|csv)\n",base);
end


function restoreDefaults(s,cfg)
%RESTOREDEFAULTS Put the firmware's shipped gains back.
    d = cfg.DEFAULT;
    setGain(s,"BKP",d.BKP); setGain(s,"BKD",d.BKD); setGain(s,"BKI",d.BKI);
    setGain(s,"PKP",d.PKP); setGain(s,"PKD",d.PKD);
    resetIntegrator(s);
    fprintf("Restored Kp=%g Kd=%g Ki=%g PKp=%g PKd=%g\n", ...
        d.BKP,d.BKD,d.BKI,d.PKP,d.PKD);
end
