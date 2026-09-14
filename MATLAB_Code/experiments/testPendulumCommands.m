function results = testPendulumCommands(s)
%TESTPENDULUMCOMMANDS Exercise every command that cannot move the cart.
%
%   s = connectPendulum("COM4");
%   testPendulumCommands(s)
%
%   Runs each helper once, checks the reply parses, and times the round
%   trip. Nothing here drives the motor, so it is safe with the rod fitted
%   and your hands on the rig.
%
%   The gain test writes a value and puts the original back. If it fails
%   part way through, check the gains with readGains(s) before running the
%   rig -- they should be 400 / 400 / 20 / 300.

arguments
    s
end

fprintf("\n===== Read-only command test =====\n\n");

names   = strings(0,1);
ok      = false(0,1);
detail  = strings(0,1);
rtt     = zeros(0,1);

    function record(name,fcn)
        t0 = tic;
        try
            txt = fcn();
            names(end+1,1)  = name;      %#ok<AGROW>
            ok(end+1,1)     = true;      %#ok<AGROW>
            detail(end+1,1) = txt;       %#ok<AGROW>
            rtt(end+1,1)    = toc(t0);   %#ok<AGROW>
            fprintf("  ok    %-14s %s\n",name,txt);
        catch ME
            names(end+1,1)  = name;                  %#ok<AGROW>
            ok(end+1,1)     = false;                 %#ok<AGROW>
            detail(end+1,1) = string(ME.message);    %#ok<AGROW>
            rtt(end+1,1)    = toc(t0);               %#ok<AGROW>
            fprintf(2,"  FAIL  %-14s %s\n",name,ME.message);
        end
    end

%% Sensors
record("PING",    @() pendulumCommand(s,"PING",0.5,3,"PONG"));

record("readAngle", @() sprintf("%d counts",readAngleWheeltec(s)));

record("readPosition", @() sprintf("%d counts",readPositionWheeltec(s)));

record("readVoltage", @() sprintf("%.2f V",readVoltage(s)));

%% Gains
record("readGains", @() gainText(readGains(s)));

g0 = [];
try
    g0 = readGains(s);
catch
end

if ~isempty(g0)
    record("setGain write", @() setGainAndVerify(s,"BKP",g0.Balance_KP-25));
    record("setGain undo",  @() setGainAndVerify(s,"BKP",g0.Balance_KP));
else
    fprintf(2,"  skip  gain write   (could not read the originals first)\n");
end

%% Status
record("readStatus", @() statusText(readStatusWheeltec(s)));

%% How fast can MATLAB actually poll?
fprintf("\n  Measuring poll rate (3 s of STATUS)...\n");
n = 0; t0 = tic;
while toc(t0) < 3
    readStatusWheeltec(s);
    n = n + 1;
end
el = toc(t0);
fprintf("  %d replies in %.1f s  ->  %.1f Hz, %.0f ms per round trip\n", ...
    n,el,n/el,1000*el/n);
fprintf("  (the firmware's own control loop runs at 200 Hz; this is the\n");
fprintf("   supervisory link only, which is why balancing stays on the board)\n");

%% Summary
results = table(names,ok,rtt*1000,detail, ...
    'VariableNames',{'Command','Pass','RoundTripMs','Detail'});

fprintf("\n===== %d of %d passed =====\n",sum(ok),numel(ok));
if all(ok)
    fprintf("All read-only commands work. Next: testPendulumMotor(s)\n");
    fprintf("Read its safety notes first -- that one does move the cart.\n\n");
else
    fprintf(2,"Fix the failures above before touching the motor.\n\n");
end
end


function txt = gainText(g)
txt = sprintf("KP=%.1f KD=%.1f  posKP=%.1f posKD=%.1f", ...
    g.Balance_KP,g.Balance_KD,g.Position_KP,g.Position_KD);
end


function txt = statusText(st)
txt = sprintf("pwm=%d pos=%d angle=%d %.2fV stop=%d rxErr=%d", ...
    st.Moto,st.Encoder,st.AngleRaw,st.VoltageVolts, ...
    st.FlagStop,st.RxErrors);
if st.RxErrors > 0
    txt = txt + "   <-- rxErr should be 0";
end
end


function txt = setGainAndVerify(s,name,value)
setGain(s,name,value);
g = readGains(s);
switch name
    case "BKP", got = g.Balance_KP;
    case "BKD", got = g.Balance_KD;
    case "PKP", got = g.Position_KP;
    case "PKD", got = g.Position_KD;
end
if abs(got - round(value)) > 0.05
    error("STM32:GainNotApplied","asked for %g, board reports %g",round(value),got);
end
txt = sprintf("%s = %g (read back)",name,got);
end
