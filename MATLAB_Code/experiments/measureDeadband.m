function result = measureDeadband(s,opts)
%MEASUREDEADBAND Find the PWM below which the cart does not move at all.
%
%   s = connectPendulum("COM4");
%   measureDeadband(s)
%
%   testPendulumMotor showed PWM 600 and 900 moving the cart nowhere while
%   1500 and 2500 moved it. That gap is static friction: below some torque
%   the cart simply does not break free. This sweeps PWM to find where the
%   threshold actually is, in BOTH directions -- belt tension and the drive
%   train are rarely symmetric, so the two can differ.
%
%   Every pulse has the same duration, so displacements are comparable,
%   and the direction alternates on each step so the cart oscillates about
%   its starting point instead of walking into an end stop.
%
%   WHY IT MATTERS
%   The balance controller computes small corrections near equilibrium. Any
%   correction smaller than this deadband produces no motion at all, so the
%   error grows until the command finally clears the threshold and the cart
%   lurches. That is a large part of why the rod jitters instead of sitting
%   still, and it is not something more PID tuning can fix.
%
%   BEFORE RUNNING: rod clear, hands clear, cart near the middle.

arguments
    s
    opts.PulseMs   (1,1) double {mustBePositive} = 250
    opts.PwmList   (1,:) double = 400:100:2000
    opts.MoveCount (1,1) double {mustBePositive} = 10   % counts = "it moved"
    opts.SafeLo    (1,1) double = 6800
    opts.SafeHi    (1,1) double = 9000
    opts.Target    (1,1) double = 7925   % POSITION_MIDDLE in control.h
end

st = readStatusWheeltec(s);
fprintf("\n===== Deadband sweep =====\n");
fprintf("  cart at %d, battery %.2f V, pulse %d ms\n\n",st.Encoder,st.VoltageVolts,opts.PulseMs);

if st.VoltageVolts < 10.5
    error("Lab:LowBattery","Battery %.2f V. Charge before measuring friction.",st.VoltageVolts);
end
% The encoder is relative and reset writes 10000 wherever the cart is, so
% a freshly connected board always reads outside this band. Establish the
% frame first, then check it.
homeCart(s,"Target",opts.Target);
st = readStatusWheeltec(s);
if st.Encoder < opts.SafeLo || st.Encoder > opts.SafeHi
    fprintf("  Cart at %d, outside %d..%d.\n",st.Encoder,opts.SafeLo,opts.SafeHi);
    fprintf("  Slide it to the middle by hand and re-home.\n");
    homeCart(s,"Force",true,"Target",opts.Target);
    st = readStatusWheeltec(s);
end
fprintf("  Cart at %d (homed).\n",st.Encoder);

fprintf("The cart will make %d short moves. Rod clear? Hands clear?\n",2*numel(opts.PwmList));
if ~strcmpi(strtrim(string(input("Type y and Enter: ","s"))),"y")
    fprintf("Cancelled.\n"); result = table(); return;
end

pwmCol = []; dirCol = []; movedCol = []; posCol = [];

for k = 1:numel(opts.PwmList)
    for sgn = [1 -1]                       % alternate so the cart stays put
        p = sgn*opts.PwmList(k);

        before = readPositionWheeltec(s);
        if before < opts.SafeLo || before > opts.SafeHi
            fprintf(2,"  Cart reached %d, outside the safe band. Stopping.\n",before);
            break;
        end

        setMotorPWM_wheeltec(s,p,opts.PulseMs);
        pause(opts.PulseMs/1000 + 0.5);
        after = readPositionWheeltec(s);

        d = after - before;
        pwmCol(end+1,1)   = abs(p);            %#ok<AGROW>
        dirCol(end+1,1)   = sgn;               %#ok<AGROW>
        movedCol(end+1,1) = d;                 %#ok<AGROW>
        posCol(end+1,1)   = after;             %#ok<AGROW>

        fprintf("  PWM %+5d -> %+5d counts   (cart %d)\n",p,d,after);
    end
end

pendulumCommand(s,"STOP",0.5,3,"STOPPED");

result = table(pwmCol,dirCol,movedCol,posCol, ...
    'VariableNames',{'PWM','Direction','CountsMoved','PositionAfter'});

%% Threshold per direction
fprintf("\n");
thresh = nan(1,2);
labels = ["positive PWM","negative PWM"];
for i = 1:2
    sgn = (i==1)*2 - 1;
    m = dirCol == sgn;
    p = pwmCol(m); d = abs(movedCol(m));
    idx = find(d >= opts.MoveCount,1,'first');
    if isempty(idx)
        fprintf("  %s : never broke free up to PWM %d\n",labels(i),max(p));
    else
        if idx == 1
            thresh(i) = p(1);
            fprintf("  %s : moved already at the lowest PWM tried (%d)\n",labels(i),p(1));
        else
            thresh(i) = (p(idx)+p(idx-1))/2;
            fprintf("  %s : deadband between %d and %d, call it %.0f\n", ...
                labels(i),p(idx-1),p(idx),thresh(i));
        end
    end
end

if all(~isnan(thresh))
    asym = abs(thresh(1)-thresh(2));
    fprintf("\n  asymmetry between directions: %.0f PWM counts",asym);
    if asym > 0.2*mean(thresh)
        fprintf("  <-- large; check belt tension\n");
    else
        fprintf("  (small, normal)\n");
    end
    fprintf("  as a fraction of full scale (6900): %.1f %%\n",100*mean(thresh)/6900);
end

%% Plot
figure("Name","Actuator deadband","Color","w");
plot(pwmCol(dirCol==1),abs(movedCol(dirCol==1)),"o-","LineWidth",1.2); hold on;
plot(pwmCol(dirCol==-1),abs(movedCol(dirCol==-1)),"s-","LineWidth",1.2);
yline(opts.MoveCount,"--","movement threshold");
grid on; xlabel("PWM magnitude"); ylabel("|counts moved| per pulse");
title(sprintf("Cart travel per %d ms pulse",opts.PulseMs));
legend("positive PWM","negative PWM","Location","northwest");

fprintf("\nThe flat part at the left is the deadband: torque below static\n");
fprintf("friction moves nothing. Where the curve lifts off is the threshold.\n\n");
end
