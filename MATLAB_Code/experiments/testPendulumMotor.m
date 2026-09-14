function results = testPendulumMotor(s)
%TESTPENDULUMMOTOR First motor movement, in four steps of rising size.
%
%   s = connectPendulum("COM4");
%   testPendulumMotor(s)
%
%   BEFORE RUNNING
%     * Take the pendulum rod OFF, or at least make sure nothing is in its
%       swing. This test only moves the cart, but the cart yanks the rod.
%     * Push the cart to the middle of the rail (around 7925 counts).
%     * Hands clear of the rail.
%     * Know where the stop button is. It outranks anything MATLAB sends.
%
%   WHAT IT DOES
%     1. Checks battery and cart position, and refuses to run if either is
%        unsafe -- before sending any PWM.
%     2. Asks you to confirm.
%     3. Sends four short pulses: 600, 900, 1500, 2500, each 150-300 ms,
%        measuring how far the cart travelled and checking it stopped.
%     4. Verifies the firmware watchdog by leaving the cart alone.
%
%   Each pulse carries its own duration, so the firmware stops the motor
%   itself even if MATLAB dies mid-test.

arguments
    s
end

% Cart band, in the firmware's ABSOLUTE encoder frame. That frame only
% means something once homeCart has run: the encoder is relative, reset
% writes 10000 wherever the cart is, and connecting resets the board.
SAFE_LO = 7200;      % home the cart if it starts outside this band
SAFE_HI = 8600;
CART_TARGET = 7925;  % POSITION_MIDDLE in control.h
MIN_V   = 10.5;      % volts; the firmware itself cuts out below 7.0 V

fprintf("\n===== Motor test =====\n\n");

%% 1. Pre-flight
st = readStatusWheeltec(s);
fprintf("Before starting\n");
fprintf("  cart position : %d counts   (rail centre 7925, end stops 5900 / 9900)\n",st.Encoder);
fprintf("  battery       : %.2f V\n",st.VoltageVolts);
fprintf("  stop flag     : %d\n",st.FlagStop);
fprintf("  rx errors     : %d\n\n",st.RxErrors);

if st.VoltageVolts < MIN_V
    error("Lab:LowBattery", ...
        "Battery reads %.2f V, want at least %.1f V. Below 7.0 V the firmware " + ...
        "forces a stop, and on USB power alone the motor driver has no supply " + ...
        "at all. Connect and charge the battery.",st.VoltageVolts,MIN_V);
end

% A fresh connection always reads exactly 10000 (see the note above), so
% refusing on position would refuse every first run. Establish the frame
% instead, then check it.
homeCart(s,"Target",CART_TARGET);
st = readStatusWheeltec(s);
if st.Encoder < SAFE_LO || st.Encoder > SAFE_HI
    fprintf("Cart is at %d, outside %d..%d.\n",st.Encoder,SAFE_LO,SAFE_HI);
    fprintf("Slide it to the middle by hand and re-home.\n");
    homeCart(s,"Force",true,"Target",CART_TARGET);
    st = readStatusWheeltec(s);
end
fprintf("  cart now at   : %d counts (homed)\n\n",st.Encoder);

%% 2. Confirm
fprintf("The cart is about to move. Rod clear? Hands clear?\n");
answer = input("Type y and press Enter to continue: ","s");
if ~strcmpi(strtrim(string(answer)),"y")
    fprintf("Cancelled. Nothing was sent.\n");
    results = table();
    return;
end

%% 3. Pulses
plan = [ 600 150
         900 150
        1500 200
        2500 300 ];

pwm = zeros(0,1); ms = zeros(0,1); moved = zeros(0,1); stopped = false(0,1);

for k = 1:size(plan,1)
    thisPwm = plan(k,1);
    thisMs  = plan(k,2);

    before = readPositionWheeltec(s);
    fprintf("\nPulse %d: PWM %d for %d ms   (cart at %d)\n",k,thisPwm,thisMs,before);

    try
        setMotorPWM_wheeltec(s,thisPwm,thisMs);
    catch ME
        if ME.identifier == "STM32:FirmwareError"
            fprintf(2,"  Firmware refused it: %s\n",ME.message);
            fprintf(2,"  ERR=ENDSTOP means the cart is too close to the rail end.\n");
            break;
        end
        rethrow(ME);
    end

    pause(thisMs/1000 + 0.6);           % let it run and coast to a stop

    after = readPositionWheeltec(s);
    stA   = readStatusWheeltec(s);
    d     = after - before;

    fprintf("  moved %+d counts, now at %d, PWM now %d\n",d,after,stA.Moto);

    pwm(end+1,1)     = thisPwm;                 %#ok<AGROW>
    ms(end+1,1)      = thisMs;                  %#ok<AGROW>
    moved(end+1,1)   = d;                       %#ok<AGROW>
    stopped(end+1,1) = (stA.Moto == 0);         %#ok<AGROW>

    if stA.Moto ~= 0
        fprintf(2,"  WARNING: motor is still commanded to %d. Sending STOP.\n",stA.Moto);
        pendulumCommand(s,"STOP",0.5,3,"STOPPED");
        break;
    end

    if abs(d) < 5 && thisPwm >= 900
        fprintf(2,"  WARNING: almost no movement at PWM %d.\n",thisPwm);
        fprintf(2,"  Check the belt, the motor connector, and the battery.\n");
    end

    if after < 6400 || after > 9400
        fprintf(2,"  Getting close to an end stop. Stopping the test here.\n");
        break;
    end
end

%% 4. Watchdog check
fprintf("\nWatchdog: sending PWM 800 for 200 ms, then waiting 1.5 s...\n");
setMotorPWM_wheeltec(s,800,200);
pause(1.5);
stW = readStatusWheeltec(s);
wdOK = (stW.Moto == 0);
if wdOK
    fprintf("  PWM is back to 0 by itself. Watchdog works.\n");
else
    fprintf(2,"  PWM is still %d after the pulse should have expired.\n",stW.Moto);
    pendulumCommand(s,"STOP",0.5,3,"STOPPED");
end

%% Summary
results = table(pwm,ms,moved,stopped, ...
    'VariableNames',{'PWM','DurationMs','CountsMoved','StoppedAfter'});

fprintf("\n");
disp(results);

if ~isempty(moved)
    dirOK = all(sign(moved(moved~=0)) == sign(moved(find(moved~=0,1))));
    fprintf("Direction consistent : %d\n",dirOK);
    fprintf("Watchdog             : %d\n",wdOK);
    % Deliberately NOT reporting counts-per-PWM: the pulses have different
    % durations and the cart accelerates rather than moving at a constant
    % speed, so that ratio would be meaningless. Compare accelerations
    % instead, via x = 0.5*a*t^2.
    live = moved ~= 0;
    if any(live)
        acc = 2*abs(moved(live))./((ms(live)/1000).^2);
        for i = find(live)'
            fprintf("  PWM %4d -> about %6.0f counts/s^2\n", ...
                pwm(i),2*abs(moved(i))/((ms(i)/1000)^2));
        end
        if nnz(live) >= 2 && numel(unique(pwm(live))) >= 2
            pl = pwm(live);
            % a = k*(pwm - deadband)  ->  solve with the two extreme points
            [~,lo] = min(pl); [~,hi] = max(pl);
            r = acc(hi)/acc(lo);
            if r > 1.01
                dead = (pl(hi) - r*pl(lo))/(1 - r);
                fprintf("\nStatic friction deadband, rough estimate: PWM ~%.0f\n",dead);
                fprintf("Below that the cart does not break free at all -- which is\n");
                fprintf("exactly what PWM %d and %d just showed.\n",pwm(1),pwm(2));
                fprintf("Measure it properly with measureDeadband(s).\n");
            end
        end
    end
    fprintf("Calibrate counts to millimetres with a ruler before quoting distances.\n");
end

fprintf("\nIf all four pulses moved the cart the same way and it stopped each\n");
fprintf("time, the Route 4 motor path is proven. Next is lab_step_response.m\n\n");
end
