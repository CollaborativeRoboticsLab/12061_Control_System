%% lab_step_response.m
%  Open-loop step response of the cart, driven from MATLAB.
%
%  This is the experiment that used to require editing control.c:
%
%      #define LAB1_TEST_MODE   1
%      #define LAB1_PWM         2500
%      #define LAB1_PULSE_TICKS 60
%
%  then rebuilding and reflashing for every amplitude. Now the amplitude
%  and the pulse width are just numbers in this script.
%
%  IMPORTANT, and worth saying to the students out loud:
%  the sampling here is NOT the firmware's 200 Hz control loop. Every
%  sample costs one request/response round trip over USB serial, so this
%  loop runs at roughly 25-40 Hz and the interval jitters. That is fine
%  for watching a cart accelerate. It is nowhere near fast enough to
%  balance the pendulum from MATLAB, and the script measures the actual
%  rate so the students can see the number rather than be told it.

clear; clc;

PORT      = "COM4";
BAUD      = 128000;      % firmware A's rate, set by uart_init(72,128000)
PWM_STEP  = 2500;        % same as the old LAB1_PWM
PULSE_MS  = 300;         % same as LAB1_PULSE_TICKS * 5 ms
RECORD_S  = 1.5;         % keep logging after the pulse ends

%% 1. Connect
%  connectPendulum, not connectSTM32: dropping DTR/RTS resets this board
%  into a 2 s startup delay, and the binary DataScope stream has to be
%  muted before the first readline can parse anything. connectPendulum
%  handles both; connectSTM32 is left alone for the bluepill board.
s = connectPendulum(PORT,BAUD);
cleanupObj = onCleanup(@() cleanupPort(s));   %#ok<NASGU>

st = readStatusWheeltec(s);
fprintf("\nBoard state before the step\n");
fprintf("  cart position : %d counts (rail centre is 7925)\n",st.Encoder);
fprintf("  pendulum ADC  : %d counts (hanging centre is 3100)\n",st.AngleRaw);
fprintf("  battery       : %.2f V\n",st.VoltageVolts);
fprintf("  rx errors     : %d\n\n",st.RxErrors);

if st.Encoder > 8600 || st.Encoder < 7200
    error("Lab:CartOffCentre", ...
        "Cart is at %d counts. Push it near 7925 before the step, " + ...
        "or it will hit the end stop.",st.Encoder);
end

%% 2. Fire the step and log
n    = 0;
cap  = 4000;
tLog = zeros(cap,1);
pLog = zeros(cap,1);
aLog = zeros(cap,1);
uLog = zeros(cap,1);

t0 = tic;
setMotorPWM_wheeltec(s,PWM_STEP,PULSE_MS);

while toc(t0) < RECORD_S
    st = readStatusWheeltec(s);
    n  = n + 1;
    tLog(n) = toc(t0);
    pLog(n) = st.Encoder;
    aLog(n) = st.AngleRaw;
    uLog(n) = st.Moto;
    if n >= cap, break; end
end

tLog = tLog(1:n); pLog = pLog(1:n); aLog = aLog(1:n); uLog = uLog(1:n);

%% 3. What sampling rate did we actually get?
dt = diff(tLog);
fprintf("Sampling actually achieved\n");
fprintf("  samples        : %d over %.2f s\n",n,tLog(end));
fprintf("  mean rate      : %.1f Hz\n",1/mean(dt));
fprintf("  interval       : min %.1f ms, median %.1f ms, max %.1f ms\n", ...
    1000*min(dt),1000*median(dt),1000*max(dt));
fprintf("  jitter (std)   : %.1f ms\n\n",1000*std(dt));

%% 4. Plot
figure("Name","Open-loop step response","Color","w");

subplot(3,1,1);
stairs(tLog,uLog,"LineWidth",1.2); grid on;
ylabel("PWM"); title(sprintf("Step: PWM = %d for %d ms",PWM_STEP,PULSE_MS));
ylim([-1.1 1.1]*max(abs(uLog)+1));

subplot(3,1,2);
plot(tLog,pLog - pLog(1),"LineWidth",1.2); grid on;
ylabel("cart travel (counts)");

subplot(3,1,3);
plot(tLog,aLog - aLog(1),"LineWidth",1.2); grid on;
ylabel("pendulum (ADC counts)"); xlabel("time (s)");

%% 5. Simple numbers to talk about
idx = find(uLog ~= 0,1,"first");
if ~isempty(idx)
    travel = pLog(end) - pLog(idx);
    fprintf("Cart moved %d counts after the step.\n",travel);
    fprintf("Calibrate counts -> millimetres with the ruler test " + ...
            "(Lab 2, Step 3) before quoting a velocity.\n");
end

function cleanupPort(s)
    try
        pendulumCommand(s,"STOP",0.50,2,["STOPPED"]);
    catch
    end
end
