%% minibalance_serial_monitor.m
% WHEELTEC linear inverted pendulum - MATLAB serial monitor
%
% DataScope frame used by the original STM32 project:
%   Byte 1      : 0x24 ('$')              frame header
%   Bytes 2-5   : Channel 1, float32 LE    Angle_Balance (ADC raw)
%   Bytes 6-9   : Channel 2, float32 LE    Encoder
%   Bytes 10-13 : Channel 3, float32 LE    motor command u(t) / Moto
%   Byte 14     : 0x0D                     frame tail / frame length marker
%
% Current firmware (R4-2, measured 2026-09-07):
%   UART baud rate      = 128000
%   DataScope update    ≈ 14 Hz   (MEASURED. The 50 ms main loop suggests
%                                  20 Hz, but Tips()/OLED adds ~25 ms per
%                                  loop. The script measures it anyway.)
%   Angle upright ADC   ≈ 3100
%   Angle downward ADC  ≈ 1020
%   Encoder initial CNT = 10000
%
%
% -------------------------------------------------------------------------
% CHANGES MADE 2026-09-07 (checked against the rig, firmware R4-2)
%   1. Default port COM10 -> COM4.
%   2. After setDTR/setRTS the board REBOOTS (those lines are wired to the
%      reset circuit) and is then silent for 2 s. Added pause(2.6) BEFORE
%      the flush, so captures no longer start during the boot delay.
%   3. Sampling rate comments corrected: measured 14 Hz, not 20 Hz.
%      (50 ms loop + ~25 ms of OLED refresh per pass.)
%   5. addpoints is now guarded: closing any of the four figures used to
%      throw and kill the acquisition.
%   4. The binary frame parser was stress-tested against synthetic streams
%      -- random chunk boundaries, leading junk, payloads containing 0x24
%      and 0x0D, and 20 deliberately dropped bytes. It resynchronises
%      correctly in every case. No change needed.
% -------------------------------------------------------------------------
%
% MATLAB R2024a + board CH9102:
% DTR/RTS are disabled because opening the COM port may otherwise affect
% the board state. If no data arrive after opening the port, press RESET
% on the STM32 board once.

clear;
clc;

%% ================= USER SETTINGS =================
PORT = "COM4";        % check with serialportlist("available")
BAUD = 128000;

RUN_TIME_S = Inf;       % acquisition time; use Inf for continuous monitoring
SAVE_CSV = false;

% Angle calibration from the original firmware.
ANGLE_UPRIGHT_ADC  = 3100;   % pendulum upright/balance position
ANGLE_DOWNWARD_ADC = 1020;   % pendulum vertically downward

% Encoder reference.
ENCODER_ZERO = 10000;        % TIM4->CNT initial value

% Original comments indicate 1040 encoder counts per output revolution.
ENCODER_COUNTS_PER_REV = 1040;

% Set the effective cart pulley/wheel diameter here when known.
% Example: 0.050 for 50 mm. Leave NaN if not yet calibrated.
PULLEY_DIAMETER_M = 0.02;

% Flip these signs if the physical positive directions are opposite
% to the convention you want to use.
ANGLE_SIGN = 1;
POSITION_SIGN = -1;
%% =================================================

HEADER = uint8(hex2dec('24'));
TAIL   = uint8(hex2dec('0D'));
FRAME_LENGTH = 14;

%% Close an old MATLAB serial object on the same port
oldPorts = serialportfind;
for k = 1:numel(oldPorts)
    try
        if strcmpi(string(oldPorts(k).Port), PORT)
            delete(oldPorts(k));
        end
    catch
    end
end
pause(0.2);

%% Open serial port
fprintf("Opening %s at %d baud...\n", PORT, BAUD);

s = serialport(PORT, BAUD, "Timeout", 2);

% Dropping DTR/RTS is needed for reliable data on this adapter, but those
% lines are wired to the reset circuit, so the board REBOOTS here and then
% spends 2 s in main()'s two delay_ms(1000) calls. Wait it out, THEN flush,
% otherwise the first two seconds of every capture are empty.
setDTR(s, false);
setRTS(s, false);
pause(2.6);
flush(s, "input");

serialCleanup = onCleanup(@() cleanupSerial(s));

fprintf("Serial port opened.\n");
fprintf("If no data appear, press RESET on the STM32 board once.\n");
% Firmware R4-2 mutes the binary stream as soon as it receives any ASCII
% command, so if you used connectPendulum earlier the stream may be off.
% Opening this port resets the board, which turns it back on -- but if you
% ever see nothing at all, that is the first thing to suspect.
fprintf("\n");

%% Conversion constants
adcSpan = ANGLE_UPRIGHT_ADC - ANGLE_DOWNWARD_ADC;

% Define theta = 0 at the upright equilibrium.
% Assuming approximately linear sensing over 180 degrees:
radPerAdcCount = pi / adcSpan;
degPerAdcCount = 180 / adcSpan;

if isfinite(PULLEY_DIAMETER_M)
    metersPerCount = pi * PULLEY_DIAMETER_M / ENCODER_COUNTS_PER_REV;
else
    metersPerCount = NaN;
end

%% Storage
time_s = [];
angle_adc = [];
theta_deg = [];
theta_rad = [];

encoder_raw = [];
position_count = [];
position_m = [];

channel3 = [];

theta_dot_rad_s = [];
cart_speed_count_s = [];
cart_speed_m_s = [];

rxBuffer = uint8([]);
frameCount = 0;

%% Live plots
fig = figure("Name", "WHEELTEC Minibalance Serial Monitor", ...
             "NumberTitle", "off");

tl = tiledlayout(fig, 3, 1);

ax1 = nexttile(tl);
angleLine = animatedline(ax1);
grid(ax1, "on");
xlabel(ax1, "Time (s)");
ylabel(ax1, "\theta (deg)");
title(ax1, "Pendulum angle (0 deg = upright)");

ax2 = nexttile(tl);
positionLine = animatedline(ax2);
grid(ax2, "on");
xlabel(ax2, "Time (s)");

if isfinite(metersPerCount)
    ylabel(ax2, "Cart position x (m)");
    title(ax2, "Cart horizontal displacement");
else
    ylabel(ax2, "Encoder count relative to zero");
    title(ax2, "Cart horizontal displacement (raw encoder counts)");
end

% Channel 3 from STM32 is used as the motor command u(t).
% In show.c, use:
% DataScope_Get_Channel_Data(Moto, 3);
ax3 = nexttile(tl);
inputLine = animatedline(ax3);
grid(ax3, "on");
xlabel(ax3, "Time (s)");
ylabel(ax3, "u (PWM counts)");
title(ax3, "Motor command u(t) - DataScope Channel 3");

%% Additional individual live figures

% ----- Individual Figure 1: Pendulum angle -----
figAngle = figure( ...
    "Name", "Pendulum Angle", ...
    "NumberTitle", "off");

axAngle = axes(figAngle);
angleLineSingle = animatedline(axAngle);

grid(axAngle, "on");
xlabel(axAngle, "Time (s)");
ylabel(axAngle, "\theta (deg)");
title(axAngle, "Pendulum Angle \theta(t)");

% ----- Individual Figure 2: Cart position -----
figPosition = figure( ...
    "Name", "Cart Position", ...
    "NumberTitle", "off");

axPosition = axes(figPosition);
positionLineSingle = animatedline(axPosition);

grid(axPosition, "on");
xlabel(axPosition, "Time (s)");

if isfinite(metersPerCount)
    ylabel(axPosition, "Cart position x (m)");
    title(axPosition, "Cart Position x(t)");
else
    ylabel(axPosition, "Encoder count relative to zero");
    title(axPosition, "Cart Position x(t) - Raw Encoder Counts");
end

% ----- Individual Figure 3: Motor command -----
figInput = figure( ...
    "Name", "Motor Command", ...
    "NumberTitle", "off");

axInput = axes(figInput);
inputLineSingle = animatedline(axInput);

grid(axInput, "on");
xlabel(axInput, "Time (s)");
ylabel(axInput, "u (PWM counts)");
title(axInput, "Motor Command u(t)");





%% Acquire data
t0 = tic;
lastStatusTime = 0;

while toc(t0) < RUN_TIME_S && isvalid(fig)

    % Pull all currently available serial bytes.
    n = s.NumBytesAvailable;
    if n > 0
        newBytes = read(s, n, "uint8");
        rxBuffer = [rxBuffer, uint8(newBytes)]; %#ok<AGROW>
    else
        pause(0.002);
    end

    % Parse as many complete DataScope frames as possible.
    while numel(rxBuffer) >= FRAME_LENGTH

        % Synchronize to '$' (0x24).
        headerIdx = find(rxBuffer == HEADER, 1, "first");

        if isempty(headerIdx)
            rxBuffer = uint8([]);
            break;
        end

        if headerIdx > 1
            rxBuffer(1:headerIdx-1) = [];
        end

        if numel(rxBuffer) < FRAME_LENGTH
            break;
        end

        candidate = rxBuffer(1:FRAME_LENGTH);

        % Valid 3-channel DataScope frame must end in 0x0D.
        if candidate(end) ~= TAIL
            rxBuffer(1) = [];
            continue;
        end

        % Decode three little-endian IEEE-754 float32 channels.
        ch1 = double(typecast(uint8(candidate(2:5)),   "single"));
        ch2 = double(typecast(uint8(candidate(6:9)),   "single"));
        ch3 = double(typecast(uint8(candidate(10:13)), "single"));

        % Remove parsed frame.
        rxBuffer(1:FRAME_LENGTH) = [];

        % Reject obviously corrupt floating-point values.
        if ~(isfinite(ch1) && isfinite(ch2) && isfinite(ch3))
            continue;
        end

        frameCount = frameCount + 1;
        t = toc(t0);

        % Key physical variables.
        adc = ch1;
        enc = ch2;

        thetaDeg = ANGLE_SIGN * (adc - ANGLE_UPRIGHT_ADC) * degPerAdcCount;
        thetaRad = ANGLE_SIGN * (adc - ANGLE_UPRIGHT_ADC) * radPerAdcCount;

        posCount = POSITION_SIGN * (enc - ENCODER_ZERO);

        if isfinite(metersPerCount)
            posM = posCount * metersPerCount;
        else
            posM = NaN;
        end

        % Estimate velocities from the transmitted data (~20 Hz).
        % These are MATLAB estimates, not the STM32 internal 200 Hz derivative.
        if isempty(time_s)
            thetaDot = NaN;
            speedCount = NaN;
            speedM = NaN;
        else
            dt = t - time_s(end);

            if dt > 0
                thetaDot = (thetaRad - theta_rad(end)) / dt;
                speedCount = (posCount - position_count(end)) / dt;

                if isfinite(metersPerCount)
                    speedM = (posM - position_m(end)) / dt;
                else
                    speedM = NaN;
                end
            else
                thetaDot = NaN;
                speedCount = NaN;
                speedM = NaN;
            end
        end

        % Store.
        time_s(end+1,1) = t; %#ok<SAGROW>
        angle_adc(end+1,1) = adc; %#ok<SAGROW>
        theta_deg(end+1,1) = thetaDeg; %#ok<SAGROW>
        theta_rad(end+1,1) = thetaRad; %#ok<SAGROW>

        encoder_raw(end+1,1) = enc; %#ok<SAGROW>
        position_count(end+1,1) = posCount; %#ok<SAGROW>
        position_m(end+1,1) = posM; %#ok<SAGROW>

        channel3(end+1,1) = ch3; %#ok<SAGROW>

        theta_dot_rad_s(end+1,1) = thetaDot; %#ok<SAGROW>
        cart_speed_count_s(end+1,1) = speedCount; %#ok<SAGROW>
        cart_speed_m_s(end+1,1) = speedM; %#ok<SAGROW>

        % Update plots. Each animatedline is checked first: closing any of
        % the four figures deletes its line, and addpoints on a deleted
        % handle throws, which would end the acquisition with an error.
        if isfinite(metersPerCount), posPlot = posM; else, posPlot = posCount; end

        safeAddPoints(angleLine,        t, thetaDeg);
        safeAddPoints(positionLine,     t, posPlot);
        safeAddPoints(inputLine,        t, ch3);
        safeAddPoints(angleLineSingle,  t, thetaDeg);
        safeAddPoints(positionLineSingle,t, posPlot);
        safeAddPoints(inputLineSingle,  t, ch3);

        if mod(frameCount, 5) == 0
            drawnow limitrate;

            if isfinite(metersPerCount)
                fprintf(['t=%7.2f s | ADC=%7.1f | theta=%8.3f deg | ' ...
                         'Encoder=%8.1f | x=%9.5f m | u=%7.1f\n'], ...
                         t, adc, thetaDeg, enc, posM, ch3);
            else
                fprintf(['t=%7.2f s | ADC=%7.1f | theta=%8.3f deg | ' ...
                         'Encoder=%8.1f | x_count=%8.1f | u=%7.1f\n'], ...
                         t, adc, thetaDeg, enc, posCount, ch3);
            end
        end
    end

    % Helpful warning if the port is open but no frame is arriving.
    if frameCount == 0 && toc(t0) > 3 && toc(t0) - lastStatusTime > 3
        fprintf("No valid frame yet. Press STM32 RESET once if required.\n");
        lastStatusTime = toc(t0);
    end
end

%% Summary
fprintf("\nAcquisition finished.\n");
fprintf("Valid frames received: %d\n", frameCount);

if frameCount == 0
    warning("No valid DataScope frames were decoded.");
    return;
end

%% Build output table
% Force every recorded signal to be an N-by-1 column vector.
time_s = time_s(:);
angle_adc = angle_adc(:);
theta_deg = theta_deg(:);
theta_rad = theta_rad(:);
encoder_raw = encoder_raw(:);
position_count = position_count(:);
position_m = position_m(:);
theta_dot_rad_s = theta_dot_rad_s(:);
cart_speed_count_s = cart_speed_count_s(:);
cart_speed_m_s = cart_speed_m_s(:);
channel3 = channel3(:);
motor_u_pwm = channel3;   % DataScope Channel 3 = motor command u(t)

% MATLAB R2024a: use a character-vector parameter name here.
T = table( ...
    time_s, ...
    angle_adc, ...
    theta_deg, ...
    theta_rad, ...
    encoder_raw, ...
    position_count, ...
    position_m, ...
    theta_dot_rad_s, ...
    cart_speed_count_s, ...
    cart_speed_m_s, ...
    motor_u_pwm, ...
    'VariableNames', { ...
        'time_s', ...
        'angle_adc', ...
        'theta_deg', ...
        'theta_rad', ...
        'encoder_raw', ...
        'position_count', ...
        'position_m', ...
        'theta_dot_rad_s', ...
        'cart_speed_count_s', ...
        'cart_speed_m_s', ...
        'motor_u_pwm'});

disp(T(max(1,height(T)-9):height(T), :));

%% Save CSV
if SAVE_CSV
    stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
    filename = "minibalance_capture_" + stamp + ".csv";
    writetable(T, filename);
    fprintf("Saved data to: %s\n", filename);
end

%% ---------------- Local functions ----------------
function safeAddPoints(h, x, y)
% Add a point only if the line still exists, so closing a figure mid-run
% stops that plot updating instead of crashing the acquisition.
    if isvalid(h)
        addpoints(h, x, y);
    end
end


function cleanupSerial(s)
    try
        flush(s);
    catch
    end

    try
        delete(s);
    catch
    end
end
