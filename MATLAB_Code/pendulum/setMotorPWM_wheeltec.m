function [appliedPWM,measurement] = setMotorPWM_wheeltec(device,requestedPWM,durationMs)
%SETMOTORPWM_WHEELTEC Open-loop motor command, WHEELTEC pendulum board.
%
%   setMotorPWM_wheeltec(device,pwm)
%       Runs the motor at PWM for the firmware default of 1000 ms.
%
%   setMotorPWM_wheeltec(device,pwm,durationMs)
%       Runs the motor at PWM for durationMs, then the firmware
%       watchdog zeroes it. This replaces the old compile-time
%       LAB1_TEST_MODE / LAB1_PWM / LAB1_PULSE_TICKS constants:
%
%           #define LAB1_PWM         2500
%           #define LAB1_PULSE_TICKS 60      % 60 x 5 ms
%
%       is exactly   setMotorPWM_wheeltec(device,2500,300).
%
%   WHY THIS IS A SEPARATE FILE, NOT A PATCH TO setMotorPWM.m
%       setMotorPWM.m talks to the bluepill test board, which is known
%       to work: PWM limit +/-999 (TIM3 ARR = 999, 1 kHz) and a two-field
%       "M,<pwm>" command. This board is different on both counts:
%       +/-6900 (ARR = 7199, 10 kHz, Xianfu_Pwm() keeps 4 % headroom) and
%       a three-field "M,<pwm>,<ms>". Overwriting the working file would
%       silently break the bluepill setup, so the two live side by side.
%
%   The firmware clamps as well, so a request outside the range comes
%   back as the clamped value and raises the usual mismatch warning.

arguments
    device
    requestedPWM (1,1) double {mustBeFinite}
    durationMs   (1,1) double {mustBeFinite,mustBeNonnegative} = 1000
end

PWM_LIMIT = 6900;      % must match UARTCMD_PWM_LIMIT in usart_cmd.h
MAX_MS    = 10000;     % must match UARTCMD_MAX_MS

requestedPWM = round(requestedPWM);
requestedPWM = max(-PWM_LIMIT,min(PWM_LIMIT,requestedPWM));

durationMs = round(durationMs);
durationMs = min(MAX_MS,durationMs);

command = "M," + string(requestedPWM) + "," + string(durationMs);

[reply,measurement] = pendulumCommand( ...
    device,command,0.50,2,["PWM="]);

tokens = regexp(reply,"^PWM=(-?\d+)$","tokens","once");

if isempty(tokens)
    error("STM32:MalformedPWM", ...
        "Malformed PWM acknowledgement: %s",reply);
end

appliedPWM = str2double(tokens{1});

if appliedPWM ~= requestedPWM
    warning("STM32:PWMMismatch", ...
        "Requested PWM=%d but STM32 acknowledged PWM=%d.", ...
        requestedPWM,appliedPWM);
end
end
