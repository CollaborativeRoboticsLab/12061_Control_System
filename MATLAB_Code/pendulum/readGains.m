function [gains,measurement] = readGains(device)
%READGAINS Read the controller parameters currently live in the firmware.
%
%   VALUES, BY POSITION
%     1 Balance_KP    angle loop, proportional
%     2 Balance_KD    angle loop, derivative
%     3 Balance_KI    angle loop, integral      (added in R4-5)
%     4 Position_KP   cart loop, proportional
%     5 Position_KD   cart loop, derivative
%     6 Position_KI   cart loop, integral       (added in R4-6)
%     7 Angle_Zero    angle setpoint, ADC counts (added in R4-6)
%
%   Angle_Zero is not a gain, but it belongs here: it is a controller
%   parameter that changes the closed-loop equilibrium, and a capture is
%   not interpretable without it. A residual angle offset means nothing
%   unless you know what the loop was aiming at.
%
%   THIS IS ALSO HOW LAB 5 LEARNS ITS BASELINE
%   Opening the serial port resets the board, so the gains always come back
%   to the values compiled into Minibalance.c. Reading them here, once,
%   straight after connecting, makes that file the single source of truth:
%   change the baseline by editing those four numbers and reflashing, and
%   the lab script needs no edit at all.
%
%   TOLERANT BY DESIGN
%   Requires AT LEAST five values and ignores any beyond the ones it knows,
%   so this one file works against R4-5 (five values) and R4-6 (seven). The
%   old version anchored the end of the line and matched exactly five,
%   which meant every firmware revision that appended a value broke it and
%   had to be special-cased. Fields are addressed by position, so the
%   protocol is append-only: never reorder, never remove.

[reply,measurement] = pendulumCommand(device,"K",0.50,2,["K="]);

parts = split(extractAfter(reply,"K="),",");
n     = numel(parts);

if n < 4
    error("STM32:MalformedGains","Malformed K response: %s",reply);
end

v = str2double(parts);
if any(isnan(v))
    error("STM32:MalformedGains","Non-numeric value in K response: %s",reply);
end

if n < 5
    % Four values means a pure-PD angle loop: R4-4 or older. Report it as
    % the firmware-version problem it is, because the fix is to reflash,
    % not to debug MATLAB.
    error("STM32:FirmwareTooOld", ...
        ['The board returned four gains, so it is running R4-4 or older ' ...
         'and its angle loop is pure PD.\nLab 4 needs the integral term ' ...
         'added in R4-5, Lab 5 needs R4-6. Rebuild and upload ' ...
         'source_code/ in VS Code.']);
end

gains = struct( ...
    "Balance_KP",  v(1), ...
    "Balance_KD",  v(2), ...
    "Balance_KI",  v(3), ...
    "Position_KP", v(4), ...
    "Position_KD", v(5), ...
    "Position_KI", NaN, ...   % R4-6
    "Angle_Zero",  NaN, ...   % R4-6
    "NumValues",   n);

if n >= 6, gains.Position_KI = v(6); end
if n >= 7, gains.Angle_Zero  = v(7); end
end
