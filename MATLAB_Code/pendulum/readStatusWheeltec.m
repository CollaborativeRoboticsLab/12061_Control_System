function status = readStatusWheeltec(device)
%READSTATUSWHEELTEC Decode STATUS= with the WHEELTEC field names.
%
%   Voltage arrives in hundredths of a volt (1184 = 11.84 V) and is
%   converted here.
%
%   FIELDS, BY POSITION
%     1 Moto          current PWM command, -6900..6900
%     2 Encoder       cart position, TIM4 counts
%     3 AngleRaw      pendulum ADC counts, 0..4095
%     4 Voltage       battery, hundredths of a volt
%     5 FlagStop      1 = motor disabled
%     6 ManualMode    1 = PWM is coming from the host
%     7 AutoRun       1 = automatic swing-up selected
%     8 StreamEnable  1 = binary DataScope frames are being sent
%     9 RxErrors      dropped bytes
%    10 KickTicks     R4-6: 5 ms units of disturbance still to run
%
%   WHY THIS PARSER IS DELIBERATELY TOLERANT
%   It used to match exactly nine fields with the end of the line anchored:
%
%       "^STATUS=(-?\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)$"
%
%   That anchor meant a board sending a TENTH field failed to match at all,
%   so one extra field threw away the nine this function already understood
%   and the whole call became "Malformed STATUS response". R4-6 appends
%   exactly such a field, and a future revision may append another.
%
%   So: require AT LEAST nine, read the ones we know, ignore the rest. A
%   genuinely short or non-numeric reply is still rejected -- tolerant is
%   not the same as uncritical.
%
%   THE PRICE, AND THE RULE THAT PAYS IT
%   Fields are addressed by POSITION, so a firmware that reordered them
%   would be read silently wrong rather than loudly rejected. The protocol
%   is therefore append-only: new fields go on the end, existing ones never
%   move and never disappear. VER / connectPendulum's version check is the
%   backstop if that rule is ever broken.

[reply,measurement] = pendulumCommand(device,"STATUS",0.50,2,["STATUS="]);

parts = split(extractAfter(reply,"STATUS="),",");
n     = numel(parts);

if n < 9
    error("STM32:MalformedStatus", ...
        "STATUS needs at least 9 fields, got %d: %s",n,reply);
end

v = str2double(parts);
if any(isnan(v))
    error("STM32:MalformedStatus", ...
        "Non-numeric field in STATUS response: %s",reply);
end

status = struct( ...
    "Moto",             v(1), ...   % current PWM, -6900..6900
    "Encoder",          v(2), ...   % cart position, TIM4 counts
    "AngleRaw",         v(3), ...   % pendulum ADC, 0..4095
    "VoltageVolts",     v(4)/100, ...   % firmware sends hundredths of a volt
    "FlagStop",         logical(v(5)), ...
    "ManualMode",       logical(v(6)), ...
    "AutoRun",          logical(v(7)), ...
    "StreamEnable",     logical(v(8)), ...
    "RxErrors",         v(9), ...
    "KickTicks",        NaN, ...    % filled in below if the board sends it
    "NumFields",        n, ...
    "RoundTripSeconds", measurement.RoundTripSeconds, ...
    "RawReply",         reply);

% R4-6 and later. On R4-5 this stays NaN, which is the honest answer: that
% firmware has no disturbance mechanism at all, so there is no tick count
% to report. Lab 1-4 never look at it.
if n >= 10
    status.KickTicks = v(10);
end
end
