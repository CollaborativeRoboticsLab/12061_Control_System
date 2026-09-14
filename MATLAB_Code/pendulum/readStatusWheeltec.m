function status = readStatusWheeltec(device)
%READSTATUSWHEELTEC Decode STATUS= with the WHEELTEC field names.
%
%   Voltage arrives in hundredths of a volt (1184 = 11.84 V) and is
%   converted here.
%
%   The wire format is unchanged (9 comma-separated integers, first one
%   signed), so the older readStatus.m still parses it. Only the field
%   NAMES differ: the bluepill firmware reported UART error counters,
%   this firmware reports the pendulum state.

[reply,measurement] = pendulumCommand(device,"STATUS",0.50,2,["STATUS="]);

tokens = regexp(reply, ...
    "^STATUS=(-?\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)$", ...
    "tokens","once");

if isempty(tokens)
    error("STM32:MalformedStatus","Malformed STATUS response: %s",reply);
end

v = str2double(tokens);

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
    "RoundTripSeconds", measurement.RoundTripSeconds, ...
    "RawReply",         reply);
end
