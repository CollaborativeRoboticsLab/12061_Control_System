function [volts,measurement] = readVoltage(device)
%READVOLTAGE Battery voltage in volts.
%
%   The firmware's Voltage variable is in HUNDREDTHS of a volt, not
%   millivolts: Get_battery_volt() computes adc*3.3*11*100/4096, and the
%   OLED prints it as Voltage/100 and Voltage%100. So 1184 means 11.84 V.
%   This function does that conversion, so it returns volts.

[reply,measurement] = pendulumCommand(device,"V",0.50,3,"VOLT=");

tokens = regexp(reply,"^VOLT=(-?\d+)$","tokens","once");
if isempty(tokens)
    error("STM32:MalformedVoltage","Malformed VOLT response: %s",reply);
end
volts = str2double(tokens{1})/100;

if volts < 7.0
    warning("STM32:NoBattery", ...
        "Battery reads %.2f V. Below 7.0 V the firmware's Turn_Off() forces " + ...
        "a stop, and the motor driver has no supply -- is the battery " + ...
        "connected, or is the board running on USB alone?",volts);
elseif volts < 10.5
    warning("STM32:LowBattery", ...
        "Battery is %.2f V. Charge it before expecting full motor torque.",volts);
end
end
