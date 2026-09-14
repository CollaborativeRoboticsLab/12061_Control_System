function measurement = setGain(device,name,value)
%SETGAIN Change one controller gain at run time, without reflashing.
%
%   name is one of:
%       "BKP"  Balance_KP    (default 400)   angle proportional
%       "BKD"  Balance_KD    (default 400)   angle derivative
%       "BKI"  Balance_KI    (default   0)   angle integral, added in R4-5
%       "PKP"  Position_KP   (default  20)   cart proportional
%       "PKD"  Position_KD   (default 300)   cart derivative
%
%   The firmware stores these as float but the command carries an
%   integer, so value is rounded. Range is clamped to +/-20000.

arguments
    device
    name  (1,1) string {mustBeMember(name,["BKP","BKD","BKI","PKP","PKD"])}
    value (1,1) double {mustBeFinite}
end

value   = round(value);
command = "GAIN," + name + "," + string(value);

[reply,measurement] = pendulumCommand(device,command,0.50,2,["GAIN="]);

expected = "GAIN=" + name + "," + string(value);
if reply ~= expected
    error("STM32:MalformedGain", ...
        "Expected %s but received %s.",expected,reply);
end
end
