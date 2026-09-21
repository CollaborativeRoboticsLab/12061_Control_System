function measurement = setGain(device,name,value)
%SETGAIN Change one controller parameter at run time, without reflashing.
%
%   name is one of:
%       "BKP"  Balance_KP    (default 400)   angle proportional
%       "BKD"  Balance_KD    (default 400)   angle derivative
%       "BKI"  Balance_KI    (default   0)   angle integral,  R4-5
%       "PKP"  Position_KP   (default  20)   cart proportional
%       "PKD"  Position_KD   (default 300)   cart derivative
%       "PKI"  Position_KI   (default   0)   cart integral,   R4-6
%       "AZ"   Angle_Zero    (default 3100)  angle SETPOINT,  R4-6
%
%   AZ IS NOT A GAIN. It is the angle the controller aims at, in raw ADC
%   counts, and it is the right tool for a rig whose pendulum potentiometer
%   sits a degree or two out of true. Such a rig shows a permanent rod lean
%   AND a permanent cart offset, and no gain can remove either: the loop is
%   faithfully holding the rod where it was told to, and the cart has to
%   sit off-centre to balance the books. Trim AZ instead.
%
%   The firmware refuses an AZ outside 500..3600 rather than clamping it,
%   because its motor-enable window is only +/-500 counts wide around this
%   value -- a typo would look exactly like dead hardware. A refusal
%   arrives here as STM32:FirmwareError carrying ERR=RANGE.
%
%   Changing AZ also clears the angle integrator in firmware: whatever it
%   had accumulated was measured against the old target.
%
%   The firmware stores these as float but the command carries an integer,
%   so value is rounded. Gains are clamped to +/-20000.

arguments
    device
    name  (1,1) string {mustBeMember(name, ...
              ["BKP","BKD","BKI","PKP","PKD","PKI","AZ"])}
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
