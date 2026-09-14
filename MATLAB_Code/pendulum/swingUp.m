function measurement = swingUp(device)
%SWINGUP Start the firmware's automatic energy swing-up (Auto_run).
%
%   Resets autorun_step0/1/2, success_flag, success_count and wait_count
%   to their power-on values, then releases the stop flag.
%
%   Keep clear of the rail. The cart accelerates hard.

[reply,measurement] = pendulumCommand(device,"SWING",0.50,2,["SWING="]);

if reply ~= "SWING=1"
    error("STM32:MalformedSwing","Unexpected SWING response: %s",reply);
end
end
