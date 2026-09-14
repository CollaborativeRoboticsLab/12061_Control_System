function measurement = resetIntegrator(device)
%RESETINTEGRATOR Zero the angle-loop integral accumulator (firmware R4-5+).
%
%   resetIntegrator(s)
%
%   WHY YOU NEED THIS BETWEEN TRIALS
%   The integrator keeps accumulating whenever Ki is non-zero. A trial that
%   ended with the rod lying on the bench leaves a large wound-up value, so
%   the NEXT trial would begin with a step of integral action that has
%   nothing to do with the gains you are testing -- and you would blame the
%   gains. Call this before every Lab 4 run that uses Ki.
%
%   It does not touch any gain: only the accumulator is cleared. The
%   firmware also clears it automatically on a stop and when balancing is
%   re-enabled, so this is for the case where neither has happened.

arguments
    device
end

[reply,measurement] = pendulumCommand(device,"IRESET",0.50,2,["IRESET="]);

if strtrim(reply) ~= "IRESET=0"
    error("STM32:MalformedIReset","Unexpected IRESET response: %s",reply);
end
end
