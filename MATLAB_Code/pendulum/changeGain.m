function g = changeGain(g,name,value)
%CHANGEGAIN Copy a gain set and change exactly ONE angle-loop gain.
%
%   g2 = changeGain(g1,"Kp",500)     name is "Kp", "Ki" or "Kd"
%
%   Changing one gain per trial is the lab rule, so this is the only way the
%   Lab 6 script builds a new controller: each trial is its predecessor with
%   a single gain altered.

arguments
    g struct
    name (1,1) string {mustBeMember(name,["Kp","Ki","Kd"])}
    value (1,1) double
end
if isnan(value)
    error("Lab:GainNotSet","The new %s is not set yet (NaN). Fill in the TODO first.",name);
end
field = "Balance_" + upper(name);
fprintf("  %s: %g -> %g\n",name,g.(field),value);
g.(field) = value;
end
