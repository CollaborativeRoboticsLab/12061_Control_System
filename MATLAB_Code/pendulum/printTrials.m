function T = printTrials(rs)
%PRINTTRIALS Print trials as the handout's table, and return it as a table.
%
%   printTrials(rs)       rs is a struct array from kickTrial
%   T = printTrials(rs)   also returns a MATLAB table (for writetable)
%
%   Osc  = number of excursions outside the recovery band; * = sustained.
%   Spec = your meetsSpec result (- if not checked).

fprintf("\n%-14s %5s %4s %5s | %6s %7s %7s %4s %6s %4s | %s\n", ...
    "Trial","Kp","Ki","Kd","Peak","Recov","Max|x|","Osc","Stable","Spec","Why / decision");
fprintf("%-14s %5s %4s %5s | %6s %7s %7s %4s %6s %4s |\n","","","","","(deg)","(s)","(m)","","","");
fprintf("%s\n",repmat('-',1,96));
for r = rs
    if isnan(r.recoverS), rec = "--"; else, rec = sprintf("%.2f",r.recoverS); end
    osc = sprintf("%d",r.nOsc);  if r.sustained, osc = osc + "*"; end
    if isnan(r.pass), sp = "-"; elseif r.pass, sp = "Y"; else, sp = "N"; end
    fprintf("%-14s %5g %4g %5g | %6.2f %7s %7.3f %4s %6s %4s | %s\n", ...
        r.label,r.gains.Balance_KP,r.gains.Balance_KI,r.gains.Balance_KD, ...
        r.peakDeg,rec,r.cartTravelMm/1000,osc,yn(r.stable),sp,r.decision);
end
fprintf("Peak from the pre-kick mean; recovery from the END of the kick; Max|x| = cart travel caused by the kick.\n");

if nargout
    T = table([rs.label]',arrayfun(@(r) r.gains.Balance_KP,rs)',arrayfun(@(r) r.gains.Balance_KI,rs)', ...
        arrayfun(@(r) r.gains.Balance_KD,rs)',[rs.peakDeg]',[rs.recoverS]',[rs.cartTravelMm]'/1000, ...
        [rs.nOsc]',[rs.sustained]',[rs.stable]',[rs.pass]',[rs.effortRms]',[rs.satPct]', ...
        [rs.residualDeg]',string({rs.decision}'), ...
        'VariableNames',{'trial','Kp','Ki','Kd','peak_deg','recovery_s','max_x_m','n_osc', ...
                         'sustained','stable','meets_spec','effort_rms','sat_pct','residual_deg','decision'});
end
end

function t = yn(b)
    if b, t = "Y"; else, t = "N"; end
end
