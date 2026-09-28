function plotTrials(rs,titleText)
%PLOTTRIALS Overlay trials, aligned on the kick: angle, cart and motor command.
%
%   plotTrials(rs,"Activity 2 - Kp")
%
%   Angle and cart are plotted as deviations from their pre-kick values, so
%   different starting offsets do not look like different controllers. The
%   kick is marked, and the dotted band is the recovery band.

figure("Name",titleText,"NumberTitle","off");
tl = tiledlayout(3,1,"TileSpacing","compact");  title(tl,titleText);
k0 = rs(1).kickMs/1000;  band = rs(1).band;

ax1 = nexttile;  hold on;
for r = rs, plot(r.t - r.tKick, r.dev, "LineWidth",1.3); end
yline([-band band],"k:");  ylabel("\theta - pre-kick (deg)");
ax2 = nexttile;  hold on;
for r = rs, plot(r.t - r.tKick, r.x - r.x0, "LineWidth",1.1); end
ylabel("cart (mm)");
ax3 = nexttile;  hold on;
for r = rs, stairs(r.t - r.tKick, r.u); end
yline([-6900 6900],"r--");  ylabel("u (PWM)");  xlabel("Time from the kick (s)");

for ax = [ax1 ax2 ax3], xline(ax,[0 k0],"r:");  grid(ax,"on"); end
linkaxes([ax1 ax2 ax3],"x");
legend(ax1,[rs.label],"Location","best","Interpreter","none");
end
