az = readGains(s).Angle_Zero;            %  cfg.AZ
figure; h = animatedline("LineWidth",1.2); grid on;
ylabel("\theta (deg)"); xlabel("Time (s)");
t0 = tic;
while toc(t0) < 30                        % 30 s,
    a = readAngleWheeltec(s);
    t = toc(t0);
    addpoints(h, t, (a - az)*180/2080);   % angle
    xlim([max(0,t-10) max(10,t)]);        %  10 s
    drawnow limitrate
end