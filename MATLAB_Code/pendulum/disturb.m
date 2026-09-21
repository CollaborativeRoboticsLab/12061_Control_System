function [appliedPWM,measurement] = disturb(device,pwm,durationMs)
%DISTURB Apply a repeatable disturbance to a running balance controller.
%
%   disturb(device,pwm)              pulse for the firmware default, 100 ms
%   disturb(device,pwm,durationMs)   pulse for durationMs, then it expires
%
%   This is the measurement instrument Lab 5 is built on. Every comparison
%   in that lab -- baseline against modified, trial against trial, and the
%   three-run repeatability check -- is only meaningful if the disturbance
%   is IDENTICAL every time. A hand push is not: the spread between two
%   pushes is wider than the spread between two controllers, so a
%   hand-pushed experiment measures the experimenter, not the gains.
%
%   WHAT THE FIRMWARE DOES WITH IT
%       Moto = Balance_Pwm - Position_Pwm + kick_pwm;
%
%   The pulse is ADDED to what the controller already decided, so the loop
%   stays closed for the whole event and the angle trace afterwards is a
%   recovery, not a fall.
%
%   WHY NOT setMotorPWM_wheeltec
%   That sends M, which puts the firmware in manual mode, and manual mode
%   REPLACES the balance loop rather than disturbing it. An M sent while
%   the rod is balancing simply drops it. The two commands look similar and
%   do opposite things.
%
%   FAILURES YOU SHOULD EXPECT TO SEE
%     ERR=NOTBALANCING  no controller is running (Flag_Stop set, manual
%                       mode, or mid swing-up). Call setBalance(s,true)
%                       first, with the rod already near vertical.
%     ERR=ENDSTOP       the cart is at a rail end and this kick would push
%                       it further out. Kick the other way, or re-centre.
%   Both arrive as STM32:FirmwareError from pendulumCommand.
%
%   LIMITS, matching UARTCMD_KICK_* in usart_cmd.h: |pwm| <= 4000, and
%   duration <= 500 ms. Deliberately smaller than an M pulse. The point of
%   the experiment is the recovery; a disturbance large enough to saturate
%   the motor by itself tells you nothing about the gains.

arguments
    device
    pwm        (1,1) double {mustBeFinite}
    durationMs (1,1) double {mustBeFinite,mustBeNonnegative} = 100
end

KICK_PWM_MAX = 4000;     % must match UARTCMD_KICK_PWM_MAX
KICK_MAX_MS  = 500;      % must match UARTCMD_KICK_MAX_MS

pwm = round(pwm);
pwm = max(-KICK_PWM_MAX,min(KICK_PWM_MAX,pwm));

durationMs = round(durationMs);
durationMs = min(KICK_MAX_MS,durationMs);

command = "KICK," + string(pwm) + "," + string(durationMs);

[reply,measurement] = pendulumCommand(device,command,0.50,2,["KICK="]);

tokens = regexp(reply,"^KICK=(-?\d+)$","tokens","once");
if isempty(tokens)
    error("STM32:MalformedKick","Malformed KICK acknowledgement: %s",reply);
end

appliedPWM = str2double(tokens{1});

if appliedPWM ~= pwm
    warning("STM32:KickMismatch", ...
        "Requested KICK=%d but the board acknowledged KICK=%d.",pwm,appliedPWM);
end
end
