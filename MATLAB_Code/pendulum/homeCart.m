function pos = homeCart(s,opts)
%HOMECART Tell the firmware where the cart is, so the rail limits mean
%         something. One command, no motor travel.
%
%   homeCart(s)                      % prompts you to centre the cart
%   homeCart(s,"Confirm",false)      % cart is already centred, just do it
%   homeCart(s,"Force",true)         % re-home even if it was homed before
%
%   WHY THIS IS NEEDED
%   The encoder is relative. Encoder_Init_TIM4 writes TIM4->CNT = 10000 at
%   every reset, wherever the cart physically happens to be, and connecting
%   resets the board (dropping DTR/RTS does that). So straight after
%   connectPendulum the cart ALWAYS reads 10000, and every absolute number
%   baked into the WHEELTEC firmware is measuring from an origin nobody
%   chose:
%
%       POSITION_MIDDLE 7925    the balance controller's position setpoint
%       5900 and 9900           the edge protection in Auto_run
%       the Auto_run offsets    POSITION_MIDDLE-668, +800, -482, ...
%
%   HOME declares the origin instead of trying to deduce it. You slide the
%   cart to the middle of the rail by hand -- two seconds, no motor, no
%   fighting static friction -- and the firmware writes the count to 7925
%   and arms the limits.
%
%   WHY NOT DRIVE THERE WITH THE MOTOR
%   That was the first version and it was worse in every way. It had to
%   assume which PWM sign moves the cart which way, and if that assumption
%   were ever wrong the firmware's own end-stop test would refuse exactly
%   the direction that escapes the limit -- the cart would be stuck with no
%   way out over the serial link. It also had to hunt across the actuator
%   deadband, which on this rig sits somewhere between PWM 900 and 1500, so
%   it took 10 to 50 pulses and sometimes gave up. Declaring the origin
%   removes all of that.
%
%   WHAT CHANGES AFTERWARDS
%   Before HOME the firmware skips the limit test entirely (an unknown
%   frame cannot be tested, only guessed at) and caps every M command to
%   PWM 2500 for 300 ms, so the cart can be jogged but cannot build up a
%   run. After HOME the full +/-6900 range is available and the directional
%   end-stop guard is live.
%
%   Needs firmware R4-4 or later.

arguments
    s
    opts.Confirm (1,1) logical = true
    opts.Force   (1,1) logical = false
    opts.Verbose (1,1) logical = true
    opts.Target  (1,1) double  = 7925   % UARTCMD_ENC_HOME / POSITION_MIDDLE
    opts.Tol     (1,1) double {mustBePositive} = 200
end

% Already done? Re-homing mid-experiment would silently move the balance
% controller's setpoint, so it is not something to do by accident.
alreadyHomed = false;
try
    r = pendulumCommand(s,"HOMED",0.50,3,["HOMED="]);
    alreadyHomed = strtrim(r) == "HOMED=1";
catch
    error("Lab:FirmwareTooOld", ...
        ['This board does not understand HOMED, so it is running ' ...
         'firmware older than R4-4. Rebuild and upload ' ...
         'Control_System_lab_1 first -- on the old build the cart ' ...
         'position is meaningless and the end-stop guard either ' ...
         'refuses everything or protects nothing.']);
end

if alreadyHomed && ~opts.Force
    pos = readPositionWheeltec(s);
    if opts.Verbose
        fprintf("Already homed; cart reads %d counts.\n",pos);
        if abs(pos-opts.Target) > opts.Tol
            fprintf("  It has since travelled %+d counts from the middle.\n", ...
                pos-opts.Target);
            fprintf("  That is expected after an experiment. Only re-home\n");
            fprintf("  (homeCart(s,""Force"",true)) if you have moved the\n");
            fprintf("  cart by hand or reset the board.\n");
        end
    end
    return;
end

if opts.Confirm
    fprintf("\n===== Home the cart =====\n");
    fprintf("Slide the cart by hand to the MIDDLE of the rail, as close to\n");
    fprintf("halfway between the two end stops as you can judge by eye.\n");
    fprintf("A centimetre either way does not matter -- the rail is about\n");
    fprintf("4000 counts end to end, and the limits keep 100 counts of\n");
    fprintf("margin at each end.\n\n");
    fprintf("The motor is off and will stay off during this step.\n");
    if ~strcmpi(strtrim(string(input("Cart centred? Type y and Enter: ","s"))),"y")
        pos = readPositionWheeltec(s);
        fprintf("Cancelled. Not homed; cart still reads %d.\n",pos);
        return;
    end
end

reply = pendulumCommand(s,"HOME",0.50,3,["HOME="]);
tok   = regexp(strtrim(reply),"^HOME=(-?\d+)$","tokens","once");
if isempty(tok)
    error("Lab:HomeFailed","Board answered HOME with: %s",reply);
end
declared = str2double(tok{1});

% Read it back rather than trusting the acknowledgement. If TIM4->CNT did
% not take the write, everything downstream would be quietly wrong.
pause(0.05);
pos = readPositionWheeltec(s);
if abs(pos-declared) > 10
    error("Lab:HomeNotApplied", ...
        ['HOME acknowledged %d but the cart now reads %d. The encoder ' ...
         'count did not take the write, so the rail limits are still ' ...
         'not trustworthy.'],declared,pos);
end

if opts.Verbose
    fprintf("\nHomed.\n");
    fprintf("  cart position : %d counts (declared, not measured)\n",pos);
    fprintf("  rail ends     : %d and %d, about %d counts of travel\n", ...
        5900,9900,9900-5900);
    fprintf("  end-stops     : armed\n");
    fprintf("  motor range   : full +/-6900 (the nudge cap is lifted)\n\n");
end
end
