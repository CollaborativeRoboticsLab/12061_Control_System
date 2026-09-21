function device = connectPendulum(port,baudRate,opts)
%CONNECTPENDULUM Open the WHEELTEC pendulum and get it talking ASCII.
%
%   s = connectPendulum("COM4")
%   s = connectPendulum("COM4",128000,"SettleSeconds",3)
%
%   Use this instead of connectSTM32 for the pendulum rig. connectSTM32 is
%   left untouched -- it works for the bluepill board and is not the
%   problem here. This function exists because the pendulum needs two
%   things connectSTM32 does not do:
%
%   1. WAIT OUT THE REBOOT.
%      Dropping DTR/RTS is required for reliable data on this adapter, but
%      it also resets the STM32, and main() then spends 2.0 s in two
%      delay_ms(1000) calls before USART1 exists. connectSTM32 issues its
%      three PING attempts at roughly 0.8, 1.3 and 1.8 s after opening --
%      every one of them lands before the board is listening, so it always
%      times out on a first connect. Measured, not guessed: a capture right
%      after opening shows 1.82 s of silence, then frames.
%
%   2. MUTE THE BINARY STREAM FIRST.
%      Out of reset the firmware is sending 14-byte DataScope frames. Those
%      bytes sit in the buffer ahead of any reply, so the first readline
%      returns binary junk with "PONG" stuck on the end and the parser
%      rejects it. The firmware mutes the stream as soon as it parses any
%      valid ASCII command, so this function spends one throwaway PING to
%      trigger that, flushes, and only then starts the real handshake.
%
%   Returns a serialport object configured exactly like connectSTM32's, so
%   every existing helper (readAngle, setMotorPWM_wheeltec, ...) works with
%   it unchanged.
%
%   ONE THING TO KNOW ABOUT THE CART POSITION
%   The encoder is RELATIVE. Encoder_Init_TIM4 writes TIM4->CNT = 10000 at
%   every reset, wherever the cart happens to be sitting, and connecting
%   resets the board. Every absolute number in the WHEELTEC firmware --
%   centre 7925, end stops 5900 and 9900 -- is therefore only meaningful if
%   the cart was parked at the correct end of the rail BEFORE the reset.
%   That end is the one positive PWM drives AWAY from (positive PWM makes
%   the count decrease), i.e. 10000 is the top end and the rail runs down
%   to about 5900.
%
%   Rather than rely on that, firmware R4-4 lets the host DECLARE the
%   origin: put the cart in the middle of the rail by hand and call
%   homeCart(s), which sends HOME. The firmware writes the count to 7925
%   and arms the end-stops. Until then it skips the limit test altogether
%   and caps every motor command to PWM 2500 for 300 ms, so nothing can
%   take a run at the end of the rail while its position is unknown.
%
%   The lab scripts call homeCart for you.

arguments
    port     (1,1) string = "COM4"
    baudRate (1,1) double {mustBePositive,mustBeInteger} = 128000
    opts.SettleSeconds (1,1) double {mustBeNonnegative} = 2.6
    opts.Verbose       (1,1) logical = true
    opts.ExpectVersion (1,1) string  = "R4-6"
end

allPorts = string(serialportlist("all"));
if ~any(strcmpi(allPorts,port))
    error("STM32:PortNotDetected","%s is not detected by Windows/MATLAB.",port);
end

availablePorts = string(serialportlist("available"));
if ~any(strcmpi(availablePorts,port))
    error("STM32:PortUnavailable", ...
        "%s exists but is busy. Close PlatformIO Monitor, the WHEELTEC " + ...
        "upper-computer, other terminals and other MATLAB sessions.",port);
end

device = [];

try
    device = serialport(port,baudRate, ...
        "DataBits",8,"StopBits",1,"Parity","none", ...
        "FlowControl","none","Timeout",0.75);

    % Required for this USB adapter. Also resets the board -- hence step 2.
    setDTR(device,false);
    setRTS(device,false);

    configureTerminator(device,"CR/LF","CR");
    configureCallback(device,"off");

    if opts.Verbose
        fprintf("Waiting %.1f s for the board to finish booting...\n",opts.SettleSeconds);
    end
    pause(opts.SettleSeconds);
    flush(device);

    % Throwaway command: its only job is to make the firmware set
    % stream_enable = 0 so the binary frames stop. The reply is expected to
    % be tangled up with leftover binary, so it is not parsed.
    write(device,uint8([80 73 78 71 13]),"uint8");    % PING + CR
    pause(0.25);
    flush(device);

    % Now the line is clean ASCII. This is the handshake that counts.
    % pendulumCommand, not sendSTM32Command: readline never finds the
    % terminator on this link, so we do our own framing over raw bytes.
    reply = pendulumCommand(device,"PING",0.50,3,["PONG"]);
    if reply ~= "PONG"
        error("STM32:UnexpectedPingReply","Expected PONG but received: %s",reply);
    end

    % Which firmware is actually on the board? Without this, an unflashed
    % board looks exactly like a buggy one, and you debug the wrong thing.
    fwVersion = "unknown (pre-R4-2)";
    try
        vreply = pendulumCommand(device,"VER",0.50,3,["VER="]);
        tok = regexp(vreply,"^VER=(.+)$","tokens","once");
        if ~isempty(tok), fwVersion = string(tok{1}); end
    catch
        % Older firmware has no VER command, so it either says ERR=UNKNOWN
        % or nothing at all. Both mean the same thing.
    end

    if fwVersion ~= opts.ExpectVersion
        % Say what still works, not just that something is different. Most
        % of this toolbox is backward compatible on purpose -- the STATUS
        % and K parsers accept "at least N" fields rather than exactly N --
        % so an older board is usually fine for the earlier labs and the
        % student does not need to stop and reflash mid-session.
        if fwVersion == "R4-5"
            warning("STM32:FirmwareMismatch", ...
                "Board reports R4-5; this toolbox expects %s.\n" + ...
                "  Labs 1-4 will run normally on R4-5.\n" + ...
                "  Lab 5 will NOT: it needs the KICK disturbance command,\n" + ...
                "  the run-time angle setpoint (GAIN,AZ) and STATUS field 10,\n" + ...
                "  all added in R4-6. Open source_code/ in VS Code and\n" + ...
                "  upload before the week 7 session.",opts.ExpectVersion);
        else
            warning("STM32:FirmwareMismatch", ...
                "Board reports firmware ""%s"" but this toolbox expects ""%s"".\n" + ...
                "Open source_code/ in VS Code and upload before trusting any\n" + ...
                "result. Symptoms of an old build: rxErr climbing, ERR=UNKNOWN\n" + ...
                "or ERR=SYNTAX on longer commands, replies going missing.", ...
                fwVersion,opts.ExpectVersion);
        end
    end

    if opts.Verbose
        fprintf("Pendulum connected.\n");
        fprintf("  Firmware      : %s\n",fwVersion);
        fprintf("  Port          : %s\n",device.Port);
        fprintf("  Baud rate     : %d\n",device.BaudRate);
        fprintf("  DTR / RTS     : low\n");
        fprintf("  Binary stream : muted (send setStream(s,true) to restore it)\n");
        fprintf("  Verification  : PING -> PONG\n");
        try
            pos = readPositionWheeltec(device);
            fprintf("  Cart position : %d counts (10000 = boot value, whatever\n",pos);
            fprintf("                  the cart's physical position was)\n");
            if abs(pos-10000) < 5
                fprintf("  Cart limits   : NOT armed yet -- run homeCart(s)\n");
            end
        catch
        end
    end

catch ME
    if ~isempty(device)
        try, configureCallback(device,"off"); catch, end
    end
    clear device
    rethrow(ME);
end
end
