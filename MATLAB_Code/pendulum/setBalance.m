function measurement = setBalance(device,enable)
%SETBALANCE Enable or disable the on-board balancing PD controller.
%
%   setBalance(device,true)  -> firmware runs Balance() + Position()
%                               at 200 Hz, exactly as the stock code does
%                               when KEY5 releases the stop flag.
%   setBalance(device,false) -> stop (Flag_Stop = 1).
%
%   Enabling also re-zeroes the cart: the firmware sets Swing_up = 0,
%   which makes the next control interrupt take Position_Zero = Encoder.
%   Put the cart where you want the setpoint to be BEFORE calling this.

arguments
    device
    enable (1,1) logical
end

if enable
    command = "BAL,1"; expected = "BAL=1";
else
    command = "BAL,0"; expected = "BAL=0";
end

[reply,measurement] = pendulumCommand(device,command,0.50,2,["BAL="]);

if reply ~= expected
    error("STM32:MalformedBalance","Unexpected BAL response: %s",reply);
end
end
