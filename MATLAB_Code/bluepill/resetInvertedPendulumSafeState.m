function resetInvertedPendulumSafeState()
%RESETINVERTEDPENDULUMSAFESTATE
% Force the Simulink model into a safe startup state.
%
% Safe values:
%   Motor_PWM              = 0
%   Motor_Enable_Command   = false
%   Emergency_Stop_Command = true

modelName = "Uni_Canberra_Inverted_Pendulum";

if ~bdIsLoaded(char(modelName))
    load_system(char(modelName));
end

pwmPath = ...
    modelName + "/Motor_PWM";

enablePath = ...
    modelName + "/Motor_Enable_Command";

emergencyPath = ...
    modelName + "/Emergency_Stop_Command";

set_param( ...
    char(pwmPath), ...
    "Value","0");

set_param( ...
    char(enablePath), ...
    "Value","false");

set_param( ...
    char(emergencyPath), ...
    "Value","true");

set_param( ...
    char(modelName), ...
    "SimulationCommand","update");

save_system(char(modelName));

fprintf("\n");
fprintf("Safe state applied successfully.\n");
fprintf("Motor_PWM              = 0\n");
fprintf("Motor_Enable_Command   = false\n");
fprintf("Emergency_Stop_Command = true\n");
fprintf("\n");

end