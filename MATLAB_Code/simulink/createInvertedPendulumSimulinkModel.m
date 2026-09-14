function createInvertedPendulumSimulinkModel()
%CREATEINVERTEDPENDULUMSIMULINKMODEL Create the complete STM32 Simulink
%model and build the associated monitoring and motor-control Dashboard.
%
% This function:
%   1. Verifies that STM32SerialSystem.m is available.
%   2. Verifies that buildInvertedPendulumDashboard.m is available.
%   3. Tests construction of STM32SerialSystem.
%   4. Stops and closes the existing target model.
%   5. Creates a timestamped backup of the existing SLX file.
%   6. Creates a new blank Simulink model.
%   7. Configures the fixed-step discrete solver.
%   8. Adds a MATLAB System block.
%   9. Associates STM32SerialSystem with the block.
%  10. Verifies that the block has 3 inputs and 12 outputs.
%  11. Saves the initial Simulink model.
%  12. Runs buildInvertedPendulumDashboard.m.
%  13. Verifies that all expected Dashboard blocks exist.
%  14. Forces the final safe startup state.
%  15. Updates, saves, and opens the completed model.
%
% SAFE INITIAL STATE
%   Motor_PWM               = 0
%   Motor_Enable_Command    = false
%   Emergency_Stop_Command  = true
%
% Run this function from the MATLAB Command Window:
%
%   createInvertedPendulumSimulinkModel
%
% Required project files:
%   STM32SerialSystem.m
%   buildInvertedPendulumDashboard.m
%   connectSTM32.m
%   disconnectSTM32.m
%   drainSTM32Input.m
%   readSTM32Data.m
%   sendSTM32Command.m
%   setMotorPWM.m
%   stopMotor.m
%
% Important:
%   Close PlatformIO Serial Monitor and all other COM4 applications before
%   running the completed model.

clc;

fprintf("\n");
fprintf("====================================================\n");
fprintf("Creating the inverted-pendulum Simulink model\n");
fprintf("====================================================\n");


%% 1. Configuration

modelName = "Uni_Canberra_Inverted_Pendulum";

modelFile = modelName + ".slx";

systemObjectClass = "STM32SerialSystem";

systemBlockName = "Inverted Pendulum";

systemBlockPath = ...
    modelName + "/" + systemBlockName;

dashboardBuilderName = ...
    "buildInvertedPendulumDashboard";


%% 2. Verify that Simulink is available

simulinkInformation = ver("simulink");

if isempty(simulinkInformation)

    error( ...
        "STM32:SimulinkNotAvailable", ...
        "Simulink is not available in this MATLAB installation.");
end

fprintf("\nSimulink is available.\n");


%% 3. Verify that STM32SerialSystem.m is available

systemObjectFile = which(char(systemObjectClass));

if isempty(systemObjectFile)

    error( ...
        "STM32:SystemObjectNotFound", ...
        [ ...
        "STM32SerialSystem.m could not be found. " ...
        "Change MATLAB's Current Folder to the project MATLAB folder " ...
        "or add the project MATLAB folder to the MATLAB path."]);
end

fprintf("\nSystem object found:\n");
fprintf("  %s\n",systemObjectFile);


%% 4. Verify that the Dashboard builder is available

dashboardBuilderFile = which( ...
    char(dashboardBuilderName));

if isempty(dashboardBuilderFile)

    error( ...
        "STM32:DashboardBuilderNotFound", ...
        [ ...
        "buildInvertedPendulumDashboard.m could not be found. " ...
        "Both builder files must be located in the project MATLAB folder."]);
end

fprintf("\nDashboard builder found:\n");
fprintf("  %s\n",dashboardBuilderFile);


%% 5. Verify that required communication functions are available

requiredFunctionNames = { ...
    'connectSTM32', ...
    'drainSTM32Input', ...
    'readSTM32Data', ...
    'sendSTM32Command', ...
    'setMotorPWM', ...
    'stopMotor'};

missingFunctionNames = strings( ...
    numel(requiredFunctionNames),1);

missingFunctionCount = 0;


for functionIndex = 1:numel(requiredFunctionNames)

    requiredFunctionName = ...
        requiredFunctionNames{functionIndex};

    functionFile = which(requiredFunctionName);

    if isempty(functionFile)

        missingFunctionCount = ...
            missingFunctionCount + 1;

        missingFunctionNames(missingFunctionCount) = ...
            string(requiredFunctionName);
    end
end


missingFunctionNames = ...
    missingFunctionNames(1:missingFunctionCount);


if ~isempty(missingFunctionNames)

    fprintf("\nMissing communication functions:\n");

    for functionIndex = 1:numel(missingFunctionNames)

        fprintf( ...
            "  %s.m\n", ...
            missingFunctionNames(functionIndex));
    end

    error( ...
        "STM32:RequiredFunctionsMissing", ...
        "One or more required communication functions are missing.");
end

fprintf("\nRequired communication functions are available.\n");


%% 6. Verify that the System object class can be constructed

try

    constructionTestObject = STM32SerialSystem();

    if ~isa(constructionTestObject,"STM32SerialSystem")

        error( ...
            "STM32:UnexpectedConstructionResult", ...
            "The constructor returned an unexpected object type.");
    end

    clear constructionTestObject;

catch ME

    error( ...
        "STM32:SystemObjectConstructionFailed", ...
        [ ...
        "STM32SerialSystem could not be constructed. " ...
        "Correct STM32SerialSystem.m before building the model.\n\n" ...
        "Original error:\n%s"], ...
        ME.message);
end

fprintf("\nSTM32SerialSystem construction test passed.\n");


%% 7. Stop and close the existing target model

if bdIsLoaded(char(modelName))

    simulationStatus = string( ...
        get_param(char(modelName),"SimulationStatus"));

    if simulationStatus ~= "stopped"

        fprintf("\nStopping the existing simulation...\n");

        set_param( ...
            char(modelName), ...
            "SimulationCommand","stop");

        pause(0.5);
    end

    fprintf("\nClosing the existing model...\n");

    close_system( ...
        char(modelName), ...
        0);
end


%% 8. Back up an existing model file

if isfile(modelFile)

    timeStamp = string( ...
        datetime( ...
            "now", ...
            "Format","yyyyMMdd_HHmmss"));

    backupFile = ...
        modelName + "_backup_" + timeStamp + ".slx";

    backupCreated = copyfile( ...
        char(modelFile), ...
        char(backupFile));

    if ~backupCreated

        error( ...
            "STM32:ModelBackupFailed", ...
            "The existing Simulink model could not be backed up.");
    end

    fprintf("\nExisting model backed up as:\n");
    fprintf("  %s\n",backupFile);

    delete(char(modelFile));
end


%% 9. Create a new blank Simulink model

new_system(char(modelName));

open_system(char(modelName));

fprintf("\nNew model created:\n");
fprintf("  %s\n",modelName);


%% 10. Configure the model solver

set_param( ...
    char(modelName), ...
    "SolverType","Fixed-step", ...
    "Solver","FixedStepDiscrete", ...
    "FixedStep","0.1", ...
    "StartTime","0", ...
    "StopTime","10");

fprintf("\nSolver configured:\n");
fprintf("  Solver type: Fixed-step\n");
fprintf("  Solver:      Discrete\n");
fprintf("  Step size:   0.1 seconds\n");
fprintf("  Stop time:   10 seconds\n");


%% 11. Configure model-data logging

try

    set_param( ...
        char(modelName), ...
        "SignalLogging","on", ...
        "SignalLoggingName","logsout", ...
        "ReturnWorkspaceOutputs","on");

    fprintf("\nSignal logging configured.\n");

catch ME

    warning( ...
        "STM32:LoggingConfigurationFailed", ...
        "Some logging settings could not be configured: %s", ...
        ME.message);
end


%% 12. Add the MATLAB System block

try

    add_block( ...
        "simulink/User-Defined Functions/MATLAB System", ...
        char(systemBlockPath), ...
        "Position",[650 100 950 880], ...
        "ShowName","off");

catch ME

    close_system( ...
        char(modelName), ...
        0);

    error( ...
        "STM32:MATLABSystemBlockCreationFailed", ...
        [ ...
        "The MATLAB System block could not be created.\n\n" ...
        "Original error:\n%s"], ...
        ME.message);
end

fprintf("\nMATLAB System block created:\n");
fprintf("  %s\n",systemBlockPath);


%% 13. Assign STM32SerialSystem to the MATLAB System block

systemObjectAssigned = false;

assignmentParameter = "";


% First try the standard MATLAB System parameter.
try

    set_param( ...
        char(systemBlockPath), ...
        "System", ...
        char(systemObjectClass));

    systemObjectAssigned = true;

    assignmentParameter = "System";

catch
    % Search the available parameters below.
end


%% 14. Search available block parameters if necessary

if ~systemObjectAssigned

    dialogParameters = get_param( ...
        char(systemBlockPath), ...
        "DialogParameters");

    availableParameterNames = ...
        fieldnames(dialogParameters);

    candidateParameterNames = { ...
        'System', ...
        'SystemObject', ...
        'SystemObjectName', ...
        'SystemName'};


    for parameterIndex = 1:numel(candidateParameterNames)

        candidateParameter = ...
            candidateParameterNames{parameterIndex};

        parameterIsAvailable = ismember( ...
            candidateParameter, ...
            availableParameterNames);

        if ~parameterIsAvailable
            continue;
        end

        try

            set_param( ...
                char(systemBlockPath), ...
                candidateParameter, ...
                char(systemObjectClass));

            systemObjectAssigned = true;

            assignmentParameter = ...
                string(candidateParameter);

            break;

        catch
            % Try the next possible parameter.
        end
    end
end


%% 15. Stop if the System object was not assigned

if ~systemObjectAssigned

    availableParameters = get_param( ...
        char(systemBlockPath), ...
        "DialogParameters");

    fprintf("\nAvailable MATLAB System block parameters:\n");

    disp(fieldnames(availableParameters));

    close_system( ...
        char(modelName), ...
        0);

    error( ...
        "STM32:SystemObjectAssignmentFailed", ...
        [ ...
        "The MATLAB System block was created, but " ...
        "STM32SerialSystem could not be assigned automatically."]);
end

fprintf("\nSystem object assigned successfully.\n");
fprintf("  Class:     %s\n",systemObjectClass);
fprintf("  Parameter: %s\n",assignmentParameter);


%% 16. Try to configure exposed System-object properties

systemPropertyNames = { ...
    'Port', ...
    'BaudRate', ...
    'ResponseTimeout', ...
    'CommandRetries', ...
    'MotorRefreshPeriod', ...
    'SampleTime', ...
    'MaximumConsecutiveFailures'};

systemPropertyValues = { ...
    'COM4', ...
    '9600', ...
    '0.75', ...
    '1', ...
    '0.20', ...
    '0.10', ...
    '3'};


for propertyIndex = 1:numel(systemPropertyNames)

    propertyName = ...
        systemPropertyNames{propertyIndex};

    propertyValue = ...
        systemPropertyValues{propertyIndex};

    try

        set_param( ...
            char(systemBlockPath), ...
            propertyName, ...
            propertyValue);

    catch
        % The defaults in STM32SerialSystem already contain these values.
    end
end


%% 17. Refresh MATLAB and update the model

rehash;

try

    set_param( ...
        char(modelName), ...
        "SimulationCommand","update");

catch ME

    close_system( ...
        char(modelName), ...
        0);

    error( ...
        "STM32:InitialModelUpdateFailed", ...
        [ ...
        "The model could not update after STM32SerialSystem " ...
        "was assigned.\n\nOriginal error:\n%s"], ...
        ME.message);
end


%% 18. Verify that the expected ports were generated

systemPorts = get_param( ...
    char(systemBlockPath), ...
    "PortHandles");

numberOfInputs = ...
    numel(systemPorts.Inport);

numberOfOutputs = ...
    numel(systemPorts.Outport);

fprintf("\nGenerated System-block ports:\n");
fprintf("  Inputs:  %d\n",numberOfInputs);
fprintf("  Outputs: %d\n",numberOfOutputs);


if numberOfInputs ~= 3

    close_system( ...
        char(modelName), ...
        0);

    error( ...
        "STM32:IncorrectInputCount", ...
        [ ...
        "The Inverted Pendulum block generated %d inputs. " ...
        "STM32SerialSystem must generate exactly 3 inputs."], ...
        numberOfInputs);
end


if numberOfOutputs ~= 12

    close_system( ...
        char(modelName), ...
        0);

    error( ...
        "STM32:IncorrectOutputCount", ...
        [ ...
        "The Inverted Pendulum block generated %d outputs. " ...
        "STM32SerialSystem must generate exactly 12 outputs."], ...
        numberOfOutputs);
end


%% 19. Apply initial central-block formatting

try

    set_param( ...
        char(systemBlockPath), ...
        "Position",[650 100 950 880], ...
        "ShowName","off", ...
        "FontName","Arial", ...
        "FontSize","18", ...
        "FontWeight","bold", ...
        "ForegroundColor","blue", ...
        "BackgroundColor","lightBlue");

catch

    set_param( ...
        char(systemBlockPath), ...
        "Position",[650 100 950 880], ...
        "ShowName","off", ...
        "FontName","Arial", ...
        "FontSize","18", ...
        "ForegroundColor","blue", ...
        "BackgroundColor","lightBlue");
end


%% 20. Save the initial model

save_system( ...
    char(modelName), ...
    char(modelFile));

fprintf("\nInitial model saved:\n");
fprintf("  %s\n",modelFile);


%% 21. Run the main Dashboard builder

fprintf("\nRunning the main Dashboard builder...\n");

try

    run(char(dashboardBuilderFile));

catch ME

    save_system(char(modelName));

    error( ...
        "STM32:DashboardBuilderFailed", ...
        [ ...
        "The basic Inverted Pendulum block was created successfully, " ...
        "but buildInvertedPendulumDashboard.m failed.\n\n" ...
        "Original error:\n%s"], ...
        ME.message);
end


%% 22. Define the final required blocks

requiredFinalBlocks = [ ...
    modelName + "/Inverted Pendulum"
    modelName + "/Motor_PWM"
    modelName + "/Motor_Enable_Command"
    modelName + "/Emergency_Stop_Command"
    modelName + "/Dashboard_PWM_Slider"
    modelName + "/Dashboard_Motor_Enable"
    modelName + "/Dashboard_Emergency_Stop"
    modelName + "/Monitor_Angle_Raw"
    modelName + "/Monitor_Encoder_Position"
    modelName + "/Monitor_Commanded_PWM"
    modelName + "/Monitor_Applied_PWM"
    modelName + "/Monitor_STM32_Time_ms"
    modelName + "/Monitor_Round_Trip_ms"
    modelName + "/Monitor_Data_Valid"
    modelName + "/Monitor_Connection_State"
    modelName + "/Monitor_Successful_Packets"
    modelName + "/Monitor_Missed_Packets"
    modelName + "/Monitor_Malformed_Packets"
    modelName + "/Monitor_Consecutive_Failures"];


%% 23. Preallocate the missing-block list

missingBlocks = strings( ...
    numel(requiredFinalBlocks),1);

missingBlockCount = 0;


%% 24. Verify the final required blocks

for blockIndex = 1:numel(requiredFinalBlocks)

    blockPath = ...
        requiredFinalBlocks(blockIndex);

    blockExists = ...
        getSimulinkBlockHandle(char(blockPath)) > 0;

    if ~blockExists

        missingBlockCount = ...
            missingBlockCount + 1;

        missingBlocks(missingBlockCount) = ...
            blockPath;
    end
end


missingBlocks = ...
    missingBlocks(1:missingBlockCount);


if ~isempty(missingBlocks)

    fprintf("\nMissing final blocks:\n");

    for blockIndex = 1:numel(missingBlocks)

        fprintf( ...
            "  %s\n", ...
            missingBlocks(blockIndex));
    end

    error( ...
        "STM32:IncompleteDashboardBuild", ...
        "The Dashboard build finished with missing blocks.");
end

fprintf("\nAll required Dashboard blocks were created.\n");


%% 25. Force the final safe startup state

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


%% 26. Update the completed model

set_param( ...
    char(modelName), ...
    "SimulationCommand","update");


%% 27. Read back the safe startup values

pwmValue = string( ...
    get_param( ...
        char(pwmPath), ...
        "Value"));

enableValue = string( ...
    get_param( ...
        char(enablePath), ...
        "Value"));

emergencyValue = string( ...
    get_param( ...
        char(emergencyPath), ...
        "Value"));


%% 28. Verify the safe startup values

if pwmValue ~= "0"

    error( ...
        "STM32:UnsafePWMValue", ...
        "Motor_PWM did not reset to zero.");
end


if ~strcmpi(enableValue,"false")

    error( ...
        "STM32:UnsafeEnableValue", ...
        "Motor_Enable_Command did not reset to false.");
end


if ~strcmpi(emergencyValue,"true")

    error( ...
        "STM32:UnsafeEmergencyValue", ...
        "Emergency_Stop_Command did not reset to true.");
end


%% 29. Save and open the completed model

save_system(char(modelName));

open_system(char(modelName));


%% 30. Completion report

fprintf("\n");
fprintf("====================================================\n");
fprintf("Inverted-pendulum Simulink model created successfully\n");
fprintf("====================================================\n");
fprintf("Model file:\n");
fprintf("  %s\n",modelFile);
fprintf("\n");
fprintf("System block:\n");
fprintf("  %s\n",systemBlockPath);
fprintf("\n");
fprintf("System object:\n");
fprintf("  %s\n",systemObjectClass);
fprintf("\n");
fprintf("Safe initial state:\n");
fprintf("  Motor PWM      = %s\n",pwmValue);
fprintf("  Motor Enable   = %s\n",enableValue);
fprintf("  Emergency Stop = %s\n",emergencyValue);
fprintf("\n");
fprintf("Before running the motor, verify:\n");
fprintf("  Data_Valid = 1\n");
fprintf("  Connection_State = 3\n");
fprintf("  Applied_PWM = 0\n");
fprintf("====================================================\n");

end