%% buildInvertedPendulumDashboard.m
%
% Builds the complete Simulink Dashboard interface for the STM32
% inverted-pendulum teaching platform.
%
% FINAL CONTROL ARCHITECTURE
%
%   Dashboard PWM Slider
%       binds to Motor_PWM Constant Value
%
%   Dashboard Motor Enable Toggle
%       binds to Motor_Enable_Command Constant Value
%
%   Dashboard Emergency Stop Toggle
%       binds to Emergency_Stop_Command Constant Value
%
% The three Constant blocks connect directly to the three inputs of the
% STM32SerialSystem MATLAB System block.
%
% SAFE INITIAL STATE
%
%   Motor_PWM             = 0
%   Motor_Enable_Command  = false
%   Emergency_Stop_Command = true
%
% SYSTEM INPUT ORDER
%
%   Input 1: Requested PWM
%   Input 2: Motor Enable
%   Input 3: Emergency Stop
%
% SYSTEM OUTPUT ORDER
%
%   Output 1:  Angle Raw
%   Output 2:  Encoder Position
%   Output 3:  Commanded PWM
%   Output 4:  Applied PWM
%   Output 5:  STM32 Timestamp
%   Output 6:  Round-Trip Time
%   Output 7:  Data Valid
%   Output 8:  Connection State
%   Output 9:  Successful Packets
%   Output 10: Missed Packets
%   Output 11: Malformed Packets
%   Output 12: Consecutive Failures
%
% This script:
%
%   1. Finds the unique 3-input, 12-output MATLAB System block.
%   2. Stops the simulation.
%   3. Deletes old Manual Switch controls.
%   4. Deletes previous Dashboard controls.
%   5. Deletes previous generated Constants and Displays.
%   6. Removes associated lines and dangling lines.
%   7. Formats the Inverted Pendulum block.
%   8. Creates three Constant blocks.
%   9. Aligns each Constant with the exact System input position.
%  10. Connects clean horizontal input lines.
%  11. Creates 12 compact output Displays.
%  12. Connects and labels all output lines.
%  13. Creates one-click Dashboard controls.
%  14. Binds Dashboard controls to Constant parameters.
%  15. Applies and verifies the safe startup state.
%  16. Updates and saves the model.

clc;


%% 1. Define the model and System object

modelName = "Uni_Canberra_Inverted_Pendulum";
systemObjectClass = "STM32SerialSystem";
systemBlockName = "Inverted Pendulum";

systemPath = ...
    modelName + "/" + systemBlockName;


%% 2. Confirm that STM32SerialSystem is available

systemObjectFile = which(systemObjectClass);

if isempty(systemObjectFile)

    error( ...
        "STM32:SystemObjectNotFound", ...
        [ ...
        "STM32SerialSystem.m was not found on the MATLAB path. " ...
        "Open the project MATLAB folder or add that folder to the path."]);
end

fprintf("System object found:\n%s\n\n",systemObjectFile);


%% 3. Close another active model if necessary

currentRoot = string(bdroot);

if strlength(currentRoot) > 0 && ...
        currentRoot ~= modelName

    if string(get_param(char(currentRoot), ...
            "SimulationStatus")) ~= "stopped"

        set_param( ...
            char(currentRoot), ...
            "SimulationCommand","stop");

        pause(0.5);
    end

    save_system(char(currentRoot));
end


%% 4. Create or load the Simulink model

if bdIsLoaded(char(modelName))

    fprintf("Model already loaded:\n%s\n\n",modelName);

elseif isfile(modelName + ".slx")

    load_system(char(modelName));

    fprintf("Existing model loaded:\n%s.slx\n\n",modelName);

else

    new_system(char(modelName));

    fprintf("New model created:\n%s\n\n",modelName);
end

open_system(char(modelName));


%% 5. Stop the model if it is running

simulationStatus = string( ...
    get_param(char(modelName),"SimulationStatus"));

if simulationStatus ~= "stopped"

    fprintf("Stopping the current simulation...\n");

    set_param( ...
        char(modelName), ...
        "SimulationCommand","stop");

    pause(0.5);
end


%% 6. Configure basic model settings

set_param( ...
    char(modelName), ...
    "SolverType","Fixed-step", ...
    "Solver","FixedStepDiscrete", ...
    "FixedStep","0.1", ...
    "StartTime","0", ...
    "StopTime","10");

try
    set_param( ...
        char(modelName), ...
        "SignalLogging","on", ...
        "SignalLoggingName","logsout", ...
        "ReturnWorkspaceOutputs","on");
catch ME
    warning( ...
        "STM32:ModelLoggingConfigurationFailed", ...
        "Could not configure all logging options: %s", ...
        ME.message);
end


%% 7. Check whether the Inverted Pendulum block already exists

systemBlockExists = ...
    getSimulinkBlockHandle(char(systemPath)) > 0;


%% 8. If necessary, search for an existing compatible block

if ~systemBlockExists

    allBlocks = find_system( ...
        char(modelName), ...
        "LookUnderMasks","all", ...
        "FollowLinks","on", ...
        "Type","Block");

    compatibleBlocks = strings(0,1);

    for blockIndex = 1:numel(allBlocks)

        candidate = string(allBlocks{blockIndex});

        if candidate == modelName
            continue;
        end

        try
            candidatePorts = get_param( ...
                char(candidate), ...
                "PortHandles");

            hasThreeInputs = ...
                numel(candidatePorts.Inport) == 3;

            hasTwelveOutputs = ...
                numel(candidatePorts.Outport) == 12;

            if hasThreeInputs && hasTwelveOutputs
                compatibleBlocks(end+1,1) = ...
                    candidate; %#ok<SAGROW>
            end

        catch
            % Ignore blocks without standard ports.
        end
    end


    if numel(compatibleBlocks) == 1

        existingBlock = compatibleBlocks(1);

        existingBlockParent = string( ...
            get_param(char(existingBlock),"Parent"));

        if existingBlockParent == modelName

            set_param( ...
                char(existingBlock), ...
                "Name",char(systemBlockName));

            systemBlockExists = true;

            fprintf( ...
                "Existing compatible block renamed:\n%s\n\n", ...
                systemPath);
        end
    end
end


%% 9. Create the MATLAB System block if it does not exist

if ~systemBlockExists

    fprintf("Creating the MATLAB System block...\n");

    add_block( ...
        "simulink/User-Defined Functions/MATLAB System", ...
        char(systemPath), ...
        "Position",[650 100 950 880], ...
        "ShowName","off");

    systemBlockExists = true;
end


%% 10. Associate the block with STM32SerialSystem

systemParameterConfigured = false;

% Standard MATLAB System block parameter.
try
    set_param( ...
        char(systemPath), ...
        "System",char(systemObjectClass));

    systemParameterConfigured = true;

catch
    % Continue to the release-independent parameter search below.
end


%% 11. Release-independent System-object parameter search

if ~systemParameterConfigured

    dialogParameters = get_param( ...
        char(systemPath), ...
        "DialogParameters");

    parameterNames = fieldnames(dialogParameters);

    possibleParameterNames = { ...
        'System', ...
        'SystemObject', ...
        'SystemObjectName', ...
        'SystemName'};

    for candidateIndex = 1:numel(possibleParameterNames)

        candidateParameter = ...
            possibleParameterNames{candidateIndex};

        if ismember(candidateParameter,parameterNames)

            try
                set_param( ...
                    char(systemPath), ...
                    candidateParameter, ...
                    char(systemObjectClass));

                systemParameterConfigured = true;

                fprintf( ...
                    "System object assigned using parameter: %s\n", ...
                    candidateParameter);

                break;

            catch
            end
        end
    end
end


%% 12. Stop if the System object could not be assigned

if ~systemParameterConfigured

    dialogParameters = get_param( ...
        char(systemPath), ...
        "DialogParameters");

    fprintf("\nAvailable MATLAB System block parameters:\n");

    disp(fieldnames(dialogParameters));

    error( ...
        "STM32:SystemObjectAssignmentFailed", ...
        [ ...
        "The MATLAB System block was created, but " ...
        "STM32SerialSystem could not be assigned automatically."]);
end


%% 13. Configure the System object properties

systemPropertyValues = { ...
    'Port',                       'COM4'; ...
    'BaudRate',                   '9600'; ...
    'ResponseTimeout',            '0.75'; ...
    'CommandRetries',             '1'; ...
    'MotorRefreshPeriod',         '0.20'; ...
    'SampleTime',                 '0.10'; ...
    'MaximumConsecutiveFailures', '3'};


for propertyIndex = 1:size(systemPropertyValues,1)

    propertyName = ...
        systemPropertyValues{propertyIndex,1};

    propertyValue = ...
        systemPropertyValues{propertyIndex,2};

    try
        set_param( ...
            char(systemPath), ...
            propertyName, ...
            propertyValue);

    catch
        % Some releases expose System object properties through an
        % internal mask rather than direct block parameters. The class
        % defaults already contain the intended values.
    end
end


%% 14. Update the model to generate the ports

clear STM32SerialSystem
rehash

set_param( ...
    char(modelName), ...
    "SimulationCommand","update");


%% 15. Verify that the block generated the expected ports

systemPorts = get_param( ...
    char(systemPath), ...
    "PortHandles");

numberOfInputs = ...
    numel(systemPorts.Inport);

numberOfOutputs = ...
    numel(systemPorts.Outport);


if numberOfInputs ~= 3

    error( ...
        "STM32:IncorrectCreatedInputCount", ...
        [ ...
        "The newly created Inverted Pendulum block has %d inputs. " ...
        "Expected 3 inputs."], ...
        numberOfInputs);
end


if numberOfOutputs ~= 12

    error( ...
        "STM32:IncorrectCreatedOutputCount", ...
        [ ...
        "The newly created Inverted Pendulum block has %d outputs. " ...
        "Expected 12 outputs."], ...
        numberOfOutputs);
end


fprintf("Inverted Pendulum block created successfully.\n");
fprintf("  System object: %s\n",systemObjectClass);
fprintf("  Inputs:        %d\n",numberOfInputs);
fprintf("  Outputs:       %d\n\n",numberOfOutputs);

save_system(char(modelName));
%% 16. Define input and output signal names

inputSignalNames = { ...
    'Requested_PWM', ...
    'Motor_Enable', ...
    'Emergency_Stop'};


outputSignalNames = { ...
    'Angle_Raw', ...
    'Encoder_Position', ...
    'Commanded_PWM', ...
    'Applied_PWM', ...
    'STM32_Time_ms', ...
    'Round_Trip_ms', ...
    'Data_Valid', ...
    'Connection_State', ...
    'Successful_Packets', ...
    'Missed_Packets', ...
    'Malformed_Packets', ...
    'Consecutive_Failures'};


monitorBlockNames = { ...
    'Monitor_Angle_Raw', ...
    'Monitor_Encoder_Position', ...
    'Monitor_Commanded_PWM', ...
    'Monitor_Applied_PWM', ...
    'Monitor_STM32_Time_ms', ...
    'Monitor_Round_Trip_ms', ...
    'Monitor_Data_Valid', ...
    'Monitor_Connection_State', ...
    'Monitor_Successful_Packets', ...
    'Monitor_Missed_Packets', ...
    'Monitor_Malformed_Packets', ...
    'Monitor_Consecutive_Failures'};


%% 5. Define blocks managed by the script

oldManualControlNames = { ...
    'Enable_FALSE', ...
    'Enable_TRUE', ...
    'Motor_Enable_Switch', ...
    'Emergency_TRUE', ...
    'Emergency_FALSE', ...
    'Emergency_Stop_Switch', ...
    'PWM_Initial_Value'};


dashboardControlNames = { ...
    'Dashboard_PWM_Slider', ...
    'Dashboard_Motor_Enable', ...
    'Dashboard_Emergency_Stop'};


constantBlockNames = { ...
    'Motor_PWM', ...
    'Motor_Enable_Command', ...
    'Emergency_Stop_Command'};


%% 6. Remove lines connected to the System block

systemPorts = get_param( ...
    char(systemPath), ...
    "PortHandles");

allSystemPorts = [ ...
    systemPorts.Inport(:); ...
    systemPorts.Outport(:)];


for portIndex = 1:numel(allSystemPorts)

    lineHandle = get_param( ...
        allSystemPorts(portIndex), ...
        "Line");

    if lineHandle ~= -1

        try
            delete_line(lineHandle);

        catch ME
            warning( ...
                "STM32:SystemLineCleanupFailed", ...
                "Could not remove a System-block line: %s", ...
                ME.message);
        end
    end
end


%% 7. Remove old Manual Switch control lines

for blockIndex = 1:numel(oldManualControlNames)

    oldBlockPath = ...
        modelName + "/" + oldManualControlNames{blockIndex};

    deleteAllBlockLines(oldBlockPath);
end


%% 8. Remove previous Constant-block lines

for blockIndex = 1:numel(constantBlockNames)

    oldBlockPath = ...
        modelName + "/" + constantBlockNames{blockIndex};

    deleteAllBlockLines(oldBlockPath);
end


%% 9. Remove previous Display-block lines

for blockIndex = 1:numel(monitorBlockNames)

    displayPath = ...
        modelName + "/" + monitorBlockNames{blockIndex};

    deleteAllBlockLines(displayPath);
end


%% 10. Delete old Manual Switch controls

for blockIndex = 1:numel(oldManualControlNames)

    oldBlockPath = ...
        modelName + "/" + oldManualControlNames{blockIndex};

    deleteBlockIfPresent(oldBlockPath);
end


%% 11. Delete previous Dashboard controls

for blockIndex = 1:numel(dashboardControlNames)

    dashboardPath = ...
        modelName + "/" + dashboardControlNames{blockIndex};

    deleteBlockIfPresent(dashboardPath);
end


%% 12. Delete previous Constant blocks

for blockIndex = 1:numel(constantBlockNames)

    constantPath = ...
        modelName + "/" + constantBlockNames{blockIndex};

    deleteBlockIfPresent(constantPath);
end


%% 13. Delete previous output Displays

for blockIndex = 1:numel(monitorBlockNames)

    displayPath = ...
        modelName + "/" + monitorBlockNames{blockIndex};

    deleteBlockIfPresent(displayPath);
end


%% 14. Remove remaining dangling lines

allLines = find_system( ...
    char(modelName), ...
    "FindAll","on", ...
    "Type","line");

for lineIndex = 1:numel(allLines)

    try
        sourcePort = get_param( ...
            allLines(lineIndex), ...
            "SrcPortHandle");

        destinationPorts = get_param( ...
            allLines(lineIndex), ...
            "DstPortHandle");

        sourceMissing = ...
            isempty(sourcePort) || ...
            any(sourcePort == -1);

        destinationMissing = ...
            isempty(destinationPorts) || ...
            any(destinationPorts == -1);

        if sourceMissing || destinationMissing
            delete_line(allLines(lineIndex));
        end

    catch
        % Ignore a line handle that has already been removed.
    end
end


%% 15. Update once after cleanup

set_param( ...
    char(modelName), ...
    "SimulationCommand","update");


%% 16. Format and position the Inverted Pendulum block

systemPosition = [ ...
    650, ...  % left
    100, ...  % top
    950, ...  % right
    880];     % bottom


try
    set_param( ...
        char(systemPath), ...
        "Position",systemPosition, ...
        "ShowName","off", ...
        "FontName","Arial", ...
        "FontSize","18", ...
        "FontWeight","bold", ...
        "ForegroundColor","blue", ...
        "BackgroundColor","lightBlue");

catch
    % Some MATLAB System blocks do not expose FontWeight.
    set_param( ...
        char(systemPath), ...
        "Position",systemPosition, ...
        "ShowName","off", ...
        "FontName","Arial", ...
        "FontSize","18", ...
        "ForegroundColor","blue", ...
        "BackgroundColor","lightBlue");
end


%% 17. Refresh System ports after resizing

set_param( ...
    char(modelName), ...
    "SimulationCommand","update");

systemPorts = get_param( ...
    char(systemPath), ...
    "PortHandles");


if numel(systemPorts.Inport) ~= 3

    error( ...
        "STM32:IncorrectInputCount", ...
        "Expected 3 System inputs, but found %d.", ...
        numel(systemPorts.Inport));
end


if numel(systemPorts.Outport) ~= 12

    error( ...
        "STM32:IncorrectOutputCount", ...
        "Expected 12 System outputs, but found %d.", ...
        numel(systemPorts.Outport));
end


%% 18. Obtain exact System input coordinates

pwmInputPosition = get_param( ...
    systemPorts.Inport(1), ...
    "Position");

enableInputPosition = get_param( ...
    systemPorts.Inport(2), ...
    "Position");

emergencyInputPosition = get_param( ...
    systemPorts.Inport(3), ...
    "Position");

pwmInputY = pwmInputPosition(2);
enableInputY = enableInputPosition(2);
emergencyInputY = emergencyInputPosition(2);


%% 19. Define final Constant-block paths

pwmConstantPath = ...
    modelName + "/Motor_PWM";

enableConstantPath = ...
    modelName + "/Motor_Enable_Command";

emergencyConstantPath = ...
    modelName + "/Emergency_Stop_Command";


%% 20. Define Constant-block dimensions

constantLeft = 100;
constantWidth = 165;
constantHeight = 44;


%% 21. Create Motor PWM Constant aligned with System input 1

pwmConstantTop = round( ...
    pwmInputY - constantHeight/2);

add_block( ...
    "simulink/Sources/Constant", ...
    char(pwmConstantPath), ...
    "Value","0", ...
    "OutDataTypeStr","double", ...
    "SampleTime","0.1", ...
    "Position",[ ...
        constantLeft, ...
        pwmConstantTop, ...
        constantLeft + constantWidth, ...
        pwmConstantTop + constantHeight], ...
    "ShowName","on");


%% 22. Create Motor Enable Constant aligned with System input 2

enableConstantTop = round( ...
    enableInputY - constantHeight/2);

add_block( ...
    "simulink/Sources/Constant", ...
    char(enableConstantPath), ...
    "Value","false", ...
    "OutDataTypeStr","boolean", ...
    "SampleTime","0.1", ...
    "Position",[ ...
        constantLeft, ...
        enableConstantTop, ...
        constantLeft + constantWidth, ...
        enableConstantTop + constantHeight], ...
    "ShowName","on");


%% 23. Create Emergency Stop Constant aligned with System input 3

emergencyConstantTop = round( ...
    emergencyInputY - constantHeight/2);

add_block( ...
    "simulink/Sources/Constant", ...
    char(emergencyConstantPath), ...
    "Value","true", ...
    "OutDataTypeStr","boolean", ...
    "SampleTime","0.1", ...
    "Position",[ ...
        constantLeft, ...
        emergencyConstantTop, ...
        constantLeft + constantWidth, ...
        emergencyConstantTop + constantHeight], ...
    "ShowName","on");


%% 24. Obtain Constant-block ports

pwmPorts = get_param( ...
    char(pwmConstantPath), ...
    "PortHandles");

enablePorts = get_param( ...
    char(enableConstantPath), ...
    "PortHandles");

emergencyPorts = get_param( ...
    char(emergencyConstantPath), ...
    "PortHandles");


%% 25. Confirm all System input destinations are empty

for inputIndex = 1:3

    existingLine = get_param( ...
        systemPorts.Inport(inputIndex), ...
        "Line");

    if existingLine ~= -1
        delete_line(existingLine);
    end
end


%% 26. Connect Motor PWM Constant

pwmLine = add_line( ...
    char(modelName), ...
    pwmPorts.Outport(1), ...
    systemPorts.Inport(1), ...
    "autorouting","on");

set_param( ...
    pwmLine, ...
    "Name",inputSignalNames{1});


%% 27. Connect Motor Enable Constant

enableLine = add_line( ...
    char(modelName), ...
    enablePorts.Outport(1), ...
    systemPorts.Inport(2), ...
    "autorouting","on");

set_param( ...
    enableLine, ...
    "Name",inputSignalNames{2});


%% 28. Connect Emergency Stop Constant

emergencyLine = add_line( ...
    char(modelName), ...
    emergencyPorts.Outport(1), ...
    systemPorts.Inport(3), ...
    "autorouting","on");

set_param( ...
    emergencyLine, ...
    "Name",inputSignalNames{3});


%% 29. Route the input lines

try
    Simulink.BlockDiagram.routeLine(pwmLine);
    Simulink.BlockDiagram.routeLine(enableLine);
    Simulink.BlockDiagram.routeLine(emergencyLine);
catch
    % The Constants are already vertically aligned with the inputs.
end


%% 30. Create compact output Displays

displayLeft = 1140;
displayWidth = 115;
displayHeight = 34;


for outputIndex = 1:numel(outputSignalNames)

    sourcePort = systemPorts.Outport(outputIndex);

    sourcePosition = get_param( ...
        sourcePort, ...
        "Position");

    sourceY = sourcePosition(2);

    displayTop = round( ...
        sourceY - displayHeight/2);

    displayPath = ...
        modelName + "/" + monitorBlockNames{outputIndex};


    add_block( ...
        "simulink/Sinks/Display", ...
        char(displayPath), ...
        "Position",[ ...
            displayLeft, ...
            displayTop, ...
            displayLeft + displayWidth, ...
            displayTop + displayHeight], ...
        "ShowName","on");


    displayPorts = get_param( ...
        char(displayPath), ...
        "PortHandles");


    outputLine = add_line( ...
        char(modelName), ...
        sourcePort, ...
        displayPorts.Inport(1), ...
        "autorouting","on");


    set_param( ...
        outputLine, ...
        "Name",outputSignalNames{outputIndex});


    fprintf( ...
        "Connected output %02d: %s\n", ...
        outputIndex, ...
        outputSignalNames{outputIndex});
end


%% 31. Define Dashboard control paths

pwmSliderPath = ...
    modelName + "/Dashboard_PWM_Slider";

motorTogglePath = ...
    modelName + "/Dashboard_Motor_Enable";

emergencyTogglePath = ...
    modelName + "/Dashboard_Emergency_Stop";


%% 32. Define Dashboard control positions

dashboardLeft = 325;
dashboardRight = 555;

sliderHeight = 105;
toggleHeight = 90;

dashboardClearance = 45;


%% 33. Create PWM Slider above Requested_PWM signal

pwmSliderBottom = ...
    pwmInputY - dashboardClearance;

pwmSliderTop = ...
    pwmSliderBottom - sliderHeight;

add_block( ...
    "simulink_hmi_blocks/Slider", ...
    char(pwmSliderPath), ...
    "Position",[ ...
        dashboardLeft, ...
        pwmSliderTop, ...
        dashboardRight, ...
        pwmSliderBottom], ...
    "ShowName","on");


%% 34. Bind PWM Slider to Motor_PWM Constant

pwmBinding = Simulink.HMI.ParamSourceInfo;

pwmBinding.BlockPath = ...
    Simulink.BlockPath(char(pwmConstantPath));

pwmBinding.ParamName = 'Value';

set_param( ...
    char(pwmSliderPath), ...
    "Binding",pwmBinding);


%% 35. Attempt to configure PWM Slider limits

try
    set_param( ...
        char(pwmSliderPath), ...
        "Limits",[-999 0 999]);

catch ME
    warning( ...
        "STM32:PWMSliderLimitsNotApplied", ...
        ["PWM Slider limits could not be set automatically. " ...
         "Set minimum to -999 and maximum to 999 manually. " ...
         "Details: %s"], ...
        ME.message);
end


%% 36. Create Motor Enable Toggle above Motor_Enable signal

motorToggleBottom = ...
    enableInputY - dashboardClearance;

motorToggleTop = ...
    motorToggleBottom - toggleHeight;

add_block( ...
    "simulink_hmi_blocks/Toggle Switch", ...
    char(motorTogglePath), ...
    "Position",[ ...
        dashboardLeft + 35, ...
        motorToggleTop, ...
        dashboardRight - 35, ...
        motorToggleBottom], ...
    "ShowName","on");


%% 37. Bind Motor Enable Toggle to Constant

enableBinding = Simulink.HMI.ParamSourceInfo;

enableBinding.BlockPath = ...
    Simulink.BlockPath(char(enableConstantPath));

enableBinding.ParamName = 'Value';

set_param( ...
    char(motorTogglePath), ...
    "Binding",enableBinding);


%% 38. Create Emergency Stop Toggle above Emergency_Stop signal

emergencyToggleBottom = ...
    emergencyInputY - dashboardClearance;

emergencyToggleTop = ...
    emergencyToggleBottom - toggleHeight;

add_block( ...
    "simulink_hmi_blocks/Toggle Switch", ...
    char(emergencyTogglePath), ...
    "Position",[ ...
        dashboardLeft + 35, ...
        emergencyToggleTop, ...
        dashboardRight - 35, ...
        emergencyToggleBottom], ...
    "ShowName","on");


%% 39. Bind Emergency Stop Toggle to Constant

emergencyBinding = Simulink.HMI.ParamSourceInfo;

emergencyBinding.BlockPath = ...
    Simulink.BlockPath(char(emergencyConstantPath));

emergencyBinding.ParamName = 'Value';

set_param( ...
    char(emergencyTogglePath), ...
    "Binding",emergencyBinding);


%% 40. Force the safe initial Constant values

set_param( ...
    char(pwmConstantPath), ...
    "Value","0");

set_param( ...
    char(enableConstantPath), ...
    "Value","false");

set_param( ...
    char(emergencyConstantPath), ...
    "Value","true");


%% 41. Update the completed model

set_param( ...
    char(modelName), ...
    "SimulationCommand","update");


%% 42. Reapply input signal labels

systemPorts = get_param( ...
    char(systemPath), ...
    "PortHandles");


for inputIndex = 1:numel(inputSignalNames)

    lineHandle = get_param( ...
        systemPorts.Inport(inputIndex), ...
        "Line");

    if lineHandle ~= -1

        set_param( ...
            lineHandle, ...
            "Name",inputSignalNames{inputIndex});
    end
end


%% 43. Reapply output signal labels

for outputIndex = 1:numel(outputSignalNames)

    lineHandle = get_param( ...
        systemPorts.Outport(outputIndex), ...
        "Line");

    if lineHandle ~= -1

        set_param( ...
            lineHandle, ...
            "Name",outputSignalNames{outputIndex});
    end
end


%% 44. Verify safe initial values

pwmValue = string( ...
    get_param(char(pwmConstantPath),"Value"));

enableValue = string( ...
    get_param(char(enableConstantPath),"Value"));

emergencyValue = string( ...
    get_param(char(emergencyConstantPath),"Value"));


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


%% 45. Save the completed model

save_system(char(modelName));


%% 46. Completion report

fprintf("\n");
fprintf("================================================\n");
fprintf("Inverted-pendulum Dashboard created successfully\n");
fprintf("================================================\n");
fprintf("Model: %s\n",modelName);
fprintf("\n");
fprintf("Safe initial state:\n");
fprintf("  Motor PWM      = %s\n",pwmValue);
fprintf("  Motor Enable   = %s\n",enableValue);
fprintf("  Emergency Stop = %s\n",emergencyValue);
fprintf("\n");
fprintf("Input routing:\n");
fprintf("  Three Constant blocks aligned with System inputs\n");
fprintf("  Dashboard controls positioned above signal lines\n");
fprintf("\n");
fprintf("Output Displays connected: %d\n", ...
    numel(outputSignalNames));
fprintf("================================================\n");


%% Local helper: remove all lines connected to a block

function deleteAllBlockLines(blockPath)

    if getSimulinkBlockHandle(char(blockPath)) <= 0
        return;
    end

    try
        blockPorts = get_param( ...
            char(blockPath), ...
            "PortHandles");

        portFieldNames = fieldnames(blockPorts);

        for fieldIndex = 1:numel(portFieldNames)

            currentPorts = ...
                blockPorts.(portFieldNames{fieldIndex});

            for portIndex = 1:numel(currentPorts)

                currentPort = currentPorts(portIndex);

                if currentPort == -1
                    continue;
                end

                try
                    lineHandle = get_param( ...
                        currentPort, ...
                        "Line");

                    if lineHandle ~= -1
                        delete_line(lineHandle);
                    end

                catch
                end
            end
        end

    catch
    end
end


%% Local helper: delete a block if it exists

function deleteBlockIfPresent(blockPath)

    if getSimulinkBlockHandle(char(blockPath)) <= 0
        return;
    end

    try
        deleteAllBlockLines(blockPath);
        delete_block(char(blockPath));

    catch ME
        warning( ...
            "STM32:GeneratedBlockDeleteFailed", ...
            "Could not delete block %s: %s", ...
            blockPath, ...
            ME.message);
    end
end