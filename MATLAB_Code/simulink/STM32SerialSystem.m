classdef STM32SerialSystem < matlab.System
    %STM32SERIALSYSTEM Persistent STM32 serial communication interface.
    %
    % Inputs
    %   1. Requested PWM
    %   2. Motor Enable
    %   3. Emergency Stop
    %
    % Outputs
    %   1.  Raw angle
    %   2.  Encoder position
    %   3.  Commanded PWM
    %   4.  Applied PWM
    %   5.  STM32 timestamp
    %   6.  Round-trip time
    %   7.  Data-valid flag
    %   8.  Connection state
    %   9.  Successful packets
    %   10. Missed packets
    %   11. Malformed packets
    %   12. Consecutive failures

    properties (Nontunable)
        Port string = "COM10"
        BaudRate double = 9600
        ResponseTimeout double = 0.75
        CommandRetries double = 1
        MotorRefreshPeriod double = 0.20
        SampleTime double = 0.10
        MaximumConsecutiveFailures double = 3
    end

    properties (Access = private)
        pDevice = []

        pIsConnected logical = false
        pMotorEnabled logical = false
        pStopAlreadySent logical = false

        pLastCommandedPWM double = 0
        pLastAppliedPWM double = 0
        pLastAngleRaw double = 0
        pLastEncoderPosition double = 0
        pLastSTM32TimestampMs uint32 = uint32(0)
        pLastRoundTripMs double = NaN

        pSuccessfulPackets uint32 = uint32(0)
        pMissedPackets uint32 = uint32(0)
        pMalformedPackets uint32 = uint32(0)
        pConsecutiveFailures uint32 = uint32(0)

        pSamplesSinceMotorCommand uint32 = uint32(0)
        pMotorRefreshSamples uint32 = uint32(1)
    end

    methods
        function obj = STM32SerialSystem(varargin)
            setProperties(obj,nargin,varargin{:});
        end
    end

    methods (Access = protected)

        function setupImpl(obj)
            obj.initialiseRuntimeState();

            refreshSamples = round( ...
                obj.MotorRefreshPeriod / obj.SampleTime);

            obj.pMotorRefreshSamples = ...
                uint32(max(1,refreshSamples));

            try
                obj.pDevice = connectSTM32( ...
                    obj.Port,obj.BaudRate);

                obj.pIsConnected = true;

                if ~stopMotor(obj.pDevice)
                    error( ...
                        "STM32:SystemSetupStopFailed", ...
                        "Initial STOP was not acknowledged.");
                end

                obj.pStopAlreadySent = true;
                obj.pMotorEnabled = false;
                obj.pLastCommandedPWM = 0;
                obj.pLastAppliedPWM = 0;

                fprintf("\nSTM32SerialSystem setup complete.\n");
                fprintf("  Port: %s\n",obj.Port);
                fprintf("  Baud rate: %.0f\n",obj.BaudRate);
                fprintf( ...
                    "  Sample time: %.3f seconds\n", ...
                    obj.SampleTime);
                fprintf( ...
                    "  Motor refresh: %.3f seconds\n", ...
                    obj.MotorRefreshPeriod);
                fprintf("  Initial motor state: STOPPED\n");

            catch ME
                obj.releaseDeviceSafely();

                error( ...
                    "STM32:SystemSetupFailed", ...
                    "STM32SerialSystem setup failed: %s", ...
                    ME.message);
            end
        end


        function [ ...
                angleRaw, ...
                encoderPosition, ...
                commandedPWM, ...
                appliedPWM, ...
                stm32TimestampMs, ...
                roundTripMs, ...
                dataValid, ...
                connectionState, ...
                successfulPackets, ...
                missedPackets, ...
                malformedPackets, ...
                consecutiveFailures] = ...
                stepImpl( ...
                obj, ...
                requestedPWM, ...
                enableMotor, ...
                emergencyStop)

            requestedPWM = round(double(requestedPWM));
            requestedPWM = max(-999,min(999,requestedPWM));

            enableMotor = logical(enableMotor);
            emergencyStop = logical(emergencyStop);

            dataValid = false;

            if ~obj.pIsConnected
                connectionState = uint8(0);

                [ ...
                    angleRaw, ...
                    encoderPosition, ...
                    commandedPWM, ...
                    appliedPWM, ...
                    stm32TimestampMs, ...
                    roundTripMs, ...
                    successfulPackets, ...
                    missedPackets, ...
                    malformedPackets, ...
                    consecutiveFailures] = ...
                    obj.currentOutputs();

                return;
            end

            try
                if emergencyStop || ~enableMotor
                    obj.applySafeStop();

                else
                    obj.pStopAlreadySent = false;

                    commandChanged = ...
                        requestedPWM ~= obj.pLastCommandedPWM;

                    refreshRequired = ...
                        obj.pSamplesSinceMotorCommand >= ...
                        obj.pMotorRefreshSamples;

                    enablingMotor = ~obj.pMotorEnabled;

                    if commandChanged || ...
                            refreshRequired || ...
                            enablingMotor

                        [acknowledgedPWM,~] = setMotorPWM( ...
                            obj.pDevice,requestedPWM);

                        obj.pLastCommandedPWM = requestedPWM;
                        obj.pLastAppliedPWM = acknowledgedPWM;
                        obj.pSamplesSinceMotorCommand = uint32(0);

                    else
                        obj.pSamplesSinceMotorCommand = ...
                            obj.pSamplesSinceMotorCommand + uint32(1);
                    end

                    obj.pMotorEnabled = true;
                end

                data = readSTM32Data(obj.pDevice);

                obj.pLastAngleRaw = data.AngleRaw;
                obj.pLastEncoderPosition = data.Position;
                obj.pLastAppliedPWM = data.AppliedPWM;
                obj.pLastSTM32TimestampMs = ...
                    uint32(data.STM32TimestampMs);
                obj.pLastRoundTripMs = ...
                    data.RoundTripSeconds * 1000;

                obj.pSuccessfulPackets = ...
                    obj.pSuccessfulPackets + uint32(1);

                obj.pConsecutiveFailures = uint32(0);
                dataValid = true;

            catch ME
                obj.processCommunicationFailure(ME);
            end

            if emergencyStop
                connectionState = uint8(3);

            elseif obj.pConsecutiveFailures >= ...
                    uint32(obj.MaximumConsecutiveFailures)

                connectionState = uint8(5);

            elseif obj.pConsecutiveFailures > 0
                connectionState = uint8(4);

            elseif obj.pMotorEnabled
                connectionState = uint8(2);

            else
                connectionState = uint8(1);
            end

            [ ...
                angleRaw, ...
                encoderPosition, ...
                commandedPWM, ...
                appliedPWM, ...
                stm32TimestampMs, ...
                roundTripMs, ...
                successfulPackets, ...
                missedPackets, ...
                malformedPackets, ...
                consecutiveFailures] = ...
                obj.currentOutputs();
        end


        function releaseImpl(obj)
            fprintf("\nSTM32SerialSystem release started.\n");

            obj.releaseDeviceSafely();

            fprintf("STM32SerialSystem release complete.\n");
        end


        function resetImpl(obj)
            % Do not clear pDevice here. resetImpl can be called while the
            % System object still owns COM4.

            obj.pLastCommandedPWM = 0;
            obj.pLastAppliedPWM = 0;
            obj.pLastAngleRaw = 0;
            obj.pLastEncoderPosition = 0;
            obj.pLastSTM32TimestampMs = uint32(0);
            obj.pLastRoundTripMs = NaN;

            obj.pSuccessfulPackets = uint32(0);
            obj.pMissedPackets = uint32(0);
            obj.pMalformedPackets = uint32(0);
            obj.pConsecutiveFailures = uint32(0);

            obj.pSamplesSinceMotorCommand = uint32(0);
            obj.pMotorEnabled = false;
            obj.pStopAlreadySent = false;
        end


        function validatePropertiesImpl(obj)
            if strlength(strtrim(obj.Port)) == 0
                error( ...
                    "STM32:InvalidPort", ...
                    "Port cannot be empty.");
            end

            validateattributes( ...
                obj.BaudRate, ...
                {'numeric'}, ...
                {'scalar','positive','finite'});

            validateattributes( ...
                obj.ResponseTimeout, ...
                {'numeric'}, ...
                {'scalar','positive','finite'});

            validateattributes( ...
                obj.CommandRetries, ...
                {'numeric'}, ...
                {'scalar','integer','positive'});

            validateattributes( ...
                obj.SampleTime, ...
                {'numeric'}, ...
                {'scalar','positive','finite'});

            validateattributes( ...
                obj.MotorRefreshPeriod, ...
                {'numeric'}, ...
                {'scalar','positive','finite'});

            validateattributes( ...
                obj.MaximumConsecutiveFailures, ...
                {'numeric'}, ...
                {'scalar','integer','positive'});

            if obj.MotorRefreshPeriod >= 1
                warning( ...
                    "STM32:UnsafeRefreshPeriod", ...
                    "MotorRefreshPeriod must be shorter " + ...
                    "than the firmware watchdog period.");
            end
        end


        function validateInputsImpl( ...
                ~,requestedPWM,enableMotor,emergencyStop)

            validateattributes( ...
                requestedPWM, ...
                {'numeric'}, ...
                {'scalar','real','finite'});

            validateattributes( ...
                enableMotor, ...
                {'logical','numeric'}, ...
                {'scalar','real','finite'});

            validateattributes( ...
                emergencyStop, ...
                {'logical','numeric'}, ...
                {'scalar','real','finite'});
        end


        function numberOfInputs = getNumInputsImpl(~)
            numberOfInputs = 3;
        end


        function numberOfOutputs = getNumOutputsImpl(~)
            numberOfOutputs = 12;
        end


        function [name1,name2,name3] = getInputNamesImpl(~)
            % Keep input port labels blank inside the block.

            name1 = '';
            name2 = '';
            name3 = '';
        end


        function [ ...
                name1,name2,name3,name4,name5,name6, ...
                name7,name8,name9,name10,name11,name12] = ...
                getOutputNamesImpl(~)
            % Keep output port labels blank inside the block.

            name1 = '';
            name2 = '';
            name3 = '';
            name4 = '';
            name5 = '';
            name6 = '';
            name7 = '';
            name8 = '';
            name9 = '';
            name10 = '';
            name11 = '';
            name12 = '';
        end


        function icon = getIconImpl(~)
            icon = 'Inverted Pendulum';
        end


        function sampleTimeSpecification = getSampleTimeImpl(obj)
            sampleTimeSpecification = createSampleTime( ...
                obj, ...
                "Type","Discrete", ...
                "SampleTime",obj.SampleTime);
        end


        function flag = isInputSizeMutableImpl(~,~)
            flag = false;
        end


        function flag = isInputComplexityMutableImpl(~,~)
            flag = false;
        end


        function flag = isInputDataTypeMutableImpl(~,~)
            flag = false;
        end


        function [ ...
                size1,size2,size3,size4,size5,size6, ...
                size7,size8,size9,size10,size11,size12] = ...
                getOutputSizeImpl(~)

            size1 = [1 1];
            size2 = [1 1];
            size3 = [1 1];
            size4 = [1 1];
            size5 = [1 1];
            size6 = [1 1];
            size7 = [1 1];
            size8 = [1 1];
            size9 = [1 1];
            size10 = [1 1];
            size11 = [1 1];
            size12 = [1 1];
        end


        function [ ...
                fixed1,fixed2,fixed3,fixed4,fixed5,fixed6, ...
                fixed7,fixed8,fixed9,fixed10,fixed11,fixed12] = ...
                isOutputFixedSizeImpl(~)

            fixed1 = true;
            fixed2 = true;
            fixed3 = true;
            fixed4 = true;
            fixed5 = true;
            fixed6 = true;
            fixed7 = true;
            fixed8 = true;
            fixed9 = true;
            fixed10 = true;
            fixed11 = true;
            fixed12 = true;
        end


        function [ ...
                complex1,complex2,complex3,complex4,complex5,complex6, ...
                complex7,complex8,complex9,complex10,complex11,complex12] = ...
                isOutputComplexImpl(~)

            complex1 = false;
            complex2 = false;
            complex3 = false;
            complex4 = false;
            complex5 = false;
            complex6 = false;
            complex7 = false;
            complex8 = false;
            complex9 = false;
            complex10 = false;
            complex11 = false;
            complex12 = false;
        end


        function [ ...
                type1,type2,type3,type4,type5,type6, ...
                type7,type8,type9,type10,type11,type12] = ...
                getOutputDataTypeImpl(~)

            type1 = "double";
            type2 = "double";
            type3 = "double";
            type4 = "double";
            type5 = "uint32";
            type6 = "double";
            type7 = "logical";
            type8 = "uint8";
            type9 = "uint32";
            type10 = "uint32";
            type11 = "uint32";
            type12 = "uint32";
        end
    end


%% Force interpreted execution in Simulink

    methods (Static, Access = protected)
    
        function simulationMode = getSimulateUsingImpl()
            % Force the MATLAB System block to use the MATLAB execution engine.
            %
            % This System object uses host-side resources that are not
            % supported for C/C++ code generation, including:
            %   - serialport
            %   - try/catch
            %   - setDTR and setRTS
            %   - persistent COM-port ownership
            %   - MATLAB communication helper functions
    
            simulationMode = "Interpreted execution";
        end
    
    
        function showOption = showSimulateUsingImpl()
            % Display the Simulate using option in the block dialog.
    
            showOption = true;
        end
    
    end


    methods (Access = private)

        function initialiseRuntimeState(obj)
            obj.pDevice = [];

            obj.pIsConnected = false;
            obj.pMotorEnabled = false;
            obj.pStopAlreadySent = false;

            obj.pLastCommandedPWM = 0;
            obj.pLastAppliedPWM = 0;
            obj.pLastAngleRaw = 0;
            obj.pLastEncoderPosition = 0;
            obj.pLastSTM32TimestampMs = uint32(0);
            obj.pLastRoundTripMs = NaN;

            obj.pSuccessfulPackets = uint32(0);
            obj.pMissedPackets = uint32(0);
            obj.pMalformedPackets = uint32(0);
            obj.pConsecutiveFailures = uint32(0);

            obj.pSamplesSinceMotorCommand = uint32(0);
            obj.pMotorRefreshSamples = uint32(1);
        end


        function applySafeStop(obj)
            if obj.pStopAlreadySent
                obj.pMotorEnabled = false;
                obj.pLastCommandedPWM = 0;
                obj.pLastAppliedPWM = 0;
                return;
            end

            if ~stopMotor(obj.pDevice)
                error( ...
                    "STM32:StopNotConfirmed", ...
                    "STM32 did not acknowledge STOP.");
            end

            obj.pStopAlreadySent = true;
            obj.pMotorEnabled = false;
            obj.pLastCommandedPWM = 0;
            obj.pLastAppliedPWM = 0;
            obj.pSamplesSinceMotorCommand = uint32(0);
        end


        function processCommunicationFailure(obj,ME)
            obj.pConsecutiveFailures = ...
                obj.pConsecutiveFailures + uint32(1);

            if contains(string(ME.identifier),"Malformed")
                obj.pMalformedPackets = ...
                    obj.pMalformedPackets + uint32(1);
            else
                obj.pMissedPackets = ...
                    obj.pMissedPackets + uint32(1);
            end

            if obj.pConsecutiveFailures >= ...
                    uint32(obj.MaximumConsecutiveFailures)

                obj.pLastCommandedPWM = 0;
                obj.pLastAppliedPWM = 0;
                obj.pMotorEnabled = false;

                try
                    writeline(obj.pDevice,"STOP");
                catch
                end

                obj.pIsConnected = false;
                obj.pStopAlreadySent = false;

            else
                % Force a command refresh after one isolated missed packet.
                obj.pSamplesSinceMotorCommand = ...
                    obj.pMotorRefreshSamples;
            end
        end


        function releaseDeviceSafely(obj)
            if ~isempty(obj.pDevice)
                try
                    stopMotor(obj.pDevice);
                catch
                    try
                        writeline(obj.pDevice,"STOP");
                    catch
                    end
                end

                try
                    configureCallback(obj.pDevice,"off");
                catch
                end

                try
                    setDTR(obj.pDevice,false);
                    setRTS(obj.pDevice,false);
                catch
                end

                try
                    flush(obj.pDevice);
                catch
                end
            end

            obj.pDevice = [];
            obj.pIsConnected = false;
            obj.pMotorEnabled = false;
            obj.pStopAlreadySent = true;
            obj.pLastCommandedPWM = 0;
            obj.pLastAppliedPWM = 0;
        end


        function [ ...
                angleRaw, ...
                encoderPosition, ...
                commandedPWM, ...
                appliedPWM, ...
                stm32TimestampMs, ...
                roundTripMs, ...
                successfulPackets, ...
                missedPackets, ...
                malformedPackets, ...
                consecutiveFailures] = ...
                currentOutputs(obj)

            angleRaw = obj.pLastAngleRaw;
            encoderPosition = obj.pLastEncoderPosition;
            commandedPWM = obj.pLastCommandedPWM;
            appliedPWM = obj.pLastAppliedPWM;
            stm32TimestampMs = obj.pLastSTM32TimestampMs;
            roundTripMs = obj.pLastRoundTripMs;

            successfulPackets = obj.pSuccessfulPackets;
            missedPackets = obj.pMissedPackets;
            malformedPackets = obj.pMalformedPackets;
            consecutiveFailures = obj.pConsecutiveFailures;
        end
    end
end