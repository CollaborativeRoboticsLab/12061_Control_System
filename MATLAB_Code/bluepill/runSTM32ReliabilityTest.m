function [results,summary] = runSTM32ReliabilityTest( ...
    device,durationMinutes,periodSeconds)
%RUNSTM32RELIABILITYTEST Sustained PING/STATUS/D reliability test.
%
% This test does not send nonzero motor commands.

arguments
    device
    durationMinutes (1,1) double {mustBePositive} = 10
    periodSeconds (1,1) double {mustBePositive} = 0.10
end

durationSeconds = durationMinutes * 60;
estimatedRows = ceil(durationSeconds / periodSeconds) + 10;

sample = NaN(estimatedRows,1);
pcTime = NaT(estimatedRows,1);
elapsedSeconds = NaN(estimatedRows,1);
command = strings(estimatedRows,1);
success = false(estimatedRows,1);
roundTripMs = NaN(estimatedRows,1);
sampleIntervalMs = NaN(estimatedRows,1);

stm32TimestampMs = NaN(estimatedRows,1);
angleRaw = NaN(estimatedRows,1);
position = NaN(estimatedRows,1);
appliedPWM = NaN(estimatedRows,1);

statusCurrentPWM = NaN(estimatedRows,1);
statusCCR4 = NaN(estimatedRows,1);
overrunErrors = NaN(estimatedRows,1);
framingErrors = NaN(estimatedRows,1);
noiseErrors = NaN(estimatedRows,1);
parityErrors = NaN(estimatedRows,1);
txTimeouts = NaN(estimatedRows,1);
adcTimeouts = NaN(estimatedRows,1);
bufferOverflows = NaN(estimatedRows,1);

ignoredAlive = zeros(estimatedRows,1);
malformedPackets = zeros(estimatedRows,1);
missedPackets = zeros(estimatedRows,1);
errorIdentifier = strings(estimatedRows,1);
errorMessage = strings(estimatedRows,1);

testTimer = tic;
previousStart = NaN;
k = 0;

fprintf("Starting %.1f-minute reliability test.\n",durationMinutes);
fprintf("No nonzero motor command will be sent.\n");

while toc(testTimer) < durationSeconds
    iterationStart = toc(testTimer);
    k = k + 1;

    if k > numel(sample)
        error("STM32:TestStorageExceeded", ...
            "Preallocated test storage was exceeded.");
    end

    sample(k) = k;
    pcTime(k) = datetime("now");
    elapsedSeconds(k) = iterationStart;

    if ~isnan(previousStart)
        sampleIntervalMs(k) = ...
            (iterationStart-previousStart)*1000;
    end
    previousStart = iterationStart;

    selector = mod(k-1,3);

    try
        switch selector
            case 0
                command(k) = "PING";

                [reply,measurement] = sendSTM32Command( ...
                    device,"PING",0.75,1,["PONG"]);

                if reply ~= "PONG"
                    error("STM32:MalformedPong", ...
                        "Expected PONG but received %s.",reply);
                end

                roundTripMs(k) = ...
                    measurement.RoundTripSeconds*1000;
                ignoredAlive(k) = measurement.IgnoredAlive;

            case 1
                command(k) = "STATUS";

                status = readStatus(device);

                roundTripMs(k) = ...
                    status.RoundTripSeconds*1000;
                statusCurrentPWM(k) = status.CurrentPWM;
                statusCCR4(k) = status.CCR4;
                overrunErrors(k) = status.OverrunErrors;
                framingErrors(k) = status.FramingErrors;
                noiseErrors(k) = status.NoiseErrors;
                parityErrors(k) = status.ParityErrors;
                txTimeouts(k) = status.TxTimeouts;
                adcTimeouts(k) = status.AdcTimeouts;
                bufferOverflows(k) = status.BufferOverflows;

            otherwise
                command(k) = "D";

                data = readSTM32Data(device);

                roundTripMs(k) = ...
                    data.RoundTripSeconds*1000;
                stm32TimestampMs(k) = ...
                    double(data.STM32TimestampMs);
                angleRaw(k) = data.AngleRaw;
                position(k) = data.Position;
                appliedPWM(k) = data.AppliedPWM;
                ignoredAlive(k) = data.IgnoredAlive;
        end

        success(k) = true;

    catch ME
        errorIdentifier(k) = string(ME.identifier);
        errorMessage(k) = string(ME.message);

        if contains(ME.identifier,"Malformed")
            malformedPackets(k) = 1;
        else
            missedPackets(k) = 1;
        end
    end

    nextTarget = k * periodSeconds;
    remaining = nextTarget-toc(testTimer);

    if remaining > 0
        pause(remaining);
    end

    if mod(k,100) == 0
        fprintf( ...
            "Elapsed %.1f s | samples %d | failures %d\n", ...
            toc(testTimer),k,sum(~success(1:k)));
    end
end

validRows = 1:k;

results = table( ...
    sample(validRows), ...
    pcTime(validRows), ...
    elapsedSeconds(validRows), ...
    command(validRows), ...
    success(validRows), ...
    sampleIntervalMs(validRows), ...
    roundTripMs(validRows), ...
    stm32TimestampMs(validRows), ...
    angleRaw(validRows), ...
    position(validRows), ...
    appliedPWM(validRows), ...
    statusCurrentPWM(validRows), ...
    statusCCR4(validRows), ...
    overrunErrors(validRows), ...
    framingErrors(validRows), ...
    noiseErrors(validRows), ...
    parityErrors(validRows), ...
    txTimeouts(validRows), ...
    adcTimeouts(validRows), ...
    bufferOverflows(validRows), ...
    ignoredAlive(validRows), ...
    malformedPackets(validRows), ...
    missedPackets(validRows), ...
    errorIdentifier(validRows), ...
    errorMessage(validRows), ...
    'VariableNames',{ ...
        'Sample', ...
        'PCTime', ...
        'ElapsedSeconds', ...
        'Command', ...
        'Success', ...
        'SampleIntervalMs', ...
        'RoundTripMs', ...
        'STM32TimestampMs', ...
        'AngleRaw', ...
        'Position', ...
        'AppliedPWM', ...
        'StatusCurrentPWM', ...
        'StatusCCR4', ...
        'OverrunErrors', ...
        'FramingErrors', ...
        'NoiseErrors', ...
        'ParityErrors', ...
        'TxTimeouts', ...
        'AdcTimeouts', ...
        'BufferOverflows', ...
        'IgnoredAlive', ...
        'MalformedPackets', ...
        'MissedPackets', ...
        'ErrorIdentifier', ...
        'ErrorMessage'});

validRTT = results.RoundTripMs(results.Success & ...
    ~isnan(results.RoundTripMs));

validIntervals = results.SampleIntervalMs( ...
    ~isnan(results.SampleIntervalMs));

summary = struct;
summary.DurationMinutes = durationMinutes;
summary.PeriodSeconds = periodSeconds;
summary.TotalTransactions = height(results);
summary.SuccessfulTransactions = sum(results.Success);
summary.FailedTransactions = sum(~results.Success);
summary.SuccessRatePercent = ...
    100*summary.SuccessfulTransactions/summary.TotalTransactions;
summary.MalformedPackets = sum(results.MalformedPackets);
summary.MissedPackets = sum(results.MissedPackets);
summary.IgnoredAlive = sum(results.IgnoredAlive);

if isempty(validRTT)
    summary.MeanRTTMs = NaN;
    summary.MinimumRTTMs = NaN;
    summary.MaximumRTTMs = NaN;
    summary.StdRTTMs = NaN;
else
    summary.MeanRTTMs = mean(validRTT);
    summary.MinimumRTTMs = min(validRTT);
    summary.MaximumRTTMs = max(validRTT);
    summary.StdRTTMs = std(validRTT);
end

if isempty(validIntervals)
    summary.MeanSampleIntervalMs = NaN;
    summary.MaximumSampleIntervalMs = NaN;
    summary.SampleIntervalJitterMs = NaN;
else
    summary.MeanSampleIntervalMs = mean(validIntervals);
    summary.MaximumSampleIntervalMs = max(validIntervals);
    summary.SampleIntervalJitterMs = ...
        max(validIntervals)-min(validIntervals);
end

fprintf("\nRELIABILITY TEST SUMMARY\n");
disp(summary);
end