function results = measureSTM32Latency(device,numberOfSamples)
%MEASURESTM32LATENCY Measure PING, STATUS and DATA request timing.

arguments
    device
    numberOfSamples (1,1) double ...
        {mustBePositive,mustBeInteger} = 500
end

command = strings(numberOfSamples,1);
success = false(numberOfSamples,1);
roundTripMs = NaN(numberOfSamples,1);
startTimeSeconds = NaN(numberOfSamples,1);
errorIdentifier = strings(numberOfSamples,1);
errorMessage = strings(numberOfSamples,1);

testStart = tic;

for k = 1:numberOfSamples
    startTimeSeconds(k) = toc(testStart);

    switch mod(k-1,3)
        case 0
            command(k) = "PING";
            expected = "PONG";

        case 1
            command(k) = "STATUS";
            expected = "STATUS=";

        otherwise
            command(k) = "D";
            expected = "DATA=";
    end

    try
        [~,measurement] = sendSTM32Command( ...
            device,command(k),0.75,1,expected);

        success(k) = true;
        roundTripMs(k) = ...
            measurement.RoundTripSeconds * 1000;

    catch ME
        errorIdentifier(k) = string(ME.identifier);
        errorMessage(k) = string(ME.message);
    end

    % Keep the test load controlled.
    pause(0.02);
end

sampleIntervalMs = [NaN; diff(startTimeSeconds) * 1000];

results = table( ...
    (1:numberOfSamples)', ...
    command, ...
    success, ...
    startTimeSeconds, ...
    sampleIntervalMs, ...
    roundTripMs, ...
    errorIdentifier, ...
    errorMessage, ...
    'VariableNames',{ ...
        'Sample', ...
        'Command', ...
        'Success', ...
        'StartTimeSeconds', ...
        'SampleIntervalMs', ...
        'RoundTripMs', ...
        'ErrorIdentifier', ...
        'ErrorMessage'});

validRTT = results.RoundTripMs(results.Success);

fprintf("\nLATENCY TEST SUMMARY\n");
fprintf("Samples:             %d\n",height(results));
fprintf("Successful:          %d\n",sum(results.Success));
fprintf("Failed:              %d\n",sum(~results.Success));

if ~isempty(validRTT)
    fprintf("Mean RTT:            %.3f ms\n",mean(validRTT));
    fprintf("Median RTT:          %.3f ms\n",median(validRTT));
    fprintf("Minimum RTT:         %.3f ms\n",min(validRTT));
    fprintf("Maximum RTT:         %.3f ms\n",max(validRTT));
    fprintf("RTT standard dev.:   %.3f ms\n",std(validRTT));
    fprintf("RTT peak-to-peak:    %.3f ms\n", ...
        max(validRTT)-min(validRTT));
    fprintf("95th percentile RTT: %.3f ms\n", ...
        localPercentile(validRTT,95));
    fprintf("99th percentile RTT: %.3f ms\n", ...
        localPercentile(validRTT,99));
end
end

function value = localPercentile(data,percent)

data = sort(data(:));
position = 1 + (numel(data)-1) * percent / 100;
lowerIndex = floor(position);
upperIndex = ceil(position);

if lowerIndex == upperIndex
    value = data(lowerIndex);
else
    fraction = position-lowerIndex;
    value = data(lowerIndex) + ...
        fraction*(data(upperIndex)-data(lowerIndex));
end
end