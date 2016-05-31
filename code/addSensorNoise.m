% sensor noise table for faster computation
function [noise] = addSensorNoise(time,s0)
    noise = randn(12,numel(time))*0.001;
end