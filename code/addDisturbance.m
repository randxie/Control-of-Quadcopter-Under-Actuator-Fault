% calculate disturbance table
% assume pose angle disturbance is smaller than position disturbance
function [disturb_s] = addDisturbance(time, s)
    disturb_s = [zeros(6,numel(time));randn(3,numel(time))*0.05;randn(3,numel(time))*0.001];
end