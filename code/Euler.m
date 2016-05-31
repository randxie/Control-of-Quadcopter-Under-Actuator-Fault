% Euler method for solving stochastic system
function [t, s]=Euler(dyneqn,time,s0)
    s=zeros(numel(time),numel(s0));
    s(1,:)=s0';
    h = time(2)-time(1);
    for n=2:numel(time)
        s(n,:)=s(n-1,:) + (h*dyneqn(time(n),s(n-1,:)'))';
    end
    t = time;
end