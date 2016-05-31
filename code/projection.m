function [pnew] = projection(p, alpha, gamma, y, dataMtx)
global para constants
dataMtx = dataMtx';
M = alpha*eye(size(dataMtx,2)) + dataMtx'* dataMtx;

if (rcond(M)>10^-15)
    Minv = inv(M);
    dp = (gamma * dataMtx * Minv) * (y - dataMtx' * p);
else
    dp = 0;
    
end
pnew = p + dp;
% constraint pnew (actuator only losses thrust/moment)
for i = 1:numel(pnew)
    if (pnew(i) > constants.maxKL)
        pnew(i) = constants.maxKL;
    end
end

end