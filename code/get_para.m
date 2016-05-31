% get current parameter (assign fault here)
function [cur_para] = get_para(time, s)
    global para constants isFault
    cur_para = para;
    if(isFault)
        if time>100
            cur_para.kL(1) = constants.maxKL * (0.5 + exp(-(time-100))* 0.5);
        end
    end
end
