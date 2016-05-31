% define the trajectory for quadcopter
function [my_map] = trajectory(t)
    x_d = 0; y_d = 0; z_d = 0.5; phi_d = 0; theta_d = 0; psi_d = 0;
    x_dot_d = 0; y_dot_d = 0; z_dot_d = 0; phi_dot_d = 0; theta_dot_d = 0; psi_dot_d = 0;

    if t<50
        x_d = 0; 
    elseif t>=50 && t<100
        x_d = 0.25; 
    elseif t>=100 && t<150
        x_d = 0.25; y_d = 0.25;
    elseif t>=150 && t<200
        x_d = 0; y_d = 0.25;
    elseif t>=200
        x_d = 0; y_d = 0;
    end
    
    my_map = [x_d;y_d;z_d;phi_d;theta_d;psi_d;
              x_dot_d; y_dot_d; z_dot_d; phi_dot_d; theta_dot_d; psi_dot_d];
end