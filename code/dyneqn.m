% s is the state vector, 12 states in total
% state in following order:
% x,y,z,phi,theta,psi,xdot,ydot,zdot,phi_dot,theta_dot,psi_dot
function [sdot] = dyneqn(time, s)
    
    [fs, gs_thrust, gs_moment] = calThrustMoment(s);
    % assume only motor parameters will be changed
    cur_para = get_para(time, s);
    kL = cur_para.kL; kD = kL/cur_para.LD_ratio;
    
    % calculate inputs based on propeller rotational speeds omega
    % support there is a close-loop in omega
    if time==0
        U=[0;0;0;0];
    else
        U = controller(time, s);
    end

    u1=U(1); u2=U(2); u3=U(3); u4=U(4);
    %   denote u1 = o1^2, u2 = o2^2, ....
    FT = (kL(1)*u1 + kL(2)*u2 + kL(3)*u2 + kL(4)*u4);
    M1 = (kL(4)*u4 - kL(2)*u2); 
    M2 = (-kL(1)*u1 + kL(3)*u3); 
    M3 = (kD(1)*u1 + kD(3)*u3) - (kD(2)*u2 + kD(4)*u4);
   
    MT = [M1;M2;M3];
    
    %%
    sdot = zeros(size(s));
    sdot(1:6) = s(7:12);
    sdot(7:12) = fs + [gs_thrust*FT;gs_moment.*MT];
    
    %% add disturbance
    sdot = sdot + cur_para.disturb_table(:,round(time/cur_para.h+1));
    
    %% store last states
    global Uold sdot_old s_old
    Uold = U;
    sdot_old = sdot;
    s_old = s;
end

function [U] = controller(time, s)
    global ctlpara constants para isAdapt
    % add sensor noise
    s = s + para.noise_table(:,round(time/para.h+1));
    
    x=s(1); y=s(2); z=s(3); phi=s(4); theta=s(5); psi=s(6); 
    x_dot=s(7); y_dot=s(8); z_dot=s(9); phi_dot=s(10); theta_dot=s(11); psi_dot=s(12);

    % be careful here, when doing motion planning, need to transform global
    % rotational speed to body rotational speed
    my_map = trajectory(time);
    
    x_d = my_map(1); y_d = my_map(2); z_d = my_map(3); 
    phi_d = my_map(4); theta_d = my_map(5); psi_d = my_map(6);
    x_dot_d = my_map(7); y_dot_d = my_map(8); z_dot_d = my_map(9); 
    phi_dot_d = my_map(10); theta_dot_d = my_map(11); psi_dot_d = my_map(12);
    
    %% online parameter identification through projection
    if(isAdapt)
        if(time>0.5)
            global sdot_old Uold s_old kLMonitor
            pmotor = para.kL';
            [fs, gs_thrust, gs_moment] = calThrustMoment(s_old);
            y_id = sdot_old([7,8,9,10,11,12]) - fs([1,2,3,4,5,6]);

            dataMtx = [gs_thrust(1)*Uold';
                        gs_thrust(2)*Uold';
                        gs_thrust(3)*Uold';
                        -gs_moment(1)*[0 Uold(2) 0 -Uold(4)];...
                        -gs_moment(2)*[Uold(1) 0 -Uold(3) 0];
                        gs_moment(3)*[Uold(1) -Uold(2) Uold(3) -Uold(4)]/para.LD_ratio];
            para.kL = projection(pmotor, 2, 0.01, y_id, dataMtx)';
            kLMonitor = [kLMonitor;para.kL];
        end
    end
    
    %% firstly, do an altitude control
    uz = ctlpara.KP_z*(z_d - z) + ctlpara.KP_zdot*(z_dot_d-z_dot);
    FT_des = (constants.g + uz)*para.m/(cos(phi)*cos(theta));
    
    %% then we do a position control
    % calculate desired ux and uy
    ux = ctlpara.KP_x*(x_d - x) + ctlpara.KP_xdot*(x_dot_d-x_dot);
    uy = ctlpara.KP_y*(y_d - y) + ctlpara.KP_ydot*(y_dot_d-y_dot);
    
    % calculate required theta and phi
    theta_d = atan((ux*cos(psi)+uy*sin(psi))/(constants.g + uz));
    phi_d = atan((ux*sin(psi)-uy*cos(psi))/(constants.g + uz))*cos(theta);
    
    % regularization, do not overturn the quadcopter
    if (abs(theta_d)>(60/180*pi))
        theta_d = 15/180*pi*sign(theta_d);
    end
    
    if (abs(phi_d)>(60/180*pi))
        phi_d = 15/180*pi*sign(phi_d);
    end
    
    M1_des = ctlpara.KP_phi*(phi_d-phi)+ctlpara.KP_phi_dot*(phi_dot_d-phi_dot);
    M2_des = ctlpara.KP_theta*(theta_d-theta)+ctlpara.KP_theta_dot*(theta_dot_d-theta_dot);
    M3_des = ctlpara.KP_psi*(psi_d-psi)+ctlpara.KP_psi_dot*(psi_dot_d-psi_dot);
    
    kL = para.kL; kD = kL/para.LD_ratio;
    transMtx = [kL;0,-kL(2),0,kL(4);-kL(1),0,kL(3),0;kD(1),-kD(2),kD(3),-kD(4)];
    U = ((transMtx \ [FT_des;M1_des;M2_des;M3_des]));


end

function [fs, gs_thrust, gs_moment] = calThrustMoment(s)
    global constants para
    cur_para = para;
    phi=s(4); theta=s(5); psi=s(6);
    phi_dot=s(10); 
    theta_dot=s(11); 
    psi_dot=s(12);
    
    Ixx = cur_para.Ixx; Iyy = cur_para.Iyy; Izz = cur_para.Izz; m=cur_para.m;
    
    fs = [0;0;-constants.g; ...
        (Ixx+Iyy-Izz)/Ixx*psi_dot*theta_dot; ...
        (-Ixx-Iyy+Izz)/Iyy*phi_dot*psi_dot;
        (Ixx-Iyy+Izz)/Izz*phi_dot*theta_dot];

    gs_thrust = [sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);...
            -sin(phi)*cos(psi)+cos(phi)*sin(psi)*sin(theta);...
            cos(phi)*cos(theta)]/m;

    gs_moment = [1/Ixx; 1/Iyy; 1/Izz];
end
