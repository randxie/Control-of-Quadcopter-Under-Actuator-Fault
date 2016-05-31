clc;
clear all;
close;

%% Decide whether to do adaptation or not; Also decide whether there is thrust loss or not
    global isAdapt isFault
    isAdapt = 0;
    isFault = 1;
    
%% Define initial states
    % Define system states
    x0=0;   y0=0;   z0=0;   phi0=0;    theta0=0;    psi0=0;
    x_dot0=0;   y_dot0=0;   z_dot0=0;   phi_dot0=0; theta_dot0=0;   psi_dot0=0;
    sNorm0=[x0,y0,z0,phi0,theta0,psi0,x_dot0,y_dot0,z_dot0,phi_dot0,theta_dot0,psi_dot0]';

    % Define simulation states
    s0 = sNorm0;

%% set system structure and control parameters

% for detecting parameter change
    global wt st rt gamma1 gamma2 r0 rstat kLMonitor
    kLMonitor = [];
    wt = zeros(4,1);
    rt = zeros(4,1);
    r0 = 1;
    gamma1 = 0.3;
    gamma2 = 0.3;

% define parameters
    global constants para 
    constants.g = 9.8;
    para.m=1;
    para.Ixx=10^-1;
    para.Iyy=10^-1;
    para.Izz=10^-1;
    para.kL = para.m*constants.g/(4*200^2) * ones(1,4);
    para.LD_ratio = 15; % assume such ratio does not change
    
    constants.maxKL = para.m*constants.g/(4*200^2); % known

% PD controller parameter
    global ctlpara 
    % pose angle control
    ctlpara.KP_phi=3;
    ctlpara.KP_phi_dot=0.9;

    ctlpara.KP_theta=3;
    ctlpara.KP_theta_dot=0.9;

    ctlpara.KP_psi=3;
    ctlpara.KP_psi_dot=0.9;

    % position control
    ctlpara.KP_x = 0.5;
    ctlpara.KP_xdot = 2;

    ctlpara.KP_y = 0.5;
    ctlpara.KP_ydot = 2;

    % altitude control
    ctlpara.KP_z = 3;
    ctlpara.KP_zdot = 2;

    
%% simulaiton
    h = 0.005;
    time=0:h:250;
    para.h = h;
    
    % generate process noise for sensor and disturbance
    para.noise_table = addSensorNoise(time,sNorm0);
    para.disturb_table = addDisturbance(time,sNorm0);
    
    % find desired trajectory
    ref_traj = zeros(numel(time), numel(sNorm0));
    for i = 1:numel(time)
        ref_traj(i,:) = trajectory(time(i))';
    end

    % use Euler method for solving system
    [t,sout]=Euler(@dyneqn,time,s0); 

%% plotting
% plot different states
figure;
subplot(2,2,1);
    plot(t,(sout(:,1)),t,(sout(:,2)),'r--',t,(sout(:,3)),'k:', 'linewidth', 2);
    xlabel('Time(s)');
    ylabel('Displacement(m)');
    legend('x','y','z');

subplot(2,2,3);
    plot(t,(sout(:,7)),t,(sout(:,8)),'r--',t,(sout(:,9)),'k:', 'linewidth', 2);
    xlabel('Time(s)');
    ylabel('Velocity(m/s)');
    l1 = legend('$\dot{x}$','$\dot{y}$','$\dot{z}$');
    set(l1, 'interpreter', 'latex')

subplot(2,2,2);
    plot(t,(sout(:,4))/pi*180,t,(sout(:,5))/pi*180,'r--',t,(sout(:,6))/pi*180,'k:', 'linewidth', 2);
    xlabel('Time(s)');
    ylabel('Euler angle(degree)');
    legend('\phi','\theta','\psi');

subplot(2,2,4);
    plot(t,(sout(:,10)),t,(sout(:,11)),'r--',t,(sout(:,12)),'k:', 'linewidth', 2);
    xlabel('Time(s)');
    ylabel('Rotational Velocity (degree/s)');
    l2 = legend('$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$');
    set(l2, 'interpreter', 'latex')

% 3D trajectory
figure;
    plot3((sout(:,1)),(sout(:,2)),(sout(:,3)),'k:', 'linewidth', 2);
    hold on;
    plot3((ref_traj(:,1)),(ref_traj(:,2)),(ref_traj(:,3)),'r--', 'linewidth', 0.5);
    xlabel('x (m)');
    ylabel('y (m)');
    zlabel('z (m)');
    axis([-0.5 0.5 -0.5 0.5 0 1])

% tracking error
figure;
    distance = sout(:,1:3)-ref_traj(:,1:3);
    distance = sqrt(mean((distance.^2),2));
    plot(t,distance,'k:', 'linewidth', 2);
    xlabel('Time(s)');
    ylabel('3D tracking error (m)');

