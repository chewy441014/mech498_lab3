function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();

m_1=robot.m_1;
m_2=robot.m_2;
m_r1=robot.m_r1;
m_r2=robot.m_r2;
l_1=robot.l_1;
l_2=robot.l_2;
g=robot.g;

I1=1/12*m_r1*l_1^2+m_1*(l_1/2)^2
I2=1/12*m_r2*l_2^2+m_2*(l_2/2)^2

M1=m_1+m_r1
M2=m2+m_r2

Lc1=((m1+1/2*m_r1)*l_1)/(M1)
Lc2=((m2+1/2*m_r2)*l_2)/(M2)


% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.01; % [s]
t_f = 10; % [s]

% Initial Conditions
X_0 = [0; 0];

% Control Gains (Scalar)
K_p = 1;
K_v = 1;

% Numerical Integration
t = 0:dt:t_f;
X = []; % initialize variable to hold state vector
X_dot = []; % initialize variable to hold state vector derivatives
for i = 1:length(t)
    if i == 1

    else

    end
    
    % Control torques
    tau = [];
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M1*Lc1^2+M2*(l_1+Lc2*cos(X(2)))^2+I1+I2, 0 ; 0, M2*Lc2^2+I2];
    C = [-2*M2*Lc2*sin(X(2))*(l_1+Lc2*cos(X(2)))*X_dot(1)*X_dot(2);...
        M2*Lc2*sin(X(2))*(l_1+Lc2*cos(X(2)))*X_dot(1)^2];
    G = [0;M2*g*Lc2*cos(X(2))];

    X_dot(i,:) = [];
    
    % Trapezoidal Integration
    if i > 1
        
    end
    
    % Plot Energy
    
end

% Graphical Simulation
robot.handles = drawRR([],robot);
for i = 2:length(t)
    setRR([],robot);
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output





end

