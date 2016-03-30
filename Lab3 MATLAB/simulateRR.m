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

% Joint Torque Limit
tau_max = []; % [N-m] (Scalar)

% Time
dt = []; % [s]
t_f = []; % [s]

% Initial Conditions
X_0 = [];

% Control Gains (Scalar)
K_p = [];
K_v = [];

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
    M = [];
    C = [];
    G = [];

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

