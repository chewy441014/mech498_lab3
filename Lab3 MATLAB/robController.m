function [ tau ] = robController( trajectory, Theta, Theta_dot, t , rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function sends command torques in the vector tau to move the
%    robot. The controller determines tau based on the desired position and
%    velocity and the actual position and velocity. Desired position and
%    velocity are obtained from trajectory and the current time t. The
%    actua position is Theta, and the actual velocity is Theta_dot.
%    Calculate the torques needed to add gravity compensation using the
%    robot parameters from the structure rob.
%

% Robot Parameters from rob
g = rob.parameters.g;
m1 = rob.parameters.m1;
m2 = rob.parameters.m2;
m3 = rob.parameters.m3;
m4 = rob.parameters.m4;
l1 = rob.parameters.l1/1000;
l2 = rob.parameters.l2/1000;
l3 = rob.parameters.l3/1000;
...
    
% Gravity Compensation Vector
G2=m2*g*l2/2*cos(Theta(2))+...
    m3*g*l2*cos(Theta(2))+m3*g*l3/2*cos(Theta(2)+Theta(3))+...
    m4*g*l2*cos(Theta(2))+m4*g*l3*cos(Theta(2)+Theta(3));

G3=m3*g*l3/2*cos(Theta(2)+Theta(3))+m4*g*l3*cos(Theta(2)+Theta(3));

G = [0;G2;G3]; %[3x1] vector

% Trajectory interpolation (DO NOT CHANGE)
Theta_ref = zeros(3,1);
Theta_dot_ref = zeros(3,1);
for i = 1:3
    Theta_ref(i) = interp1(trajectory(1,:),trajectory(i+1,:),t);
    Theta_dot_ref(i) = interp1(trajectory(1,:),trajectory(i+4,:),t);
end

% Gravity Compensation Control

K_p = [3000;3000;3000]; % Proportional gain matrix containing gains K_p1 to K_p3
K_v = [300;300;300]; % Derivative gain matrix containing gains K_v1 to K_v3

tau = -K_p.*(Theta-Theta_ref)-K_v.*(Theta_dot-Theta_dot_ref)+G; % control input (torque)

end

