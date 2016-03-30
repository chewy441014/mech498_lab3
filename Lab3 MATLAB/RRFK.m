function [T, robot_T] = RRFK(joint_angles,robot)
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Laura Blumenschein 
% 
%    DESCRIPTION - Calculate the forward kinematics of the RR robot for
%    plotting
%

L_1 = robot.l_1;
theta_1 = joint_angles(1);
theta_2 = joint_angles(2);

T01 = [cos(theta_1) -sin(theta_1) 0 0;...
       sin(theta_1) cos(theta_1) 0 0;...
       0 0 1 0;...
       0 0 0 1];
T12 = [cos(theta_2) -sin(theta_2) 0 L_1;...
       0 0 -1 0;...
       sin(theta_2) cos(theta_2) 0 0;...
       0 0 0 1];
T = T01*T12;
robot_T = {T01,T12};
end