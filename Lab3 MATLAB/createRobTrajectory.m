function [ trajectory ] = createRobTrajectory( via, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - Generate a joint position and velocity trajectory to be
%    used by the controller.
%
%    The input via is a 3x4 matrix containing via points for the end
%    effector. Each column contains a point in 3D space. The trajectory
%    should move sequentially through each via point. The best approach is
%    to simply move in a straight line in cartesian space between the
%    points. Use robIK() to find the corresponding trajectory in joint
%    space.
%
%    The output trajectory is a 7xn matrix. The first row will be
%    equally-spaced time stamps starting at time zero and finishing at time
%    t_f given to you below. Rows 2 through 4 contain joint angles,
%    and rows 5 7 should contain joint velocities.

t_f = 30; % final time (do not change) [s]

trajectory(1,:) = []; %Time
trajectory(2:4,:) = []; %Joint angles
trajectory(5:7,:) = []; %Joiint velocities

end

