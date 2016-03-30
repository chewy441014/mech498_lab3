function [ T ] = dhtf( alpha, a, d, theta )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - Generate a homogeneous transformation matrix from D-H parameters.
%    

T = [            cos(theta),           -sin(theta),           0,             a;
      sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
      sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d;
                          0,                     0,           0,             1];


end

