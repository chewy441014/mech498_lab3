function [ rob_struct ] = robInit()
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    Initialize a structure rob_struct to represent important robot
%    information that will be passed into various Rob simulation
%    functions.
%

% Fill in robot D-H parameters and other necessary parameters 
b = 10; % joint damping [N-m/rad/s]
g = 9.81; % gravity constant [kg/m^2]
m1 = 20; % mass of link 1 [kg]
m2 = 10; % mass of link 2 [kg]
m3 = 10; % mass of link 3 [kg]
m4 = 0; % mass at the end effector [kg]
l1 = 1000; % length of link 1 [mm]
l2 = 700; % length of link 2 [mm]
l3 = 700; % length of link 3 [mm]
lc3 = l3*(m3/2+m4)/(m3+m4); % distance from frame 3 origin to link 3 COM [mm]
I1 = m1*(l1*1e-3)^2/12; % inertia of link 1 [kg-m^2]
I2 = m2*(l2*1e-3)^2/12; % inertia of link 2 [kg-m^2]
I3 = m3*(l3*1e-3)^2/12+m3*((lc3*1e-3)-(l3*1e-3)/2)^2+m4*((l3*1e-3)-(lc3*1e-3))^2; % inertia of link 3 [kg-m^2]
J1 = I1/2; % inertia of actuator 1 [kg-m^2]
J2 = I2/4; % inertia of actuator 2 [kg-m^2]
J3 = I3/4; % inertia of actuator 3 [kg-m^2]
rob_struct.parameters.b = b;
rob_struct.parameters.g = g;
rob_struct.parameters.m1 = m1;
rob_struct.parameters.m2 = m2;
rob_struct.parameters.m3 = m3;
rob_struct.parameters.m4 = m4;
rob_struct.parameters.l1 = l1;
rob_struct.parameters.l2 = l2;
rob_struct.parameters.l3 = l3;
rob_struct.parameters.I1 = I1;
rob_struct.parameters.I2 = I2;
rob_struct.parameters.J1 = J1;
rob_struct.parameters.J2 = J2;
rob_struct.parameters.J3 = J3;


% Robot joint limits (deg)
deg2rad = pi/180;
rob_struct.joint_limits{1} = [-180,180]*deg2rad;
rob_struct.joint_limits{2} = [-90,90]*deg2rad;
rob_struct.joint_limits{3} = [-180,180]*deg2rad;

% Set bounds on the cartesian workspace of the robot
r_xy = 2000;
r_z = 2500;
rob_struct.workspace = [-r_xy,r_xy,-r_xy,r_xy,-r_z/4,r_z];

% Set colors to be drawn for each link and associated frame, including the
% tool
rob_struct.colors = default_colors(4);


    function [ C ] = default_colors( n_colors )
    % Outputs the default colors for MATLAB 2015

    C = {[     0,    0.4470,    0.7410];
         [0.8500,    0.3250,    0.0980];
         [0.9290,    0.6940,    0.1250];
         [0.4940,    0.1840,    0.5560];
         [0.4660,    0.6740,    0.1880];
         [0.3010,    0.7450,    0.9330];
         [0.6350,    0.0780,    0.1840]};
    C = C(1:n_colors);

    end

end

