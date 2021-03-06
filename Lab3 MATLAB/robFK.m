function [ T, rob_T ] = robFK( joint_angles, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
% 
%    Fast computation of robot forward kinematics with a check on joint
%    angle limitations.
% 
%    joint_angles is a 6-elemnt vector of robot joint angles
%
%    rob is a structure generated by robInit()
%
%    T is the transform that relates the end effector to robot base, and rob_T is a cell
%    array of the transforms relating neighboring links

for i = 1:length(joint_angles)
    if rob.joint_limits{i}(1) > joint_angles(i)...
            || rob.joint_limits{i}(2) < joint_angles(i)
        warning(['Joint angle ',num2str(i),' outside of limits.']);
    end
end

% FANUC DH transforms
T_0_1 = dhtf(0,0,rob.parameters.l1,joint_angles(1));
T_1_2 = dhtf(pi/2,0,0,joint_angles(2));
T_2_3 = dhtf(0,rob.parameters.l2,0,joint_angles(3)+pi/2);
T_3_4 = dhtf(pi/2,0,rob.parameters.l3,0);

% Populate FANUC cell array with FK transforms
rob_T = {T_0_1,T_1_2,T_2_3,T_3_4};

% Output full transform
T = T_0_1*T_1_2*T_2_3*T_3_4;


end



