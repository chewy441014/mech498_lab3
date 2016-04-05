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
dt=.01;
t=0:dt:t_f;

home_pos = via(:,1);
ballstart = via(:,2);
ballstop = via(:,3);
home_pos2 = via(:,4);

path_cart_1 = ballstart - home_pos;
path_cart_1 = home_pos*ones(1,1001) + [0:path_cart_1(1)/1000:path_cart_1(1); ...
    0:path_cart_1(2)/1000:path_cart_1(2); ...
    0:path_cart_1(3)/1000:path_cart_1(3)];
path_cart_2 = ballstop - ballstart;
path_cart_2 = ballstart*ones(1,1000) + [0:path_cart_2(1)/999:path_cart_2(1); ...
    0:path_cart_2(2)/999:path_cart_2(2); ...
    0:path_cart_2(3)/999:path_cart_2(3)];
path_cart_3 = home_pos2 - ballstop;
path_cart_3 = ballstop*ones(1,1000) + [0:path_cart_3(1)/999:path_cart_3(1); ...
    0:path_cart_3(2)/999:path_cart_3(2); ...
    0:path_cart_3(3)/999:path_cart_3(3)];

joint_angles_mat = zeros(3,3001);
joint_velocity_mat = zeros(3,3001);
for i = 1:length(t)
    if i <= 1001
        pos = path_cart_1(:,i);
    elseif i > 1001 && i <= 2001
        pos = path_cart_2(:,i-1001);
    elseif i > 2001 && i <= 3001
        pos = path_cart_3(:,i-2001);
    else
        disp('Something fucked up')
    end
    
    if i > 1
        [is_solution, joint_angles] = robIK(pos, joint_angles_mat(:,i-1), rob);
    else
        [is_solution, joint_angles] = robIK(pos, [0 0 0], rob);
    end
    if ~is_solution
        error('Inverse Kinematics returned "Not a solution".')
    end
    
    joint_angles_mat(:,i) = joint_angles';
    if i > 1
        joint_velocity_mat(:,i) = (joint_angles_mat(:,i) - ...
            joint_angles_mat(:,i-1))/dt;
    end
end

trajectory(1,:) = t; %Time
trajectory(2:4,:) = joint_angles_mat; %Joint angles
trajectory(5:7,:) = joint_velocity_mat; %Joiint velocities

end

