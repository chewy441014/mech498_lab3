function setRR( angles, robot )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
% 
%    DESCRIPTION - Update the position of the robot after calling drawRR()
% 

[~,robot_T] = RRFK(angles,robot);
set(robot.handles(1),'Matrix',robot_T{1});
set(robot.handles(2),'Matrix',robot_T{2});
drawnow;

end

