function setRob( angles, rob )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
% 
%    DESCRIPTION - Update the position of the robot after calling drawRob()
% 
%    This function can be used as is once robFK() and drawRob() have been
%    completed.

[~,rob_T] = robFK(angles,rob);
set(rob.handles(1),'Matrix',rob_T{1});
set(rob.handles(2),'Matrix',rob_T{2});
set(rob.handles(3),'Matrix',rob_T{3});
drawnow;

end

