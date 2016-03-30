function [ J ] = robJacobian( joint_angles, rob )


s1 = sin(joint_angles(1));
c1 = cos(joint_angles(1));
s2 = sin(joint_angles(2));
c2 = cos(joint_angles(2));
s23 = sin(joint_angles(2)+joint_angles(3));
c23 = cos(joint_angles(2)+joint_angles(3));
l2 = rob.parameters.l2*1e-3;
l3 = rob.parameters.l3*1e-3;

J = [ -s1*(c1*l2+c23*l3), -c1*(s2*l2+s23*l3), -c1*s23*l3;
       c1*(c1*l2+c23*l3), -s1*(s2*l2+s23*l3), -s1*s23*l3;
                       0,       c2*l2+c23*l3,     c23*l3 ];
                   


end

