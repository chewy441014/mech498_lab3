function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

    close all;

    % Initialize robot
    robot = RRInit();

    m_1=robot.m_1;
    m_2=robot.m_2;
    m_r1=robot.m_r1;
    m_r2=robot.m_r2;
    l_1=robot.l_1;
    l_2=robot.l_2;
    g=robot.g;

    I1=1/12*m_r1*l_1^2+m_1*(l_1/2)^2;
    I2=1/12*m_r2*l_2^2+m_2*(l_2/2)^2;

    M1=m_1+m_r1;
    M2=m_2+m_r2;

    Lc1=((m_1+1/2*m_r1)*l_1)/(M1);
    Lc2=((m_2+1/2*m_r2)*l_2)/(M2);


    % Joint Torque Limit
    tau_max = 20; % [N-m] (Scalar)

    % Time
    dt = 0.01; % [s]
    t_f = 10; % [s]

    % Initial Conditions
    X_0 = [pi/3; 0; pi/2; 0];
    % [theta1 theta1' theta2 theta2']

    % Control Gains (Scalar)
    K_p = 100;
    K_v = 50;

    % Numerical Integration
    t = 0:dt:t_f;
    X = zeros(4,length(t)); % initialize variable to hold state vector
    X_dot = zeros(4,length(t)); % initialize variable to hold state vector derivatives
    thetad=[0;pi/2];
    u = zeros(2,length(t));
    k = zeros(2,length(t));
    for i = 1:length(t)
        if i == 1
            X(:,1)=X_0;
        else
            %Use X calculated from previous calculation

        % Control torques
        tau = -K_p*(X([1,3],i-1)-thetad)-K_v*X([2,4],i-1);

        % Apply joint torque limits
        tau(tau>tau_max) = tau_max;
        tau(tau<-tau_max) = -tau_max;

        % Dynamic Model
        M = [M1*Lc1^2+M2*(l_1+Lc2*cos(X(3,i-1)))^2+I1+I2, 0 ; 0, M2*Lc2^2+I2];
        C = [-2*M2*Lc2*sin(X(3,i-1))*(l_1+Lc2*cos(X(3,i-1)))*X(2,i-1)*X(4,i-1);...
            M2*Lc2*sin(X(3,i-1))*(l_1+Lc2*cos(X(3,i-1)))*X(2,i-1)^2];
        G = [0;M2*g*Lc2*cos(X(3,i-1))];

        thetapp=inv(M)*(tau-C-G);
        X_dot(:,i) = [0;thetapp(1);0;thetapp(2)];


        % Trapezoidal Integration
        if i > 1
            X_dot(1,i)=X_dot(1,i-1)+.5*(X_dot(2,i-1)+X_dot(2,i))*dt;
            X_dot(3,i)=X_dot(3,i-1)+.5*(X_dot(4,i-1)+X_dot(4,i))*dt;
            X(2,i)=X_dot(1,i);
            X(4,i)=X_dot(3,i);

            X(1,i)=X(1,i-1)+.5*(X(2,i-1)+X(2,i))*dt;
            X(3,i)=X(3,i-1)+.5*(X(4,i-1)+X(4,i))*dt;

        end

        % Plot Energy
        k(:,i) = 1/2*X_dot([1,3],i)'*M*X_dot([1,3],i);
        u(2,i) = M2*[0;0;g]'*[0;0;sin(X(3,i))*Lc2];
        
        end

    end

    joint_angles1=X(1,:);
    joint_angles2=X(3,:);
    
    % Graphical Simulation
    robot.handles = drawRR([X_0(1),X_0(3)],robot);
    for i = 2:length(t)
        setRR([joint_angles1(i), joint_angles2(i)],robot);
        pause(.0001); % adjustable pause in seconds
    end

    % Plot Output
    figure;
    subplot(5,1,1)
    plot(t,k(1,:))
    ylabel('KE \theta_1')
    title('Energy Plots')
    subplot(5,1,2)
    plot(t,u(1,:))
    ylabel('PE \theta_1')
    subplot(5,1,3)
    plot(t,k(2,:))
    ylabel('KE \theta_2')
    subplot(5,1,4)
    plot(t,u(2,:))
    ylabel('PE \theta_2')
    subplot(5,1,5)
    plot(t,u(1,:)+u(2,:)+k(1,:)+k(2,:))
    ylabel('Total Energy')
    xlabel('Time (sec)')
    figure;
    plot(t,X(1,:))
    xlabel('Time (s)')
    ylabel('Theta 1 position')
    title('Theta 1 Position vs. Time')
    
    figure
    plot(t,X(3,:))
    xlabel('Time (s)')
    ylabel('Theta 2 position')
    title('Theta 2 position vs. Time')
    

end

