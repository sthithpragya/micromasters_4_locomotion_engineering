
function u = controller(params, t, X)
  
u=[0; 0];
l1 = 0.5;
l2 = 0.5; 

kp = 1000;
kd = 5;
    theta1 = X(1);
    theta2 = X(2);
    theta1_dot = X(3);
    theta2_dot = X(4);
    p = [l1*cos(theta1)+l2*cos(theta1+theta2);l1*sin(theta1)+l2*sin(theta1+theta2)];
    e = p - ; 
    v = [theta1_dot; theta2_dot];
    J = [-l1*sin(theta1) -l2*sin(theta1+theta2),-l2*sin(theta1+theta2);l1*cos(theta1) +l2*cos(theta1+theta2),l2*cos(theta1+theta2)];
    u = -kp*J.'*e - kd*v;
  % 2. Let e = p - params.traj(t) be the task-space error
  % 3. Calculate the manipulator Jacobian J = d p / d theta
  % 4. Use a "natural motion" PD controller, u = - kp * J^T * e - kd * [dth1; dth2]
end

