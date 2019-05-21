% Main Script **********************
% Asigning physical specification
param.g = 9.8;
param.l = 1;    % pendulum length
param.m = 1;    % pendulum mass

% Initial position q0 = [q dq]
q0 = [pi/4 0];
tbegin = 0;
tfinal = 5;

% ODE solving
[T,Q] = ode45(@(t,Q)Pendulum(t, Q, param), [tbegin, tfinal], q0);
% plot(T,Q(:,1))
% xlabel('time');ylabel('position');
RunPendulumSimulation(T, Q, param, 'none')

% Equation of motion ****************
function dQ = Pendulum(t, Q, param)
    % Describe the system equation of motion derived by Lagrangian method
    % Reminder: the state Q = [q dq]
    % States output dQ = [dq ddq]
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -(param.g/param.l)*sin(Q(1));
end