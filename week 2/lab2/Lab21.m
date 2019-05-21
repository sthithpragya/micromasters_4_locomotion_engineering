% Main Script **********************
% Asigning physical specification
param.g = 9.8;
param.l = 1;    % spring rest length
param.m = 1;    % block mass
param.k = 100;  % spring constant
param.b = 2;    % damping ratio

% Initial position q0 = [q dq]
q0 = [1.25*param.l 0];
tbegin = 0;
tfinal = 5;

% ODE solving
[T,Q] = ode45(@(t,Q)DHO(t, Q, param), [tbegin, tfinal], q0);
% plot(Q(:,1),Q(:,2))
% xlabel('position');ylabel('velocity');
RunSHOSimulation(T, Q, param, 'none')

% Equation of motions ****************
function dQ = DHO(t, Q ,param)
    % Describe the system equation of motion derived by Lagrangian method
    % Reminder: the state Q = [q dq]
    % States output dQ = [dq ddq]
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.k/param.m*(Q(1)-param.l)-param.b/param.m*Q(2);
end