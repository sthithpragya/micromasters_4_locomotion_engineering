% Main Script **********************
% Asigning physical specification
param.g = 9.8;
param.l = 1;    % pendulum length
param.m = 1;    % pendulum mass
param.b = 2;    % damping ratio

% Initial position q0 = [q dq]
q0 = [pi/6 0];
tbegin = 0;
tfinal = 5;

% ODE solving
[T,Q] = ode45(@(t,Q)ForcedPendulum(t, Q, param), [tbegin, tfinal], q0);
plot(T,Q(:,1))
xlabel('time');ylabel('position');

% Equation of motion ****************
function dQ = ForcedPendulum(t, Q, param)

    dQ = zeros(2,1);
    tau  = param.b*param.l*param.l*Q(2);
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -(param.g/param.l)*Q(1)-(param.b/param.m)*Q(2)+(1/(param.m*param.l^2))*tau;
end