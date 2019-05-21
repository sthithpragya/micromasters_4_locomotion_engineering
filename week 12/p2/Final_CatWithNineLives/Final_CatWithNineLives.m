% Main Script **********************
% Assigning cat specification
param.g = 9.8;
param.r = 0.2;   % leg/ spring rest length
param.mb = 2.4;  % body mass
param.Ib = 1;    % body inertia
param.lb = 0.3;  % body length
param.mt = 0.2;  % tail mass
param.lt = 0.4;  % tail length

% Control parameters
param.desiredpitch = 0;
param.desireddpitch = 0;
param.Kp = 100;
param.Kv = 15;

% Initial position Q0 = [q dq pitch dpitch theta dtheta]
Q0 = [2 0 pi/2 0 0 0];
tbegin = 0;
tfinal = 5;

% ODE solving
options = odeset('Events',@(t, Q)EventTouchDown(t, Q, param));
[T, Q, te, Qe, ie] = ode45(@(t, Q)EOMFlight(t, Q, param),...
            [tbegin, tfinal], Q0, options);
RunJerboaSimulation(T, Q, param, 'none')

function dQ = EOMFlight(t,Q,param)
    % Describe the system equation of motion
    % Reminder: the state Q = [q dq pitch dpitch theta dtheta]; 
    % state output dQ = [dq ddq dpitch ddpitch dtheta ddtheta]
    
    % Hint for Tau: use param.desiredpitch and param.desireddpitch as your reference
    % and param.Kp for porportional control and param.Kv for derivative control
    Tau = -param.Kp*(Q(3))-param.Kv*(Q(4)); 
    dQ = zeros(6,1);
    dQ(1) = Q(2);
    dQ(2) = -param.g;
    dQ(3) = Q(4);
    dQ(4) = -2*Tau/param.Ib;
    dQ(5) = Q(6);
    dQ(6) = Tau/(param.mt*param.lt^2)+2*Tau/param.Ib;
end

% Event function *********************
function [value,isterminal,direction] = EventTouchDown(t, Q, param)
    % Insert the exact function you wrote in project 2
    value = Q(1)-param.r;         % event point: touch down
    isterminal = 1;
    direction = -1;
end