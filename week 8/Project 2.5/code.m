% Main Script **********************
% Assigning robot specification
param.g = 9.8;
param.d = 0.08;     % ball diameter
param.r = 0.3;      % spring rest length/ chi0
param.m = 3;        % ball mass
param.k = 5000;     % vitual spring eleastic constant
param.b = 6;        % damping constant
param.tau = 750;    % thrust yalue applied (you don't need to change this)
param.thrusttime = 0.005;  % duration of thrust
param.l1 = 0.15;    % first linkage
param.l2 = 0.3;     % second linkage


% Initial position Q0 = [q dq]
Q0 = [1 0];
tbegin = 0;
tfinal = 5;
phase = 4;

% ODE solving
T = tbegin;
Q = Q0;
while T(end) < tfinal
    if phase == 1
        % touch down, into stance phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventBottom(t, Q, param)));
        phase = 2;
    elseif phase == 2
        % stance phase, injecting thrust
        [Ttemp, Qtemp] = ...
            ode45(@(t, Q)EOMStanceThrusted(t, Q, param),...
            [tbegin, tbegin + param.thrusttime], Q0);
        phase = 3;
    elseif phase == 3
        % stance phase without thrust
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventLiftOff(t, Q, param)));
        phase = 4;
    elseif phase == 4
        % lift off, into flight phase
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMFlight(t, Q, param),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventTouchDown(t, Q, param)));
        phase = 1;
    end
    nT= length(Ttemp);
    T = [T; Ttemp(2:nT)];
    Q = [Q; Qtemp(2:nT,:)];
    Q0 = Qtemp(nT,:);
    tbegin = Ttemp(nT);
end

xi = fivebarIK(Q , param)
plot(T,xi)
legend('Position','COM Velocity')

function dQ = EOMFlight(t, Q, param)
    % Flight phase (from liftoff to touchdown)
    % insert your previous code here
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.g;    
end
function dQ = EOMStance(t, Q, param)
    % Stance phase (from touchdown to liftoff)
    % insert your previous code here
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.g-param.b*Q(2)/param.m-param.k*(Q(1)-param.r)/param.m;
end
function dQ = EOMStanceThrusted(t,Q,param)
    % Define Thrust phase (applied at the bottom for param.thrusttime)
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = -param.g-param.b*Q(2)/param.m-param.k*(Q(1)-param.r)/param.m + param.tau/param.m;
end

% Event functions *********************
function [value,isterminal,direction] = EventTouchDown(t, Q, param)
    % insert your previous code here
    value = Q(1)-param.r;         % event point: touch down
    isterminal = 1;
    direction = -1;
end
function [value,isterminal,direction] = EventLiftOff(t, Q, param)
    % insert your previous code here
    value = param.r-Q(1);         % event point: touch down
    isterminal = 1;
    direction = -1;
    
end
function [value,isterminal,direction] = EventBottom(t, Q, param)
    % Define the lowest point
    value = Q(2);
    isterminal = 1;
    direction = 1;
end

% Find Inverse Kinematics ****************
function xi = fivebarIK(Q , param)
    % Find the inverse kinematics of the five bar linkage so that the out put is the motor angle xi
    m = size(Q,1);
    xi = zeros(m,1);
    for i = 1:m
        if Q(i,1) > param.r
            xi(i) = pi-acos((param.r^2+param.l1^2-param.l2^2)/(2*param.r*param.l1)); 
        else
            xi(i) = pi-acos((Q(i,1)^2+param.l1^2-param.l2^2)/(2*Q(i,1)*param.l1));
        end
    end
end

