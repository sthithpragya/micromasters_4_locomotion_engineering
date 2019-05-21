% Robot specification
param.g = 9.81;
param.d = 0.08;     % ball diameter
param.r = 0.3;      % spring rest length
param.m = 3;        % ball mass
param.k = 5000;     % virtual spring eleastic constant
param.b = 6;        % damping constant

% Control parameters
param.phitd = 0.;           % nominal (first) touchdown angle
% --- COMPLETE THIS SECTION --- %
param.N = 13/5;          % ratio of virtual spring eleastic constant at thrust
param.xddes = 0.8;       % desired forward speed
param.xdgain = 0.025;      % gain for Raibert's speed control
param.thrusttime = 5/1000;  % duration of thrust

% Initial conditions Q0 = [x y dx dy]
Q0 = [0 0.8 0.5 0];
tbegin = 0;
tfinal = 5;
phase = 4;

% Position of the foot
xfoot = Q0(end,1) + param.r*sin(param.phitd);
yfoot = Q0(end,2) - param.r*cos(param.phitd);
saved_data = [Q0 [xfoot yfoot]];

% ODE solving
T = tbegin;
Q = Q0;
while T(end) < tfinal
    if phase == 1
        % stance phase before thrust
        stance_start = Ttemp(end);
        
        options = odeset('Events',@(t, Q)EventBottom(t, Q, param, xfoot_stance),'MaxStep',1e-4,'AbsTol',1e-4,'RelTol',1e-4);
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param, xfoot_stance),[tbegin, tfinal], Q0,...
            odeset('Events',@(t, Q)EventBottom(t, Q, param, xfoot_stance)));
        xfoot = xfoot_stance*ones(size(Ttemp,1),1);
        yfoot = zeros(size(Ttemp,1),1);
        phase = 2;
    elseif phase == 2
        % stance phase, injecting thrust
        [Ttemp, Qtemp] = ...
            ode45(@(t, Q)EOMSpringThrust(t, Q, param, xfoot_stance),...
            [tbegin, tbegin + param.thrusttime], Q0);
        xfoot = xfoot_stance*ones(size(Ttemp,1),1);
        yfoot = zeros(size(Ttemp,1),1);
        phase = 3;
    elseif phase == 3
        % stance phase without thrust
        options = odeset('Events',@(t, Q)EventLiftOff(t, Q, param, xfoot_stance),'MaxStep',1e-4,'AbsTol',1e-4,'RelTol',1e-4);
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMStance(t, Q, param, xfoot_stance),[tbegin, tfinal], Q0, options);
        xfoot = xfoot_stance*ones(size(Ttemp,1),1);
        yfoot = zeros(size(Ttemp,1),1);
        
        % Compute forward speed of the robot
        xd = Qtemp(end,3);
        
        % Compute stance time
        stance_end = Ttemp(end);
        stance_duration = stance_end - stance_start;
        
        % Compute touchdown angle
        % --- COMPLETE THIS SECTION --- %
        param.phitd = asin(xd*stance_duration/(2*param.r)+param.xdgain*(xd-param.xddes)/param.r);
        
        phase = 4;
    elseif phase == 4
        % flight phase
        options = odeset('Events',@(t, Q)EventTouchDown(t, Q, param),'MaxStep',1e-4,'AbsTol',1e-4,'RelTol',1e-4);
        [Ttemp, Qtemp, te, Qe, ie] = ...
            ode45(@(t, Q)EOMFlight(t, Q, param),[tbegin, tfinal], Q0, options);
        xfoot = Qtemp(:,1) + param.r*sin(param.phitd);
        yfoot = Qtemp(:,2) - param.r*cos(param.phitd);
        xfoot_stance = xfoot(end);
        phase = 1;
    end
    nT= length(Ttemp);
    T = [T; Ttemp(2:nT)];
    Q = [Q; Qtemp(2:nT,:)];
    Q0 = Qtemp(nT,:);
    tbegin = Ttemp(nT);
    
    xfoot_saved = xfoot(2:nT,1);
    yfoot_saved = yfoot(2:nT,1);
    saved_data  = [saved_data ; [Qtemp(2:nT,:) xfoot_saved yfoot_saved]];
end

T2 = [0: 1e-2: tfinal];
Q2 = interp1(T, saved_data, T2);
% Simulation Visualization
RunHopperSimulation(T2, Q2, param, 'none');


% Equations of motion
function dQ = EOMStance(t, Q, param, xfoot)
    % Leg angle
    phi = atan2(xfoot-Q(1),Q(2));
    
    % Leg length
    r = sqrt((Q(1)-xfoot)^2+Q(2)^2);
    
    % Leg angle velocity
    phidot = (Q(3)*cos(phi)+Q(4)*sin(phi))/r;
    
    % Leg length velocity
    rdot = -Q(3)*sin(phi)+Q(4)*cos(phi);
    
    % Force and torque to be applied
    % --- COMPLETE THIS SECTION --- %
    F = -param.k*(r-param.r);
    tau = 0;
    
    % Transform F,tau to cartesian coordinates
    % --- COMPLETE THIS SECTION --- %
    Fx = tau*cos(phi)*(-1/r)+F*(-sin(phi));
    Fy = tau*sin(phi)*(-1/r)+F*cos(phi);
    
    % Describe the system equations of motion
    % Reminder: the state Q = [q dq]; state output dQ = [dq ddq]
    dQ = zeros(4,1);
    dQ(1) = Q(3);
    dQ(2) = Q(4);
    dQ(3) = Fx/param.m + param.b*rdot*sin(phi)/param.m;
    dQ(4) = -param.g + Fy/param.m - param.b*rdot*cos(phi)/param.m;
end

function dQ = EOMSpringThrust(t, Q, param, xfoot)
    % Leg angle
    phi = atan2(xfoot-Q(1),Q(2));
    
    % Leg length
    r = sqrt((Q(1)-xfoot)^2+Q(2)^2);
    
    % Leg angle velocity
    phidot = (Q(3)*cos(phi)+Q(4)*sin(phi))/r;
    
    % Leg length velocity
    rdot = -Q(3)*sin(phi)+Q(4)*cos(phi);
    
    % Force and torque to be applied
    % --- COMPLETE THIS SECTION --- %
    F = -param.N*param.k*(r-param.r);
    tau = 0;
    
    % Transform F,tau to cartesian coordinates
    % --- COMPLETE THIS SECTION --- %
    Fx = tau*cos(phi)*(-1/r)+F*(-sin(phi));
    Fy = tau*sin(phi)*(-1/r)+F*cos(phi); 
    
    % Describe the system equations of motion
    % Reminder: the state Q = [q dq]; state output dQ = [dq ddq]
    dQ = zeros(4,1);
    dQ(1) = Q(3);
    dQ(2) = Q(4);
    dQ(3) = Fx/param.m + param.b*rdot*sin(phi)/param.m;
    dQ(4) = -param.g + Fy/param.m - param.b*rdot*cos(phi)/param.m;
end