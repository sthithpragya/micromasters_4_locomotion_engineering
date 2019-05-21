% Robot specification
param.g = 9.81;      % acceleration of gravity
param.r = 0.3;       % spring rest length
param.m = 3;         % ball mass
param.k = 5000;      % vitual spring eleastic constant

% Desired forward speed to regulate
param.xddes = 0.8;

% Initial conditions Q0 = [x y dx dy]
Q0 = [0.2088 0.2984 0.8392 3.0636];

% Search limits for touchdown angles
search_start = deg2rad(0);
search_end   = deg2rad(8);

phitd = ReturnMap(Q0, search_start, search_end, param)

% Equations of motion
function dQ = EOMStance_polar(t, Q, param)
    % Describe the system equations of motion
    % Reminder: the state Q = [q dq]; state output dQ = [dq ddq]
    dQ = zeros(4,1);
    % --- COMPLETE THIS SECTION --- %
    dQ(1) = Q(3);
    dQ(2) = Q(4); 
    dQ(3) = (1/param.m)*(param.m*Q(1)*Q(4)^2-param.k*(Q(1)-param.r)-param.m*param.g*cos(Q(2)));
    dQ(4) = (1/(Q(1)))*(param.g*sin(Q(2))-2*Q(3)*Q(4));
end

% Event function to detect liftoff
function [value,isterminal,direction] = EventLiftOff_polar(t, Q, param)
    % Locate the time when spring length passes through zero and stop
    % integration.
    % --- COMPLETE THIS SECTION --- %
    value = Q(1)-param.r;
    direction = 1;
    isterminal = [1];              % stop the integration
end

function phitd = ReturnMap(Q_current, search_start, search_end, param)
    % Tolerance to stop the simulation
    epsilon = 5*1e-2;
    
    % Number of samples for the search
    NumSampleSpaces = 100;
    
    % Simulation start and end time for each touchdown angle to be searched
    tbegin = 0;
    tfinal = 5;
    
    for i = 1:NumSampleSpaces+1
        % Touchdown angle to check
        thetatd = search_start + ((i-1)/NumSampleSpaces)*(search_end-search_start);
        
        % Find initial conditions in polar coordinates for this specific candidate, thetatd
        % --- COMPLETE THIS SECTION --- %
        x = Q_current(1);
        y = Q_current(2);
        xdot = Q_current(3);
        ydot = Q_current(4);
        Y = param.r*cos(thetatd)-y;
        t = 0.5*(2*ydot/param.g + sqrt(4*ydot^2/param.g^2 - 8*Y/param.g));
        ydot_td = ydot-param.g*t;
        xdot_td = xdot;
        
        ldot_td = ydot_td*cos(thetatd)-xdot_td*sin(thetatd);
        thetadot_td = (xdot_td*cos(thetatd)+ydot_td*sin(thetatd))/(-param.r);
            
        
        
        Qstart = [param.r; thetatd; ldot_td; thetadot_td];
        
        % Run the simulation for this candidate, thetatd
        options = odeset('Events',@(t, Q)EventLiftOff_polar(t, Q, param),'MaxStep',1e-4,'AbsTol',1e-4,'RelTol',1e-4);
        [Tfinal, Qfinal, te, Qe, ie] = ode45(@(t, Q)EOMStance_polar(t, Q, param),[tbegin, tfinal], Qstart, options);
        
        % Find the liftoff forward speed after the sim ends and check if it is epsilon-close to the desired speed
        l_lo = Qe(1);
        theta_lo = Qe(2);
        ldot_lo = Qe(3);
        thetadot_lo = Qe(4);
        xdlo = -l_lo*thetadot_lo*cos(theta_lo)-ldot_lo*sin(theta_lo);
        phitd = thetatd;
        if abs(xdlo - param.xddes) < epsilon
            break
        end
    end
end
