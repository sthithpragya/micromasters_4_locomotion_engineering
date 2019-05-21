% Main Script **********************
% Asigning physical specification
param.g = 9.8;
param.l = 1;    % pendulum length
param.m = 1;    % pendulum mass
param.b = 2;    % damping ratio
Edes = (1-cos(1))*param.m*param.g*param.l;

% Initial position q0 = [q dq]
q0 = [pi/100 0];
tbegin = 0;
tfinal = 10;

% ODE solving
[T,Q] = ode45(@(t,Q)AmpControledPendulum(t, Q, param, Edes), [tbegin, tfinal], q0);
plot(T, param.m/2*(param.l^2.*Q(:,2).^2) + param.m*param.g*param.l.*(ones(length(Q))-cos(Q(:,1))))
hold on;
plot(T,Q(:,1))
xlabel('time');ylabel('position');
% RunPendulumSimulation(T, Q, param, 'none')

% Equation of motion ****************
function dQ = AmpControledPendulum(t, Q, param, Edes)
    G = param.g;
    L = param.l;   
    M = param.m;   
    B = param.b;
   
    E = (1/2)*M*(L^2)*(Q(2))^2 + M*G*L*(1-cos(Q(1)));
    kp = 3;
    E_in = (Edes - E);
%     tau = (kp*E_in)*atan(Q(2)) - (0.5)*B*Q(2);
  
    
    tau = -kp*(E_in)*atan(Q(2));    
    dQ = zeros(2,1);
    dQ(1) = Q(2);
    dQ(2) = (-tau-B*Q(2)-M*G*L*Q(1))/(M*L^2);
end