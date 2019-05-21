function [ ] = brachiatingRobot( )
%BRACHIATINGROBOT Simulates a brachiating robot
%   Detailed explanation goes 
close('all');
m1 = 3.499;
m2 = 1.232;
lc1 = 0.141;
lc2 = 0.333;
l1 = 0.5;
l2 = 0.5;
g = 9.8;
i1 = 0.090;
i2 = 0.033;

y0 = [-0.643501,-1.85459,0,0];

tspan = [0, 5];
abstol=1e-4; 
reltol=1e-3; 
maxstep=1e-3;
options=odeset('RelTol',reltol,'MaxStep',maxstep,'AbsTol',abstol);
[T,Y]=ode23s(@brachiatingDynamics,tspan,y0,options,m1,m2,lc1,lc2,l1,l2,i1,i2,g);
plotBrachBot(Y,T,l1,l2,lc1,lc2); 

end

