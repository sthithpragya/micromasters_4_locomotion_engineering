tspan = [0, 40];
abstol=1e-4; 
reltol=1e-3; 
maxstep=1e-3;
y0 = [pi/4,-pi/6,0,0];
options=odeset('Events',@swingEvent,'RelTol',reltol,'MaxStep',maxstep,'AbsTol',abstol);
[T,Y]=ode23s(@swingDynamics,tspan,y0,options);
tspan = [T(end), 41];
y0 = Y(end,:);
y0(3) = 0;% -y0(3);
y0(1) = abs(y0(1))*pi/12;
options=odeset('Events',@swingEvent,'RelTol',reltol,'MaxStep',maxstep,'AbsTol',abstol);
[T1,Y1]=ode23s(@swingDynamics,tspan,y0,options);
plot([T;T1],[Y(:,1);Y1(:,1)]);
hold on;
plot([T;T1],[Y(:,2);Y1(:,2)]);



function [value, isterminal, direction] = swingEvent(t,yi)
l=5;
g=9.8;
th1 = yi(1);
th2 = yi(2);
th1_d = yi(3);
th2_d = yi(4);
e1 = (1/2)*(l^2*th1_d^2) + g*(l-(l*cos(th1)));
e2 = (1/2)*(l^2*th2_d^2) + g*(l-(l*cos(th2)));

pe1 = g*(l-(l*cos(th1)));
pe2 = g*(l-(l*cos(th2)));
edif = abs(pe1/e1-pe2/e2) %edif = abs(g*(l-(l*cos(th1)))/e1-g*(l-(l*cos(th2)))/e2);
v1 = 1;
if(edif<.5)
    if sign(th1)*sign(th2) > 0
        if sign(th1_d)*sign(th2_d) > 0
            v1 = -1;
        end
    end
end


value = v1; %abs((yi(1)-yi(2))+sign(yi(3))*1000 - sign(yi(4))*1000)-.02;
isterminal  = 1 ; 
direction  = 0;
end

function sys = swingDynamics(t,yi)
th1 = yi(1);
th2 = yi(2);
th1_d = yi(3);
th2_d = yi(4);
g = 9.8;
l = 5;
km = .3;
kd = .2;
emax = l*(1-cos(pi/4));
e1 = (1/2)*(l^2)*(th1_d)^2 + g*l*(1-cos(th1));
e2 = (1/2)*(l^2)*(th2_d)^2 + g*l*(1-cos(th2));
th1_dd = -g/l*th1 + km*(th1-th2) + 2*e1*atan(th1_d)*(emax-e1) - kd*(th1_d-th2_d) ;
th2_dd = -g/l*th2 + km*(th2-th1) + 2*e2*atan(th2_d)*(emax-e2)- kd*(th2_d-th1_d) ;

sys(1) = th1_d;
sys(2) = th2_d;
sys(3) = th1_dd;
sys(4) = th2_dd;

sys = sys';

end