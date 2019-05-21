syms L1 L2 R Phi Xdot Ydot Tau1 Tau2 Phidot t Ldot
k1 = (L1^2-L2^2-R^2)/(2*L1*R^2);
P = (L2^2-L1^2-R^2)/(2*L1*R);
M = k1/sqrt(1-P^2);
A = [-M -1; M -1];
B = [-sin(Phi) cos(Phi); -cos(Phi)/R -sin(Phi)/R];
Pt = [Tau1 Tau2]*A*B*[Xdot;Ydot];
Fx = diff(Pt, Xdot);
Fy = diff(Pt, Ydot);

param = [Tau1, Tau2, L1, L2, R, Phi];
%param_val = [tau1,tau2,l1,l2,r,phi];

th1 = -Phidot*t + acos((L2^2-L1^2-(Ldot*t)^2)/(2*L1*Ldot*t));
th2 = -Phidot*t - acos((L2^2-L1^2-(Ldot*t)^2)/(2*L1*Ldot*t));
th1d = simplify(diff(th1,t))
th2d = diff(th2,t);




