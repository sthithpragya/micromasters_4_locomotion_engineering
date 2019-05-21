syms theta_1 theta_2 l_1 l_c1 l_2 l_c2 theta_d1 theta_d2 m_2


x2y = -l_1*cos(theta_1) - l_c2*cos(theta_1 + theta_2);
x2x = l_1*sin(theta_1) + l_c2*sin(theta_1 + theta_2);
ex = l_1*sin(theta_1);
ey = -l_1*cos(theta_1);

jv2 = [-x2y, -x2y+ey;
    x2x, x2x-ex;
    0, 0];

td = [theta_d1; theta_d2];

x2d = l_1*cos(theta_1)*theta_d1 + l_c2*cos(theta_1 + theta_2)*(theta_d1+theta_d2);
y2d = l_1*sin(theta_1)*theta_d1 + l_c2*sin(theta_1 + theta_2)*(theta_d1+theta_d2);

simplify(((m_2)*(x2d^2 + y2d^2))/2)