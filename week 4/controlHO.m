function [ tau ] = controlHO( yi )
%CONTROLHO Controller to force brachiating robot to be a harmonic
%oscillator
%   Detailed explanation goes here
m_1 = 3.499;
m_2 = 1.232;
l_c1 = 0.141;
l_c2 = 0.333;
l_1 = 0.5;
l_2 = 0.5;
g = 9.8;
I_1 = 0.090;
I_2 = 0.033;

theta_1 = yi(1);
theta_2 = yi(2);
theta_d1 = yi(3);
theta_d2 = yi(4);

theta = yi(1)+.5*yi(2);
omega = 3.321;
%Copy in the controller you made here:

tau = (((g*(sin(theta_1)*(l_1*m_2 + l_c1*m_1) + l_c2*m_2*sin(theta_1 + theta_2)) - l_1*l_c2*m_2*theta_d2*sin(theta_2)*(2*theta_d1 + theta_d2))*(m_2*l_c2^2 + I_2))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2)) - ((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)*(l_1*l_c2*m_2*sin(theta_2)*theta_d1^2 - tau + g*l_c2*m_2*sin(theta_1 + theta_2)))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2)))*(m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2) - (m_2*l_c2^2 + I_2)*(((g*(sin(theta_1)*(l_1*m_2 + l_c1*m_1) + l_c2*m_2*sin(theta_1 + theta_2)) - l_1*l_c2*m_2*theta_d2*sin(theta_2)*(2*theta_d1 + theta_d2))*(m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2)) - ((l_1*l_c2*m_2*sin(theta_2)*theta_d1^2 - tau + g*l_c2*m_2*sin(theta_1 + theta_2))*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2))) + g*l_c2*m_2*sin(theta_1 + theta_2) + l_1*l_c2*m_2*theta_d1^2*sin(theta_2);

end

