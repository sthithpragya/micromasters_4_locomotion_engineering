function sys = brachiatingDynamics(t,yi,m_1,m_2,l_c1,l_c2,l_1,l_2,I_1,I_2,g)
%BRACHIATINGDYNAMICS This function contains the EOM for a two-link
%brachiating robot
%   Detailed explanation goes here
theta_1 = yi(1);     theta_2 = yi(2);  %Positions
theta_d1 = yi(3); theta_d2=yi(4); %Velocities

tau = controlHO(yi);

theta1_ddot = ((g*(l_c1*sin(theta_1)*(m_1 + m_2) + l_c2*m_2*sin(theta_1 + theta_2)) - l_c1*l_c2*m_2*theta_d2*sin(theta_2)*(2*theta_d1 + theta_d2))*(m_2*l_c2^2 + I_2))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2)) - ((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)*(l_c1*l_c2*m_2*sin(theta_2)*theta_d1^2 - tau + g*l_c2*m_2*sin(theta_1 + theta_2)))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2));
theta2_ddot = ((l_c1*l_c2*m_2*sin(theta_2)*theta_d1^2 - tau + g*l_c2*m_2*sin(theta_1 + theta_2))*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2)) - ((g*(l_c1*sin(theta_1)*(m_1 + m_2) + l_c2*m_2*sin(theta_1 + theta_2)) - l_c1*l_c2*m_2*theta_d2*sin(theta_2)*(2*theta_d1 + theta_d2))*(m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2))/((m_2*l_c2^2 + l_1*m_2*cos(theta_2)*l_c2 + I_2)^2 - (m_2*l_c2^2 + I_2)*(m_2*l_1^2 + 2*m_2*cos(theta_2)*l_1*l_c2 + m_1*l_c1^2 + m_2*l_c2^2 + I_1 + I_2));

%Updates
sys(1) = theta_d1;
sys(2) = theta_d2;
sys(3) = theta1_ddot;
sys(4) = theta2_ddot;

sys = sys';
end

