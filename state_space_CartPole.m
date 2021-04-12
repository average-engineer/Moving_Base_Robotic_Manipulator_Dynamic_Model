function [dw_dt] = state_space_CartPole(t,w,m,g,L,M)
theta_dotdot = -0.005*sin(t);
x_dotdot = -((m*L)/((2)*(m + M)))*theta_dotdot;
Q = -(L)*(((1/3)*(2*M + (m/2)))*(x_dotdot) + ((m*g)/(2))*(w(2)));
% dw_dt_1 = w(3);
% dw_dt_2 = w(4);
% dw_dt_4 = ((1)/(((m*(L)^(2))/(3)) - (((m*L)^2)/((4)*(M + m)))))*((Q) + (((m*g*L)/(2))*(w(2))));
% dw_dt_3 = ((-m*L)/((2)*(M + m)))*(dw_dt_4);
% dw_dt = [dw_dt_1;dw_dt_2;dw_dt_3;dw_dt_4];
% Mass Matrix
M_mat = [m + M,(m*L)/2;(m*L)/2,(m*(L)^2)/3];
% Stiffness Matrix
K_mat = [0,0;0,(-m*g*L)/2];
% system matrix
A = [zeros(2,2),eye(2,2);-M_mat\K_mat,zeros(2,2)];
% input matrix
B = [zeros(2,2);M_mat\eye(2,2)];
% input vector
u = [0;Q];

% state space representation
dw_dt = A*w + B*u;
end