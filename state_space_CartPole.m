function [dw_dt] = state_space_CartPole(t,w,m,g,L,M)
theta_dotdot = -0.005*sin(t);
x_dotdot = -((m*L)/((2)*(m + M)))*theta_dotdot;
Q = -(L)*(((1/3)*(2*M + (m/2)))*(x_dotdot) + ((m*g)/(2))*(w(2)));
dw_dt_1 = w(3);
dw_dt_2 = w(4);
dw_dt_4 = ((1)/(((m*(L)^(2))/(3)) - (((m*L)^2)/((4)*(M + m)))))*((Q) + (((m*g*L)/(2))*(w(2))));
dw_dt_3 = ((-m*L)/((2)*(M + m)))*(dw_dt_4);
dw_dt = [dw_dt_1;dw_dt_2;dw_dt_3;dw_dt_4];
end