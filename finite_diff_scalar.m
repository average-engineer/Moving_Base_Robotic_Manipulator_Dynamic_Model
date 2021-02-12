%function for applying the finite difference method in order to compute the
%differentiation of a scalar data set
function output = finite_diff_scalar(sim_time,X)
%for the first data point 
%each data point is a scalar quantity
%On = (Xn+1 - Xn)/(tn+1 - tn)
%for the last data point
%On = (Xn - Xn-1)/(tn - tn-1)
%for the rest of the data
%On = (Xn+1 - Xn-1)/(tn+1 - tn-1)
for i = 1:length(sim_time)
    if i == 1
        output(i)  = (X(i+1) - X(i))/(sim_time(i+1) - sim_time(i));
    elseif i == length(sim_time)
        output(i) = (X(i) - X(i-1))/(sim_time(i) - sim_time(i-1));
    else
        output(i) = (X(i+1) - X(i-1))/(sim_time(i+1) - sim_time(i-1));
    end
end 
%output is a double (numerical value) matrix having the same length as the
%kinematic time vector and each element is a scalar value
end