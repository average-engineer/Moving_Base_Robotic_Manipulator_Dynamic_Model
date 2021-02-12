function [h_star] = centri_cor_force_matrix(sim_time,U,U1,J,NJ,theta_dot)

for ii = 1:length(sim_time(1:end))
    for i = 1:NJ
        for k = 1:NJ
            for m = 1:NJ
                h{ii}{i}(k,m) = 0;
                for j = max([i,k,m]):NJ
                    h1 = trace(U1{ii}{j}{k,m}*J{ii}{j}*(U{ii}{j,i}'));
                    h{ii}{i}(k,m) = h{ii}{i}(k,m) + h1;
                end
            end
        end
    end
    for i = 1:NJ
        h_star{ii}(i) = (theta_dot{ii}')*h{ii}{i}*theta_dot{ii};
    end
    h_star{ii} = h_star{ii}';
end
end