function [p_v] = centri_cor_base_matrix(sim_time,U,J,T_b,T_b_dot,NJ)

for ii = 1:length(sim_time)
    for i = 1:NJ
        for r = 1:NJ
            p_v{ii}(i,r) = 0;
            for j = max(i,r):NJ
                pv_1 = 2*trace(T_b_dot{ii}*U{ii}{j,r}*J{ii}{j}*(U{ii}{j,i}')*(T_b{ii}'));
                p_v{ii}(i,r) = pv_1 + p_v{ii}(i);
            end
        end 
    end
end
end