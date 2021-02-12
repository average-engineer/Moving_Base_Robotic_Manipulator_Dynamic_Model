function [p_f] = inertial_force_base(sim_time,U,J,A,T_b,T_b_dot_dot,NJ)

for ii = 1:length(sim_time)
    for i = 1:NJ
        p_f{ii}(i) = 0;
        for j = i:NJ
            T1 = T_Concat_dist(A{ii},j,0);
            p_f1 = trace(T_b_dot_dot{ii}*T1*J{ii}{j}*(U{ii}{j,i}')*(T_b{ii}'));
            p_f{ii}(i) = p_f{ii}(i) + p_f1;
        end
    end
    p_f{ii} = p_f{ii}';
end
end