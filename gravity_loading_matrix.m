function [c] = gravity_loading_matrix(sim_time,m,g,U,r,T_b,NJ)

for ii = 1:length(sim_time(1:end))
    c{ii} = zeros(NJ,1);
    for i = 1:NJ
        c{ii}(i) = 0;
        for j = i:NJ
            c1 = -m(j)*g*T_b{ii}*U{ii}{j,i}*r{ii}{j};
            c{ii}(i) = c{ii}(i) + c1;
        end
    end
end
end