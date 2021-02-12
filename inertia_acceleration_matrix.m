function [D] = inertia_acceleration_matrix(sim_time,U,J,NJ)

for ii = 1:length(sim_time(1:end))
    for i = 1:NJ
        for k = 1:NJ
          D{ii}(i,k) = 0;  
            for j = max(i,k):NJ
                D_1 = trace(U{ii}{j,k}*J{ii}{j}*U{ii}{j,i}');
                D{ii}(i,k) = D{ii}(i,k) + D_1;
            end
        end
    end
end
end