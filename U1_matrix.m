function [U1] = U1_matrix(A,sim_time,NJ)

%Q matrix for revolute joint
Q = [0,-1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];
for ii = 1:length(sim_time(1:end))
    for i = 1:NJ
        for j = 1:NJ
            for k = 1:NJ
                if j<=k&&k<=i
                    T1 = T_Concat_dist(A{ii},j-1,0);
                    T2 = T_Concat_dist(A{ii},k-1,j-1);
                    T3 = T_Concat_dist(A{ii},i,k-1);
                    U1{ii}{i}{j,k} = T1*Q*T2*Q*T3;
                elseif k<=j&&j<=i    
                    T4 = T_Concat_dist(A{ii},k-1,0);
                    T5 = T_Concat_dist(A{ii},j-1,k-1);
                    T6 = T_Concat_dist(A{ii},i,j-1);
                    U1{ii}{i}{j,k} = T4*Q*T5*Q*T6;
                elseif j>i||k>i
                    U1{ii}{i}{j,k} = zeros(4,4);
                end
            end
        end
    end
end

end