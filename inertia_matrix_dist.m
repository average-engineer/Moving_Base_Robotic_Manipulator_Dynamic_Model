function [J] = inertia_matrix_dist(link_lengths,m,r,NJ,sim_time)
%units: kgm2
%ASSUMPTIONS
%the inertias of each segment along the X axis of its distal joint frame is assumed
%to be negligible
%the inertias of each segment along the Y and Z axes of its distal frame are
%assumed to be equal
%cross inertias (Ixx,Ixy,Ixz) are assumed to be zero

%inertias of link i about the frame of distal joint
for ii = 1:length(sim_time)
    Ixx{ii} = zeros(NJ,1);
    Iyy{ii} = zeros(NJ,1);
    Izz{ii} = zeros(NJ,1);
    Ixy{ii} = zeros(NJ,1);
    Iyz{ii} = zeros(NJ,1);
    Ixz{ii} = zeros(NJ,1);
end

for ii = 1:length(sim_time)
    for i = 1:NJ
        %inertia matrix for each link i
        %the inertias are about the joints distal to the links/segments
        %applying parallel axis theorem
        Iyy{ii}(i) = (m(i)*(link_lengths{ii}(i))^2)/3;
        Izz{ii}(i) =(m(i)*(link_lengths{ii}(i))^2)/3;
        
        J{ii}{i} = [(-Ixx{ii}(i) + Iyy{ii}(i) + Izz{ii}(i))/2,Ixy{ii}(i),Ixz{ii}(i),m(i)*r{ii}{i}(1);
            Ixy{ii}(i),(Ixx{ii}(i) - Iyy{ii}(i) + Izz{ii}(i))/2,Iyz{ii}(i),m(i)*r{ii}{i}(2);
            Ixz{ii}(i),Iyz{ii}(i),(Ixx{ii}(i) + Iyy{ii}(i) - Izz{ii}(i))/2,m(i)*r{ii}{i}(3);
            m(i)*r{ii}{i}(1),m(i)*r{ii}{i}(2),m(i)*r{ii}{i}(3),m(i)];
    end
end
end