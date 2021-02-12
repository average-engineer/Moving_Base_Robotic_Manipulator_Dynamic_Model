function [joint_torque] = Inv_Dynamics_LE_CartPole

%%
%**************************************************************************
%******************** INVERSE DYNAMICS of SINGLE LEG **********************
%**************************************************************************
% WHERE
% NJ = Number of Joints
% NF = Number of Frames
% DOF = Degree of Freedom of the Cartesian Space
% flag = Decision about the Joint Type
% For Rotary Joint (Flag=1),
% For Prismatic Joint (Flag=0)
%**************************************************************************


%% 
% clear all
% close all
% clc
%% GLOBAL VARIABLES
global NJ link_lengths link_masses COM_prox pole_angle sim_time g
global cart_x
sim_time = linspace(0,10,1000);

%acceleration due to gravity
g = 9.81;

%number of joints
NJ = 1;

for ii = 1:length(sim_time)
    link_lengths{ii} = 0.1;
end

%position of COM wrt proximal frames
for i = 1:NJ
    COM_prox(i) = 0.5;
end

%position of link i COM wrt to ith frame (frame of distal joint of link i) (right)
for ii = 1:length(sim_time)
    for i = 1:NJ
        r{ii}{i} = [-link_lengths{ii}(i)*(1 - COM_prox(i));0;0;1];
    end
end

%pole angle
%rad
pole_angle = 0.005*sin(sim_time);

%pole mass
for i = 1:NJ
    link_masses(i) = 50;
end

%cart mass
cart_mass = 1000;

%cart (base) movement
for ii = 1:length(sim_time)
    for i = 1:NJ
    cart_x(ii) = ((0.005*link_masses(i)*link_lengths{ii}(i))/(cart_mass + link_masses(i)))*sin(sim_time(ii));
    end
end

%cart velocity
cart_x_dot = finite_diff_scalar(sim_time,cart_x);

%cart acceleration
cart_x_dot_dot = finite_diff_scalar(sim_time,cart_x_dot);


%pole angle speeds (rad/s)
pole_angle_vel = finite_diff_scalar(sim_time,pole_angle);

%pole angle accelerations (rad/s2)
pole_angle_acc = finite_diff_scalar(sim_time,pole_angle_vel);

%% DISTAL DH PARAMETERS
%joint variables (according to distal DH parameters of human leg)
%radians
%according to distal DH parameters, there will be 3 joint variables
for ii = 1:length(sim_time)
    theta{ii} = zeros(NJ,1);
    for i = 1:NJ
        theta{ii}(i) = (pi/2) - pole_angle(ii); %radians
    end
end


%joint velocities (1st order time derivative of joint variables)
for ii = 1:length(sim_time)
    theta_dot{ii} = zeros(NJ,1);
    for i = 1:NJ
        theta_dot{ii}(i) = -pole_angle_vel(ii); %radians/second
    end
end

%joint accelerations (2nd order time derivatives of joint variables
for ii = 1:length(sim_time)
    theta_dot{ii} = zeros(NJ,1);
    for i = 1:NJ
        theta_dot_dot{ii}(i) = -pole_angle_acc(ii); %radians/second^2
    end
end
   
%Acceleration due to gravity matrix
gravity_acc = zeros(1,4);
gravity_acc(2) = -g;

%MEMORY ALLOCATION
for i = 1:length(sim_time)
    A{i} = zeros(4,4,NJ);% arm matrix
end

%% FUNCTION CALLS
% Calculate homogeneous tranforms of each frame
[A] = ArmMatrix_dist(A,sim_time,theta,link_lengths,NJ);

%transformation matrix of base frame (hip) wrt inertial global FOR
%the base frame is considered to have the same orientation as the inertial
%frame 
%the translation of the base frame is decribed by the hip trajectory in the
%sagittal plane (X-Y Plane)
for ii = 1:length(sim_time)
    T_b{ii} = [1,0,0,cart_x(ii);
        0,1,0,0;
        0,0,1,0;
        0,0,0,1];
end

%first time derivative of base transformation matrix
T_b_dot = finite_diff_vector(sim_time,T_b);

%second time derivative of base transformation matrix

T_b_dot_dot = finite_diff_vector(sim_time,T_b_dot);

%inertia matrix/tensor for each link wrt to its distal joint frame
J = inertia_matrix_dist(link_lengths,link_masses,r,NJ,sim_time);

%matrices for effect of the movement of one joint on other joint (Uij)
U = U_matrix(A,sim_time,NJ);

%matrices for effect of the movement of two joints on another joint (Uijk)
U1 = U1_matrix(A,sim_time,NJ);


%dynamic coeffient matrices
%inertial acceleration based matrix
D = inertia_acceleration_matrix(sim_time,U,J,NJ);
%coroilis and centrifugal force matrix
h = centri_cor_force_matrix(sim_time,U,U1,J,NJ,theta_dot);
%gravity loading based matrix
c = gravity_loading_matrix(sim_time,link_masses,gravity_acc,U,r,T_b,NJ);
%inertial force due to base movement matrix
p_f = inertial_force_base(sim_time,U,J,A,T_b,T_b_dot_dot,NJ);
%centrifugal and coroilis force due to base movement matrix
p_v = centri_cor_base_matrix(sim_time,U,J,T_b,T_b_dot,NJ);

aa = 1;


%generalized torque matrix at each time instant
for ii = 1:length(sim_time)
    joint_torques{ii} = D{ii}*theta_dot_dot{ii} + h{ii} + c{ii} + p_v{ii}*theta_dot{ii} + p_f{ii};
    joint_torque(ii) = joint_torques{ii};
end

aa = 1;
end