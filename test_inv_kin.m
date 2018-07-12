clear all

REVOLUTE = 1;
PRISMATIC = 0;

d_z = [10, 5, 4+3.5, 3.5, 0 2]; 
d_x = [0, 0, 0, 0, 0, 0];
phi_x = [90, 90, 0, -90, 90, 0];
phi_z = [0, 90, 0, 0, 0, 0];
q_max = [3*pi/4, pi/2, 10, pi, pi/2, pi];
q_min = [-3*pi/4, -pi/2, 0, -pi, -pi/2, -pi];

rho = [REVOLUTE REVOLUTE PRISMATIC REVOLUTE REVOLUTE REVOLUTE];

RRP_arm = arm(6,d_z,d_x,phi_z,phi_x,rho, q_max, q_min);

do_Forward_Kinematics(RRP_arm)
plot_arm(RRP_arm.P);

gen_rand_valid_point(RRP_arm)
