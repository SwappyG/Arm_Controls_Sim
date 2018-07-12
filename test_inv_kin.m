clear all

REVOLUTE = 1;
PRISMATIC = 0;

d_z = [10, 9, 4+3.5+3.5, 0, 0 ,5]; 
d_x = [0, 0, 0, 0, 0, 0];
phi_x = [pi/2, pi/2, 0, -pi/2, pi/2, 0];
phi_z = [0, pi/2, 0, 0, 0, 0];
q_max = [3*pi/4, pi/2, 10, pi, pi/2, pi];
q_min = [-3*pi/4, -pi/2, 0, -pi, -pi/2, -pi];

rho = [REVOLUTE REVOLUTE PRISMATIC REVOLUTE REVOLUTE REVOLUTE];

RRP_arm = arm(6,d_z,d_x,phi_z,phi_x,rho, q_max, q_min);

target_point = gen_rand_valid_point(RRP_arm);
target_pose = roty(pi/4);
do_Inverse_Kinematics(RRP_arm, target_point, target_pose);

inv_kin_point = do_Forward_Kinematics(RRP_arm);
plot_arm(RRP_arm.P);

[target_point RRP_arm.P{7}]
RRP_arm.An;