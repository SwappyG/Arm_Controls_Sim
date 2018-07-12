d_z = [3, 10.5, 4+3.5]; 
d_x = [0, 0, 0];
phi_x = [90, 90, 0];
phi_z = [0, 90, 0];
rho = [1 1 0];

r_min = norm([d_z(2) d_z(3)]);
r_max = norm([d_z(2) d_z(3)+24]);
r_delta = r_max-r_min;

r = r_delta*rand()+r_min;

pos = rand(3,1);
pos = pos.*r./norm(pos)

do_Inverse_Kinematics(pos)


