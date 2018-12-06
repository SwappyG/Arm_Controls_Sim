%% Constants
REVOLUTE = 1;
PRISMATIC = 0;

% DH convention parameters for each joint
d_z = [6.03, -1.39, -1.24, 1];%12.17];
d_x = [1.79, 12.47, 0, 0];
phi_z = deg2rad([0, 0, 90, 0]);
phi_x = deg2rad([90, 0, 90, -90]); 
rho = [REVOLUTE, REVOLUTE, REVOLUTE, REVOLUTE];

q_min = [-3*pi/6, -pi/6, -5*pi/6, 0];
q_max = [3*pi/6, 5*pi/6, 0, 0];

c_m = [1, 1, 1];
mass = [1, 1, 1];



% Other arm parameters
num_joints = 3;

%% Variables

% Current position of each joint stored in an array

