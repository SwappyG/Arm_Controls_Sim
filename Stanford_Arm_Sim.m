%% Description

% Determining the Jacobian of a 3DOF arm + 3DOF wrist

% Trying to find X_dot = [v w]' as a func of qi_dot
% I.E. [v w]' = [Jv Jw]' * [q1_dot ... qn_dot]

%% Robot Parameters

NUM_JOINTS = 6;
NUM_FRAMES = 7;
REVOLUTE = 1;
PRISMATIC = 0;
q = [0 0 0 0 0 0];

% q = 90*(rand(1,6)-0.5);
% q(3) = 5*rand();

% Stanford Manipulator (RRP - RRR)

%   ||   ~  Rev joint, z axis left/right
%   (O)  ~  Rev joint, z axis in/out
%  <[]>  ~  Prsm Joint, z axis left/right
%   =    ~  Rev Joint, z axis up/down
%   -+-  ~  Base

%  (O)
%   | \    
%   =  <[]>--||--(O)--||--
%   |
%  -+-

%% -- -- -- --

% Arm Geometry --
% Define the geometry of the robot using D-H Convention
    % All z_i axis assigned to joint in direction of motion
        % z_i is either in line with linear motion for prismatic or
        % z_i is in line with ang velo vector for revolute
    % origin is selected on z_i where normal to z_i and z_i-1 intersect
        % If z_i and z_i-1 intersect, origin is selecte there
    % x_i should be normal to both z_i and z_i-1 going through origin
        % if z_i and z_i-1 intersect, x_i is normal to z_i and z_i-1 plane
    % Choose y_i to make a right handed co-ordinate system
    
    % d_x is dist between origin and intersect of x_i and z_i-1
    % d_z is dist along z_i-1 from prev origin to intersection of x_i
        % and z_i-1
    % phi_x is angle between z_i-1 and z_i about x_i
    % phi_z is angle between x_i-1 and x_i about z_i-1
d_z = [3, 10.5, 4+3.5, 3.5, 0 2]; 
d_x = [0, 0, 0, 0, 0, 0];
phi_x = [90, 90, 0, -90, 90, 0];
phi_z = [0, 90, 0, 0, 0, 0];
rho = [REVOLUTE REVOLUTE PRISMATIC REVOLUTE REVOLUTE REVOLUTE];

% Forward Kinematics --
    % T is a cell array, each cell is 4x4 transform from i to i-1
    % An is the 4x4 transform from end effector to base frame
    % R is a cell array, each cell is a 3x3 rotation from i to base frame
    % P is a cell array, each cell is a 3x1 vector from i to base frame
    % Z is a cell arrat, each cell is a 3x3 rot. of k_hat_i to base frame
[T, An, R, P, Z] = do_Forward_Kinematics(d_z, d_x, phi_x, phi_z, rho, q);

plot_arm(P); 

% Calculating the Jacobian --
    % J holds the jacobian of the end effector
    % J(1:3, :) is the velocity Jacobian
    % J(4:6, :) is the angular velocity Jacobian
J = calc_Jacobian(Z, P, rho);
%J_inv = inv(J);