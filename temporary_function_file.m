%% Constants
REVOLUTE = 1;
PRISMATIC = 0;

% DH convention parameters for each joint
robot.d_z = [3, 0, 0];
robot.d_x = [0, 12, 12];
robot.phi_z = [-90, 0, 0];
robot.phi_x = [0, 0, 0]; 
robot.rho = [REVOLUTE, REVOLUTE, REVOLUTE];

% Other arm parameters
robot.num_joints = 3;

%% Variables

% Current position of each joint stored in an array
robot.q = [0, 0, 0];

% Holds intermediate transforms, rotations, positions and z-axis orientations
robot.T = cell(robot.num_joints + 1, 1);
robot.R = cell(robot.num_joints + 1, 1);
robot.P = cell(robot.num_joints + 1, 1);
robot.Z = cell(robot.num_joints + 1, 1);


% Provides one of 4 DH basic transforms
function A_sub = get_DH_component(param, type)

	A_sub = eye(4,4);

	switch type
		case 'd_z'
			A_sub(3,4) = param;
		case 'd_x'
			A_sub(1,4) = param;
		case 'phi_z'
			A_sub(1:3, 1:3) = rotz( param );
		case 'phi_x'
			A_sub(1:3, 1:3) = rotx( param );
		otherwise
			error("Not a valid DH component");
	end
	
end


% Calculates the transform a joint given all DH params
function A = get_DH_transform(d_z, d_x, phi_z, phi_x)

	A_Dz = get_DH_component(d_z, 'd_z');
	A_Dx = get_DH_component(d_x, 'd_x');
	A_Rz = get_DH_component(phi_z, 'phi_z');
	A_Rx = get_DH_component(phi_z, 'phi_x');

	A = A_Rz * A_Dz * A_Dx * A_Rx;

end


% Performs forward kinematics given state and DH params
function do_forward_kinematics(robot)

	% Holds transform of end effector
	A_n = eye(4,4);
	
	% z dir unit vector, for extract orientation of Z-axis of each joint
	k_hat = [0, 0, 1]';
	
	% iterate through all joints
	for index = 1:robot.num_joints
	
		% Combine arm geometry and joint state for current DH params
		d_z = robot.d_z(index) + ~robot.rho(index)*robot.q(index);
		d_x = robot.d_x(index);
		phi_z = robot.phi_z(index) + robot.rho(index)*robot.q(index);
		phi_x = robot.phi_x(index)
	
		% Calculate transform between this joint and prev joint
		robot.T(index) = { get_DH_transform( d_z, d_x, phi_z, phi_x ) };
		
		% Chain transforms to get transform between end effector and base
		A_n = A_n * robot.T{index};
		
		% Extract out rotation, position and z-axis orientation for this joint
		robot.R(index+1) = { A_n(1:3,1:3) };
		robot.P(index+1) = { A_n(1:3,4) };
		robot.Z(index+1) = { A_n(1:3,1:3) * k_hat) };
	
	end

	robot.A_n = A_n;

end



function q = do_inverse_position_kinematics(robot, target)

	d_z = robot.d_z;
	d_x = robot.d_x;
	phi_z = robot.phi_z;
	phi_x = robot.phi_x;

	% find the radius from center to target, and its project on XY plane
	r = norm(target);
	r_proj = norm([x, y]);

	% Find the sum of the offsets for joints 2 and 3
	offset = d_z(2) + d_z(3);

	% find the additional height that joints 2 and 3 need to add
	z_planar = target(3) - d_z(1);

	% Consider the arm from a top-down view
	% Assume that q1 has been selected such that the arm is along some axis A
	% By changing q2 and q3, the arm will move along the A-Z plane
		% Note that the A axis is normally a function of q2 and q3
		% To avoid this, A is defined by the CORRECT values of q2 and q3
		% Thus, it is now a constant, even though we don't know it
	% In this frame, the offset does not contribute to the height at all
	% Thus: z_planar = d_x2*sin(q2) + d_x3*sin(q2+q3)
		% This is equation 1

	% r_proj is how far from the origin the arm needs to be
	% This distance is made up of:
		% d_x2_proj + dx_3_proj going forward
			% this is the projections of d_x2 and d_x3 onto the X-Y Plane
		% d_z2 and d_z3 going sideways
	% thus: r_proj^2 = (d_x2_proj + d_x3_proj + d_x1_proj)^2 + offset^2
		% Let K_planar = sqrt( r_proj^2 - offset^2 ) - d_x1
		% Then: K_planar = d_x2_proj + d_x3_proj

	K_planar = sqrt( r_proj^2 - offset^2 ) - d_x(1);

	% the projections can be found by taking the cos of the lengths
		% d_x1_proj = d_x1 * cos(0) = d_x1
		% d_x2_proj = d_x2 * cos(q2)
		% d_x3_proj = d_x3 * cos(q2+q3)

	% Combining these equations, we get the following
	% K_planar = d_x2*cos(q2) + d_x3*cos(q2+q3)
		% This is equation 2

	% Notice the similarities of equations 1 and 2. Rearrange them:
		% (z_planar - d_x2*sin(q2))/d_x3 = sin(q2+q3)    <= eq1 rearranged
		% (K_planar - d_x2*cos(q2))/d_x3 = cos(q2+q3)	 <= eq2 rearranged

	a1 = (z_planar - d_x(2)*sind(q2))/d_x(3);
	a2 = (K_planar - d_x(2)*cosd(q2))/d_x(3);
		
	% The two equations describe the same triangle, with d_x3 as the hypotenuse
	% Using pythagorean theorem:
		% d_x3^2 = (z_planar - d_x2*sin(q2))^2 + (K_planar - d_x2*cos(q2))^2
	% Expand and collect terms
		% (-d_x3^2 + z_planar^2 + K_planar^2)/(2*d_x2) = z_planar*sin(q2) + K_planar*cos(q2)
		% Combine the left side into a single constant:
			% C = z_planar*sin(q2) + K_planar*cos(q2)

	C = (-d_x(3)^2 + z_planar^2 + K_planar^2 + d_x(2)^2)/(2*d_x(2));

	
	% This is equivalent to a C = Asinx + Bcosx problem
	% First, scale everything by sqrt(A^2 + B^2)
	
	% let cos(alpha) = A and sin(alpha) = B
		% Thus cos(theta + alpha) = C / scale_factor
		% tan(alpha) = z_planar/K_planar
	
	scale_factor = sqrt(z_planar^2 + K_planar^2);
	
	q2 = acos(C / scale_factor) + atan(z_planar / K_planar);
	
	% With q2, we can easily find q3
	
	q3 = asin( ( z_planar - d_x(2)*sin(q2) ) / d_x(3) ) - q;
	
	% We still need to find q1
	% We can use the offsets and projections to get the arm angle about Z from X-Axis
	% we can use atan2 to get the required angle from the target X and Y points
	% subtract the former from the latter gives us q1
	
	q1 = atan2(y, x) - atan( ( offset) / ( d_x(1) + d_x(2)*cos(q2) + d_x(3)*cos(q3)) );
	
		
end


