function q = do_inverse_position_kinematics(robot, target)

    q = [inf, inf, inf];
    
	d_z = robot.d_z;
	d_x = robot.d_x;
	phi_z = robot.phi_z;
	phi_x = robot.phi_x;

	% find the radius from center to target, and its project on XY plane
	r = norm(target);
	r_proj = norm([target(1), target(2)]);

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
		% d_x1_proj + d_x2_proj + dx_3_proj going forward
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
		
	% The two equations describe the same triangle, with d_x3 as the hypotenuse
	% Using pythagorean theorem:
		% d_x3^2 = (z_planar - d_x2*sin(q2))^2 + (K_planar - d_x2*cos(q2))^2
	% Expand and collect terms
		% (-d_x3^2 + z_planar^2 + K_planar^2)/(2*d_x2) = z_planar*sin(q2) + K_planar*cos(q2)
		% Combine the left side into a single constant:
			% C = z_planar*sin(q2) + K_planar*cos(q2)

	C = (-d_z(4)^2 + z_planar^2 + K_planar^2 + d_x(2)^2)/(2*d_x(2));
	
	% This is equivalent to a C = Asinx + Bcosx problem
	% First, scale everything by sqrt(A^2 + B^2)
	
	% let cos(alpha) = A and sin(alpha) = B
		% Thus cos(theta + alpha) = C / scale_factor
		% tan(alpha) = z_planar/K_planar
	
	scale_factor = sqrt(z_planar^2 + K_planar^2);
	
	q(2) = acos(C / scale_factor) + atan2(z_planar , K_planar);
    
    if ~isreal(q(2))
        'shit'
    end
    
	% With q2, we can easily find q3
	% Note than when taking the arcsin, there are two possible answers
	q3_a = asin( ( z_planar - d_x(2)*sin(q(2)) ) / d_z(4) ) - q(2);
    q3_b = pi - q3_a - 2*q(2);
    
    if ~isreal(q3_a)
        'shit'
    end
    
    % Check to see which q3 is the correct one
    K_planar_test_a = K_planar - (d_x(2)*cos(q(2)) + d_z(4)*cos(q(2)+q3_a));
    K_planar_test_b = K_planar - (d_x(2)*cos(q(2)) + d_z(4)*cos(q(2)+q3_b));
    
    if abs(K_planar_test_a) <= abs(K_planar_test_b)
        q(3) = q3_a;
    else
        q(3) = q3_b;
    end
    
	% We still need to find q1
	% We can use the offsets and projections to get the arm angle about Z from X-Axis
	% we can use atan2 to get the required angle from the target X and Y points
	% subtract the former from the latter gives us q1
	
	q(1) = atan2(target(2), target(1)) + atan( ( offset) / ( d_x(1) + d_x(2)*cos(q(2)) + d_z(4)*cos(q(2)+q(3))) );
    
end




%% 



%     K_planar = sqrt( r_proj^2 - offset^2 ) - d_x(1);
% 
% 	C = (-d_z(4)^2 + z_planar^2 + K_planar^2 + d_x(2)^2)/(2*d_x(2));
% 
% 	scale_factor = sqrt(z_planar^2 + K_planar^2);
% 	
% 	q_b(2) = acos(C / scale_factor) + atan(z_planar / K_planar);
%     
% 	q_b(3) = asin( ( z_planar - d_x(2)*sin(q_b(2)) ) / d_z(4) ) - q_b(2);
% 
% 	q_b(1) = atan2(target(2), target(1)) + atan( ( offset) / ( d_x(1) + d_x(2)*cos(q_b(2)) + d_z(4)*cos(q_b(3))) );
%     
%     
%     
%     K_planar = sqrt( r_proj^2 - offset^2 ) - d_x(1);
% 
% 	C = (-d_z(4)^2 + z_planar^2 + K_planar^2 + d_x(2)^2)/(2*d_x(2));
% 
% 	scale_factor = sqrt(z_planar^2 + K_planar^2);
% 	
% 	q_c(2) = acos(C / scale_factor) + atan(z_planar / K_planar);
% 	q_c(3) = asin( ( z_planar - d_x(2)*sin(q_c(2)) ) / d_z(4) ) - q_c(2);
% 
% 	q_c(1) = -atan2(target(2), target(1)) + atan( ( offset) / ( d_x(1) + d_x(2)*cos(q_c(2)) + d_z(4)*cos(q_c(3))) );
%        
%        
%     K_planar = sqrt( r_proj^2 - offset^2 ) - d_x(1);
% 
% 	C = (-d_z(4)^2 + z_planar^2 + K_planar^2 + d_x(2)^2)/(2*d_x(2));
% 
% 	scale_factor = sqrt(z_planar^2 + K_planar^2);
% 	
% 	q_d(2) = acos(C / scale_factor) + atan(z_planar / K_planar);
% 	q_d(3) = asin( ( z_planar - d_x(2)*sin(q_d(2)) ) / d_z(4) ) - q_d(2);
% 
% 	q_d(1) = -atan2(target(2), target(1)) - atan( ( offset) / ( d_x(1) + d_x(2)*cos(q_d(2)) + d_z(4)*cos(q_d(3))) );
%     
%     