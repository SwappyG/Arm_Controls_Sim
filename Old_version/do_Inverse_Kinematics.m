function q = do_Inverse_Kinematics(robot, point, pose)

    % Check array dimensions match
%     assert(class(robot) ~= 'arm', ...
%                 'argument to this function must be of type "arm"');
%     assert(isvector(xyz_point) && length(xyz_point) == 3, ...
%                 'Dimension of point mismatch');
        
    d1 = robot.d_z(1);
    d2 = robot.d_z(2);
    d3 = robot.d_z(3);
    d6 = robot.d_z(6);
    
    % Split problem into pos and orientation
        % first 3DOF handles pos, wrist handles orientation
    
    target = point - pose*[0 0 d6]';
    
    % It's much easier to work in spherical co-ordinates
        % [r=radius theta=polar_angle phi=azumithal_angle]
        % Define r as the distance from joint q1 target point
        % Define phi as angle between 
    
    % Co-ordinate frame defined at joint 2
        % Thus, height of d1 needs to be subtracted from target
    target = target - [0 0 d1]';
    [r, phi, theta] = cartesian_to_spherical(target);
         
    % r^2 = d2^2 + (d3+q3)^2;
    % Rearranging this:
    q3 = sqrt(r^2 - d2^2) - d3; 
        % Here: r must be > d2,
        %       sqrt(r^2 - d2^2) - d3 must be > q3_min
    
    % let h = d3 + q3  
    % let proj_h be the projection of h onto the horizontal plane
    % let proj_r be the project of r on to the horizontal plane
        % Then (proj_h, proj_r, d2) form a right angle triangle
            % let alpha be the angle between d2 and proj_r
            
	% define perp_r such that (perp_r, proj_r, r) form a right triangle
    % define perp_h such that (perp_h, proj_h, h) form a right triangle
        % h and r intersect at one end
        % proj_h and proj_r also intersect at one end
        % perp_h connects proj_h and h at same point as proj_r and r
            % therefore perp_r and perp_h are the same line!
    
    % By convention of spherical coord, phi is angle between perp_r and r
        % thus: r * sin(phi) = perp_h
        %       h * sin(q2) = perp_h
        
    q2 = asin(r*sin(phi)/(d3+q3));   
    % sin(phi) < (d3+q3)/r
    
    % Lastly, we just need to find alpha
        %   cos(alpha) = d2 / proj_r
    alpha = acos( d2 / (r*cos(phi)) );
        % cos(phi) > d2/r
        % No new equalities are create, this is same as the prev one
    
    q1 = pi/2 - alpha + theta; %theta - (pi/2 - alpha); 

    q = [q1 q2 q3];
    robot.Q(1:3) = q;
    do_Forward_Kinematics(robot);
    
    R6_3 = robot.R{4}' * pose;
    
    q5 = acos(R6_3(3,3));
    q4 = atan2( R6_3(2,3) , R6_3(1,3) );
    q6 = atan2( R6_3(3,2) , R6_3(3,1) );
    
    q = [q1 q2 q3 q4 q5 q6];
    
    robot.Q(:) = q;
    
        
end












%% INCORRECT MATH

%     % By cosine law:
%         % h^2 = (d3+q3)^2 + (d3+q3)^2 - 2*(d3+q3)^2*cos(q2)
%     % Also by cosine law:
%         % h^2 = r^2 + r^2 - 2*r^2*cos(phi)
%     % I.E. the value of q2 creates two triangles (r,r,h) and
%         % (d3+q3,d3+q3,h) which share a border 'h'. Also, the angle between
%         % r|q2=0 and r|q2=k is the azumithal angle, hence its just phi
%     % Equating these two relationships, we get:
%     
%     q2 = acos(1 - r^2*(1-cos(phi))/((d3+q3)^2));
%         % rearranging this, an equality must be satisfied:
%             % cos(phi) > d2^2/r^2 // uses r^2 = d2^2 + (d3+q3)^2
%     
%     % let d3_prime be the projection of d3+q3 on horizontal plane
%         % note, this is a projection, not a rotation
%     d3_prime = (d3+q3)*cos(q2);
%     % Then define r_prime as projection of r on horizontal plane
%         % a triangle(d2, d3_prime, r3_prime) is formed on horizontal plane
%     % Then define alpha as angle between d2 and r_prime
%     alpha = atan(d3_prime/d2);
%     % Theta is the angle CCW starting at the x0 axis
%         % (90-alpha) is an angle CW from x0 axis
%         % thus theta =  q1 - (90-alpha)
%     q1 = 90-alpha + theta;
%         % q1 has no restrictions, though theta should have some limits

