d_z = [6.03, 1.39, 1.24];
d_x = [1.79, 12.47, 12.17];
z = 22.23;
y = 10.41;
x = 16.10;
target = [x,y,z];

q1 = 25;
q2 = 27;
q3 = 33;

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

q2_calc = -acosd(C / scale_factor) + atand(z_planar / K_planar);

% With q2, we can easily find q3

q3_calc = asind( ( z_planar - d_x(2)*sind(q2) ) / d_x(3) ) - q2;


% We still need to find q1
% We can use the offsets and projections to get the arm angle about Z from X-Axis
% we can use atan2 to get the required angle from the target X and Y points
% subtract the former from the latter gives us q1

q1_calc = atan2d(y, x) - atand( ( offset) / ( d_x(1) + d_x(2)*cosd(q2) + d_x(3)*cosd(q3)) );




test_1 = z_planar - (d_x(2)*sind(q2) + d_x(3)*sind(q2+q3));
test_2 = K_planar - (d_x(2)*cosd(q2) + d_x(3)*cosd(q2+q3));
test_3 = d_x(3)^2 - ( z_planar^2 - 2*z_planar*d_x(2)*sind(q2) + (d_x(2)*sind(q2))^2 + ...
                      K_planar^2 - 2*K_planar*d_x(2)*cosd(q2) + (d_x(2)*cosd(q2))^2 );
test_4 = C - ( z_planar*sind(q2) + K_planar*cosd(q2) );



tests = [test_1, test_2, test_3, test_4]


