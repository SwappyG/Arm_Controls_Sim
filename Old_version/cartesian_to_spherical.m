function [r, phi, theta] = cartesian_to_spherical(point)

    % r is simply the pythagorian distance of point P
    r = norm(point);

    % phi can be calculated by project r onto the horizontal plane
        % the sign of phi is the same as the sign of z
    phi = asin(point(3)/r);
    
    % theta is then found using the rev around normal vect of proj_r
    theta = atan2(point(2), point(1));

end