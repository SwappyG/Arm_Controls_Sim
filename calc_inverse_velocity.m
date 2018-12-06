% Takes in:
    % x_dot: target vel and ang vel of a point (6x1)
    % J: Jacobian of same point (6xn)
        % n is the number of preceding joints

% Returns
    % Q_dot: required joint vels that'll achieve x_dot (nx1)

function Q_dot = calc_inverse_velocity(x_dot, J)

    % Check dimensions to make sure inputs are valid
    if rank(J) < size(J,2)
        error('The jacobian is singular')
    end

    if size(J,1) ~= 6
        error('The jacobian must have 6 rows')
    end
    
    if size(x_dot) ~=6
        error('The req velocity must be 1D array of size 6')
    end
    
    % TODO: Handle Singular J and redundant robots
    
    Q_dot = J\x_dot;



end