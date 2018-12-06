% Takes in:
    % x_ddot: target accel and ang accel of point of interest(6x1)
    % J: Jacobian of point of interest (6xn)
        % n is the number of preceding joints
    % J_dot: derivative of Jacobian at point of interest (6xn)
    % Q_dot: vel and ang vel of point of interest (nx1)
    
% Returns
    % Q_ddot: required joint accels that'll achieve x_ddot (nx1)


function Q_ddot = calc_inverse_accel(x_ddot, J, J_dot, Q_dot)

    % Check dimensions to make sure inputs are valid
    if rank(J) < size(J,2)
        error('The jacobian is singular')
    end

    if size(J,1) ~= 6
        error('The jacobian must have 6 rows')
    end
    
    if size(x_ddot) ~=6
        error('The req velocity must be 1D array of size 6')
    end

    % TODO: Handle Singular J and redundant robots
    
    % Intermediate calculation for legibility
    B = x_ddot - J_dot*Q_dot;
    
    Q_ddot = J\B;

end