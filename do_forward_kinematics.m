% Performs forward kinematics given state and DH params
function end_point = do_forward_kinematics(robot, generate_point, q_gen)

	% Holds transform of end effector
	An = eye(4,4);
	
	% z dir unit vector, for extract orientation of Z-axis of each joint
	k_hat = [0, 0, 1]';
	
    % If just generating point, use supplied q_gen
    if generate_point
        Q = q_gen;
    else
        Q = robot.Q;
    end
    
    % Combine arm geometry and joint state for current DH params
    d_z = robot.d_z;
    d_x = robot.d_x;
    phi_z = robot.phi_z;
    phi_x = robot.phi_x;
    rho = robot.rho;

	% iterate through all joints
	for index = 1:robot.num_joints+1
	    
        % if generating point, don't update robot
        if generate_point
            An = An * get_DH_transform( ...
                            d_z(index) + ~rho(index)*Q(index), ...
                            d_x(index), ...
                            phi_z(index) + rho(index)*Q(index), ...
                            phi_x(index) ...
                                );
        else
            % Calculate transform between this joint and prev joint
            robot.T(index) = { get_DH_transform( ...
                                    d_z(index) + ~rho(index)*Q(index), ...
                                    d_x(index), ...
                                    phi_z(index) + rho(index)*Q(index), ...
                                    phi_x(index) ...
                                        ) };

            % Chain transforms to get transform between end effector and base
            An = An * robot.T{index};

            % Extract out rotation, position and z-axis orientation for this joint
            robot.R(index+1) = { An(1:3,1:3) };
            robot.P(index+1) = { An(1:3,4) };
            robot.Z(index+1) = { An(1:3,1:3) * k_hat };
        end
        
    end
    
    % Only update robot if not generating point
    if generate_point
        end_point = An(1:3,4);
    else
        robot.An = An;
        end_point = robot.P{end};
    end

end




% 		% Combine arm geometry and joint state for current DH params
% 		d_z = robot.d_z(index) + ~robot.rho(index)*Q(index);
% 		d_x = robot.d_x(index);
% 		phi_z = robot.phi_z(index) + robot.rho(index)*Q(index);
% 		phi_x = robot.phi_x(index);