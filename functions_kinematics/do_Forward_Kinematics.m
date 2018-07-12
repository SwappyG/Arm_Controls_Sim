function do_Forward_Kinematics(robot)

    if class(robot) ~= 'arm'
        error('argument to this function must be of type "arm"');
    end

    An = eye(4,4);
    k_hat = [0 0 1]';
    
    for index = 1:robot.num_joints
        % Find the DH Transformation given the 4 params from i-1 to i
        robot.T(index) = { get_DH_transform( robot, index ) };

        % Multiply the current transform with the running product
        An = An*robot.T{index};

        % Extract the rotation matrix from running product (rot_i_to_base)
        robot.R(index+1) = { An(1:3,1:3) };

        % Extract the pos vector from running product (Pos_i_to_base)
        robot.P(index+1) = { An(1:3, 4) };

        % Calculate Z, orientation of ith z axis in base frame
        robot.Z(index+1) = { An(1:3,1:3) * k_hat };
    end

    robot.An = An;
    
end


%%


%  % Check that the dimensions are all good
%     if length(q) ~= NUM_JOINTS
%         error('There must be as many joint params q as NUM_JOINTS: %d!', NUM_JOINTS);
%     elseif length(d_z) ~= NUM_JOINTS
%         error('d_z must have length of NUM_JOINTS: %d', NUM_JOINTS);
%     elseif length(d_x) ~= NUM_JOINTS
%         error('d_x must have length of NUM_JOINTS: %d', NUM_JOINTS);
%     elseif length(phi_x) ~= NUM_JOINTS
%         error('phi_x must have length of NUM_JOINTS: %d', NUM_JOINTS);
%     elseif length(phi_z) ~= NUM_JOINTS
%         error('phi_z must have length of NUM_JOINTS: %d', NUM_JOINTS);
%     end



%%

%     % Check length of rho to determine number of joints
%     NUM_JOINTS = length(rho);
%     NUM_FRAMES = NUM_JOINTS + 1;
    
   

%  % Create variable to hold 4x4 transformation matrices
%     % [ R_3x3   D_3x1
%     %   0_1x3   1_1x1 ]
%     T = cell(NUM_FRAMES, 1);
%     T(:) = {zeros(4,4)}; %Transformatation of i+1 in i frame
% 
%     % Create variable to hold End Effector Transform for end effector
%     An = eye(4,4);
% 
%     % R(i, 3, 3) is the rotation matrix to ith frame
%     % R(1, 3, 3) is the rotation matrix to base frame, ie. identity matrix
%     R = cell(NUM_FRAMES, 1);
%     R(:) = {zeros(3, 3)};
%     R(1) = {eye(3)};
% 
%     % k_hat is the unit z axis vector in base frame
%     k_hat = [0 0 1]';
% 
%     % Z is the product of R * k_hat
%     Z = cell(NUM_FRAMES, 1);
%     Z(:) = {zeros(3, 1)};
%     Z(1) = {R{1} * k_hat};
% 
%     % Pos is the vector from ith frame origin to base frame origin
%     % Pos(:, 1) = [0 0 0]';
%     P = cell(NUM_FRAMES, 1);
%     P(:) = {zeros(3, 1)};
%     P(1) = {[0 0 0]'};