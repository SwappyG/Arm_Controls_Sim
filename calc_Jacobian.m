% Takes in axis of motion unit vectors Z, Length vectors P, and booleans
% rho which indicate joint type

% Z must be a cell array with NUM_JOINTS + 1 cells
    % Z{i} is a 3x1 col vector representing the axis of motion of the ith
    % joint in base frame. Z{1} is just [0 0 1]' since its already in base
    % frame
% P must be a cell array with NUM_JOINTS + 1 cells
    % P{i} is the location of the origin of the ith joint in base frame
    % P{1} is simply [0 0 0]' since its the origin of the base frame
    % represented in the base frame

% rho should be a standard array of length NUM_JOINTS
    % each value in rho should be either a 1 for REVOLUTE joints or a 0 for
    % PRISMATIC JOINTS
    
function J = calc_Jacobian(Z, P, rho)
    
    % Constants
    REVOLUTE = 1;
    PRISMATIC = 0;
    DOF_3D_SPACE = 6;

    % Check length of rho to determine number of joints
    NUM_JOINTS = length(rho);
    
    % Check that the dimensions of Z and P are good
    if length(Z) ~= NUM_JOINTS + 1
        error('Z does not have enough col vectors, must have NUM_JOINTS + 1 cols');
    elseif length(P) ~= NUM_JOINTS + 1
        error('P does not have enough col vectors, must have NUM_JOINTS + 1 cols');
    end
    
    J = zeros(DOF_3D_SPACE, NUM_JOINTS);
    
    % Iterate through all joints
    for index = 1:NUM_JOINTS
        
        if rho(index) == REVOLUTE
            % Velocity co-efficient is the cross product of rotation axis 
            % unit vector in base frame and distance from end effector to 
            % ith frame
            J(1:3, index) = cross( Z{index} , P{end} - P{index} );
            
            % Angular velocity coefficient is simply the axis of rotation
            % represented in base frame
            J(4:6, index) = Z{index};
        
        elseif rho(index) == PRISMATIC
            
            % Velocity coefficient is simply the axis of translation
            % represented in base frame
            J(1:3, index) = Z{index};
            
            % Angular velocity of a prismatic joint is zero
            J(4:6, index) = 0;
        
        else
            error('Joint must be either revolute or prismatic');
        
        end 
        
    end
    
end