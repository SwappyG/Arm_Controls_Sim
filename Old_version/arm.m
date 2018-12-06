classdef arm < handle

    properties 
        
        REVOLUTE = 1;
        PRISMATIC = 0;
        
        num_joints = [];
        
        % -- Arm Geometry --
        % Define the geometry of the robot using D-H Convention
            % All z_i axis assigned to joint in dir of motion
                % z_i is either in line with lin motion for prismatic or
                % z_i is in line with ang velo vector for revolute
            % O_i is selected on z_i where norm to z_i and z_i-1 intersect
                % If z_i & z_i-1 intersect, O_i is selecte there
            % x_i should be norm to both z_i & z_i-1 going through O_i
                % if z_i & z_i-1 intersect, x_i is norm to z_i/z_i-1 plane
            % Choose y_i to make a right handed co-ordinate system

        % d_x is dist between O_i and intersect of x_i and z_i-1
        % d_z is dist along z_i-1 from O_i-1 to intersection of x_i
            % and z_i-1
        % phi_x is angle between z_i-1 and z_i about x_i
        % phi_z is angle between x_i-1 and x_i about z_i-1
        
		% Denavit-Hartenberg parameters for defining the arm geometry
        d_z = []; %
        d_x = [];
        phi_x = [];
        phi_z = [];
		
		% holds params indicating whether joint is revolute or prismatic
        rho = [];
        
		c_m = []; % Holds the centre of mass for each link w.r.t. prev ref frame
		mass = []; % Holds the mass of each link
		
        q_max = [];
        q_min = [];
        
        % -- Arm State --
            % These parameters update over time
        Q = []; % Holds the state of the joints
        
        T = {}; % Holds 4x4 tf from i to i-1
        Z = {}; % Holds dir of z axis of frame i in base frame
        R = {}; % Holds rot mat of frame i to base frame
        P = {}; % Holds pos vect of origin of frame i to base frame
        An = []; % Holds 4x4 tf of end effector in base frame
        
        J = []; % Holds jacobian of end effector in base frame
        Jn = {}; % Holds the jacobians for all center of masses
		
        hist_q = []; % Holds the history of Q
        hist_t = []; % Holds time stamp history of Q
        
    end
    
    methods
       
        function obj = arm(num_joints, d_z, d_x, phi_z, ...
                            phi_x, rho, q_max, q_min, ...
							c_m, mass)
        
            if nargin > 0
                
                if num_joints <= 0
                    error('num_joints must be more than 1');
                end

                if  ~isvector(d_z) 		|| length(d_z) 		~= num_joints || ...
                    ~isvector(d_x) 		|| length(d_x) 		~= num_joints || ...
                    ~isvector(phi_z)	|| length(phi_z) 	~= num_joints || ...
                    ~isvector(phi_x) 	|| length(phi_x) 	~= num_joints || ...
                    ~isvector(rho) 		|| length(rho) 		~= num_joints || ...
                    ~isvector(q_max) 	|| length(q_max) 	~= num_joints || ...
                    ~isvector(q_min) 	|| length(q_min) 	~= num_joints || ...
                    ~isvector(c_m) 		|| length(q_min) 	~= num_joints || ...
                    ~isvector(mass) 	|| length(q_min) 	~= num_joints

                    error('all params must be of length num_joints');
              
                end

                obj.num_joints = num_joints;
                obj.d_z = d_z;
                obj.d_x = d_x;
                obj.phi_z = phi_z;
                obj.phi_x = phi_x;
                obj.rho = rho;
                obj.q_max = q_max;
                obj.q_min = q_min;
				obj.c_m = c_m;
				obj.mass = mass;

                obj.T = cell(num_joints+1, 1);
                obj.T(:) = {zeros(4,4)}; 
        
                obj.An = eye(4,4);

                obj.R = cell(num_joints+1, 1);
                obj.R(:) = {zeros(3, 3)};
                obj.R(1) = {eye(3)};

                obj.Z = cell(num_joints+1, 1);
                obj.Z(:) = {zeros(3, 1)};
                obj.Z(1) = {[0 0 1]'};

                obj.P = cell(num_joints+1, 1);
                obj.P(:) = {zeros(3, 1)};
                obj.P(1) = {[0 0 0]'};
               
                obj.Q = (q_max-q_min).*rand(1,6)+q_min;
                
            end
            
            
        end
        
        
    end

end