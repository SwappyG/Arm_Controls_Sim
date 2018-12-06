function point = gen_rand_valid_point(robot)
    
    THRES = 0.95;

    d1 = robot.d_z(1);
    d2 = robot.d_z(2);
    d3 = robot.d_z(3);
    q3max = robot.q_max(3);
    
    L_max = sqrt( (d3+q3max)^2 + d2^2 );
    L_min = sqrt( d3^2 + d2^2 );
    L = (L_max-L_min)*rand() + L_min;

    theta = pi*rand() - pi/2;
    
    x = L * cos(theta);
    y = L * sin(theta);
    
    z_min = sqrt(L_min^2 - x^2 - y^2) + d1;
    
    if ~isreal(z_min)
        z_min = d1;
    end
    
    z_max = sqrt(L_max^2 - x^2 - y^2) + d1;
    
    z = THRES * (z_max-z_min) * rand() + z_min;


    point = [x y z]';
    
    
    [L z_min z_max]
    
end
