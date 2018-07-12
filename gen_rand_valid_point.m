function point = gen_rand_valid_point(robot)
    
    THRES = 0.95;

    d1 = robot.d_z(1);
    d2 = robot.d_z(2);
    d3 = robot.d_z(3);
    q3max = robot.q_max(3);
    
    xy_sq = THRES * (d3+q3max)^2 *rand() + d2^2;

    theta = pi*rand() - pi/2;
    
    x = sqrt(xy_sq) * cos(theta);
    y = sqrt(xy_sq) * sin(theta);
    
    z_d1 = THRES * (d3+q3max)^2 *rand() + d2^2 + d3^2 - xy_sq;
    
    z = sqrt(z_d1)+d1;
    z = z*sign(rand()-0.5);

    point = [x y z]';
    
end
