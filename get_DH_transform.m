function H = get_DH_transform(robot, index)

    H_Rz = eye(4,4);
    H_Rz(1:3,1:3) = rotz( robot.phi_z(index) + ...
                            robot.rho(index)*robot.Q(index) );
    
    H_Dz = eye(4,4);
    H_Dz(3,4) = robot.d_z(index) + ~robot.rho(index)*robot.Q(index);
    
    H_Dx = eye(4,4);
    H_Dx(1,4) = robot.d_x(index);
    
    H_Rx = eye(4,4);
    H_Rx(1:3,1:3) = rotx( robot.phi_x(index) );
    
    H = H_Rz*H_Dz*H_Dx*H_Rx;
    
end
