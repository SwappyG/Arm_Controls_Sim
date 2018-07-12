function H = get_DH_transform(d, a, r, t)

    H_Rz = eye(4,4);
    H_Rz(1:3,1:3) = rotz(t);
    
    H_Dz = eye(4,4);
    H_Dz(3,4) = d;
    
    H_Dx = eye(4,4);
    H_Dx(1,4) = a;
    
    H_Rx = eye(4,4);
    H_Rx(1:3,1:3) = rotx(r);
    
    H = H_Rz*H_Dz*H_Dx*H_Rx;
    
end
