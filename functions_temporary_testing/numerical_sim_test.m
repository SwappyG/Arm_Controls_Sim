d_z = [3, 10.5, 4+3.5]; 
d_x = [0, 0, 0];
phi_x = [90, 90, 0];
phi_z = [0, 90, 0];
rho = [1 1 0];

index = 1; 
point = zeros(50*50*50, 3);

% for q1 = linspace(0,10,50)
    for q2 = linspace(-90,90,50)
        for q3 = linspace(0,5,50)
            [~, An, R, P, ~] = do_Forward_Kinematics(d_z,d_x,phi_x,phi_z,rho,[q1 q2 q3]);
            point(index, :) = An(1:3,4)';
            index = index + 1;
        end
    end
% end


tic