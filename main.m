clc
constants

this_arm = arm(	num_joints, ...
				d_z, d_x, phi_z, phi_x, rho, ...
				q_max, q_min, ...
				c_m, mass );
				
this_arm.Q = deg2rad([0, 0, 0, 0]);

end_point = do_forward_kinematics(this_arm, false, [0,0,0,0])

jacobian_end_pt = calc_jacobian(this_arm.Z, this_arm.P, this_arm.rho, 4);





%% Inverse Kinematics Function Testing
% 
% TOL = 1e-6;
% alt_soln_cnt = 0;
% flop_cnt = 0;
% tic
% for ii = 1:10000
%     q_gen = gen_rand_angs(q_min, q_max);
% 
%     end_gen = do_forward_kinematics(this_arm, true, q_gen);
% 
%     q_calc = do_inverse_position_kinematics(this_arm, end_gen);
%     end_calc = do_forward_kinematics(this_arm, true, [q_calc, 0]);
% 
%     if norm(q_gen - [q_calc, 0]) > TOL
%         alt_soln_cnt = alt_soln_cnt + 1;
%     end
%     
%     if norm(end_gen-end_calc) > TOL
%         flop_cnt = flop_cnt + 1;
%     end
%        
% end
% toc
% disp(toc/10000)
% fprintf("flop_count: %d, alt_soln_cnt: %d", flop_cnt, alt_soln_cnt);
% 
