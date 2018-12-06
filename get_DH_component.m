% Provides one of 4 DH basic transforms
function A_sub = get_DH_component(param, type)

	A_sub = eye(4,4);

	switch type
		case 'd_z'
			A_sub(3,4) = param;
		case 'd_x'
			A_sub(1,4) = param;
		case 'phi_z'
			A_sub(1:3, 1:3) = rotz( param );
		case 'phi_x'
			A_sub(1:3, 1:3) = rotx( param );
		otherwise
			error("Not a valid DH component");
	end
	
end