% Calculates the transform a joint given all DH params
function A = get_DH_transform(d_z, d_x, phi_z, phi_x)

	A_Dz = get_DH_component(d_z, 'd_z');
	A_Dx = get_DH_component(d_x, 'd_x');
	A_Rz = get_DH_component(phi_z, 'phi_z');
	A_Rx = get_DH_component(phi_x, 'phi_x');

	A = A_Rz * A_Dz * A_Dx * A_Rx;

end