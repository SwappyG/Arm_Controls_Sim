function angs = gen_rand_angs(min_ang, max_ang)

	THRES = 0.9;

	avg = (max_ang + min_ang) ./ 2;
	delta = max_ang - min_ang;
	angs = THRES.*delta.*(rand()-0.5) + avg;

end