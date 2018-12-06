syms q1 q2 q3

q3_max = 5;
d = [3 10.5 7.5];
r = norm([d(2), d(3)+q3_max]);
x = r*rand();
y = sqrt(r^2 - x^2)*(2*rand()-1);
z = sqrt(r^2 - x^2 - y^2)*(2*rand()-1);

assume(q1, 'real');
assume(q2, 'real');
assume(q3, 'real');

R1_0 = rotz(q1)*round(rotx(pi/2));
R2_1 = rotz(q2 + pi/2)*round(rotx(pi/2));
R3_2 = eye(3);

P1_0 = [0 0 d(1)]';
P2_1 = [0 0 d(2)]';
P3_2 = [0 0 d(3)+q3]';

P3_1 = P2_1 + R2_1*P3_2;
P3_0 = P1_0 + R1_0*P3_1;

eqn = vpa([x y z]' == P3_0);
[q1 q2 q3] = vpasolve(eqn, [q1 q2 q3],[-pi, pi; -pi, pi; 0, 10])
