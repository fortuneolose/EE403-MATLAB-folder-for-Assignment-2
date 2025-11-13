% This is our prescribed transfer function, with the prescribed values for b2, b3, a1, a2, & a3. 
Gps = tf([5.5 13], [1 6.1 12 7.1]); % We take the values of our transfer function and transpose them into a discrete time z transform.
Kp = 1; Ki = 0; Kd = 0; T = 1;
Gz = c2d(Gps, T, 'zoh'); % Discretize plant.
num = [(T*Kp + (0.5*((T)^2)*Ki) + Kd) (((0.5*(T)^2) * Ki) - (T * Kp) - (2 * Kd)) Kd]; % The numerator of our transfer function.
den = [ T,  -T,  0 ]; % The denominator of our transfer function
Dz = tf(num, den, T, 'variable', 'z'); % PID PTF
DGz = Dz * Gz; % D(z) in series with G(z)
SYSz = feedback(DGz, 1); % Y(z) / R(z)
t = 0:T:20;
r = ones(size(t));
y = lsim(SYSz, r, t);
plot(t, r, 'g--')
hold on
stem(t, y, 'r', 'filled');
xlabel('time (seconds)'); ylabel('r(k) & y(k) ')
title('Kp = 1 Ki = 0 Kd = 0');


