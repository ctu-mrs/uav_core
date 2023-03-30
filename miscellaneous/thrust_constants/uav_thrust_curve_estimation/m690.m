% computes ka and kb from two-point measured thrust

pwm_min = 1020;
pwm_max = 2020;

% masses of UAV
mass = [
4.65;
6.85;
7.95;
9.05;
];

thrust_pwm = [
1360;
1590;
1700;
1830;
];

thrust = (thrust_pwm - pwm_min) ./ (pwm_max - pwm_min)

n_motors = 4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 2);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g)/n_motors);
end

% print A
A

% compute the linear coeficients
X = A\thrust;

% plot the constants
ka = X(1)
kb = X(2)
