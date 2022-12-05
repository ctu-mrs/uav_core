% computes ka and kb from two-point measured thrust

pwm_min = 993;
pwm_max = 1993;

% masses of UAV
mass = [
7.75;
9.2;
10.25;
];

thrust_pwm = [
1470;
1562;
1615;
];

thrust = (thrust_pwm - pwm_min) ./ (pwm_max - pwm_min)

n_motors = 8;

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
