% computes ka and kb from two-point measured thrust

% * Weight/pwm (pwm min: 1102, max: 1903)
% * 2 base batteries: 6479g/1402
% * plus battery: 7582g/1445
% * plus battery: 8685g/1522
% * plus battery: 9788g/1558
% * plus battery: 10891g/1602

pwm_min = 1044;
pwm_max = 1944;

% masses of UAV
mass = [
8.75;
10.8;
12.9
];

thrust_pwm = [
1650;
1730;
1810
];

n_motors = 4;

% thrusts needed to hover
thrust = (thrust_pwm - pwm_min) ./ (pwm_max - pwm_min);

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
