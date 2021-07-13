% computes ka and kb from two-point measured thrust
% pwm min: 1102
% pwm max: 1905
% 5.45 kg: 1420
% 7.00 kg: 1450 
% 7.55 kg: 1520 
% 8.95 kg: 1550

% masses of UAV
mass = [
5.45
7.00
7.55
8.95
];

% thrusts needed to hover
thrust = [
0.3960149439601494
0.43337484433374845
0.5205479452054794
0.5579078455790785
];

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
