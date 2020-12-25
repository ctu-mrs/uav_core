% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
4.3;
4.9;
];

% thrusts needed to hover
thrust = [
0.67;
0.82;
];

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
