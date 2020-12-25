% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
1.466;
1.833;
2.190;
];

% thrusts needed to hover
thrust = [
0.57;
0.68;
0.80;
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
