% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
2.441;
3.008;
3.565
];

% thrusts needed to hover
thrust_carbon = [
0.5188;
0.6062;
0.6750
];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 2);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g));
end

% print A
A

% compute the linear coeficients
X = A\thrust_carbon;

% plot the constants
ka = X(1)
kb = X(2)
