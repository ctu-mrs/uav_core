% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
0.401;
0.787;
1.024
];

% thrusts needed to hover
thrust = [
5688.0/9068.0;
7788.0/9068.0;
9068.0/9068.0;
];

thrust

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
X = A\thrust;

% plot the constants
ka = X(1)
kb = X(2)
