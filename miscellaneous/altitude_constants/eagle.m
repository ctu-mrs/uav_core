% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
6.664;
8.784;
10.904
];

% thrusts needed to hover
thrust = [
0.385;
0.446;
0.5
];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% fix kf constant at something
kf = 1;

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 2);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g)/kf);
end

% print A
A

% compute the linear coeficients
X = A\thrust;

% plot the constants
kf
ka = X(1)
kb = X(2)
