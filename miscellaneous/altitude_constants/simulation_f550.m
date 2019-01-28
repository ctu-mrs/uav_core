% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
3.0;
4.0
5.0;
];

% thrusts needed to hover
thrust = [
0.27
0.41
0.59
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

