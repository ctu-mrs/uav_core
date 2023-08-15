% computes ka and kb from two-point measured thrust

thr_min = 993
thr_max = 1993

thr1 = 1162
thr2 = 1245

% masses of UAV
mass = [
  0.825;
  1.185;
];

% thrusts needed to hover
thrust = [
  0.169;
  0.252;
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
