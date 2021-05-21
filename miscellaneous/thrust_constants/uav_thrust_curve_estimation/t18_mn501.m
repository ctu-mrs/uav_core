% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
7.30;
10.20;
13.15;
16.05;
];

% thrusts needed to hover
thrust = [
0.354839;
0.463343;
0.57478;
0.652981;
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

% plot
fig = figure(1);
y = 0:0.01:mass(end);
x = [];
for i=1:length(y)
  x(i) = ka*sqrt(y(i)*g/n_motors) + kb;
end

hold off
plot(x, y, 'linewidth', 3)
hold on
scatter(thrust, mass, 'x', 'linewidth', 3)
xlabel('throttle [-]')
ylabel('thrust [kg]')
