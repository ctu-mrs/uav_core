% computes ka and kb from two-point measured thrust

% masses of UAV
mass = [
0.011;
0.041;
0.089;
0.154;
0.238;
0.337;
0.453;
0.577;
0.711;
0.85;
1.004;
1.154;
1.304;
1.414;
1.622;
1.806;
1.967;
];

% thrusts needed to hover
thrust = [
0.05;
0.1;
0.15;
0.2;
0.25;
0.3;
0.35;
0.4;
0.45;
0.5;
0.55;
0.6;
0.65;
0.7;
0.75;
0.8;
0.85;
];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% the gravitational acceleration
g = 9.81;

% create the main matrix
A = ones(length(mass), 3);

for i=1:length(mass)
  A(i, 1) = sqrt((mass(i)*g));
  A(i, 2) = 1.0;
  A(i, 3) = mass(i)*g*0;
end

% print A
A;

% compute the linear coeficients
X = A\thrust;

% plot the constants
ka = X(1)
kb = X(2)
kc = X(3)

% plot
fig = figure(1);
y = 0:0.01:mass(end);
x = [];
for i=1:length(y)
  x(i) = ka*sqrt(y(i)*g) + kb + kc*y(i)*g*0;
end

hold off
plot(x, y, 'linewidth', 3)
hold on
scatter(thrust, mass, 'x', 'linewidth', 3)
xlabel('throttle [-]')
ylabel('thrust [kg]')
