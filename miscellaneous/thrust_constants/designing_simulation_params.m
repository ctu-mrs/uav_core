clear all

% input = the desired thrust from the MRS pipeline
% output = number [0, 1]  which is send to the motors (ESCs)
function output = PixhawkOutput(input)

  % params from component_snippets.xacro
  zero_position_armed = 0.15;
  input_scaling = 1;
  input_offset = 0;

  output = (input + input_offset) * input_scaling + zero_position_armed;
end

% inverse of the PixhawkOutput
function input = PixhawkOutputInv(output)

  % params from component_snippets.xacro
  zero_position_armed = 0.15;
  input_scaling = 1;
  input_offset = 0;

  input = ((output - zero_position_armed) / input_scaling) - input_offset;
end

% returns the thrust [N]
function thrust = MotorModel(input, motor_constant)

  thrust = motor_constant * (input^(2.0));

end

% inverse of the MotorModel
% returns the input
function input = MotorModelInv(thrust, motor_constant)

  input = sqrt(thrust / motor_constant);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

g = 9.81;

% how much propellers does your UAV have?
n_propellers = 4;

% the motor constant for the simulated motor model
% = max thrust in [N] per motor
motor_constant = 8.91;

% define a list of different UAV masses for computing the thrust points
mass = [
 1.4;
 1.8;
 2.2;
];

% calculate the thrust outputs of the MRS pipeline for the defined masses
for i=1:length(mass)

  % calculate what thrust ([0, 1]) will the Pixhawk return to achieve the
  % desired hover thrust?
  pixhawk_output(i) = MotorModelInv((g * mass(i))/n_propellers, motor_constant);

  % calculate what thrust ([0, 1]) will the MRS ControlManager return to
  % achieve the desired hover thrust?
  mrs_thrust(i, 1) = PixhawkOutputInv(pixhawk_output(i));

  % feed it back throught the pipeline to get the thrust
  out_thrust_N(i, 1) = n_propellers * MotorModel(PixhawkOutput(mrs_thrust(i, 1)), motor_constant);

end

mrs_thrust

% calculate the ka and kb thrust constants based on the masses and the mrs_thrust
A = ones(length(mass), 2);

for i=1:length(mass)
  A(i, 1) = sqrt(((mass(i)*g)/n_propellers));
end

X = A\mrs_thrust;

ka = X(1)
kb = X(2)

% Use the ka and kb constants to deduce what hover thrust mass-equivalent will be created
% using the mrs_thrust. Those should be equal to the "mass" vector defined previously.
% If not, something is wrong in this script.
for i=1:length(mrs_thrust)

  mass_thrust_equivalent(i, 1) = (n_propellers * (((mrs_thrust(i) - kb) / ka)^(2.0))) / g;

end

% Calculate the error of estimating the mass_thrust_equivalent
mass_thrust_error_error = abs(mass_thrust_equivalent - mass)
