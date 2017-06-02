% Read data file
data = load('flexibleArm_test_4');

% Extract data into vectors
time_exp = data(:,1);
theta_exp = data(:,2);
deflect_exp = data(:,3);
omega_exp = data(:,4);

% Shift data based on inherent offset
time_exp = (time_exp - time_exp(1))/1000; % convert time stamps to sec.
theta_exp = theta_exp + .3;
deflect_exp = -(deflect_exp - deflect_exp(1));

% trim data
time_exp = time_exp(1:4750);
theta_exp = theta_exp(5251:end);
deflect_exp = deflect_exp(5251:end);
omega_exp = omega_exp(5251:end);

% Plot hub angle
figure
plot(time_exp, theta_exp)
title('Motion of Flexible Arm')
xlabel('Time (s)')
ylabel('Hub Angle (rad)')

% Plot tip deflection
figure
plot(time_exp, deflect_exp)
title('Tip Deflection of Flexible Arm')
xlabel('Time (s)')
ylabel('Deflection (m)')

% Plot hub velocity
figure
plot(time_exp, omega_exp)
title('Ang. Velocity of Flexible Arm')
xlabel('Time (s)')
ylabel('Ang. Vel. (rad/s)')
