%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% ASEN 2003 Lab 6: Rotory Position Control Experiment
% 
% The purpose of this program is to model the movement of a rigid and
% flexible arm (and deflection) in order to find optimal gain values to
% achieve critical dampending or minimum tip deflection.
% 
% This program does the following:
% 	- Defines values pertinent to the physical system in use
% 	- Models the movement of the rigid arm
%   - Finds the optimal Kp and Kd values for crit. dampening
%   - Models the movement of the flexible arm
%   - Finds the optimal gains to achieve crit. dampending
%   - Finds the optimal gains to achieve minimum tip deflection
% 
% Author:	Keith Covington
% Created:	04/19/17
% Modified:	04/26/17
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Housekeeping
clear all; close all; clc;

%% Definitions

% System parameters
Kg = 48.4;          % total gear ratio
Km = 0.0107;		% motor constant [V/(rad/s)] or [Nm/amp]
Rm = 3.29;          % armature resistance [ohms]
J_hub = 0.002;		% base inertia (includes motors and gears) [kg*m^2]
J_load = 0.0015;	% load inertia of bar [kg*m^2]
J = J_hub+J_load;	% total inertia [kg*m^2]
L = 0.45;           % link length [m]
Marm = 0.06;		% link mass of ruler [kg]
J_arm = Marm*(L^2)/3;	% link rigid body (should be 0.004) [kg*m^2]
Mtip = 0.05;		% tip mass [kg]
J_M = Mtip*(L^2);	% tip mass inertia (should be 0.01) [kg*m^2]
fc = 1.8;           % natural frequency [Hz]
J_L = J_arm+J_M;
Karm = ((2*pi*fc)^2)*J_L; % flexible link stiffness

% Gains
Kp = linspace(-5,20,10);
Kd = linspace(-0.5,15,10);

thetad = .3; % desired theta
settling5 = 0.05*thetad; % required settling
workingKs = [];


%% Model rigid arm
figure;
hold on

wbar = waitbar(0,'Simulating models...'); % progress bar
for i = 1:length(Kp)
    for j = 1:length(Kd)
        % Closed loop system
        num = Kp(i)*Kg*Km / (J*Rm);
        den1 = 1;
        den2 = ((Kg^2)*(Km^2) + Kd(j)*Kg*Km) / (J*Rm);
        den3 = (Kp(i)*Kg*Km)/(J*Rm);
        den = [den1 den2 den3];
        sysTF = tf(num,den); % define transfer function

        % Step response
        [x,t] = step(sysTF);
        theta = 2*thetad*x; % thetad is desired arm angle

        % Plot only if it's reasonable i.e. leave out diverging models
        if ~any(abs(theta) > 0.9)
            plot(t,theta);
        end
        
        % Find when theta is within tolerance (5% settling
        inTolerance = any(abs((thetad - theta/2 <= settling5)));
        
        % Find t where this occured
        timeDone = t(find(abs((thetad - theta/2 <= settling5)),1));
        
        % Check for overshoot
        overShot = any((thetad - theta/2) < 0) && any((thetad - theta/2) > 0);
        
        %Check to make sure we're not exceding allowable Vin
        w = diff(theta)./diff(t);
        Vin = w.*Km;
        excedeV = any(Vin>=5);
        
        % If it's within tolerance and didn't overshoot, record Ks
        if inTolerance && (timeDone < 0.5) && ~overShot && ~excedeV && ~any(abs(theta) > 0.9)
            workingKs = [workingKs; [timeDone Kp(i) Kd(j)]];
        end
    end
    waitbar(i/length(Kp)); % update progress bar
end
close(wbar); % close progress bar

plot(50*t, 2*thetad.*ones(1,length(t)),'k:')
plot(50*t, 2*(thetad.*ones(1,length(t))-settling5),'k--')
plot(50*t, 2*(thetad.*ones(1,length(t))+settling5),'k--')
title('Rigid Arm Movement')
xlabel('Time (s)')
ylabel('Theta (rad)')


%% Find fastest one
bestCase = workingKs(find(min(workingKs(:,1)) == workingKs(:,1)),:);
KpBest = bestCase(2)
KdBest = bestCase(3)

% Vary best case to test near values
Kp = [5; 10; KpBest];
Kd = [0; 10; KdBest];

% Show these cases
figure;
hold on
for i = 1:length(Kp)
    % Closed loop system
    num = Kp(i)*Kg*Km / (J*Rm);
    den1 = 1;
    den2 = ((Kg^2)*(Km^2) + KdBest*Kg*Km) / (J*Rm);
    den3 = (Kp(i)*Kg*Km)/(J*Rm);
    den = [den1 den2 den3];
    sysTF = tf(num,den); % define transfer function

    % Step response
    [x,t] = step(sysTF);
    theta = 2*thetad*x; % thetad is desired arm angle

    plot(t,theta);
end
plot(2*t, 2*thetad.*ones(1,length(t)),'k:')
plot(2*t, 2*(thetad.*ones(1,length(t))-settling5),'k--')
plot(2*t, 2*(thetad.*ones(1,length(t))+settling5),'k--')
title('Rigid Arm Movement')
xlim([0 1]);
xlabel('Time (s)')
ylabel('Theta (rad)')
legend('Kp = 5', 'Kp = 10', 'Kp = 20 (optimal)', 'Desired angle', '5% settling')

figure;
hold on
for j = 1:length(Kd)
    % Closed loop system
    num = KpBest*Kg*Km / (J*Rm);
    den1 = 1;
    den2 = ((Kg^2)*(Km^2) + Kd(j)*Kg*Km) / (J*Rm);
    den3 = (KpBest*Kg*Km)/(J*Rm);
    den = [den1 den2 den3];
    sysTF = tf(num,den); % define transfer function

    % Step response
    [x,t] = step(sysTF);
    theta = 2*thetad*x; % thetad is desired arm angle

    plot(t,theta);
end
plot(2*t, 2*thetad.*ones(1,length(t)),'k:')
plot(2*t, 2*(thetad.*ones(1,length(t))-settling5),'k--')
plot(2*t, 2*(thetad.*ones(1,length(t))+settling5),'k--')
title('Rigid Arm Movement')
xlim([0 1]);
xlabel('Time (s)')
ylabel('Theta (rad)')
legend('Kd = 0 (underdamped)', 'Kd = 10 (overdamped)', 'Kd = 1.2222 (crit. damped)', 'Desired angle', '5% settling')



%% Model Flexible arm

workingKs = [];

Kp = linspace(0,10,5);
Kd = linspace(-15,0,5);
KP = linspace(0,1.5,5);
KD = linspace(0,1.5,5);

figure
wbar = waitbar(0,'Simulating models...'); % progress bar
for i = 1:length(Kp)
    for j = 1:length(Kd)
        for k = 1:length(KP)
            for l = 1:length(KD)
                %% Model movement of arm
                p1 = -(Kg^2)*(Km^2)/(J_hub*Rm);
                p2 = (Kg^2)*(Km^2)*L/(J_hub*Rm);
                q1 = Karm/(L*J_hub);
                q2 = -Karm*(J_hub+J_L)/(J_L*J_hub);
                r1 = Kg*Km/(J_hub*Rm);
                r2 = -Kg*Km*L/(J_hub*Rm);

                lamda3 = -p1 + KP(k)*r1 + KD(l)*r2;
                lamda2 = -q2 + Kp(i)*r1 + Kd(j)*r2 + KD(l)*(p2*r1-r2*p1);
                lamda1 = p1*q2 - q1*p2 + KP(k)*(q1*r2-r1*q2) + Kd(j)*(p2*r1-r2*p1);
                lamda0 = Kp(i)*(q1*r2-r1*q2);

                num = [Kp(i)*r1 0 Kp(i)*(q1*r2-r1*q2)];
                den = [1 lamda3 lamda2 lamda1 lamda0];
                sysTF = tf(num,den); % define transfer function

                % Step response
                [x,t] = step(sysTF);
                theta = 2*thetad*x; % thetad is desired arm angle
                
                if ~any(abs(theta) > 0.9)
                    subplot(1,2,1)
                    hold on
                    plot(t,theta);
                    title('Flexible Arm Movement')
                    xlabel('Time (s)')
                    ylabel('Hub Angle (rad)')
                    
                    % Find when theta is within tolerance (5% settling
                    inTolerance = any(abs((thetad - theta/2 <= settling5)));

                    % Find t where this occured
                    timeDone = t(find(abs((thetad - theta/2 <= settling5)),1));

                    % Check for overshoot
                    overShot = any((thetad - theta/2) < 0) && any((thetad - theta/2) > 0);
                    
                    %Check to make sure we're not exceding allowable Vin
                    w = diff(theta)./diff(t);
                    Vin = w.*Km;
                    excedeV = any(Vin>=5);

                    
                    %% Model deflection of arm
                    num = [Kp(i)*r2 Kp(i)*(p2*r1-r2*p1) 0];
                    den = [1 lamda3 lamda2 lamda1 lamda0];
                    sysTF = tf(num,den); % define transfer function

                    % Step response
                    [x,t] = step(sysTF);
                    theta = 2*thetad*x; % thetad is desired arm angle
                    
                    % Check for residual vibrations
                    exceedsResVibe = any( (x(t>=1 & t<=1.2)) >= 0.005 );

                    subplot(1,2,2)
                    hold on
                    plot(t,theta);
                    title('Flexible Arm Deflection')
                    xlabel('Time (s)')
                    ylabel('Deflection (m)')
                    
                    % If it's within tolerance and didn't overshoot, record Ks
                    if inTolerance && (timeDone < 1) && ~overShot...
                            && ~excedeV && ~any(abs(theta) > 0.9)...
                            && ~exceedsResVibe
                        workingKs = [workingKs; [timeDone Kp(i) Kd(j) KP(k) KD(l) max(abs(x))]];
                    end
                    
                end
            end
        end
    end
    waitbar(i/length(Kp)); % update progress bar
end
close(wbar);



%% Find gains most closely modeling crit. dampening of flexible arm
bestCase = workingKs(find(min(workingKs(:,1)) == workingKs(:,1)),:);
Kp = bestCase(2)
Kd = bestCase(3)
KP = bestCase(4)
KD = bestCase(5)

% Manually define K values of higher accuracy from previous simulation
Kp = 7.7778;
Kd = -1.6667;
KP = 1.3333;
KD = 1.3333;

% Coefs used in transfer function
p1 = -(Kg^2)*(Km^2)/(J_hub*Rm);
p2 = (Kg^2)*(Km^2)*L/(J_hub*Rm);
q1 = Karm/(L*J_hub);
q2 = -Karm*(J_hub+J_L)/(J_L*J_hub);
r1 = Kg*Km/(J_hub*Rm);
r2 = -Kg*Km*L/(J_hub*Rm);

% Coefs used in transfer function
lamda3 = -p1 + KP*r1 + KD*r2;
lamda2 = -q2 + Kp*r1 + Kd*r2 + KD*(p2*r1-r2*p1);
lamda1 = p1*q2 - q1*p2 + KP*(q1*r2-r1*q2) + Kd*(p2*r1-r2*p1);
lamda0 = Kp*(q1*r2-r1*q2);

% Define transfer function
num = [Kp*r1 0 Kp*(q1*r2-r1*q2)];
den = [1 lamda3 lamda2 lamda1 lamda0];
sysTF = tf(num,den); % define transfer function

% Step response
[x,t] = step(sysTF);
theta = 2*thetad*x; % thetad is desired arm angle

% Plot optimal model movement
compareFig = figure;
subplot(1,2,1)
hold on
plot(t,theta);
title('Flexible Arm Movement (Crit. Dampening)')
xlabel('Time (s)')
ylabel('Hub Angle (rad)')

% Model deflection of arm
num = [Kp*r2 Kp*(p2*r1-r2*p1) 0];
den = [1 lamda3 lamda2 lamda1 lamda0];
sysTF = tf(num,den); % define transfer function

% Step response
[x,t] = step(sysTF);
theta = 2*thetad*x; % thetad is desired arm angle

% Plot optimal model deflection
subplot(1,2,2)
hold on
plot(t,theta);
title('Flexible Arm Deflection (Crit. Dampening)')
xlabel('Time (s)')
ylabel('Deflection (m)')



%% Analyze data

analyzeData; % load data file and create plots
figure(compareFig)
subplot(1,2,1)
plot(time_exp, theta_exp);
plot(6*t, 2*thetad.*ones(1,length(t)),'k:')
plot(6*t, 2*(thetad.*ones(1,length(t))-settling5),'k--')
plot(6*t, 2*(thetad.*ones(1,length(t))+settling5),'k--')
legend('Model', 'Experimental', 'Desired Angle', '5% Settling')

subplot(1,2,2)
plot(time_exp, deflect_exp);
plot(6*t, (zeros(1,length(t))-.005),'k--')
plot(6*t, (zeros(1,length(t))+.005),'k--')
legend('Model', 'Experimental', '0.5 cm deflection')

