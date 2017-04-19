%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% ASEN 2003 Lab 6: Rotory Position Control Experiment
% 
% The purpose of this program is to...
% 
% This program does the following:
% 	- Defines ...
% 	- Analyzes things and stuff
% 
% Author:	Keith Covington
% Created:	04/19/17
% Modified:	04/19/17
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Housekeeping
clear all; close all; clc;

%% Definitions

% System parameters
Kg = 48.4;		% total gear ratio
Km = 0.0107;		% motor constant [V/(rad/s)] or [Nm/amp]
Rm = 3.29;		% armature resistance [ohms]
J_hub = 0.002;		% base inertia (includes motors and gears) [kg*m^2]
J_load = 0.0015;	% load inertia of bar [kg*m^2]
J = J_hub+J_load;	% total inertia [kg*m^2]
L = 0.45;		% link length [m]
Marm = 0.06;		% link mass of ruler [kg]
J_arm = Marm*(L^2)/3;	% link rigid body (should be 0.004) [kg*m^2]
Mtip = 0.05;		% tip mass [kg]
J_M = Mtip*(L^2);	% tip mass inertia (should be 0.01) [kg*m^2]
fc = 1.8;		% natural frequency [Hz]
J_L = J_arm+J_M;	% uhhhhhhhhhh
Karm = ((2*pi*fc)^2)*J_L; % flexible link stiffness

% Gains
Kp = linspace(-5,20,10);
Kd = linspace(-0.5,15,10);

thetad = .45;


figure;
hold on

for i = 1:length(Kp)
    for j = 1:length(Kd)
        %% Closed loop system
        num = Kp(i)*Kg*Km / (J*Rm);
        den1 = 1;
        den2 = ((Kg^2)*(Km^2) + Kd(j)*Kg*Km) / (J*Rm);
        den3 = (Kp(i)*Kg*Km)/(J*Rm);
        den = [den1 den2 den3];
        sysTF = tf(num,den); % define transfer function

        %% Step response
        [x,t] = step(sysTF);
        theta = 2*thetad*x; % thetad is desired arm angle

        plot(t,theta);

        % Find Vin
%         w = diff(theta)./diff(t);
%         Vin = w.*Km;
%         find(Vin>=5)
    end
end

xlabel('Time (s)')
ylabel('Theta (rad)')




