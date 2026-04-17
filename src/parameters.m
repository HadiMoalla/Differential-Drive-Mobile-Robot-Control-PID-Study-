clc;
clear;
close all;

% Robot physical parameters
r = 0.05;          % Wheel radius [m]
L = 0.2;           % Distance between wheels [m]
dt = 0.05;         % Simulation time step [s]
Tsim = 30;         % Total simulation time [s]
Vmax = 0.8;        % Constant linear velocity [m/s]
Wmax = 3;          % Angular velocity limit [rad/s]
dis_max= sqrt(2);


% Robot body dimensions (visualization)
body_length = 0.3;
body_width  = 0.2;
wheel_width = 0.04;
wheel_length = 0.08;

% Heading controller gains (PID on alpha)
k_alpha  = 11;
ki_alpha = 0;
kdalpha  = 0;

% Initial robot pose
x = 0;
y = 0;
theta =(-1)*pi/2;
theta0=theta;

% Goal position (can be anywhere)
xg = 5;
yg = 5;
tol = 0.02;

dx = xg - x;
dy = yg - y;
rho = sqrt(dx^2 + dy^2);
v = rho * Vmax / dis_max;
if v> Vmax
    v = Vmax;
end