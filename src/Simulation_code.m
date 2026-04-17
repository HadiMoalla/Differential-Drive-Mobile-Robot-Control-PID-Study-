clc;
clear;
close all;

% Heading controller gains (PID on alpha)
kp_alpha  = 11;
ki_alpha = 22;
kd_alpha  = 0.12;

% Goal position
xg = 5;
yg = 5;
tol = 0.02;

% Initial robot pose
x = 0;
y = 0;
theta = 1.5*pi;  % Initial angle [rad]


% Robot physical parameters
r = 0.05;          % Wheel radius [m]
L = 0.2;           % Distance between wheels [m]
dt = 0.05;         % Simulation time step [s]
Tsim = 30;         % Total simulation time [s]
Vmax = 0.8;        % Constant linear velocity [m/s]
Wmax = 3;          % Angular velocity limit [rad/s]

%Robot visualization parameters
body_length = 0.3;
body_width  = 0.2;
wheel_width = 0.04;
wheel_length = 0.08;

theta0 = theta;


dx = xg - x;
dy = yg - y;

rho = sqrt(dx^2 + dy^2);
%v = Vmax; %this was done before fixing the goal proximity instability
v = rho * Vmax / sqrt(2); %our solution!
if v > Vmax
    v = Vmax;
end

X = x;
Y = y;
alpha_history = [];
time_history = [];

alpha_prev = 0;
alpha_sum  = 0;

margin = 1;
xmin = min(x, xg) - margin;
xmax = max(x, xg) + margin;
ymin = min(y, yg) - margin;
ymax = max(y, yg) + margin;

figure('Color','white');
hold on; axis equal; grid on;
set(gca, 'Color', 'white', 'GridAlpha', 0.3);
xlim([xmin xmax]);
ylim([ymin ymax]);

text_x = xmin + 0.05*(xmax-xmin);
text_y = ymax - 0.05*(ymax-ymin);
text(text_x, text_y, ...
     {sprintf('K_p = %.2f,   K_i = %.2f,   K_d = %.3f', kp_alpha, ki_alpha, kd_alpha), ...
      sprintf('V_{max} = %.1f m/s,   W_{max} = %.1f rad/s', Vmax, Wmax), ...
      sprintf('\\theta_0 = %.2f rad', theta)}, ...
     'FontSize', 9, 'FontName', 'Arial', ...
     'BackgroundColor', 'white', 'EdgeColor', 'none', ...
     'VerticalAlignment', 'top');

plot(xg, yg, 'rx', 'MarkerSize',10,'LineWidth',2);
hPath = plot(X,Y,'b-','LineWidth',1.5);

hBody   = patch(0,0,'b','FaceAlpha',0.3);
hWheelL = patch(0,0,'k');
hWheelR = patch(0,0,'k');
hVel    = quiver(x,y,0,0,'r','LineWidth',2,'MaxHeadSize',2);

t = 0;
for t = 0:dt:Tsim
    dx = xg - x;
    dy = yg - y;
    rho = sqrt(dx^2 + dy^2);

    if rho < tol
        break;
    end

    alpha = atan2(dy, dx) - theta;
    alpha = atan2(sin(alpha), cos(alpha));

    alpha_history(end+1) = alpha;
    time_history(end+1)  = t;

    dalpha    = (alpha - alpha_prev) / dt;
    alpha_sum = alpha_sum + alpha * dt;

    w = kp_alpha*alpha + ki_alpha*alpha_sum + kd_alpha*dalpha;
    w = max(-Wmax, min(Wmax, w));

    x     = x + v*cos(theta)*dt;
    y     = y + v*sin(theta)*dt;
    theta = theta + w*dt;

    alpha_prev = alpha;

    X(end+1) = x;
    Y(end+1) = y;
    set(hPath,'XData',X,'YData',Y);

    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

    body = R * [ body_length/2  body_length/2 -body_length/2 -body_length/2;
                 body_width/2  -body_width/2  -body_width/2   body_width/2 ];
    set(hBody,'XData',body(1,:)+x,'YData',body(2,:)+y);

    wheel_offset = L/2;
    wheel = R * [ wheel_length/2  wheel_length/2 -wheel_length/2 -wheel_length/2;
                  wheel_width/2  -wheel_width/2  -wheel_width/2   wheel_width/2 ];

    set(hWheelL,'XData',wheel(1,:)+x - wheel_offset*sin(theta), ...
                'YData',wheel(2,:)+y + wheel_offset*cos(theta));
    set(hWheelR,'XData',wheel(1,:)+x + wheel_offset*sin(theta), ...
                'YData',wheel(2,:)+y - wheel_offset*cos(theta));

    set(hVel,'XData',x,'YData',y,'UData',v*cos(theta),'VData',v*sin(theta));

    drawnow;
end

title(sprintf('Differential-Drive Robot Point-to-Point Control (Kinematic PID)\nTravel Time: %.2f seconds', t), ...
      'FontSize', 11, 'FontWeight', 'bold');
xlabel('X [m]');
ylabel('Y [m]');
legend('Goal','Path','Location','best');

% ----- Second figure: Angular Error (alpha) Response -----
figure('Color', 'white', 'Position', [100, 100, 900, 600]);

plot(time_history, alpha_history, 'b-', 'LineWidth', 1.5);
hold on; grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Angular Error \alpha [rad]', 'FontSize', 11);
title('Angular Error Response (Heading Error)', 'FontSize', 12, 'FontWeight', 'bold');

initial_error = abs(alpha_history(1));
if initial_error > 0.01
    if alpha_history(1) > 0
        overshoot = max(0, -min(alpha_history)) / initial_error * 100;
    else
        overshoot = max(0,  max(alpha_history)) / initial_error * 100;
    end
else
    overshoot = 0;
end

settle_band = max(0.02 * abs(alpha_history(1)), 0.01);
settling_time = NaN;
for i = length(time_history):-1:1
    if abs(alpha_history(i)) > settle_band
        settling_time = time_history(min(i+1, length(time_history)));
        break;
    end
end

steady_state_error = abs(alpha_history(end));

text(0.5, 0.85, ...
     {sprintf('Response Characteristics:'), ...
      sprintf('Overshoot: %.2f %%', overshoot), ...
      sprintf('Settling Time (±2%%): %.3f s', settling_time), ...
      sprintf('Steady State Error: %.4f rad', steady_state_error)}, ...
     'Units', 'normalized', ...
     'FontSize', 10, 'FontName', 'Arial', ...
     'BackgroundColor', 'white', 'EdgeColor', [0.5 0.5 0.5], ...
     'LineWidth', 1, 'Margin', 5, ...
     'HorizontalAlignment', 'center', ...
     'VerticalAlignment', 'bottom');

if ~isnan(settling_time)
    plot([settling_time, settling_time], [min(alpha_history), max(alpha_history)], 'r--', 'LineWidth', 1);
end

plot([0, max(time_history)], [0, 0], 'k--', 'LineWidth', 1);
plot([0, max(time_history)], [ settle_band,  settle_band], 'r:', 'LineWidth', 1);
plot([0, max(time_history)], [-settle_band, -settle_band], 'r:', 'LineWidth', 1);

legend('Angular Error \alpha(t)', 'Settling Time', 'Zero Reference', '\pm2% Band', ...
       'Location', 'best', 'FontSize', 9);

grid on;
xlim([0, max(time_history)]);
ylim([min(alpha_history) - 0.1*abs(min(alpha_history)), max(alpha_history) + 0.1*abs(max(alpha_history))]);

fprintf('\n--- Angular Error Response Analysis ---\n');
fprintf('Overshoot: %.2f %%\n', overshoot);
fprintf('Settling Time (+/-2%%): %.3f s\n', settling_time);
fprintf('Steady State Error: %.4f rad\n', steady_state_error);
fprintf('----------------------------------------\n');