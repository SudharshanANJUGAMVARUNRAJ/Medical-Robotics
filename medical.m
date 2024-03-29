% Link parameters
a = [1, 1, 1]; % Link lengths
alpha = [0, 0, 0]; % Link twists
d = [0, 0, 0]; % Link offsets

% Define the Transformation Matrices
T01 = @(theta) dh_matrix(a(1), alpha(1), d(1), theta);
T12 = @(theta) dh_matrix(a(2), alpha(2), d(2), theta);
T23 = @(theta) dh_matrix(a(3), alpha(3), d(3), theta);

% Animation loop
figure;
for t = 0:0.1:2*pi
    % Update joint angles
    theta1_val = sin(t);
    theta2_val = sin(t + pi/4);
    theta3_val = sin(t - pi/4);
    
    % Compute transformations
    T01_val = T01(theta1_val);
    T02_val = T01_val * T12(theta2_val);
    T03_val = T02_val * T23(theta3_val);

    % Extract joint positions
    joint_positions = [0, 0, 0; T01_val(1:3, 4)'; T02_val(1:3, 4)'; T03_val(1:3, 4)'];

    % Plotting
    plot3(joint_positions(:, 1), joint_positions(:, 2), joint_positions(:, 3), 'o-', 'LineWidth', 2);
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('3DOF Robotic Arm Movement Simulation');
    axis([-3 3 -3 3 -3 3]); % Adjust the axis limits as needed
    drawnow; % Update the plot
    pause(0.1); % Pause for a short duration
end

% DH Matrix Function
function T = dh_matrix(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end
