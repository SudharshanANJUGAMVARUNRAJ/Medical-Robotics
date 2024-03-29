# Medical-Robotics
MATLAB code animates 3-DOF robot based on user-input target coordinates, utilizing inverse kinematics.
% Define the Denavit-Hartenberg parameters for each joint
L1 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi, pi]);
L2 = Link('d', 0, 'a', 2, 'alpha', 0, 'qlim', [-pi, pi]);
L3 = Link('d', 0, 'a', 2, 'alpha', 0, 'qlim', [-pi, pi]);
%1
% Create a 3-DOF robot
robot = SerialLink([L1, L2, L3], 'name', '3-DOF Robot');
% Set initial joint angles
q = [0, 0, 0];

%2

% Define the time vector for motion
t = linspace(0, 5, 100); % You can adjust the time range and resolution

% Generate joint angles over time for theta_x motion
theta_x_values = [linspace(0, 45, 25), linspace(45, -45, 25), linspace(-45, 0, 25)];
q1_values = deg2rad(theta_x_values);
q2_values = q(2) * ones(size(theta_x_values));
q3_values = q(3) * ones(size(theta_x_values));

% Loop through the joint angles for theta_x motion and update the plot
for i = 1:length(theta_x_values)
    q = [q1_values(i), q2_values(i), q3_values(i)];
    robot.plot(q);
    title(['3-DOF Robot Motion - \theta_x = ', num2str(theta_x_values(i))]);
    drawnow;
    pause(0.000001); % Adjust the pause duration as needed
end

% Generate joint angles over time for theta_y motion
theta_y_values = [linspace(0, 45, 25), linspace(45, -45, 25), linspace(-45, 0, 25)];
q1_values = q(1) * ones(size(theta_y_values));
q2_values = deg2rad(theta_y_values);
q3_values = q(3) * ones(size(theta_y_values));

% Loop through the joint angles for theta_y motion and update the plot
for i = 1:length(theta_y_values)
    q = [q1_values(i), q2_values(i), q3_values(i)];
    robot.plot(q);
    title(['3-DOF Robot Motion - \theta_y = ', num2str(theta_y_values(i))]);
    drawnow;
    pause(0.000001); % Adjust the pause duration as needed
end
%4
% Generate joint angles over time for theta_z motion
theta_z_values = linspace(0, 60, 25);
q1_values = q(1) * ones(size(theta_z_values));
q2_values = q(2) * ones(size(theta_z_values));
q3_values = deg2rad(theta_z_values);

% Loop through the joint angles for theta_z motion and update the plot
for i = 1:length(theta_z_values)
    q = [q1_values(i), q2_values(i), q3_values(i)];
    robot.plot(q);
    title(['3-DOF Robot Motion - \theta_z = ', num2str(theta_z_values(i))]);
    drawnow;
    pause(0.000001); % Adjust the pause duration as needed
end

% Initialize a matrix to store the joint angles for each step
trajectory = zeros(3, 3);

% Create a VideoWriter object
writerObj = VideoWriter('Movie', 'MPEG-4');
writerObj.Quality = 100;
open(writerObj);

% Iterate through three steps
for step = 1:3
    % Get user input for target coordinates
    x_target = input('Enter x-coordinate: ');
    y_target = input('Enter y-coordinate: ');
    z_target = input('Enter z-coordinate: ');

    % Use inverse kinematics to find joint angles for the target coordinates
    q_target = robot.ikine(transl(x_target, y_target, z_target), 'mask', [1, 1, 1, 0, 0, 0], 'q0', q, 'itmax', 1000);

    % Convert the target joint angles from radians to degrees
    q_target_degrees = rad2deg(q_target);

    % Display the target joint angles in degrees
    disp(['Target Joint Angles (degrees) - Step ', num2str(step), ':']);
    disp(q_target_degrees');

    % Generate a joint trajectory from the current joint angles to the target
    joint_trajectory = jtraj(q, q_target, 50);

    % Animate the robot along the joint trajectory and capture frames
    for i = 1:length(joint_trajectory)
        robot.plot(joint_trajectory(i, :), 'trail', 'r-');
        drawnow;
        
        % Capture frame and write to the video
        frame = getframe(gcf);
        writeVideo(writerObj, frame);
    end

    % Store the joint angles in the trajectory matrix
    trajectory(step, :) = q_target;

    % Update the initial guess for the next iteration
    q = q_target;
end

% Close the video file
close(writerObj);

% Display the trajectory matrix
disp('Joint Angles Trajectory:');
disp(trajectory);
