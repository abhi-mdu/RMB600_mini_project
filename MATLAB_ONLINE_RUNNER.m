%% MATLAB ONLINE - Complete Project Runner
% This script runs all tests and generates visualizations
% Copy this entire file to MATLAB Online and run

%% Setup
clear; clc; close all;
fprintf('==========================================================\n');
fprintf('       ROBOTICS PROJECT - MATLAB ONLINE RUNNER           \n');
fprintf('==========================================================\n\n');

%% Part 1: Quaternion Tests
fprintf('Running Quaternion Tests...\n\n');

% Test 1: Identity Quaternion
q_identity = [1, 0, 0, 0];
R_identity = quat2rotMatrix(q_identity);
R_expected = eye(3);
error1 = norm(R_identity - R_expected);
fprintf('TEST 1: Identity Quaternion\n');
fprintf('  Error: %.10e\n', error1);
fprintf('  Result: %s\n\n', iif(error1 < 1e-6, 'PASS ✓', 'FAIL ✗'));

% Test 2: 90° Z-axis Rotation
q_90z = [0.7071, 0, 0, 0.7071];
R_90z = quat2rotMatrix(q_90z);
R_expected = [0 -1 0; 1 0 0; 0 0 1];
error2 = norm(R_90z - R_expected, 'fro');
fprintf('TEST 2: 90° Z-axis Rotation\n');
fprintf('  Error: %.10e\n', error2);
fprintf('  Result: %s\n\n', iif(error2 < 1e-5, 'PASS ✓', 'FAIL ✗'));

% Test 3: Rotation Matrix Properties
q_robot = [0.924672, 0, 0.380768, 0];
R_robot = quat2rotMatrix(q_robot);
det_val = det(R_robot);
ortho_error = norm(R_robot' * R_robot - eye(3));
fprintf('TEST 3: Rotation Matrix Properties\n');
fprintf('  Determinant: %.10f\n', det_val);
fprintf('  Orthogonality: %.2e\n', ortho_error);
fprintf('  Result: %s\n\n', iif(abs(det_val-1)<1e-6 && ortho_error<1e-10, 'PASS ✓', 'FAIL ✗'));

%% Part 2: Linear Trajectory
fprintf('Generating Linear Trajectory...\n\n');

p_start = [0.500; 0.100; 0.300];
p_end = [0.300; 0.400; 0.500];
q_start = [1, 0, 0, 0];
q_end = [0.7071, 0, 0, 0.7071];
num_points = 20;

% Generate trajectory
[trajectory_pos, trajectory_quat] = generateTrajectory(p_start, p_end, q_start, q_end, num_points);

% Verify linearity
max_deviation = 0;
for i = 2:num_points-1
    t = (i-1)/(num_points-1);
    expected = (1-t)*p_start + t*p_end;
    deviation = norm(trajectory_pos(:,i) - expected);
    max_deviation = max(max_deviation, deviation);
end

fprintf('Trajectory Generated:\n');
fprintf('  Points: %d\n', num_points);
fprintf('  Path length: %.4f m\n', norm(p_end - p_start));
fprintf('  Linearity deviation: %.10e m\n', max_deviation);
fprintf('  Result: %s\n\n', iif(max_deviation < 1e-6, 'PASS ✓', 'FAIL ✗'));

%% Part 3: Visualization 1 - Pentagon Path
fprintf('Creating Figure 1: Pentagon Path...\n');
figure('Position', [100, 100, 800, 600]);

% Pentagon points (from RAPID code)
p10 = [487.86; -57.21; 558.32] / 1000; % Convert mm to m
p20 = [487.86; -57.21; 386.46] / 1000;
p30 = [528.46; -57.21; 398.92] / 1000;
p40 = [553.51; -17.48; 436.46] / 1000;
p50 = [528.51; 22.25; 398.98] / 1000;
p60 = [487.86; 9.79; 386.46] / 1000;

% Plot path
path_points = [p10, p20, p30, p40, p50, p60, p20, p10];
plot3(path_points(1,:), path_points(2,:), path_points(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(path_points(1,:), path_points(2,:), path_points(3,:), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot3(p10(1), p10(2), p10(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Pentagon Drawing Path - Robot End-Effector Trajectory');
legend('Path', 'Target Points', 'Start Position', 'Location', 'best');
view(45, 30);
axis equal;

saveas(gcf, 'Figure1_Pentagon_Path.png');
fprintf('  ✓ Saved: Figure1_Pentagon_Path.png\n\n');

%% Part 4: Visualization 2 - Linear Trajectory
fprintf('Creating Figure 2: Linear Trajectory...\n');
figure('Position', [100, 100, 1200, 800]);

t_vals = linspace(0, 1, num_points);

% 3D trajectory
subplot(2,2,1);
plot3(trajectory_pos(1,:), trajectory_pos(2,:), trajectory_pos(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(p_start(1), p_start(2), p_start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(p_end(1), p_end(2), p_end(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Linear Trajectory');
legend('Trajectory', 'Start', 'End');
view(45, 30);

% X component
subplot(2,2,2);
plot(t_vals, trajectory_pos(1,:), 'b-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('X (m)');
title('X Position vs Time');

% Y component
subplot(2,2,3);
plot(t_vals, trajectory_pos(2,:), 'r-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('Y (m)');
title('Y Position vs Time');

% Z component
subplot(2,2,4);
plot(t_vals, trajectory_pos(3,:), 'g-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('Z (m)');
title('Z Position vs Time');

saveas(gcf, 'Figure2_Linear_Trajectory.png');
fprintf('  ✓ Saved: Figure2_Linear_Trajectory.png\n\n');

%% Part 5: Visualization 3 - SLERP Orientation
fprintf('Creating Figure 3: SLERP Orientation...\n');
figure('Position', [100, 100, 1200, 800]);

% Extract quaternion components
q_w = trajectory_quat(1,:);
q_x = trajectory_quat(2,:);
q_y = trajectory_quat(3,:);
q_z = trajectory_quat(4,:);
q_mag = sqrt(q_w.^2 + q_x.^2 + q_y.^2 + q_z.^2);

% Scalar component
subplot(2,2,1);
plot(t_vals, q_w, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('w (scalar)');
title('Quaternion Scalar Component');

% Vector components
subplot(2,2,2);
plot(t_vals, q_x, 'r-', 'LineWidth', 2); hold on;
plot(t_vals, q_y, 'g-', 'LineWidth', 2);
plot(t_vals, q_z, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('Component value');
title('Quaternion Vector Components');
legend('x', 'y', 'z', 'Location', 'best');

% Magnitude
subplot(2,2,3);
plot(t_vals, q_mag, 'k-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('Magnitude');
title('Quaternion Magnitude (should be 1)');

% All components
subplot(2,2,4);
plot(t_vals, q_w, 'b-', 'LineWidth', 2); hold on;
plot(t_vals, q_x, 'r-', 'LineWidth', 2);
plot(t_vals, q_y, 'g-', 'LineWidth', 2);
plot(t_vals, q_z, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time parameter t'); ylabel('Value');
title('All Quaternion Components');
legend('w', 'x', 'y', 'z', 'Location', 'best');

saveas(gcf, 'Figure3_SLERP_Orientation.png');
fprintf('  ✓ Saved: Figure3_SLERP_Orientation.png\n\n');

%% Summary
fprintf('==========================================================\n');
fprintf('                    EXECUTION SUMMARY                     \n');
fprintf('==========================================================\n');
fprintf('✓ All quaternion tests completed\n');
fprintf('✓ Linear trajectory generated and verified\n');
fprintf('✓ 3 figures created and saved:\n');
fprintf('  - Figure1_Pentagon_Path.png\n');
fprintf('  - Figure2_Linear_Trajectory.png\n');
fprintf('  - Figure3_SLERP_Orientation.png\n');
fprintf('\nAll files are ready for download from MATLAB Online!\n');
fprintf('==========================================================\n');

%% Helper Functions

function R = quat2rotMatrix(q)
    % Normalize quaternion
    q = q / norm(q);
    
    % Extract components
    w = q(1); x = q(2); y = q(3); z = q(4);
    
    % Compute rotation matrix
    R = [1-2*(y^2+z^2),   2*(x*y-w*z),   2*(x*z+w*y);
         2*(x*y+w*z),   1-2*(x^2+z^2),   2*(y*z-w*x);
         2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
end

function q_interp = slerp(q1, q2, t)
    % Compute dot product
    dot_product = dot(q1, q2);
    dot_product = max(-1.0, min(1.0, dot_product));
    
    % Compute angle
    theta = acos(dot_product);
    
    % Handle near-parallel quaternions
    if abs(theta) < 1e-6
        q_interp = (1-t)*q1 + t*q2;
        q_interp = q_interp / norm(q_interp);
        return;
    end
    
    % Standard SLERP
    sin_theta = sin(theta);
    w1 = sin((1-t)*theta) / sin_theta;
    w2 = sin(t*theta) / sin_theta;
    
    q_interp = w1*q1 + w2*q2;
    q_interp = q_interp / norm(q_interp);
end

function [pos, quat] = generateTrajectory(p_start, p_end, q_start, q_end, num_points)
    % Ensure quaternions take shortest path
    if dot(q_start, q_end) < 0
        q_end = -q_end;
    end
    
    % Preallocate
    pos = zeros(3, num_points);
    quat = zeros(4, num_points);
    
    % Generate trajectory
    for i = 1:num_points
        t = (i-1) / (num_points-1);
        
        % Linear position interpolation
        pos(:,i) = (1-t)*p_start + t*p_end;
        
        % SLERP orientation interpolation
        q_interp = slerp(q_start, q_end, t);
        quat(:,i) = q_interp';
    end
end

function result = iif(condition, trueVal, falseVal)
    % Inline if function
    if condition
        result = trueVal;
    else
        result = falseVal;
    end
end
