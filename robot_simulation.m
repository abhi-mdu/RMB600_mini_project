% Robot Simulation - ABB IRB1600 with MATLAB
% This script simulates a robot drawing task with linear movements
% Based on RAPID code converted to MATLAB using Robotics System Toolbox

close all
clear
clc

%% Load Robot Model
% Import the robot URDF model
robot = importrobot('robot/test.urdf');
config = robot.randomConfiguration;

%% Define Tool Frame (t4)
% Tool data from RAPID: [[-105.513,2.40649,246.356],[1,0,0,0]]
% Translation in mm, quaternion [w,x,y,z]
robot = addFrame([-105.513,2.40649,246.356],[1,0,0,0],robot,'t4','t4j','link6_passive');

%% Define User Frame (uframe)
% User frame relative to base
robot = addFrame([559.804,5.50957,-3.63248],[0.999987,-0.00156359,-0.00487101,7.47128E-05],robot,'uframe','uframej','base');

%% Define Object Frame (oframe)
% Object frame relative to user frame
robot = addFrame([5,4,0],[0.67559,0,0,-0.737277],robot,'oframe','oframej','uframe');

%% Define Target Points (p10-p60)
% All target points are defined relative to the object frame
robot = addFrame([-46.86,-7.90,235.64],[0.0498083,-0.0133606,-0.998594,-0.0123139],robot,'p10','p10j','oframe');
robot = addFrame([-21.39,-34.91,-0.63],[0.0497861,-0.0134405,-0.998589,-0.0127018],robot,'p20','p20j','oframe');
robot = addFrame([19.16,-34.92,-0.53],[0.0498128,-0.0133747,-0.998593,-0.0123192],robot,'p30','p30j','oframe');
robot = addFrame([19.17,35.63,-0.49],[0.0498143,-0.0133815,-0.998593,-0.0123221],robot,'p40','p40j','oframe');
robot = addFrame([-21.65,35.64,-0.22],[0.0498108,-0.0133915,-0.998593,-0.0123259],robot,'p50','p50j','oframe');
robot = addFrame([-21.65,35.64,-0.22],[0.0498136,-0.0133915,-0.998593,-0.0123257],robot,'p60','p60j','oframe');

%% Visualize Robot
figure('Name', 'Robot Model with Frames', 'NumberTitle', 'off');
show(robot,config,'Visuals','on','Frames','on');
title('Robot Model with All Frames');
axis equal;
grid on;

%% Execute Drawing Motion
% Simulate the RAPID draw() procedure
fprintf('Starting robot drawing simulation...\n');

% Move to starting position
fprintf('Moving to p10...\n');
MoveL(getTransform(robot,robot.homeConfiguration,"t4"),getTransform(robot,config,"p10"),robot,'t4');

% Draw the shape (pentagon)
fprintf('Drawing shape...\n');
MoveL(getTransform(robot,config,"p10"),getTransform(robot,config,"p20"),robot,'t4');
MoveL(getTransform(robot,config,"p20"),getTransform(robot,config,"p30"),robot,'t4');
MoveL(getTransform(robot,config,"p30"),getTransform(robot,config,"p40"),robot,'t4');
MoveL(getTransform(robot,config,"p40"),getTransform(robot,config,"p50"),robot,'t4');
MoveL(getTransform(robot,config,"p50"),getTransform(robot,config,"p60"),robot,'t4');

% Return to p20 and then p10
fprintf('Returning to start position...\n');
MoveL(getTransform(robot,config,"p60"),getTransform(robot,config,"p20"),robot,'t4');
MoveL(getTransform(robot,config,"p20"),getTransform(robot,config,"p10"),robot,'t4');

fprintf('Drawing complete!\n');

%% Helper Functions

function robot = addFrame(Trans,q,robot,name,jointname,parentname)
    % Add a fixed frame to the robot model
    % Trans: translation vector in mm [x, y, z]
    % q: quaternion [w, x, y, z]
    % robot: robot model
    % name: name of the new frame
    % jointname: name of the joint connecting to parent
    % parentname: name of the parent frame
    
    R = quat2rotMatrix(q);
    T = [[R;[0 0 0]],[(Trans./1000)';1]];
    frame = rigidBody(name);
    jnt1 = rigidBodyJoint(jointname,'fixed');
    setFixedTransform(jnt1,T);
    frame.Joint = jnt1;
    addBody(robot,frame,parentname);
end

function R = quat2rotMatrix(q)
    % Convert quaternion [w,x,y,z] to rotation matrix
    % Input: q = [w, x, y, z] quaternion (scalar first)
    % Output: R = 3x3 rotation matrix
    
    w = q(1);
    x = q(2);
    y = q(3);
    z = q(4);
    
    % Normalize quaternion
    norm_q = sqrt(w^2 + x^2 + y^2 + z^2);
    w = w/norm_q;
    x = x/norm_q;
    y = y/norm_q;
    z = z/norm_q;
    
    % Compute rotation matrix using standard formula
    R = [1-2*(y^2+z^2),   2*(x*y-w*z),     2*(x*z+w*y);
         2*(x*y+w*z),     1-2*(x^2+z^2),   2*(y*z-w*x);
         2*(x*z-w*y),     2*(y*z+w*x),     1-2*(x^2+y^2)];
end

function MoveL(T_start, T_end, robot, toolFrame)
    % Linear motion from start to end transformation
    % T_start: 4x4 start transformation matrix
    % T_end: 4x4 end transformation matrix
    % robot: robot model
    % toolFrame: name of the tool frame
    
    % Extract positions
    p_start = T_start(1:3, 4);
    p_end = T_end(1:3, 4);
    
    % Extract rotations
    R_start = T_start(1:3, 1:3);
    R_end = T_end(1:3, 1:3);
    
    % Number of interpolation points
    num_points = 30;
    
    % Create or get figure for visualization
    if isempty(findobj('Type', 'figure', 'Name', 'Robot Motion'))
        figure('Name', 'Robot Motion', 'NumberTitle', 'off');
    else
        figure(findobj('Type', 'figure', 'Name', 'Robot Motion'));
    end
    
    % Setup inverse kinematics
    ik = inverseKinematics('RigidBodyTree', robot);
    weights = [1 1 1 1 1 1];
    initialGuess = robot.homeConfiguration;
    
    % Trajectory storage
    trajectory = zeros(3, num_points);
    
    % Interpolate and move
    for i = 1:num_points
        t = (i-1)/(num_points-1);
        
        % Linear interpolation of position
        p_interp = (1-t)*p_start + t*p_end;
        trajectory(:, i) = p_interp;
        
        % Spherical linear interpolation (SLERP) for rotation
        q_start = rotm2quat(R_start);
        q_end = rotm2quat(R_end);
        
        % Compute dot product
        dot_prod = sum(q_start .* q_end);
        
        % If dot product is negative, negate one quaternion to take shorter path
        if dot_prod < 0
            q_end = -q_end;
            dot_prod = -dot_prod;
        end
        
        % Perform SLERP
        if dot_prod > 0.9995
            % If quaternions are very close, use linear interpolation
            q_interp = (1-t)*q_start + t*q_end;
            q_interp = q_interp / norm(q_interp);
        else
            % SLERP formula
            theta = acos(dot_prod);
            q_interp = (sin((1-t)*theta)/sin(theta))*q_start + (sin(t*theta)/sin(theta))*q_end;
        end
        
        R_interp = quat2rotm(q_interp);
        
        % Construct transformation matrix
        T_interp = [R_interp, p_interp; 0 0 0 1];
        
        % Solve inverse kinematics
        try
            [config_sol, solInfo] = ik(toolFrame, T_interp, weights, initialGuess);
            
            % Update initial guess for next iteration
            initialGuess = config_sol;
            
            % Visualize at specific intervals
            if mod(i, 5) == 0 || i == 1 || i == num_points
                show(robot, config_sol, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);
                hold on;
                plot3(trajectory(1,1:i), trajectory(2,1:i), trajectory(3,1:i), 'r-', 'LineWidth', 2);
                hold off;
                title(sprintf('Robot Linear Motion - Step %d/%d', i, num_points));
                xlabel('X (m)');
                ylabel('Y (m)');
                zlabel('Z (m)');
                grid on;
                axis equal;
                drawnow;
                pause(0.05);
            end
        catch ME
            warning('IK solution not found for interpolation point %d: %s', i, ME.message);
        end
    end
    
    % Plot final trajectory line
    hold on;
    plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'r-', 'LineWidth', 2);
    hold off;
end
