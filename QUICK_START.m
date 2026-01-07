% QUICK START GUIDE
% ===========================================

%% 1. SETUP
% Make sure you're in the project directory
cd 'd:\Masters\Robotics\mini_project'

%% 2. RUN THE SIMULATION
% Option A: Run the complete simulation (recommended)
robot_simulation

% Option B: Run the original script with fixes
missing_code

%% 3. WHAT YOU'LL SEE
% - Robot model with all coordinate frames
% - Animated robot motion following a pentagonal path
% - Red trajectory line showing the tool path
% - Step-by-step visualization of the drawing

%% 4. KEY PARAMETERS YOU CAN MODIFY

% In robot_simulation.m or missing_code.m:

% Change interpolation resolution (line 154 in robot_simulation.m)
num_points = 30; % Increase for smoother motion, decrease for faster simulation

% Change visualization update rate (line 178 in robot_simulation.m)
if mod(i, 5) == 0  % Change 5 to update more/less frequently

% Change pause time for animation speed (line 192 in robot_simulation.m)
pause(0.05); % Increase for slower animation, decrease for faster

%% 5. UNDERSTANDING THE OUTPUT
% The simulation shows:
% - Blue: Robot links
% - Red line: Tool path trajectory
% - Robot moving through positions p10 → p20 → p30 → p40 → p50 → p60 → p20 → p10

%% 6. TROUBLESHOOTING

% If you see "File not found" error:
% Make sure URDF path is correct
robot = importrobot('robot/test.urdf');

% If IK fails:
% The target might be unreachable - check joint limits or target positions

% If visualization is too slow:
% Reduce num_points in MoveL function

%% 7. EXPLORING THE ROBOT

% Get robot information
robot.NumBodies      % Number of links
robot.BodyNames      % Names of all bodies
robot.BaseName       % Base link name

% Get specific transformations
config = robot.homeConfiguration;
T_tool = getTransform(robot, config, 't4');  % Tool transformation

% Visualize robot at home position
figure;
show(robot, robot.homeConfiguration, 'Visuals', 'on', 'Frames', 'on');

%% 8. CUSTOM DRAWING PATHS

% To create your own path, add new target points:
% robot = addFrame([x,y,z], [w,x,y,z], robot, 'p70', 'p70j', 'oframe');

% Then add movement:
% MoveL(getTransform(robot,config,"p60"), getTransform(robot,config,"p70"), robot,'t4');

%% 9. EXPORTING DATA

% To save trajectory data:
% Add this in MoveL function after trajectory calculation:
% save('trajectory.mat', 'trajectory');

% To export as CSV:
% writematrix(trajectory', 'trajectory.csv');

%% 10. ADDITIONAL ANALYSIS

% Calculate workspace
% Calculate joint angles for specific pose
% Analyze singularities
% Plot velocity profiles

% Example: Get joint configuration for a target
ik = inverseKinematics('RigidBodyTree', robot);
T_target = getTransform(robot, config, 'p20');
[config_solution, solInfo] = ik('t4', T_target, [1 1 1 1 1 1], robot.homeConfiguration);

% Show solution
figure;
show(robot, config_solution);
title('Robot Configuration at p20');
