# Robot Simulation Project - ABB IRB1600

## Overview
This project simulates an ABB IRB1600 industrial robot performing a drawing task. The simulation converts RAPID programming language (ABB's native robot language) to MATLAB using the Robotics System Toolbox.

## 🔗 Quick Access
**MATLAB Online Shared Project**: https://drive.mathworks.com/sharing/6aab9a91-d00f-461e-b031-61f1169b1f2d

*Access the complete project files directly in MATLAB Online. Open the link above to view and run all code without local installation.*

## Project Structure
```
mini_project/
├── robot_simulation.m          # Complete working simulation script
├── missing_code.m              # Original script with added functions
├── robot/
│   ├── test.urdf              # Robot URDF model definition
│   └── IRB1600/               # 3D model files (STL)
│       ├── base.stl
│       ├── link1.stl
│       ├── link2.stl
│       ├── link3.stl
│       ├── link4.stl
│       ├── link5.stl
│       └── link6.stl
```

## Requirements
- MATLAB R2019b or later
- Robotics System Toolbox
- (Optional) Computer Vision Toolbox for advanced visualization

## Key Components

### 1. Robot Model
The simulation uses a URDF (Unified Robot Description Format) file that defines:
- 6 revolute joints (q1-q6)
- Robot links with 3D mesh geometry
- Joint limits and dynamics

### 2. Coordinate Frames
The robot uses a hierarchical frame system:

- **base**: Robot base frame
- **uframe**: User-defined coordinate frame (work area)
- **oframe**: Object frame where the drawing occurs
- **t4**: Tool frame (end-effector)
- **p10-p60**: Target points for the drawing path

### 3. Main Functions

#### `addFrame(Trans, q, robot, name, jointname, parentname)`
Adds a fixed coordinate frame to the robot model.
- **Trans**: Translation vector [x, y, z] in millimeters
- **q**: Orientation quaternion [w, x, y, z]
- **robot**: Robot model object
- **name**: Name of the new frame
- **jointname**: Name of the connecting joint
- **parentname**: Parent frame name

#### `quat2rotMatrix(q)`
Converts a quaternion to a 3×3 rotation matrix.
- **Input**: Quaternion [w, x, y, z] (scalar-first convention)
- **Output**: 3×3 rotation matrix
- Uses normalization to handle numerical errors
- Implements standard quaternion-to-rotation-matrix formula

#### `MoveL(T_start, T_end, robot, toolFrame)`
Simulates linear motion between two poses.
- **T_start**: Starting 4×4 transformation matrix
- **T_end**: Ending 4×4 transformation matrix
- Performs:
  - Linear interpolation for position (Cartesian path)
  - SLERP (Spherical Linear Interpolation) for orientation
  - Inverse kinematics to compute joint angles
  - Real-time visualization of robot motion

#### `MoveJ(T_start, T_end, robot, toolFrame)`
Simulates joint-space motion between two poses.
- **T_start**: Starting 4×4 transformation matrix
- **T_end**: Ending 4×4 transformation matrix
- Performs:
  - Linear interpolation in joint space
  - Each joint moves independently at constant velocity
  - Results in curved path in Cartesian space
  - Faster motion compared to MoveL
  - Real-time visualization with blue trajectory line

## Drawing Task
The robot draws a pentagonal shape by moving through these target points:
1. **p10**: Starting position (above the surface)
2. **p20**: First corner
3. **p30**: Second corner
4. **p40**: Third corner
5. **p50**: Fourth corner
6. **p60**: Fifth corner
7. Return to **p20** and **p10**

## How to Run

### Option 1: Run Complete Simulation
```matlab
% Navigate to the project directory
cd 'd:\Masters\Robotics\mini_project'

% Run the complete simulation
robot_simulation
```

### Option 2: Run Original Script
```matlab
% Run the updated missing_code.m
missing_code
```

## Understanding the Code

### Coordinate Transformations
The project uses homogeneous transformation matrices (4×4):
```
T = [R  p]
    [0  1]
```
Where:
- R = 3×3 rotation matrix
- p = 3×1 position vector

### Quaternion Convention
Quaternions are in **scalar-first** format: [w, x, y, z]
- w: scalar part
- [x, y, z]: vector part

### Inverse Kinematics
The simulation uses MATLAB's `inverseKinematics` solver to find joint angles that achieve desired end-effector poses:
- Weights: [1 1 1 1 1 1] (equal importance for position and orientation)
- Iterative solution with warm start from previous configuration

## Troubleshooting

### Issue: "Unable to find robot model"
**Solution**: Ensure you're in the correct directory and the URDF file path is correct:
```matlab
robot = importrobot('robot/test.urdf');
```

### Issue: "Inverse kinematics solution not found"
**Solution**: This can happen if target poses are unreachable. Check:
- Joint limits in the URDF file
- Target point coordinates
- Robot workspace limits

### Issue: Slow visualization
**Solution**: Reduce the number of interpolation points in `MoveL`:
```matlab
num_points = 20; % Reduce from 30
```

## Technical Details

### Frame Hierarchy
```
base
 ├── link1 (q1)
 │   └── link1_passive
 │       └── link2 (q2)
 │           └── link2_passive
 │               └── link3 (q3)
 │                   └── link3_passive
 │                       └── link4 (q4)
 │                           └── link4_passive
 │                               └── link5 (q5)
 │                                   └── link5_passive
 │                                       └── link6 (q6)
 │                                           └── link6_passive
 │                                               └── t4 (tool)
 └── uframe
     └── oframe
         ├── p10
         ├── p20
         ├── p30
         ├── p40
         ├── p50
         └── p60
```

### Linear Motion Algorithm
1. Extract start and end positions/orientations
2. Create interpolation points (default: 30)
3. For each point:
   - Linearly interpolate position: `p(t) = (1-t)*p_start + t*p_end`
   - SLERP interpolate orientation: `q(t) = slerp(q_start, q_end, t)`
4. Solve inverse kinematics for joint angles
5. Visualize robot configuration

### SLERP (Spherical Linear Interpolation)
For smooth rotation interpolation:
```
q(t) = sin((1-t)θ)/sin(θ) * q_start + sin(tθ)/sin(θ) * q_end
```
Where θ is the angle between quaternions.

## References
- ABB RAPID Programming Manual
- MATLAB Robotics System Toolbox Documentation
- Peter Corke's Robotics Toolbox
- URDF Format Specification

## Author Notes
This simulation demonstrates:
- Robot kinematics and motion planning
- Coordinate frame transformations
- Industrial robot programming concepts
- MATLAB robotics visualization

## Future Enhancements
- [ ] Add velocity and acceleration profiles
- [x] Implement joint-space motion (MoveJ)
- [ ] Add collision detection
- [ ] Export trajectory data to CSV
- [ ] Create animation video export
- [ ] Add user input for custom drawing paths
