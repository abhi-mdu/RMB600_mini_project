close all
clear
clc

% MODULE MainModule
% 	TASK PERS tooldata t4:=[TRUE,[[-105.513,2.40649,246.356],[1,0,0,0]],[0.5,[50,0,50],[1,0,0,0],0,0,0]];
% 	TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[559.804,5.50957,-3.63248],[0.999987,-0.00156359,-0.00487101,7.47128E-05]],[[5,4,0],[0.67559,0,0,-0.737277]]];
% 	CONST robtarget p10:=[[-46.86,-7.90,235.64],[0.0498083,-0.0133606,-0.998594,-0.0123139],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
% 	CONST robtarget p20:=[[-21.39,-34.91,-0.63],[0.0497861,-0.0134405,-0.998589,-0.0127018],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
% 	CONST robtarget p30:=[[19.16,-34.92,-0.53],[0.0498128,-0.0133747,-0.998593,-0.0123192],[-1,0,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
% 	CONST robtarget p40:=[[19.17,35.63,-0.49],[0.0498143,-0.0133815,-0.998593,-0.0123221],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
% 	CONST robtarget p50:=[[-21.65,35.64,-0.22],[0.0498108,-0.0133915,-0.998593,-0.0123259],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
% 	CONST robtarget p60:=[[-21.65,35.64,-0.22],[0.0498136,-0.0133915,-0.998593,-0.0123257],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
% 	VAR num xpos:=0;
% 	VAR num ypos:=0;
% 	VAR num zrot:=0;
%     VAR robtarget current_pos;
%     VAR intnum timer;
%     VAR num x;
%     VAR num y;
%     VAR num z;
%     VAR iodev logfile;
% 	PROC main()
%         CONNECT timer WITH Read;
%         Open "HOME:" \File:= "LOGFILE2.DOC", logfile \Write;
%         ITimer 0.5, timer;
% 		draw;
% 		TPReadNum xpos, "Enter the value of x ";
% 		TPReadNum ypos, "Enter the value of y ";
% 		TPReadNum zrot, "Enter the angle of z ";
% 		wobj1.oframe.trans.x := xpos;
% 		wobj1.oframe.trans.y := ypos;
% 		wobj1.oframe.rot := OrientZYX(zrot,0,0);
% 		draw;
% 	ENDPROC
% 	PROC draw()
% 		MoveJ p10, v200, fine, t4\WObj:=wobj1;
% 		MoveL p20, v200, z10, t4\WObj:=wobj1;
% 		MoveL p30, v200, z10, t4\WObj:=wobj1;
% 		MoveL p40, v200, z10, t4\WObj:=wobj1;
% 		MoveL p50, v200, z10, t4\WObj:=wobj1;
% 		MoveL p20, v200, fine, t4\WObj:=wobj1;
% 	ENDPROC
% 	TRAP Read
% 		current_pos := CRobT();
%         x:=current_pos.trans.x;
%         y:=current_pos.trans.y;
%         z:=current_pos.trans.z;
%         Write logfile,"X;"\num:=x;
%         Write logfile,"Y;"\num:=y;
%         Write logfile,"Z;"\num:=z;
% 	ENDTRAP
% ENDMODULE


robot = importrobot('robot/test.urdf');
config = robot.randomConfiguration;
tform = getTransform(robot,config,"link6_passive");
robot = addFrame([-105.513,2.40649,246.356],[1,0,0,0],robot,'t4','t4j','link6_passive');
robot = addFrame([559.804,5.50957,-3.63248],[0.999987,-0.00156359,-0.00487101,7.47128E-05],robot,'uframe','uframej','base');
robot = addFrame([5,4,0],[0.67559,0,0,-0.737277],robot,'oframe','oframej','uframe');

robot = addFrame([-46.86,-7.90,235.64],[0.0498083,-0.0133606,-0.998594,-0.0123139],robot,'p10','p10j','oframe');
robot = addFrame([-21.39,-34.91,-0.63],[0.0497861,-0.0134405,-0.998589,-0.0127018],robot,'p20','p20j','oframe');
robot = addFrame([19.16,-34.92,-0.53],[0.0498128,-0.0133747,-0.998593,-0.0123192],robot,'p30','p30j','oframe');
robot = addFrame([19.17,35.63,-0.49],[0.0498143,-0.0133815,-0.998593,-0.0123221],robot,'p40','p40j','oframe');
robot = addFrame([-21.65,35.64,-0.22],[0.0498108,-0.0133915,-0.998593,-0.0123259],robot,'p50','p50j','oframe');
robot = addFrame([-21.65,35.64,-0.22],[0.0498136,-0.0133915,-0.998593,-0.0123257],robot,'p60','p60j','oframe');

show(robot,config,Visuals="on");

%%


MoveL(getTransform(robot,robot.homeConfiguration,"tool0"),getTransform(robot,config,"p10"),robot,'t4');
MoveL(getTransform(robot,config,"p10"),getTransform(robot,config,"p20"),robot,'t4');
MoveL(getTransform(robot,config,"p20"),getTransform(robot,config,"p30"),robot,'t4');
MoveL(getTransform(robot,config,"p30"),getTransform(robot,config,"p40"),robot,'t4');
MoveL(getTransform(robot,config,"p40"),getTransform(robot,config,"p50"),robot,'t4');
MoveL(getTransform(robot,config,"p50"),getTransform(robot,config,"p60"),robot,'t4');
MoveL(getTransform(robot,config,"p60"),getTransform(robot,config,"p20"),robot,'t4');
MoveL(getTransform(robot,config,"p20"),getTransform(robot,config,"p10"),robot,'t4');





function robot = addFrame(Trans,q,robot,name,jointname,parentname)
    R=quat2rotMatrix(q);
    T = [[R;[0 0 0]],[(Trans./1000)';1]];
    frame = rigidBody(name);
    jnt1 = rigidBodyJoint(jointname,'fixed');
    setFixedTransform(jnt1,T);
    frame.Joint = jnt1;
    addBody(robot,frame,parentname);
end

function R = quat2rotMatrix(q)
    % Convert quaternion [w,x,y,z] to rotation matrix
    % Input: q = [w, x, y, z] quaternion
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
    
    % Compute rotation matrix
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
    num_points = 50;
    
    % Create figure for visualization
    figure;
    show(robot, robot.homeConfiguration, 'Visuals', 'on', 'Frames', 'off');
    hold on;
    
    % Plot the linear trajectory
    trajectory = zeros(3, num_points);
    for i = 1:num_points
        t = (i-1)/(num_points-1);
        
        % Linear interpolation of position
        p_interp = (1-t)*p_start + t*p_end;
        trajectory(:, i) = p_interp;
        
        % Spherical linear interpolation (SLERP) for rotation
        % Convert rotation matrices to quaternions for interpolation
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
            ik = inverseKinematics('RigidBodyTree', robot);
            weights = [1 1 1 1 1 1];
            initialGuess = robot.homeConfiguration;
            [config_sol, ~] = ik(toolFrame, T_interp, weights, initialGuess);
            
            % Show robot configuration
            if mod(i, 10) == 0 || i == 1 || i == num_points
                show(robot, config_sol, 'Visuals', 'on', 'Frames', 'off', 'PreservePlot', false);
                drawnow;
                pause(0.05);
            end
        catch
            warning('IK solution not found for interpolation point %d', i);
        end
    end
    
    % Plot trajectory line
    plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'r-', 'LineWidth', 2);
    hold off;
    title('Robot Linear Motion Trajectory');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    grid on;
end