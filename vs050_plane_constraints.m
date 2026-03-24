%% VS050 Robot with Plane Constraints
clear all; close all;

%% Initialize
include_namespace_dq;
vs050 = VS050RobotDH.kinematics();
q = [0; pi/3; pi/3; 0; pi/3; 0];

%% Controller Setup
lambda = 3;
eta = 0.1;
solver = DQ_QPOASESSolver();

controller = DQ_ClassicQPController(vs050, solver);
controller.set_control_objective(ControlObjective.Translation);
controller.set_gain(lambda);
controller.set_damping(eta);
controller.set_stability_threshold(0.001);

fprintf('VS050 Robot - Plane Constraints\n\n'); %[output:92ed1958]

%% Define 5cm Cube (centered at 0.3, 0.1, 0.5)
cube_center = [0.3; 0.1; 0.5];
half_side = 0.025;

planes = struct();
planes(1).n = -DQ.i; planes(1).d = cube_center(1) + half_side;
planes(2).n = DQ.i;  planes(2).d = cube_center(1) - half_side;
planes(3).n = -DQ.j; planes(3).d = cube_center(2) + half_side;
planes(4).n = DQ.j;  planes(4).d = cube_center(2) - half_side;
planes(5).n = -DQ.k; planes(5).d = cube_center(3) + half_side;
planes(6).n = DQ.k;  planes(6).d = cube_center(3) - half_side;

fprintf('Cube center: [%.3f, %.3f, %.3f]\n', cube_center); %[output:6e54a309]
fprintf('Cube size: 5cm x 5cm x 5cm\n\n'); %[output:2114da09]

%% Desired Positions
xd_list = [0.3, 0.1, 0.5;
           0.31, 0.11, 0.51;
           0.29, 0.09, 0.49;
           0.305, 0.105, 0.505]';

%% Simulation
T = 4;
dt = 0.01;
time_vec = [];
error_vec = [];
distances = zeros(6, 1);

fprintf('Starting simulation...\n'); %[output:41beed0e]

for target_idx = 1:size(xd_list, 2) %[output:group:47a5d21b]
    xd = DQ(xd_list(:, target_idx));
    t = 0;
    
    fprintf('Target %d: [%.3f, %.3f, %.3f]\n', target_idx, xd_list(:, target_idx)); %[output:5d04b99d]
    
    while t < T
        x = vs050.fkm(q);
        x_t = translation(x);
        x_t_vec = vec3(x_t);
        
        J_pose = vs050.pose_jacobian(q);
        J_trans = vs050.translation_jacobian(J_pose, x);
        
        % Constraints
        B_ineq = [];
        b_ineq = [];
        current_dist = zeros(6, 1);
        
        for i = 1:6
            % Calculate distance to plane
            if i == 1
                dist = planes(i).d - x_t_vec(1);
            elseif i == 2
                dist = x_t_vec(1) - planes(i).d;
            elseif i == 3
                dist = planes(i).d - x_t_vec(2);
            elseif i == 4
                dist = x_t_vec(2) - planes(i).d;
            elseif i == 5
                dist = planes(i).d - x_t_vec(3);
            else
                dist = x_t_vec(3) - planes(i).d;
            end
            
            current_dist(i) = dist;
            
            % Jacobian
            n_dq = vec4(planes(i).n);
            Jdist = n_dq' * J_trans;
            
            % Constraint: distance >= 0
            safety_margin = 0.001;
            B_ineq = [B_ineq; -Jdist];
            b_ineq = [b_ineq; -(dist - safety_margin)];
        end
        
        distances = [distances, current_dist];
        controller.set_inequality_constraint(-B_ineq, -b_ineq);
        
        u = controller.compute_setpoint_control_signal(q, vec4(xd)); %[output:8e3b064c]
        q = q + u * dt;
        
        time_vec(end+1) = (target_idx-1)*T + t;
        error_vec(end+1) = norm(vec4(xd) - vec4(x_t));
        t = t + dt;
    end
end %[output:group:47a5d21b]

distances = distances(:, 2:end);
fprintf('\nSimulation complete\n');

%% Plots
% Task Error
figure('Name', 'Task Error');
plot(time_vec, error_vec, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position Error (m)');
title('End-Effector Position Error');
grid on;

% Distances to Planes
figure('Name', 'Distances to Planes');
plane_names = {'Right (x+)', 'Left (x-)', 'Front (y+)', 'Back (y-)', 'Top (z+)', 'Bottom (z-)'};
for i = 1:6
    subplot(3, 2, i);
    plot(time_vec, distances(i, :), 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Distance (m)');
    title(plane_names{i});
    grid on;
    yline(0, 'r--', 'Boundary');
end

fprintf('Plots generated\n');

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":51.1}
%---
%[output:92ed1958]
%   data: {"dataType":"text","outputData":{"text":"VS050 Robot - Plane Constraints\n\n","truncated":false}}
%---
%[output:6e54a309]
%   data: {"dataType":"text","outputData":{"text":"Cube center: [0.300, 0.100, 0.500]\n","truncated":false}}
%---
%[output:2114da09]
%   data: {"dataType":"text","outputData":{"text":"Cube size: 5cm x 5cm x 5cm\n\n","truncated":false}}
%---
%[output:41beed0e]
%   data: {"dataType":"text","outputData":{"text":"Starting simulation...\n","truncated":false}}
%---
%[output:5d04b99d]
%   data: {"dataType":"text","outputData":{"text":"Target 1: [0.300, 0.100, 0.500]\n","truncated":false}}
%---
%[output:8e3b064c]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'qpOASES_options'.\n\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('DQ_QPOASESSolver\/solve_quadratic_program', '\/Users\/vishals\/Downloads\/Bonus Ras Assingnment\/lesson 8 sol ras\/DQ_QPOASESSolver.m', 40)\" style=\"font-weight:bold\">DQ_QPOASESSolver\/solve_quadratic_program<\/a> (<a href=\"matlab: opentoline('\/Users\/vishals\/Downloads\/Bonus Ras Assingnment\/lesson 8 sol ras\/DQ_QPOASESSolver.m',40,0)\">line 40<\/a>)\n            options = qpOASES_options('default', ...\n            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('DQ_TaskspaceQuadraticProgrammingController\/compute_setpoint_control_signal', '\/Users\/vishals\/Library\/Application Support\/MathWorks\/MATLAB Add-Ons\/Toolboxes\/dqrobotics\/matlab\/robot_control\/DQ_TaskspaceQuadraticProgrammingController.m', 78)\" style=\"font-weight:bold\">DQ_TaskspaceQuadraticProgrammingController\/compute_setpoint_control_signal<\/a> (<a href=\"matlab: opentoline('\/Users\/vishals\/Library\/Application Support\/MathWorks\/MATLAB Add-Ons\/Toolboxes\/dqrobotics\/matlab\/robot_control\/DQ_TaskspaceQuadraticProgrammingController.m',78,0)\">line 78<\/a>)\n                u = controller.solver.solve_quadratic_program(H,f,A,b,Aeq,beq);\n                ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"}}
%---
