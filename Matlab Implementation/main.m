%% Mech 444/544 Project #2 - Group 09
%% VARIABLES
% Only change the variables on this section!
% Variables in this part controls the program behaviour and output.
clear
clc
close all

% Load the variables
config;

Parfor_Multiplier = 1 / dTheta; % This variable is defined to speed up the C_Obstacle  
                                % Generation process. Multiple cores of the CPU can be 
                                % Utilized to execute the next for loop(change 'for' to 'parfor' for the alpha iteration)
                                % Completion time decreases significantly
                                % for smaller dTheta values. For the sake
                                % of intelligibility, this variable and dTheta can
                                % be considered as 1.
tic % Start the timer
fprintf('Collision map generation started\n');
% Use parfor instead of for, for a faster iteration
for alpha = 0  : 180 * Parfor_Multiplier % Iterate for alpha values
    
    if mod(alpha, 180 * Parfor_Multiplier / 5) == 0
        fprintf('.'); % Print dots during the map generation to indicate progress
    end
    
    for beta = 0 : 360 * Parfor_Multiplier % Iterate for beta values        
        is_point_collided = false; % Set this to false for every alpha-beta pair
        for l1 = 0 : dL : L1 % Collision check for link 1
            px = l1 * cosd(alpha / Parfor_Multiplier) + robot_base_x; % X coordinates of the discretized points on link 1
            py = l1 * sind(alpha / Parfor_Multiplier) + robot_base_y; % Y coordinates of the discretized points on link 1
            is_point_collided = isInsideCircle(px, py, obstacle_x, obstacle_y, obstacle_r + rubber_band)... % Whether the point is colliding with any of the boundary conditions
                || px < 0 || py < 0 || py > 100 || px > 100 ;
            if is_point_collided % If the link 2 violates any of the boundary conditions, terminate the link 1 check
                break;
            end
        end
        
        if is_point_collided % If link 1 collides with the given alpha-beta pair, then go for the next pair
            CSpace_Obstacles = [CSpace_Obstacles; [alpha/Parfor_Multiplier beta/Parfor_Multiplier]]; % Add the alpha-beta pair to CMap
            continue;
        end
        
        for l2 = 0 : dL : L2 % Collision check for link 2
            px = L1 * cosd(alpha/Parfor_Multiplier) + l2 * cosd(alpha/Parfor_Multiplier + beta/Parfor_Multiplier) + robot_base_x; % X coordinates of the discretized points on link 2
            py = L1 * sind(alpha/Parfor_Multiplier) + l2 * sind(alpha/Parfor_Multiplier + beta/Parfor_Multiplier) + robot_base_y; % Y coordinates of the discretized points on link 2
            is_point_collided = isInsideCircle(px, py, obstacle_x, obstacle_y, obstacle_r + rubber_band)... % Whether the point is colliding with any of the boundary conditions
                || px < 0 || py < 0 || py > 100 || px > 100;
            if is_point_collided % If the link 2 violates any of the boundary conditions, terminate the link 2 check
                break;
            end
        end
        if is_point_collided % If link 2 collides with the given alpha-beta pair, then go for the next pair
            CSpace_Obstacles = [CSpace_Obstacles; [alpha/Parfor_Multiplier beta/Parfor_Multiplier]]; % Add the alpha-beta pair to CMap
        end
    end
end
elapsed_time = toc; % End the timer
fprintf('\nCollision map generation completed in %.2f seconds.\n', elapsed_time); % Print the elapsed time

%% Plot the configuration space
% Inverse Kinematics
Start_Angles = mod(rad2deg(InverseKinematics(start_x, start_y, L1, L2)),360); % Inverse kinematics for the starting position
Goal_Angles = mod(rad2deg(InverseKinematics(goal_x, goal_y, L1, L2)),360); % Inverse kinematics for the goal position

figure
hold on
axis square

plot(CSpace_Obstacles(:,1),CSpace_Obstacles(:,2),'k.'); % Plot the obstacles
plot(Start_Angles(1,1),Start_Angles(1,2),'*m','LineWidth',1.5,'MarkerSize',8); % Mark one of the starting configurations
text(Start_Angles(1,1)+3,Start_Angles(1,2),'Start Configuration 1','Color','m') % Write the text near the marker
plot(Start_Angles(2,1),Start_Angles(2,2),'*m','LineWidth',1.5,'MarkerSize',8); % Mark one of the starting configurations
text(Start_Angles(2,1)+3,Start_Angles(2,2),'Start Configuration 2','Color','m') % Write the text near the marker

plot(Goal_Angles(1,1),Goal_Angles(1,2),'*g','LineWidth',1.5,'MarkerSize',8); % Mark one of the goal configurations
text(Goal_Angles(1,1)+3,Goal_Angles(1,2),'Goal Configuration 1','Color','g') % Write the text near the marker
plot(Goal_Angles(2,1),Goal_Angles(2,2),'*g','LineWidth',1.5,'MarkerSize',8); % Mark one of the goal configurations
text(Goal_Angles(2,1)+3,Goal_Angles(2,2),'Goal Configuration 2','Color','g') % Write the text near the marker
 
title(strcat("Configuration Space ","(L_{1}= ",num2str(L1)," , L_{2}= ",num2str(L2),")"));
xlabel('\alpha');
ylabel('\beta');
xlim([0 180]);
ylim([0 360]);
grid on
drawnow
%% Trajectory Calculation and Animation
if use_wavefront
    calculate_wavefront; % Calls this script
else
    calculate_potential_map; % Calls this script
end
%% Functions
function solutions = InverseKinematics(x, y, L1, L2)
solutions = [0 0];
global robot_base_x
global robot_base_y
x = x - robot_base_x;
y = y - robot_base_y;

if x^2 + y^2 <= (L1+L2) ^ 2 % Calculate the IK if the point is within reach
    c_2 = (x ^ 2 + y ^ 2 - L1 ^ 2 - L2 ^ 2) / (2 * L1 * L2);
    s_2 = sqrt(1 - c_2 ^ 2);
    theta2 = [atan2(-s_2,c_2); atan2(s_2,c_2)];
    
    beta = atan2(y, x);
    c_psi = (x^2 + y^2 + L1 ^ 2 - L2 ^ 2) / (2 * L1 * sqrt(x^2 + y^2));
    psi = atan2(sqrt(1 - c_psi ^ 2),c_psi);
    theta1 = [beta+psi; beta-psi];
    solutions = [theta1 theta2];
end
end
function position = ForwardKinematics(theta1, theta2, L1, L2)
global robot_base_x
global robot_base_y
position = [0 0];
position(1) = L1 * cosd(theta1) + L2 * cosd(theta1 + theta2) + robot_base_x;
position(2) = L1 * sind(theta1) + L2 * sind(theta1 + theta2) + robot_base_y;
end
function isInside = isInsideCircle(px, py, cx, cy, r)
isInside = false;
if (px - cx) ^ 2 + (py - cy) ^ 2 <= r ^ 2
    isInside = true;
end
end