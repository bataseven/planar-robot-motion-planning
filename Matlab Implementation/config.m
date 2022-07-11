global robot_base_x % Define this as a global to make it available for the functions
robot_base_x = 20;  % Base position of the robot in cartesian space (x component)
global robot_base_y % Define this as a global to make it available for the functions
robot_base_y = 0; % Base position of the robot in cartesian space (y component)

L1 = 38; % Length of link 1 determined by trial and error
L2 = 38; % Length of link 1 determined by trial and error

start_x = 20; % Starting position of the end effector in cartesian space (x component)
start_y = 70; % Starting position of the end effector in cartesian space (y component) 

goal_x = 60; % Goal position of the end effector in cartesian space (x component)
goal_y = 40; % Goal position of the end effector in cartesian space (y component) 

obstacle_x = 40; % Position of the obstacle in cartesian space (x component)
obstacle_y = 60; % Position of the obstacle in cartesian space (y component)
obstacle_r = 10; % Radius of the obstacle
rubber_band = 0; % Value of this variable is added on top of the radius of obstacle during calculations

elbow_start = 2; % "1" for elbow up | "2" for elbow down
elbow_goal = 2; %  "1" for elbow up | "2" for elbow down

use_wavefront = false; % If set to 'true', trajectory calculation is made using the wavefront approach
                      % Otherwise, trajectory is calculated using gradient
                      % descent on the potential field.
                      
record_video = false; % Set this to true to record the animation
%% Fill CSpace_Obstacles by checking collisions for alpha-beta pairs
CSpace_Obstacles = []; % This array will contain blocked alpha-beta pairs
dL = 1; % Increment to check the links for
dTheta = 1; % Increment to check the theta values for