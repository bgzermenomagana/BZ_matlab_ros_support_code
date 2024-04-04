% Task: grab red cans and place them in 
% Bottles go in blue bin
% Cans go in green bin

% Notes from watching the videos
% learn how move boxes to reach object
% learn how to rotate gripper to avoid collision with surrounding objects

% learn how to define success/failure...did the object make into the bin
% if not repeat attempt

% Initiate ROS
pause(2);
rosIP = '192.168.160.128';
rosshutdown;
rosinit(rosIP,11311);

% Reseting robot position
disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld;      % reset models through a gazebo service

%rCan1
disp('Getting goal...')
type = 'gazebo'; % gazebo, ptcloud, cam, manual

% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{24};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  

    fprintf('Picking up model: %s \n',model_name);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);

% % Manual (i.e. rCan3)
elseif strcmp(type,'manual')
    % Change this to rCan1 pose
    goal = [0.8, -0.04, 0.15, -pi/2, -pi 0];     %[px,py,pz, z y z]
    % goal = [0.4, -0.5, 0.15, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
else
    % Manually
    % Change this to rCan1 pose
    goal = [-1*-0.04, 0.8, 0.10, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
end

strategy = 'topdown';
ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G)

if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end

if ~ret
    ret = moveToQ('qr');
end

pause(2)

%rCan2
disp('Getting goal...')
type = 'gazebo'; % gazebo, ptcloud, cam, manual

% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{25};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  

    fprintf('Picking up model: %s \n', model_name);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);

% % Manual (i.e. rCan3)
elseif strcmp(type,'manual')
    goal = [0.1, 0.7, 0.35, -pi/2, -pi 0];     %[px,py,pz, z y z]
    % switch x and y from matlab matrix
    mat_R_T_M = set_manual_goal(goal);
else
    % Manually
    goal = [-1*-0.04, 0.8, 0.10, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
end

strategy = 'topdown';
ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G)

if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end

if ~ret
    ret = moveToQ('qr');
end

pause(2)

%rCan3
disp('Getting goal...')
type = 'gazebo'; % gazebo, ptcloud, cam, manual

% Via Gazebo
if strcmp(type,'gazebo')
    models = getModels;                         % Extract gazebo model list
    model_name = models.ModelNames{26};         % rCan3=26, yCan1=27,rBottle2=32...%model_name = models.ModelNames{i}  

    fprintf('Picking up model: %s \n',model_name);
    [mat_R_T_G, mat_R_T_M] = get_robot_object_pose_wrt_base_link(model_name);

% Manual (i.e. rCan3)
elseif strcmp(type,'manual')
    goal = [0.8, -0.04, 0.15, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
else
    % Manually
    goal = [-1*-0.04, 0.8, 0.10, -pi/2, -pi 0];     %[px,py,pz, z y z]
    mat_R_T_M = set_manual_goal(goal);
end

strategy = 'topdown';
ret = pick(strategy, mat_R_T_M); % Can have optional starting opse for ctraj like: ret = pick(strategy, mat_R_T_M,mat_R_T_G)

if ~ret
    disp('Attempting place...')
    greenBin = [-0.4, -0.45, 0.25, -pi/2, -pi 0];
    place_pose = set_manual_goal(greenBin);
    strategy = 'topdown';
    fprintf('Moving to bin...');
    ret = moveToBin(strategy,mat_R_T_M,place_pose);
end

if ~ret
    ret = moveToQ('qr');
end