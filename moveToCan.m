% Objective:
% an array with the [x,y,z,r,p,y] of your closed end-effector at the time it touches the top of the can

% Initiating ros connection with gazebo
rosIP = '192.168.160.128';
rosshutdown;
rosinit(rosIP,11311);

% establishing action client 
% action name is '/pos_joint_traj_controller/follow_joint_trajectory'
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct') 
trajGoal = rosmessage(trajAct)
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states")
jointStateMsg = jointSub.LatestMessage

% loading virtual robot that interacts with gazebo robot
UR5e = loadrobot('universalUR5e', DataFormat="row")

% controlling joints
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

% establishing inverse kinematics for trajectory
ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation


jointStateMsg = receive(jointSub,3) % receive current robot configuration

% initial guess of robot position
initialIKGuess = homeConfiguration(UR5e)
jointStateMsg.Name
initialIKGuess(1) = jointStateMsg.Position(4); % update configuration in initial guess
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess)

% Change x,y,z  values to have it touch the tip of the can

% Original
% gripperX = -0.01;
% gripperY = 0.56;
% gripperZ = 0.34;

% Trial 1
% Colors on gazebo
% GREEN 
gripperX = -0.06;
% RED
gripperY = 0.78;
% BLUE: decreasing number moves down
gripperZ = 0.15;

% motion of the grippers
gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation = [-pi/2 -pi 0]; %  [Z Y X]radians
tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

% computing inverse kinematics
[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

show(UR5e,configSoln)

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]'

trajGoal = packTrajGoal(UR5econfig,trajGoal)

sendGoal(trajAct,trajGoal)
