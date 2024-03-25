% an array with the [x,y,z,r,p,y] of your closed end-effector at the time
% it touches the top of the can
rosIP = '192.168.160.128';
rosshutdown;
rosinit(rosIP,11311);

% action client 
% action name is '/pos_joint_traj_controller/follow_joint_trajectory'
trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory','DataFormat','struct') 
trajGoal = rosmessage(trajAct)
trajAct.FeedbackFcn = []; 
jointSub = rossubscriber("/joint_states")
jointStateMsg = jointSub.LatestMessage

% loading virtual robot to interact with gazebo robot
UR5e = loadrobot('universalUR5e', DataFormat="row")
tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver
ikWeights = [0.25 0.25 0.25 0.1 0.1 .1]; % configuration weights for IK solver [Translation Orientation] see documentation

jointStateMsg = receive(jointSub,3) % receive current robot configuration

initialIKGuess = homeConfiguration(UR5e)

jointStateMsg.Name

initialIKGuess(1) = jointStateMsg.Position(4); % update configuration in initial guess
initialIKGuess(2) = jointStateMsg.Position(3);
initialIKGuess(3) = jointStateMsg.Position(1);
initialIKGuess(4) = jointStateMsg.Position(5);
initialIKGuess(5) = jointStateMsg.Position(6);
initialIKGuess(6) = jointStateMsg.Position(7);
show(UR5e,initialIKGuess)

% Change x,y,z values to have gripper move
% at a constant height directly on top of the rCan3
% straight top-down to hug the top pf rCan3 ready to pick it up

% Original
% gripperX = -0.01;
% gripperY = 0.56;
% gripperZ = 0.34;

% Colors on gazebo
% GREEN: decreasing number moves right
gripperX = 0.1;
% RED: decreasing number moves forward
gripperY = 0.68;
% BULE: decreasing number moves down
gripperZ = 0.3;

gripperTranslation = [gripperX gripperY gripperZ];
gripperRotation = [-pi/2 -pi 0]; %  [Z Y X] radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

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

% Summary for client
% grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory','control_msgs/FollowJointTrajectory', 'DataFormat', 'struct')
% gripGoal    = rosmessage(grip_client);
% gripPos     = 0;
% gripGoal    = packGripGoal(gripPos,gripGoal)
% sendGoal(grip_client,gripGoal)
