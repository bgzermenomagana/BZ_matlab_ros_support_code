function ret = pick(model_pose)

    %% Local variables
    traj_steps          = 10;   % Num of traj steps
    traj_duration       = 2;    % Traj duration (secs)
    tf_listening_time   = 2;    % Time (secs) to listen for transformation in ros
    
    %% 1. Get Goal|Current Pose 
    % Pick will create a cartesian trajectory from current configuration to the pose of the model. 
    
    % Convert model_pose to matlab formats.
    mat_obj_pose = ros2matlabPose(model_pose);
    
    % 1b. Current Robot Pose in Cartesian Format (if TF does not work, change strategy to joint angles).
    tftree = rostf('DataFormat','struct');

    % With this tftree object, you can see all available frames in the network:
    % tftree.AvailableFrames



    % Finally, we can get the transform we desire by calling getTransform along inputs:
    %   tftree object
    %   targetgrame: this is your reference frame
    %   sourceframe: 
    current_pose = getTransform(tftree,'base','gripper_tip_link',rostime('now'), 'Timeout', 2);


    % This tform will contain the translation and rotation. 
    % t=current_pose.Transform.Translation
    % R=current_pose.Transform.Rotation

    % Convert to matlab format
    mat_cur_pose = ros2matlabPose(current_pose);

    %% 2. Call ctraj.
    mat_traj = ctraj(mat_cur_pose,mat_obj_pose,traj_steps);
    
    %% 3. Convert to joint angles via IKs
    [mat_joint_traj,rob_joint_names] = convertPoseTraj2JointTraj(mat_traj);
    
    %% 4. Create action client, message, populate ROS trajectory goal and send
    % Instantiate the 
    pick_traj_act_client = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                                           'control_msgs/FollowJointTrajectory');
    
    % Create action goal message from client
    traj_goal = rosmessage(pick_traj_act_client); 
    
    % Convert/fill in trajectory_msgs/FollowJointTrajectory
    % TODO: update packTrajGoal
    traj_goal = convert2ROSPointVec(mat_joint_traj,rob_joint_names,traj_steps,traj_duration,traj_goal);
    %traj_goal = ros2matPackTraj(ros_points_traj,traj_goal,rob_joint_names);% TODO: need to get names out
    
    % Finally send ros trajectory with traj_steps
    [traj_result_msg,traj_result_state] = sendGoal(pick_traj_act_client,traj_goal); 
    
    %% 5. Pick if successfull (check structure of resultState). Otherwise...
    if traj_result_state
        [grip_result_msg,grip_result_state] = doGrip('pick'); %%TODO
    end
end