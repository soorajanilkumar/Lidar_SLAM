function rel_pose = findRelPose(pose1_glob, pose2_glob)
% rel_pose  Find the relative pose. 
% findRelPose(pose1_glob, pose2_glob)
% Inputs should be pose (local robot frame) in global frame. (col vector)

% Homogenous transformation of frame1 wrt to global frame
T1_glob         = rotz(pose1_glob(3));
T1_glob(1:2,3)	= pose1_glob(1:2);

% Homogenous transformation of frame2 wrt to global frame
T2_glob         = rotz(pose2_glob(3));
T2_glob(1:2,3)	= pose2_glob(1:2);

% Relative pose of frame 2 wrt frame 1
rel_pose        = zeros(3,1);
T2_1            = inv(T1_glob) * T2_glob;

rel_pose(1:2)   = T2_1(1:2,3);
rel_pose(3)     = atan2(T2_1(2,1), T2_1(1,1)); 

end