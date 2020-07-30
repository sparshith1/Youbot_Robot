function rel_pose = findPose(pose1_glob, pose2_glob)

a=cos(pose1_glob(3));
a1=sin(pose1_glob(3));
b=cos(pose2_glob(3));
b1=sin(pose2_glob(3));

T1_glob         = [a -a1 0;a1 a 0;0 0 1];
T1_glob(1:2,3)	= pose1_glob(1:2);

% Homogenous transformation of frame2 wrt to global frame
T2_glob         = [b -b1 0;b1 b 0;0 0 1];
T2_glob(1:2,3)	= pose2_glob(1:2);

% Relative pose of frame 2 wrt frame 1
rel_pose        = zeros(3,1);
T2_1            = (T1_glob) \ T2_glob;

rel_pose(1:2)   = T2_1(1:2,3);
rel_pose(3)     = atan2(T2_1(2,1), T2_1(1,1));




end








