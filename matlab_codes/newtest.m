clc; clear; close all;
%Loading the ROSBAG file 
bag = rosbag('scenario5_2020-07-30-16-38-16.bag');

%Parsing the bag file and taking the recorded topic
ranges = select(bag,'Topic','/base_scan');
odometry = select(bag,'Topic','/odom');

%Converting the messages in the structure format
range = readMessages(ranges,'DataFormat','struct');
odom = readMessages(odometry,'DataFormat','struct');

%Setting the Laser and map parameters the designed sensor range is 5.6
maxRange = 5.6; 
resolution = 40;

%I am using the slam function from the MATLAB 
%Setting the imperical values based on the trial and error
slamObj = lidarSLAM(resolution,maxRange);
slamObj.LoopClosureThreshold =142;    
slamObj.LoopClosureSearchRadius = 4;
slamObj.LoopClosureMaxAttempts = 3;
slamObj.MovementThreshold = [0.0 0.0];

angle = linspace(1.57 , -1.57 , 150);

%This variable is to check if the scans are getting added
init_no_scans=10;
scan_step=10;

firstTimeLCDetected = false;
index = 0;
for c=1:init_no_scans
    index=index+1;
    %To Scans array i am adding the sensor range values and its angle from
    %messages
    scans{index} = lidarScan(double(range{c,1}.Ranges),double(angle));
    
    %this loop is to get the pose during the time the range is captured
    %this is used since both the update rate is different lidar is 10 and
    %odom update rate is 100
    for d=1:length(odom)
        if ((range{c,1}.Header.Stamp.Sec) == (odom{d,1}.Header.Stamp.Sec)) %&& ((range{c,1}.Header.Stamp.Nsec) == (odom{d,1}.Header.Stamp.Nsec))
            %adding the current pose value to the array current_poses
            current_poses(index,:)=[odom{d,1}.Pose.Pose.Position.X odom{d,1}.Pose.Pose.Position.Y odom{d,1}.Pose.Pose.Orientation.W];
            
        end
    end
    
    %Calculating the relative pose
    if index==1
        relPose=zeros(3,1);
        prev_i  = index;
    else
        % calling the function to calculate the relative pose
        relPose = findPose(current_poses(index,:)',current_poses(prev_i,:)');
        prev_i  = index;
    end
    %Adding scans and relative pose to the Slam Function object
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj,scans{index},(relPose)');
    %this is to check if the scans are getting added
    if isScanAccepted
        fprintf('Added scan %d \n', c);
    end
end

figure;
show(slamObj);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});    
index1=0;    

%The below function is to add rest of the sensor values
for d=init_no_scans:scan_step:length(range)
    
    index=index+1;
    scans{index} = lidarScan(double(range{d,1}.Ranges),double(angle));
   
    for e=1:length(odom)
        if ((range{d,1}.Header.Stamp.Sec) == (odom{e,1}.Header.Stamp.Sec)) %&& ((range{c,1}.Header.Stamp.Nsec) == (odom{d,1}.Header.Stamp.Nsec))
            current_poses(index,:)=[odom{e,1}.Pose.Pose.Position.X odom{e,1}.Pose.Pose.Position.Y odom{e,1}.Pose.Pose.Orientation.W];
            
        end
    end
    if index==1
        relPose=zeros(3,1);
        prev_i  = index;
    else
        relPose = findPose(current_poses(index,:)',current_poses(prev_i,:)');
        prev_i  = index;
    end
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamObj,scans{index},(relPose)');
    if ~isScanAccepted
        continue;
    end
    if d > 1
        for j=0:log10(d-1)
            fprintf('\b'); % delete previous counter display
        end
    end
    fprintf('%d', d);
    pause(.05);
    
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamObj, 'Poses', 'off');
        hold on;
        show(slamObj.PoseGraph);
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end

figure('Name','Graphs and Plots of the Scenario');

for f=1:length(odom)
    Position=[odom{f,1}.Pose.Pose.Position.X odom{f,1}.Pose.Pose.Position.Y odom{f,1}.Pose.Pose.Orientation.W];
    subplot(2,2,3);
    hold on 
    plot(Position(1),Position(2),'.')
    title('Dead Reckoning path of the Robot');
end

subplot(2,2,1);
hold on
show(slamObj);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});
 
[scansSLAM,poses] = scansAndPoses(slamObj);
occMap = buildMap(scansSLAM,poses,resolution,maxRange);
%figure
subplot(2,2,2);
hold on
show(occMap)
title('Occupancy Map of Environment')



