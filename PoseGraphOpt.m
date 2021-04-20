%This code will carry out Pose Graph Optimisation 
%The output will be a map of the environment using the lidar scans and the trajectory of the robot
%Run Get_Scans.m first to get the Lidar scans

clear PoseGraphOpt.m; close all ;

load('LidarScans.mat'); %This file gets created in Get_Scans.m. It contains laser scans collected from a mobile robot in the selected environment.
maxLidarRange = 8; %Set the max lidar range slightly smaller than the max scan range (20m)
mapResolution = 20; %gives 5cm precision
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%First loop closure:
figure; %This plot shows overlaid scans and an optimized pose graph for the first loop closure. A loop closure edge is added as a red link.
firstTimeLCDetected = false;
for i=10:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        continue;
    end
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First loop closure');

%Visualize the Constructed Map and Trajectory of the Robot:
figure;
show(slamAlg);
hold on;
show(slamAlg.PoseGraph); 
hold off;
title({'Map of the Environment for ',slamAlg.LoopClosureThreshold,'Loop closures and a search radius of ', slamAlg.LoopClosureSearchRadius});

% Build Occupancy Grid Map:
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
figure;
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title({'Occupancy Grid Map Built Using Lidar SLAM for ',slamAlg.LoopClosureThreshold,'Loop closures and a search radius of ', slamAlg.LoopClosureSearchRadius});


