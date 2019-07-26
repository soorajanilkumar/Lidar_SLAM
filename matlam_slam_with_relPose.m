clc; clear; close all;
%% Initialization

addpath('runData')
testnum = 3062019;        %to be set accordingly to the test data
useScan_noisy = true; % true if you want to use noisy scans, false for ideal scans
if useScan_noisy
    load(['lidarScan_noisy' num2str(testnum)]);
    scan = scanNoisy;
    clear scanNoisy
else
    load(['lidarScan_real' num2str(testnum)]); %load lidar scans
end

load(['truepose' num2str(testnum)]); 
pose = WS_pose;
% load('lidar_scan5.mat');  % to be changed according to the data file
% scan=scan3;     % to be changed according to the data file
maxLidarRange   = 10;
mapResolution   = 20;
slamAlg         = robotics.LidarSLAM(mapResolution, maxLidarRange);


slamAlg.LoopClosureThreshold        = 210*1; % Threshold on the score from the scan matching algorithm for accepting loop closures, specified as a positive scalar. Higher thresholds correspond to a better match, but scores vary based on sensor data.
slamAlg.LoopClosureSearchRadius     = 10; % Search radius for loop closure detection, specified as a positive scalar. Increasing this radius affects performance by increasing search time. Tune this distance based on your environment and the expected robot trajectory.
slamAlg.LoopClosureMaxAttempts      = 1; % Number of attempts at finding looping closures
slamAlg.LoopClosureAutoRollback     = true; % Allow automatic rollback of added loop closures, specified as true or false. The SLAM object tracks the residual error returned by the OptimizationFcn. If it detects a sudden change in the residual error and this property is true, it rejects (rolls back) the loop closure.
slamAlg.OptimizationInterval        = 1 ; % Number of loop closures accepted to trigger optimization, specified as a positive integer. By default, the PoseGraph is optimized every time LidarSLAM adds a loop closure.
slamAlg.MovementThreshold           = [0 0];

scanStep        = 20;  % to read only 1 in scanStep scans for SLAM after the first 10 scans.
init_scan_no    = 10; %to read all the first 'init_scan_no' scans




%% to check if adding scans is successful
tic
for i=1:init_scan_no
    
    if i == 1
        relPose = zeros(3,1);
        prev_i  = i;
    else
        relPose = findRelPose(pose(prev_i,:)', pose(i,:)');
        prev_i  = i;
    end
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan{i},relPose);
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end
elapTime1 = toc;
%%
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

%% 
firstTimeLCDetected = false;

fprintf('\nCounter: ')
figure;
tic
for i=init_scan_no:scanStep :length(scan)
    
    if i == 1 
        relPose = zeros(3,1);
        prev_i  = i;
    else
        relPose = findRelPose(pose(prev_i,:)', pose(i,:)');
        prev_i  = i;
    end
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan{i}, relPose);
    if ~isScanAccepted
        continue;
    end
    
    %for counter
    if i>1
        for j=0:log10(i-1)
            fprintf('\b'); % delete previous counter display
        end
    end
    fprintf('%d', i);
    pause(.05); % allows time for display to update
    
    
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph);
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
elapTime2 =toc;
elapTimeTotal = elapTime1 + elapTime2
fprintf('\n')
title('First loop closure');

%% Visualize the Constructed Map and Trajectory of the Robot
figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});



