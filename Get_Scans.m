clc; clear Get_Scans.m; close all ;

global sim
sim=remApi('remoteApi');    %Create remote Api object and call it sim
sim.simxFinish(-1);         %Close any unopened connections
global clientID
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5); %Create connection using the local host IP address 127.0.0.1 on the 19999 port.

if (clientID>-1) %If the client ID is greater than -1, then the connection is succesful
    disp('connected')
    
    speed = -3;    %Speed of robot (deg/s), negative value if going forward
    Reversing = false; %false when robot is not reversing
    gap = 0.7; %the distance robot must keep to any obsticle 
    scans = {};
    
    while (true)
        
        %Left motor:
        [returnCode, Left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);   %Left motor handle
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,speed,sim.simx_opmode_blocking); %spin left motor at the specified speed
        
        %Right motor:
        [returnCode, Right_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking); %Right motor handle
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,speed,sim.simx_opmode_blocking); %spin right motor at the specified speed
        
        %Front Lidar:
        [returnCode, Flidar]=sim.simxGetObjectHandle(clientID,'LaserScannerLaser_2D', sim.simx_opmode_blocking);%front Lidar handle
        [returnCode,signal] = sim.simxGetStringSignal(clientID,'measuredDataAtThisTime',sim.simx_opmode_blocking);%read front Lidar
        
        if(sim.simx_return_ok == returnCode)
            data=sim.simxUnpackFloats(signal);
            reshaped_data = reshape(data, 3, [])'; %Divide data into 3 collums representing x y and z
            reshaped_data = reshaped_data(:, 2:3);
            reshaped_data = double(reshaped_data);
            scan = lidarScan(reshaped_data);
            scans(end+1)={scan};
            disp(size(scans,2))
            if mod(size(scans,2),200)==0 %50 readings will be taken
                save('LidarScans.mat', 'scans');
                disp('saved')
                disp('Stop simulation')
            end
            
            %Call function 'Reverse':
            Reverse(clientID, sim, speed, gap, reshaped_data, Reversing,'Front', Left_Motor, Right_Motor);
        else
            disp('Start simulation')
        end   
    end
end
                sim.simxFinish(-1); %Close any unopened connections
                sim.delete();
function Reverse(clientID, sim, speed, gap, detectedPoints, Reversing, which_Lidar, Left_Motor, Right_Motor)
if strcmp(which_Lidar,'Back')
    NEWspeed = speed; %go forward if the back Lidar is detecting
else
    NEWspeed = -speed;  %go backwards if a sensor other than the back sensor is detecting
end
minimum=1;
for i = 1:size(detectedPoints,1)
    current = norm(detectedPoints(i,:));
    if (current < minimum)
        minimum = current;
    end
end

if (minimum < gap) && (Reversing == false) %if front sensor is detecting something less than 0.4m away and the robot is not already reversing
    disp(['Distance to obstacle', num2str(minimum)]) %display the detected point
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left motor
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right motor
    pause(0.5)
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,NEWspeed,sim.simx_opmode_blocking);%spin right motor at the specified speed
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,NEWspeed,sim.simx_opmode_blocking); %spin left motor at the specified speed
    Reversing = true; %Reverse
        
end
end



























