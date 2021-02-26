sim=remApi('remoteApi');    %Create remote Api object and call it sim
sim.simxFinish(-1);         %Close any unopened connections 
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); %Create connection using the local host IP address 127.0.0.1 on the 19999 port.

if (clientID>-1) %If the client ID is greater than -1, then the connection is succesful
    disp('connected')
    
    %Handles
    [returnCode, Robot_position]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);          %robot handle
    [returnCode, Left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);   %left motor handle
    [returnCode, Right_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking); %right motor handle
    [returnCode, Lidar]=sim.simxGetObjectHandle(clientID,'Lidar', sim.simx_opmode_blocking);                         %lidar handle
    [returnCode,Centre_Point]=sim.simxGetObjectHandle(clientID,'Odometry',sim.simx_opmode_blocking);                 %Centre_Point handle

    
    
    
    %Setting stuff
    Reverse_Mode = 0; %Robot is not reversing
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,5,sim.simx_opmode_blocking); %spin left motor at 5 deg/s
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,5,sim.simx_opmode_blocking);%spin right motor at 5 deg/s
    
    
 
    %Doing stuff
    while (true) 
        
    [returnCode,position]=sim.simxGetObjectPosition(clientID,Robot_position,Centre_Point, sim.simx_opmode_streaming); %get position of robot with respect to the Centre_Position
    disp(position)
    
    [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,Lidar,sim.simx_opmode_streaming); %read LIDAR    
      
        if (detectionState==1) && (norm(detectedPoint) < 0.4) && (Reverse_Mode == 0) %if Lidar is detecting something less than 0.4m away and the robot is not reversing 
        disp(norm(detectedPoint))
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
        pause(0.1)
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,-5,sim.simx_opmode_blocking); %spin left motor at -5 deg/s
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,-5,sim.simx_opmode_blocking);%spin right motor at -5 deg/s
        Reverse_Mode = 1; %Reverse          

          elseif (detectionState==1) && (norm(detectedPoint) > 0.8) && (Reverse_Mode == 1) %if Lidar is detecting somethingS more than 0.8m away and the robot is reversing
          disp(norm(detectedPoint))
          [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
          [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
          break;
        end 
    end 
end 

sim.simxFinish(-1); %Close any unopened connections 

sim.delete();
