global sim
sim=remApi('remoteApi');    %Create remote Api object and call it sim
sim.simxFinish(-1);         %Close any unopened connections 
global clientID 
clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5); %Create connection using the local host IP address 127.0.0.1 on the 19999 port.


if (clientID>-1) %If the client ID is greater than -1, then the connection is succesful
    disp('connected')
    
    %Handles
    [returnCode, Robot_position]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking);          %Position of Robot handle 
    [returnCode, Left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);   %Left motor handle 
    [returnCode, Right_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking); %right motor handle 
    [returnCode, Centre_Point]=sim.simxGetObjectHandle(clientID,'Odometry',sim.simx_opmode_blocking);                %Position of centre Point handle
    [returnCode, Lidar_Front]=sim.simxGetObjectHandle(clientID,'Lidar_Front', sim.simx_opmode_blocking);             %Front Lidar handle
    [returnCode, Lidar_Back]=sim.simxGetObjectHandle(clientID,'Lidar_Back', sim.simx_opmode_blocking);               %Back Lidar handle
    [returnCode, Lidar_Right]=sim.simxGetObjectHandle(clientID,'Lidar_Right', sim.simx_opmode_blocking);             %Right Lidar handle
    [returnCode, Lidar_Left]=sim.simxGetObjectHandle(clientID,'Lidar_Left', sim.simx_opmode_blocking);               %Left Lidar handle
    [returnCode, laserScannerHandle]=sim.simxGetObjectHandle(clientID,'Lidar1',sim.simx_opmode_blocking);            %2D Lase scanner handle
    
    %Setting stuff
    Reverse_Mode = 0; %Robot is not reversing
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,-5,sim.simx_opmode_blocking); %spin left motor at -5 deg/s
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,-5,sim.simx_opmode_blocking);%spin right motor at -5 deg/s

    
%Recommended operation modes for this function are simx_opmode_streaming (the first call) and simx_opmode_buffer (the following calls). This is the first call.  
position_nill=0;
[returnCode,position]=sim.simxGetObjectPosition(clientID,Robot_position,Centre_Point, sim.simx_opmode_streaming); %get position of robot with respect to the Centre_Position  
position = position_nill;
 
    %Doing stuff
while (true) 
        
  [returnCode,position1]=sim.simxGetObjectPosition(clientID,Robot_position,Centre_Point, sim.simx_opmode_buffer); %get position of robot with respect to the Centre_Position

        while (returnCode~=0)
            [returnCode,position1]=sim.simxGetObjectPosition(clientID,Robot_position,Centre_Point, sim.simx_opmode_buffer); %get position of robot with respect to the Centre_Position
        end

  pause (1);

  [returnCode,position2]=sim.simxGetObjectPosition(clientID,Robot_position,Centre_Point, sim.simx_opmode_buffer); %get position of robot with respect to the Centre_Position

        while (returnCode~=0)
            [returnCode,position2]=sim.simxGetObjectPosition(clientID,Robot_position,Centre_Point, sim.simx_opmode_buffer); %get position of robot with respect to the Centre_Position
        end
    
Robot_Motion = minus(position2, position1);
disp(Robot_Motion)


	

    [returnCode,signal] = sim.simxGetStringSignal(clientID,'Lidar1',sim.simx_opmode_blocking);
        if(sim.simx_return_ok == returnCode)
            data=sim.simxUnpackFloats(signal);
            disp(data);
        else disp('no command reply in the input buffer')
        end 

    
    [returnCode,detectionState_Front,detectedPoint_Front,~,~]=sim.simxReadProximitySensor(clientID,Lidar_Front,sim.simx_opmode_streaming); %read Lidar_Front 
    [returnCode,detectionState_Back,detectedPoint_Back,~,~]=sim.simxReadProximitySensor(clientID,Lidar_Back,sim.simx_opmode_streaming); %read Lidar_Back
    [returnCode,detectionState_Right,detectedPoint_Right,~,~]=sim.simxReadProximitySensor(clientID,Lidar_Right,sim.simx_opmode_streaming); %read Lidar_Right
    [returnCode,detectionState_Left,detectedPoint_Left,~,~]=sim.simxReadProximitySensor(clientID,Lidar_Left,sim.simx_opmode_streaming); %read Lidar_Left
    
    
    banana(detectionState_Front, detectedPoint_Front, Reverse_Mode,'Front', Left_Motor, Right_Motor)
    banana(detectionState_Back,detectedPoint_Back, Reverse_Mode,'Back', Left_Motor, Right_Motor)
    banana(detectionState_Right,detectedPoint_Right, Reverse_Mode, 'Right', Left_Motor, Right_Motor)
    banana(detectionState_Left,detectedPoint_Left, Reverse_Mode, 'Left', Left_Motor, Right_Motor)
    end 
end 

sim.simxFinish(-1); %Close any unopened connections 
sim.delete();

function reverse = banana(detectionState, detectedPoint, Reverse_Mode,sensor_direction, Left_Motor, Right_Motor)

    global sim
    global clientID
    %if (Reverse_Mode == 1) 
   if strcmp(sensor_direction,'Back')
       speed = -5;
   else 
       speed = 5;
   end
   
       
        if (detectionState ==1) && (norm(detectedPoint) < 0.4) && (Reverse_Mode == 0) %if Lidar_Front is detecting something less than 0.4m away and the robot is not already reversing 
        disp(norm(detectedPoint))
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
        pause(0.1)
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,speed,sim.simx_opmode_blocking); %spin left motor at 5 deg/s
        [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,speed,sim.simx_opmode_blocking);%spin right motor at 5 deg/s
        Reverse_Mode = 1; %Reverse          

          elseif (detectionState==1) && (norm(detectedPoint) > 0.8) && (Reverse_Mode == 1) %if Lidar_Front is detecting something more than 0.8m away and the robot is already reversing
          disp(norm(detectedPoint))
          [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
          [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
          return;
        
    end
   

end










