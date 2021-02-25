sim=remApi('remoteApi');    %Create remote Api object and call it sim
sim.simxFinish(-1);         %Close any unopened connections 
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); %Create connection using the local host IP address 127.0.0.1 on the 19999 port.

if (clientID>-1) %If the client ID is greater than -1, then the connection is succesful
    disp('connected')
    
    %Handles
    [returnCode, Left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking); %left motor handle
    [returnCode, Right_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking); %right motor handle
    [returnCode, Lidar]=sim.simxGetObjectHandle(clientID,'Lidar', sim.simx_opmode_blocking); %lidar handle
    %[returnCode, Odometry]=sim.simxCreateDummy(clientID,0.01,[],sim.simx_opmode_blocking) %dummy handle
    [returnCode,Robot_position]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking); %robot handle
    
    retreating = 0;
   
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,5,sim.simx_opmode_blocking); %spin left motor at 5 deg/s
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,5,sim.simx_opmode_blocking); %spin right motor at 5 deg/s
    
    while (true) 
        
  
    [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,Lidar,sim.simx_opmode_streaming); %read LIDAR
       
            if (detectionState==1) && (norm(detectedPoint) < 0.4) && (retreating==0)
             
                disp(norm(detectedPoint))
                retreating = 1;
                
                for i= 1:50
                [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,-5,sim.simx_opmode_blocking); 
                [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,-5,sim.simx_opmode_blocking);
                
                 pause(0.1); %A command will be executed every 0.1 seconds 
                end 
                 
            end
            
            retreating = 0;
           
            
            disp(norm(detectedPoint))
%                                 

                       [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
                       [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
                        sim.simxFinish(-1); %Close any unopened connections 
                            end
            
%         elseif (retreating == 1)
%             [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
%             [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
            end
    
        
        if (detectionState==0) && (retreating==1)
            retreating =0;
        end
        
        %imshow(image)
%         disp(norm(detectedPoint));
        pause(0.1); %A command will be executed every 0.1 seconds 
     
        
    
%    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off left actuator
%    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Right_Motor,0,sim.simx_opmode_blocking); %Turn off right actuator
%     sim.simxFinish(-1); %Close any unopened connections 


sim.delete();
