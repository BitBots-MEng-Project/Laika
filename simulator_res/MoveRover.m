sim=remApi('remoteApi');    %Create remote Api object and call it sim
sim.simxFinish(-1);         %Close any unopened connections 
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5); %Create connection using the local host IP address 127.0.0.1 on the 19999 port.

if (clientID>-1) %If the client ID is greater than -1, then the connection is succesful
    disp('connected')
    
    %Handle
    [returnCode, Left_Motor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking);
    [returnCode, Front_Sensor]=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5', sim.simx_opmode_blocking);
    [returnCode, camera]=sim.simxGetObjectHandle(clientID,'Vision_sensor', sim.simx_opmode_blocking);
    
    %other code
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0.1,sim.simx_opmode_blocking);
    [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,Front_Sensor,sim.simx_opmode_streaming);
    [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,1,sim.simx_opmode_streaming);
    
    for i=1:50
        [returnCode,detectionState,detectedPoint,~,~]=sim.simxReadProximitySensor(clientID,Front_Sensor,sim.simx_opmode_buffer);
        [returnCode,resolution,image]=sim.simxGetVisionSensorImage2(clientID,camera,1,sim.simx_opmode_buffer);
        imshow(image)
        disp(norm(detectedPoint));
        pause(0.1);
    end 
    
    [returnCode]=sim.simxSetJointTargetVelocity(clientID,Left_Motor,0,sim.simx_opmode_blocking); %Turn off actuator
   
    sim.simxFinish(-1); %Close any unopened connections 
end

sim.delete();
