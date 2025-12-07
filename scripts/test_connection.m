function test_connection()
    addpath('../remote_api');  % adjust path as needed

    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);  % close any previous connections

    clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if clientID > -1
        disp('Connected to CoppeliaSim successfully!');

        % Get a handle to the left motor
        [res, left_motor] = vrep.simxGetObjectHandle( ...
            clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking);

        if res == 0
            disp('Got left motor handle. Spinning for 2 seconds...');
            vrep.simxSetJointTargetVelocity(clientID,left_motor,2, vrep.simx_opmode_oneshot);
            pause(2);
            vrep.simxSetJointTargetVelocity(clientID,left_motor,0, vrep.simx_opmode_oneshot);
        else
            warning('Could not get motor handle.');
        end

        vrep.simxFinish(clientID);
    else
        error('Failed to connect to CoppeliaSim.');
    end

    vrep.delete();
end
