function Data_Collection()
    clc; clear; close all;

    addpath('../remote_api');  % adjust path
    addpath('../scripts');

    %==== CONNECT TO COPPELIASIM ====%

    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);

    clientID = vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

    if clientID <= -1
        error('Failed connecting to CoppeliaSim.');
    end
    disp('Connected to CoppeliaSim for DATA COLLECTION.');

    %==== GET HANDLES ====%

    % Motors
    [~, leftMotor]  = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking);
    [~, rightMotor] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);

    % Robot & Goal
    [~, robotHandle] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx', vrep.simx_opmode_blocking);
    [~, goalHandle]  = vrep.simxGetObjectHandle(clientID,'Goal',           vrep.simx_opmode_blocking);

    % Ultrasonic sensors
    sensorNames = arrayfun(@(i)sprintf('Pioneer_p3dx_ultrasonicSensor%d',i),1:16,'uni',false);
    sensorHandles = zeros(1,16);
    for i = 1:16
        [~, sensorHandles(i)] = vrep.simxGetObjectHandle(clientID,sensorNames{i},vrep.simx_opmode_blocking);
        % start streaming
        vrep.simxReadProximitySensor(clientID,sensorHandles(i),vrep.simx_opmode_streaming);
    end

    pause(0.5); % allow streaming to initialize

    %==== KEYBOARD CONTROL SETUP ====%

    global key_input;
    key_input = [0 0]; % [forward_turn, turn]

    fig = figure('Name','Manual Driving - Click here then use arrow keys','NumberTitle','off');
    set(fig, 'WindowKeyPressFcn',  @KeyDown);
    set(fig, 'WindowKeyReleaseFcn',@KeyUp);

    training_data = [];

    disp('Drive the robot with arrow keys. Close the window to stop logging.');

    %==== MAIN LOOP ====%

    dt = 0.1;  % log period (s)

    while ishandle(fig)
        % 1) Read sensors
        distances = readAllSonars(vrep, clientID, sensorHandles);

        % Grouped features: S1, S2, S3
        S1 = min(distances(1:5));     % left group
        S2 = min(distances(6:11));    % front group
        S3 = min(distances(12:16));   % right group

        % 2) Heading error
        heading_error = computeHeadingError(vrep, clientID, robotHandle, goalHandle);

        % 3) Compute manual wheel speeds from key_input
        forward = key_input(1);
        turn    = key_input(2);

        vL = forward - turn;
        vR = forward + turn;

        % Limit speeds
        maxSpeed = 2;
        vL = max(min(vL, maxSpeed), -maxSpeed);
        vR = max(min(vR, maxSpeed), -maxSpeed);

        % 4) Send speeds to robot
        vrep.simxSetJointTargetVelocity(clientID,leftMotor, vL, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,rightMotor,vR, vrep.simx_opmode_oneshot);

        % 5) Log only when robot is moving
        if abs(vL) > 1e-3 || abs(vR) > 1e-3
            row = [S1 S2 S3 heading_error vL vR];
            training_data = [training_data; row];
        end

        pause(dt);
    end

    %==== SAVE DATA ====%

    if ~exist('../data','dir'); mkdir('../data'); end
    save('../data/training_data24.mat','training_data');
    disp('Saved training_data24.mat');

    % Stop robot
    vrep.simxSetJointTargetVelocity(clientID,leftMotor, 0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,rightMotor,0, vrep.simx_opmode_oneshot);

    vrep.simxFinish(clientID);
    vrep.delete();
end

%================= HELPER FUNCTIONS =================%

%================= HELPER FUNCTIONS =================%

function KeyDown(~, event)
    global key_input;
    switch event.Key
        case 'uparrow'
            key_input(1) = 2;   % forward
        case 'downarrow'
            key_input(1) = -2;  % reverse
        case 'leftarrow'
            key_input(2) = 1;   % turn left
        case 'rightarrow'
            key_input(2) = -1;  % turn right
    end
end

function KeyUp(~, event)
    global key_input;
    switch event.Key
        case {'uparrow','downarrow'}
            key_input(1) = 0;
        case {'leftarrow','rightarrow'}
            key_input(2) = 0;
    end
end

function distances = readAllSonars(vrep, clientID, sensorHandles)
    % Returns sonar distances, saturated to a maximum range (no more 5s)
    n = numel(sensorHandles);
    maxRange = 1.0;                     % meters – choose range you care about
    distances = ones(1,n) * maxRange;   % default = "far but in-range"

    for i = 1:n
        [res, detected, point, ~, ~] = vrep.simxReadProximitySensor( ...
            clientID, sensorHandles(i), vrep.simx_opmode_buffer);
        if res == 0 && detected
            d = norm(point);
            if d > maxRange
                d = maxRange;           % saturate
            end
            distances(i) = d;
        end
    end
end

function heading_error = computeHeadingError(vrep, clientID, robotHandle, goalHandle)
    % Robot position and orientation
    [~, robotPos] = vrep.simxGetObjectPosition(clientID,robotHandle,-1,vrep.simx_opmode_blocking);
    [~, robotOri] = vrep.simxGetObjectOrientation(clientID,robotHandle,-1,vrep.simx_opmode_blocking);
    robotYaw = robotOri(3); % assuming z axis is up

    % Goal position
    [~, goalPos] = vrep.simxGetObjectPosition(clientID,goalHandle,-1,vrep.simx_opmode_blocking);

    vecToGoal = [goalPos(1)-robotPos(1), goalPos(2)-robotPos(2)];
    angleToGoal = atan2(vecToGoal(2), vecToGoal(1));

    heading_error = angleToGoal - robotYaw;

    % wrap to [-pi, pi]
    heading_error = atan2(sin(heading_error), cos(heading_error));
    limit = pi/2;  % 90 degrees
    heading_error = max(min(heading_error,  limit), -limit);
end
