function Autonomous_Run_Final()
    clc; clear; close all;

    addpath('../remote_api');
    addpath('../fis_models');

    % Load ANFIS models
    % Ensure you have run the 'fix_fis_ranges.m' script first!
    fis_left  = readfis('fis_left');
    fis_right = readfis('fis_right');

    % Connect to CoppeliaSim
    vrep = remApi('remoteApi');
    vrep.simxFinish(-1);
    clientID = vrep.simxStart('127.0.0.1',19999,true,true,2000,5);

    if clientID <= -1
        error('Could not connect to CoppeliaSim.');
    end
    disp('Connected for AUTONOMOUS RUN.');

    % Handles
    [~, leftMotor]   = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',  vrep.simx_opmode_blocking);
    [~, rightMotor]  = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking);
    [~, robotHandle] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',            vrep.simx_opmode_blocking);
    [~, goalHandle]  = vrep.simxGetObjectHandle(clientID,'Goal',                    vrep.simx_opmode_blocking);

    % Sensors
    sensorNames   = arrayfun(@(i)sprintf('Pioneer_p3dx_ultrasonicSensor%d',i),1:16,'uni',false);
    sensorHandles = zeros(1,16);
    for i = 1:16
        [~, sensorHandles(i)] = vrep.simxGetObjectHandle(clientID,sensorNames{i},vrep.simx_opmode_blocking);
        % Start streaming sensors immediately
        vrep.simxReadProximitySensor(clientID,sensorHandles(i),vrep.simx_opmode_streaming);
    end

    % Start Streaming Position/Orientation Data
    vrep.simxGetObjectPosition(clientID, robotHandle, -1, vrep.simx_opmode_streaming);
    vrep.simxGetObjectOrientation(clientID, robotHandle, -1, vrep.simx_opmode_streaming);
    vrep.simxGetObjectPosition(clientID, goalHandle, -1, vrep.simx_opmode_streaming);

    % Allow time for the data stream to fill the buffer
    disp('Initializing data streams...');
    pause(0.5);

    dt       = 0.1;
    maxSpeed = 2.0; 
    maxTime  = 180;

    % FIS input ranges (used for clamping)
    S1_range = fis_left.Inputs(1).Range;
    S2_range = fis_left.Inputs(2).Range;
    S3_range = fis_left.Inputs(3).Range;

    openThreshold = 0.8;  
    baseSpeed     = 1.5; 
    k_turn        = 2.0; 

    tStart = tic;
    disp('Running autonomous navigation...');

    while toc(tStart) < maxTime

        % -------- Read sensors (Fast Buffer Read) --------
        distances = readAllSonars(vrep, clientID, sensorHandles);
        S1_raw = min(distances(1:5));     
        S2_raw = min(distances(6:11));    
        S3_raw = min(distances(12:16));   

        % -------- Compute Heading (Fast Buffer Read) --------
        heading_error = computeHeadingError(vrep, clientID, robotHandle, goalHandle);

        % -------- Emergency Reflex (< 15cm) --------
        if S1_raw < 0.15 || S2_raw < 0.15 || S3_raw < 0.15
             disp('CRITICAL PROXIMITY: Reversing...');
             vL = -0.5;
             vR = -0.5;
             if S2_raw < 0.15 
                 vL = -0.5; vR = -1.0; % Twist back if front is hit
             end
        else
            % Normal Logic
            isOpen = (S1_raw > openThreshold) && (S2_raw > openThreshold) && (S3_raw > openThreshold);

            if isOpen
                % GOAL-SEEKING MODE
                limit = pi; 
                heading_error = min(max(heading_error, -limit), limit);
                turn = k_turn * heading_error;

                vL = baseSpeed - turn;
                vR = baseSpeed + turn;

                vL = max(min(vL, maxSpeed), -maxSpeed);
                vR = max(min(vR, maxSpeed), -maxSpeed);

            else
                % ANFIS OBSTACLE-AVOIDANCE MODE
                S1 = min(max(S1_raw, S1_range(1)), S1_range(2));
                S2 = min(max(S2_raw, S2_range(1)), S2_range(2));
                S3 = min(max(S3_raw, S3_range(1)), S3_range(2));

                limit = pi/2;
                heading_error = min(max(heading_error, -limit), limit);

                inputVec = [S1 S2 S3 heading_error];

                vL = evalfis(fis_left,  inputVec);
                vR = evalfis(fis_right, inputVec);

                vL = max(min(vL, maxSpeed), -maxSpeed);
                vR = max(min(vR, maxSpeed), -maxSpeed);
            end
        end

        % Send motor commands
        vrep.simxSetJointTargetVelocity(clientID,leftMotor,  vL, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID,rightMotor, vR, vrep.simx_opmode_oneshot);

        % -------- Check Goal --------
        % I set threshold to 0.4 meters to ensure it catches the goal
        if isAtGoal(vrep, clientID, robotHandle, goalHandle, 0.4)
            disp('Goal reached! Stopping.');
            
            % STOP COMMANDS (Sent Immediately)
            vrep.simxSetJointTargetVelocity(clientID,leftMotor,  0, vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,rightMotor, 0, vrep.simx_opmode_oneshot);
            
            pause(0.1); % Ensure command executes
            break;      % Exit loop
        end

        pause(dt);
    end

    % Double check stop (Failsafe)
    vrep.simxSetJointTargetVelocity(clientID,leftMotor,  0, vrep.simx_opmode_oneshot);
    vrep.simxSetJointTargetVelocity(clientID,rightMotor, 0, vrep.simx_opmode_oneshot);

    vrep.simxFinish(clientID);
    vrep.delete();
    disp('Autonomous run finished.');
end

% -------- Helper Functions --------

function distances = readAllSonars(vrep, clientID, sensorHandles)
    n = numel(sensorHandles);
    maxRange = 1.0;
    distances = ones(1,n) * maxRange;
    for i = 1:n
        [res, detected, point, ~, ~] = vrep.simxReadProximitySensor( ...
            clientID, sensorHandles(i), vrep.simx_opmode_buffer);
        if res == vrep.simx_return_ok && detected
            d = norm(point);
            if d > maxRange, d = maxRange; end
            distances(i) = d;
        end
    end
end

function heading_error = computeHeadingError(vrep, clientID, robotHandle, goalHandle)
    [resR, robotPos] = vrep.simxGetObjectPosition(clientID,robotHandle,-1,vrep.simx_opmode_buffer);
    [~, robotOri]    = vrep.simxGetObjectOrientation(clientID,robotHandle,-1,vrep.simx_opmode_buffer);
    [resG, goalPos]  = vrep.simxGetObjectPosition(clientID,goalHandle,-1,vrep.simx_opmode_buffer);

    if resR ~= vrep.simx_return_ok || resG ~= vrep.simx_return_ok
        heading_error = 0;
        return;
    end

    robotYaw = robotOri(3);
    vecToGoal   = [goalPos(1)-robotPos(1), goalPos(2)-robotPos(2)];
    angleToGoal = atan2(vecToGoal(2), vecToGoal(1));
    heading_error = angleToGoal - robotYaw;
    heading_error = atan2(sin(heading_error), cos(heading_error));
end

function flag = isAtGoal(vrep, clientID, robotHandle, goalHandle, threshold)
    if nargin < 5, threshold = 0.3; end
    [resR, robotPos] = vrep.simxGetObjectPosition(clientID,robotHandle,-1,vrep.simx_opmode_buffer);
    [resG, goalPos]  = vrep.simxGetObjectPosition(clientID,goalHandle,-1,vrep.simx_opmode_buffer);
    
    if resR ~= vrep.simx_return_ok || resG ~= vrep.simx_return_ok
        flag = false;
        return;
    end

    d = norm([goalPos(1)-robotPos(1), goalPos(2)-robotPos(2)]);
    flag = d < threshold;
end